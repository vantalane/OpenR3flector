#!/usr/bin/env python3

"""
	This file is part of OpenR3flector project.

	Copyright (C) 2026 vantalane <mete@kestech.net>
	SPDX-License-Identifier: GPL-3.0-or-later

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
"""

"""
Sky SDR scanner — SoapySDR direct API + UART dish control

Usage:
  python sky_sdr_scan.py
  python sky_sdr_scan.py --az-pixels 72 --el-pixels 9
  python sky_sdr_scan.py --plot-only scan_20240101_120000/
  python sky_sdr_scan.py --debug

SoapySDR Python bindings required (e.g. python-soapysdr from your distro).
Dish must be homed ('h') before scanning.
"""

import argparse, os, time, serial, json, glob
from datetime import datetime, timezone
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CF32

# ── Config ────────────────────────────────────────────────────────────────────
SERIAL_PORT  = "/dev/ttyUSB1"
BAUD         = 115200

AZ_START     = 130.0
AZ_STOP      = 230.0
EL_START     = 10.0
EL_STOP      = 60.0
AZ_PIXELS    = 50
EL_PIXELS    = 25

# SDR — opened once for the whole scan
SOAPY_DRIVER = ""         # "" = auto-detect; or "rtlsdr", "hackrf", "airspy" …
FREQ_START   = 950e6      # Hz — IF band start  (LNB LO 9.75 GHz → 10.70 GHz = 950 MHz)
FREQ_STOP    = 1950e6     # Hz — IF band stop   (LNB LO 9.75 GHz → 11.70 GHz = 1950 MHz)
FREQ_OVERLAP = 0.1        # fractional overlap between adjacent tuning steps (avoids filter edge rolloff)
SAMPLE_RATE  = 25e6       # Hz — must be ≤ device max (LimeSDR Mini: 30.72 MHz)
SOAPY_GAIN   = 30         # dB
FFT_SIZE     = 1024       # bins per individual FFT window
AVERAGES     = 16         # FFT windows averaged per tuning step

POS_THRESH   = 0.5       # deg: both axes must be within this of target
VEL_THRESH   = 0.1        # deg/s: total speed must be below this
MAX_WAIT     = 10.0       # s: give up and measure anyway
# ─────────────────────────────────────────────────────────────────────────────

DEBUG = False


# ── Serial helpers ────────────────────────────────────────────────────────────

def send_cmd(ser, cmd):
    ser.write(cmd.encode())
    ser.flush()


def query_motion(ser, budget_s=1.0):
    """Send 'm', return 12-float tuple within budget_s seconds, else None.

    Fields from send_motion_status() in uart_comms.c:
      [0-1]  body az/alt pos
      [2-3]  body az/alt target
      [4-5]  world az/alt pos       ← current position
      [6-7]  world az/alt target    ← device's stored target
      [8-9]  az/alt velocity
      [10-11] az/alt acceleration
    """
    ser.reset_input_buffer()
    send_cmd(ser, "m\n")
    deadline = time.monotonic() + budget_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        ser.timeout = min(0.3, remaining)   # readline never overshoots budget
        try:
            raw = ser.readline().decode(errors="replace").strip()
        except serial.SerialException:
            time.sleep(0.05)
            continue
        if DEBUG and raw:
            print(f"\n  [rx] {raw!r}", end="", flush=True)
        if "," not in raw:
            continue
        parts = raw.split(",")
        if len(parts) >= 12:
            try:
                return tuple(float(p) for p in parts[:12])
            except ValueError:
                continue
    return None


def send_goto_and_wait(ser, az, el):
    """Move to az/el and wait to settle.

    Returns (actual_az, actual_el) — the position from the last successful
    motion query.  Falls back to the commanded (az, el) if no UART data at all.
    """
    send_cmd(ser, f"g{az:.2f},{el:.2f}\n")
    print(f"  GOTO az={az:.1f}° el={el:.1f}°", end="", flush=True)
    deadline = time.monotonic() + MAX_WAIT
    last_motion = None
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        motion = query_motion(ser, min(1.0, remaining))
        if motion is None:
            print("?", end="", flush=True)   # ? = no UART response
            continue
        last_motion = motion
        curr_az, curr_alt = motion[4], motion[5]
        tgt_az,  tgt_alt  = motion[6], motion[7]   # device's own stored target
        az_vel,  alt_vel  = motion[8], motion[9]
        az_err = abs(curr_az - tgt_az)
        el_err = abs(curr_alt - tgt_alt)
        speed  = (az_vel**2 + alt_vel**2) ** 0.5
        if DEBUG:
            print(f"\n  [pos] curr=({curr_az:.2f},{curr_alt:.2f}) "
                  f"tgt=({tgt_az:.2f},{tgt_alt:.2f}) "
                  f"err=({az_err:.2f},{el_err:.2f}) spd={speed:.2f}",
                  end="", flush=True)
        if az_err < POS_THRESH and el_err < POS_THRESH and speed < VEL_THRESH:
            print(f" → settled (Δaz={az_err:.2f}° Δel={el_err:.2f}°)")
            return curr_az, curr_alt
        print(".", end="", flush=True)   # . = data received, not yet settled

    if last_motion is not None:
        actual_az, actual_el = last_motion[4], last_motion[5]
        print(f" → timeout (actual az={actual_az:.2f}° el={actual_el:.2f}°)")
        return actual_az, actual_el

    print(" → timeout (no position data)")
    return az, el   # commanded position as last resort


# ── SDR helpers ───────────────────────────────────────────────────────────────

def tuning_plan():
    """Return (centers, trim) for a sweep from FREQ_START to FREQ_STOP.

    centers : array of SDR center frequencies in Hz
    trim    : number of edge bins to drop on each side of every FFT to avoid
              the analog filter rolloff zone (set by FREQ_OVERLAP)
    """
    trim = int(FFT_SIZE * FREQ_OVERLAP / 2)
    step = SAMPLE_RATE * (1.0 - FREQ_OVERLAP)
    first = FREQ_START + SAMPLE_RATE / 2.0
    last  = FREQ_STOP  - SAMPLE_RATE / 2.0
    centers = np.arange(first, last + step * 0.5, step)
    return centers, trim


def open_sdr(first_center):
    args = f"driver={SOAPY_DRIVER}" if SOAPY_DRIVER else ""
    sdr = SoapySDR.Device(args)
    sdr.setSampleRate(SOAPY_SDR_RX, 0, SAMPLE_RATE)
    sdr.setFrequency(SOAPY_SDR_RX, 0, float(first_center))
    sdr.setGain(SOAPY_SDR_RX, 0, SOAPY_GAIN)
    rx_stream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
    sdr.activateStream(rx_stream)
    return sdr, rx_stream


def close_sdr(sdr, rx_stream):
    sdr.deactivateStream(rx_stream)
    sdr.closeStream(rx_stream)


def measure_spectrum(sdr, rx_stream, centers, trim):
    """Sweep through center frequencies; return concatenated power spectrum in dBFS.

    For each center frequency:
      - retune SDR, wait 10 ms for PLL lock
      - read AVERAGES+1 FFT windows (discard first to drain stale buffer)
      - average |FFT|² over AVERAGES windows
      - drop `trim` edge bins on each side (filter rolloff zone)
    Returns array of shape (total_bins,) or None on read failure.
    """
    total  = FFT_SIZE * (AVERAGES + 1)
    buf    = np.zeros(total, dtype=np.complex64)
    window = np.hanning(FFT_SIZE)
    all_bins = []

    for i, cf in enumerate(centers):
        sdr.setFrequency(SOAPY_SDR_RX, 0, float(cf))
        time.sleep(0.010)                              # PLL relock

        sr = sdr.readStream(rx_stream, [buf], total, timeoutUs=int(5e6))
        if sr.ret < total:
            return None

        samples = buf[FFT_SIZE:]                       # skip stale first window
        accu = np.zeros(FFT_SIZE)
        for j in range(AVERAGES):
            chunk = samples[j * FFT_SIZE:(j + 1) * FFT_SIZE]
            accu += np.abs(np.fft.fft(chunk * window, FFT_SIZE)) ** 2
        accu /= AVERAGES
        power = np.fft.fftshift(10.0 * np.log10(accu + 1e-20))

        lo = trim if i > 0               else 0
        hi = FFT_SIZE - trim if i < len(centers) - 1 else FFT_SIZE
        all_bins.append(power[lo:hi])

    return np.concatenate(all_bins)


def freq_axis_mhz(centers, fft_size, sample_rate, trim):
    """Reconstruct the frequency axis matching the concatenated spectrum."""
    all_freqs = []
    for i, cf in enumerate(centers):
        f = (np.fft.fftshift(np.fft.fftfreq(fft_size, 1.0 / sample_rate)) + cf) / 1e6
        lo = trim if i > 0               else 0
        hi = fft_size - trim if i < len(centers) - 1 else fft_size
        all_freqs.append(f[lo:hi])
    return np.concatenate(all_freqs)


# ── Grid ─────────────────────────────────────────────────────────────────────

def build_grid(az_pixels, el_pixels):
    azs = np.linspace(AZ_START, AZ_STOP, az_pixels, endpoint=False)
    els = np.linspace(EL_START, EL_STOP, el_pixels)
    return azs, els


def point_filename(run_dir, az, el):
    return os.path.join(run_dir, f"p_az{az:010.4f}_el{el:08.4f}.npz")


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_results(run_dir, out_png):
    from scipy.interpolate import griddata

    files = sorted(glob.glob(os.path.join(run_dir, "p_*.npz")))
    if not files:
        print("No data to plot.")
        return

    with open(os.path.join(run_dir, "scan_config.json")) as f:
        cfg = json.load(f)

    fft_size    = cfg.get("fft_size",        FFT_SIZE)
    sample_rate = cfg.get("sample_rate_hz",  SAMPLE_RATE)
    freq_start  = cfg.get("freq_start_hz",   FREQ_START)
    freq_stop   = cfg.get("freq_stop_hz",    FREQ_STOP)
    overlap     = cfg.get("freq_overlap",    FREQ_OVERLAP)
    trim        = int(fft_size * overlap / 2)
    step        = sample_rate * (1.0 - overlap)
    centers     = np.arange(freq_start + sample_rate / 2,
                            freq_stop  - sample_rate / 2 + step * 0.5,
                            step)
    freqs = freq_axis_mhz(centers, fft_size, sample_rate, trim)

    records = []
    for fpath in files:
        name = os.path.basename(fpath)[2:-4]   # strip "p_" and ".npz"
        az_part, el_part = name.split("_el")
        cmd_az = float(az_part.replace("az", ""))
        cmd_el = float(el_part)
        data = np.load(fpath)
        spec = data["spec"]
        # Use actual measured position; fall back to commanded if not stored
        az = float(data["actual_az"]) if "actual_az" in data else cmd_az
        el = float(data["actual_el"]) if "actual_el" in data else cmd_el
        records.append((az, el, spec))

    az_vals = np.array([r[0] for r in records])
    el_vals = np.array([r[1] for r in records])
    spectra = np.array([r[2] for r in records])   # (N, fft_size)

    integ = spectra.mean(axis=1)

    n_total_bins = len(freqs)
    n_maps   = min(8, n_total_bins)
    bin_idxs = np.linspace(0, n_total_bins - 1, n_maps, dtype=int)
    az0, az1 = cfg.get("az_start", AZ_START), cfg.get("az_stop",  AZ_STOP)
    el0, el1 = cfg.get("el_start", EL_START), cfg.get("el_stop",  EL_STOP)

    # Dense output grid — 4× the measured pixel count, min 256×64
    img_w = max(cfg.get("az_pixels", AZ_PIXELS) * 4, 256)
    img_h = max(cfg.get("el_pixels", EL_PIXELS) * 4, 64)
    az_grid = np.linspace(az0, az1, img_w)
    el_grid = np.linspace(el0, el1, img_h)
    AZ, EL  = np.meshgrid(az_grid, el_grid)
    points  = np.column_stack([az_vals, el_vals])

    def to_image(vals):
        img = griddata(points, vals, (AZ, EL), method="linear")
        if np.any(np.isnan(img)):   # fill convex-hull gaps with nearest
            img_nn = griddata(points, vals, (AZ, EL), method="nearest")
            img = np.where(np.isnan(img), img_nn, img)
        return img

    def image_map(ax, vals, title, cbar_label):
        im = ax.imshow(to_image(vals), origin="lower", aspect="auto",
                       cmap="inferno", extent=[az0, az1, el0, el1],
                       interpolation="bilinear")
        plt.colorbar(im, ax=ax, label=cbar_label)
        ax.set_title(title, fontsize=8)
        ax.set_xlabel("Az (°)", fontsize=7)
        ax.set_ylabel("El (°)", fontsize=7)

    ncols = 3
    nrows = (1 + n_maps + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(6 * ncols, 4 * nrows))
    axes = list(axes.flat)

    image_map(axes[0], integ, "Integrated (all bins)", "dBFS (mean)")
    for i, bidx in enumerate(bin_idxs):
        image_map(axes[i + 1], spectra[:, bidx], f"{freqs[bidx]:.2f} MHz", "dBFS")
    for ax in axes[1 + n_maps:]:
        ax.set_visible(False)

    bin_hz = sample_rate / fft_size
    fig.suptitle(
        f"Sky RF map — {os.path.basename(run_dir)}\n"
        f"{freq_start/1e6:.0f}–{freq_stop/1e6:.0f} MHz  "
        f"({len(centers)} tuning steps × {fft_size} bins, {bin_hz/1e3:.1f} kHz/bin, "
        f"{n_total_bins} bins total)",
        fontsize=10)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    print(f"Plot saved → {out_png}  ({len(records)} points, {n_total_bins} bins each)")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Sky SDR power scan")
    parser.add_argument("--port",      default=SERIAL_PORT)
    parser.add_argument("--az-pixels", type=int, default=AZ_PIXELS)
    parser.add_argument("--el-pixels", type=int, default=EL_PIXELS)
    parser.add_argument("--plot-only", metavar="DIR",
                        help="Re-plot an existing run directory and exit")
    parser.add_argument("--debug",     action="store_true",
                        help="Print raw UART lines and position data")
    args = parser.parse_args()

    global DEBUG
    DEBUG = args.debug

    if args.plot_only:
        plot_results(args.plot_only, os.path.join(args.plot_only, "sky_scan.png"))
        return

    azs, els = build_grid(args.az_pixels, args.el_pixels)
    total = len(azs) * len(els)

    run_dir = f"scan_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    os.makedirs(run_dir, exist_ok=True)

    centers, trim = tuning_plan()
    n_steps      = len(centers)
    total_bins   = sum(
        (FFT_SIZE - (trim if i > 0 else 0) - (trim if i < n_steps - 1 else 0))
        for i in range(n_steps)
    )
    dwell_ms = n_steps * (FFT_SIZE * (AVERAGES + 1) / SAMPLE_RATE + 0.010) * 1000

    with open(os.path.join(run_dir, "scan_config.json"), "w") as f:
        json.dump({
            "az_start": AZ_START, "az_stop": AZ_STOP,
            "el_start": EL_START, "el_stop": EL_STOP,
            "az_pixels": args.az_pixels, "el_pixels": args.el_pixels,
            "freq_start_hz": FREQ_START, "freq_stop_hz": FREQ_STOP,
            "freq_overlap": FREQ_OVERLAP, "sample_rate_hz": SAMPLE_RATE,
            "fft_size": FFT_SIZE, "averages": AVERAGES, "gain_db": SOAPY_GAIN,
        }, f, indent=2)

    print(f"Scan grid : {args.az_pixels} az × {args.el_pixels} el = {total} points")
    print(f"SDR       : {FREQ_START/1e6:.0f}–{FREQ_STOP/1e6:.0f} MHz, "
          f"{n_steps} tuning steps × {SAMPLE_RATE/1e6:.0f} MHz, "
          f"{SAMPLE_RATE/FFT_SIZE/1e3:.1f} kHz/bin, "
          f"dwell ≈ {dwell_ms:.0f} ms/point")
    print(f"Data      : {run_dir}/")

    sdr, rx_stream = open_sdr(centers[0])
    print("SDR ready.")

    done = 0
    try:
        with serial.Serial(args.port, BAUD, timeout=0.5) as ser:
            time.sleep(0.5)
            for row_idx, el in enumerate(els):
                row_azs = azs if row_idx % 2 == 0 else azs[::-1]
                for az in row_azs:
                    fname = point_filename(run_dir, az, el)
                    if os.path.exists(fname):
                        done += 1
                        print(f"[{done}/{total}] az={az:.1f}° el={el:.1f}° — skip")
                        continue

                    print(f"[{done+1}/{total}] az={az:.1f}° el={el:.1f}°")
                    actual_az, actual_el = send_goto_and_wait(ser, az, el)

                    spec = measure_spectrum(sdr, rx_stream, centers, trim)
                    if spec is not None:
                        print(f"  mean={spec.mean():.1f} dBFS  peak={spec.max():.1f} dBFS")
                        np.savez(fname,
                                 spec=spec.astype(np.float32),
                                 actual_az=np.float32(actual_az),
                                 actual_el=np.float32(actual_el))
                    else:
                        print("  WARNING: SDR read failed — skipping point")
                    done += 1

    except KeyboardInterrupt:
        print(f"\nInterrupted — {done}/{total} points saved.")
    finally:
        close_sdr(sdr, rx_stream)

    plot_results(run_dir, os.path.join(run_dir, "sky_scan.png"))


if __name__ == "__main__":
    main()
