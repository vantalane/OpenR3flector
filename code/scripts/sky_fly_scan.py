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
Sky fly-scan — continuous-motion SDR survey using CMD_TRACK constant-velocity sweeps.

The dish sweeps each elevation row at a steady azimuth velocity.  The SDR runs
in a background thread that captures spectra continuously; the main loop wakes
at the configured sample rate, queries the dish position over UART, and grabs
the most-recent spectrum from the thread.  Position and spectrum are within one
integration window (~0.33 ms) of each other.

Angular resolution:
  azimuth   : az_vel / sample_hz  (deg/sample)
  elevation : el_step             (deg between rows)

Usage:
  python sky_fly_scan.py
  python sky_fly_scan.py --az-vel 8 --sample-hz 10 --el-step 2 --freq 1420.405
  python sky_fly_scan.py --az-start 130 --az-stop 230 --el-start 10 --el-stop 60
  python sky_fly_scan.py --plot-only scan_fly_20240101_120000/
  python sky_fly_scan.py --debug
"""

import argparse, os, time, serial, json, glob, threading
from datetime import datetime
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CF32

# ── Config ─────────────────────────────────────────────────────────────────────
SERIAL_PORT   = "/dev/ttyUSB0"
BAUD          = 115200

AZ_START      = 0.0        # deg
AZ_STOP       = 360.0        # deg
EL_START      = 10.0         # deg
EL_STOP       = 60.0         # deg
EL_STEP_DEG   = 1          # deg between elevation rows

AZ_SCAN_VEL   = 10.0         # deg/s — dish speed during a scan row
AZ_RETURN_VEL = 20.0         # deg/s — slew speed for inter-row moves
EL_RETURN_VEL = 20.0         # deg/s

SAMPLE_HZ     = 10.0         # samples per second during a row sweep

# SDR
SOAPY_DRIVER  = ""
CENTER_FREQ   = 1420e6       # Hz
SAMPLE_RATE   = 25e6         # Hz
SOAPY_GAIN    = 30           # dB
FFT_SIZE      = 1024
AVERAGES      = 8

# Goto settle thresholds (not used during the sweep itself)
POS_THRESH    = 0.5          # deg
VEL_THRESH    = 0.1          # deg/s
MAX_WAIT      = 20.0         # s
# ───────────────────────────────────────────────────────────────────────────────

DEBUG   = False
_WINDOW = np.hanning(FFT_SIZE)


# ── Serial helpers ─────────────────────────────────────────────────────────────

def send_cmd(ser, cmd):
    ser.write(cmd.encode())
    ser.flush()


def query_motion(ser, budget_s=0.3):
    """Send 'm', parse the 12-float CSV response.  Resends on each retry.

    Fields  [4]=world_az  [5]=world_alt  [6]=target_az  [7]=target_alt
            [8]=az_vel    [9]=alt_vel    [10]=az_accel  [11]=alt_accel
    """
    deadline = time.monotonic() + budget_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        ser.reset_input_buffer()
        send_cmd(ser, "m\n")
        ser.timeout = min(0.15, remaining)
        try:
            raw = ser.readline().decode(errors="replace").strip()
        except serial.SerialException:
            time.sleep(0.02)
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


def send_goto_and_wait(ser, az, el, max_wait=MAX_WAIT):
    """Position-mode move; blocks until settled or timeout.  Returns actual (az, el)."""
    send_cmd(ser, f"g{az:.2f},{el:.2f}\n")
    print(f"  GOTO az={az:.1f}° el={el:.1f}°", end="", flush=True)
    deadline    = time.monotonic() + max_wait
    last_motion = None
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        motion = query_motion(ser, min(0.3, remaining))
        if motion is None:
            print("?", end="", flush=True)
            continue
        last_motion = motion
        curr_az, curr_el = motion[4], motion[5]
        tgt_az,  tgt_el  = motion[6], motion[7]
        az_vel,  el_vel  = motion[8], motion[9]
        az_err = abs(curr_az - tgt_az)
        el_err = abs(curr_el - tgt_el)
        speed  = (az_vel**2 + el_vel**2) ** 0.5
        if DEBUG:
            print(f"\n  [pos] ({curr_az:.2f},{curr_el:.2f}) "
                  f"err=({az_err:.2f},{el_err:.2f}) spd={speed:.2f}",
                  end="", flush=True)
        if az_err < POS_THRESH and el_err < POS_THRESH and speed < VEL_THRESH:
            print(f" → settled (Δaz={az_err:.2f}° Δel={el_err:.2f}°)")
            return curr_az, curr_el
        print(".", end="", flush=True)

    if last_motion is not None:
        print(f" → timeout (az={last_motion[4]:.2f}° el={last_motion[5]:.2f}°)")
        return last_motion[4], last_motion[5]
    print(" → timeout (no position data)")
    return az, el


def set_max_velocity(ser, az_vel, el_vel):
    send_cmd(ser, f"v{az_vel:.2f},{el_vel:.2f}\n")
    time.sleep(0.05)


def start_track(ser, az, el, az_degps, el_degps=0.0):
    """Start CMD_TRACK: firmware sweeps az at az_degps deg/s indefinitely."""
    send_cmd(ser, f"t{az:.2f},{el:.2f},{az_degps:.4f},{el_degps:.4f}\n")


# ── SDR — background spectrum thread ──────────────────────────────────────────

class SpectrumStream:
    """Captures spectra continuously in a background thread.

    readStream runs without pause so the hardware buffer never accumulates
    stale data.  Call get() to copy the most recently completed spectrum
    (at most one integration window old: AVERAGES × FFT_SIZE / SAMPLE_RATE ≈ 0.33 ms).
    """

    def __init__(self, sdr, rx_stream):
        self._sdr    = sdr
        self._rx     = rx_stream
        self._latest = None
        self._lock   = threading.Lock()
        self._stop   = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        total = FFT_SIZE * AVERAGES
        buf   = np.zeros(total, dtype=np.complex64)
        while not self._stop.is_set():
            sr = self._sdr.readStream(self._rx, [buf], total,
                                      timeoutUs=int(500_000))
            if sr.ret < total:
                continue
            chunks = buf.reshape(AVERAGES, FFT_SIZE)
            accu   = (np.abs(np.fft.fft(chunks * _WINDOW, axis=1)) ** 2).mean(axis=0)
            spec   = np.fft.fftshift(10.0 * np.log10(accu + 1e-20)).astype(np.float32)
            with self._lock:
                self._latest = spec

    def get(self):
        """Return a copy of the most recent spectrum, or None if not yet ready."""
        with self._lock:
            return None if self._latest is None else self._latest.copy()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)


def open_sdr(center_hz):
    driver_args = f"driver={SOAPY_DRIVER}" if SOAPY_DRIVER else ""
    sdr = SoapySDR.Device(driver_args)
    sdr.setSampleRate(SOAPY_SDR_RX, 0, SAMPLE_RATE)
    sdr.setFrequency(SOAPY_SDR_RX, 0, float(center_hz))
    sdr.setGain(SOAPY_SDR_RX, 0, SOAPY_GAIN)
    rx_stream   = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
    sdr.activateStream(rx_stream)
    spec_stream = SpectrumStream(sdr, rx_stream)
    return sdr, rx_stream, spec_stream


def close_sdr(sdr, rx_stream, spec_stream):
    spec_stream.stop()
    sdr.deactivateStream(rx_stream)
    sdr.closeStream(rx_stream)


# ── Az unwrap ─────────────────────────────────────────────────────────────────

def unwrap_az(prev_unwrapped, new_raw, prev_raw):
    """Accumulate az across the firmware's automatic wrap in CMD_TRACK mode."""
    diff = new_raw - prev_raw
    if diff >  180.0: diff -= 360.0
    if diff < -180.0: diff += 360.0
    return prev_unwrapped + diff


# ── Row sweep ─────────────────────────────────────────────────────────────────

def sweep_row(ser, spec_stream, el, az_start, az_end, az_vel, sample_hz):
    """Sweep one elevation row at constant velocity, sampling at sample_hz.

    At each sample tick:
      1. query_motion  — records the dish position right now over UART
      2. spec_stream.get() — grabs the freshest completed spectrum (~0.33 ms old)

    Returns (az_vals, el_vals, spectra) lists.
    """
    direction       = 1 if az_end >= az_start else -1
    scan_vel        = abs(az_vel) * direction
    sample_interval = 1.0 / sample_hz
    row_timeout     = abs(az_end - az_start) / abs(az_vel) * 3.0

    # query budget must not exceed the sample interval so it cannot block the next tick
    q_budget = min(0.15, sample_interval)

    send_goto_and_wait(ser, az_start, el)
    start_track(ser, az_start, el, scan_vel)
    time.sleep(0.1)  # let firmware process track command

    seed = query_motion(ser, 0.5)
    if seed is None:
        print("  WARNING: no UART response — skipping row")
        send_cmd(ser, f"g{az_end:.2f},{el:.2f}\n")
        return [], [], []

    prev_raw_az  = seed[4]
    unwrapped_az = seed[4]

    az_vals, el_vals, spectra = [], [], []
    row_deadline = time.monotonic() + row_timeout
    t_next       = time.monotonic()

    print(f"  sweep {az_start:.1f}°→{az_end:.1f}° @ {scan_vel:+.1f}°/s  "
          f"{sample_hz:.0f} Hz", end="", flush=True)

    while time.monotonic() < row_deadline:

        sleep_s = t_next - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        t_next += sample_interval

        # 1. position at this moment
        motion = query_motion(ser, q_budget)
        if motion is None:
            print("?", end="", flush=True)
            continue

        raw_az  = motion[4]
        curr_el = motion[5]

        unwrapped_az = unwrap_az(unwrapped_az, raw_az, prev_raw_az)
        prev_raw_az  = raw_az

        if direction > 0 and unwrapped_az >= az_end:
            break
        if direction < 0 and unwrapped_az <= az_end:
            break

        # 2. freshest spectrum from background thread
        spec = spec_stream.get()
        if spec is None:
            continue

        az_vals.append(float(unwrapped_az))
        el_vals.append(float(curr_el))
        spectra.append(spec)
        print(".", end="", flush=True)

    send_cmd(ser, f"g{az_end:.2f},{el:.2f}\n")
    print(f" [{len(az_vals)} samples]")
    return az_vals, el_vals, spectra


# ── Per-row file I/O ───────────────────────────────────────────────────────────

def row_filename(run_dir, el):
    return os.path.join(run_dir, f"row_el{el:09.4f}.npz")


def save_row(fname, az_vals, el_vals, spectra):
    np.savez(fname,
             az_vals=np.array(az_vals, dtype=np.float32),
             el_vals=np.array(el_vals, dtype=np.float32),
             spectra=np.array(spectra, dtype=np.float32))


# ── Plotting ───────────────────────────────────────────────────────────────────

def plot_results(run_dir, out_png):
    from scipy.interpolate import griddata

    files = sorted(glob.glob(os.path.join(run_dir, "row_*.npz")))
    if not files:
        print("No data to plot.")
        return

    with open(os.path.join(run_dir, "scan_config.json")) as f:
        cfg = json.load(f)

    center_hz   = cfg["center_freq_hz"]
    fft_size    = cfg.get("fft_size",       FFT_SIZE)
    sample_rate = cfg.get("sample_rate_hz", SAMPLE_RATE)
    freqs_mhz   = (np.fft.fftshift(np.fft.fftfreq(fft_size, 1.0 / sample_rate))
                   + center_hz) / 1e6

    az_all, el_all, spec_all = [], [], []
    for fpath in files:
        d = np.load(fpath)
        az_all.append(d["az_vals"])
        el_all.append(d["el_vals"])
        spec_all.append(d["spectra"])

    az_vals = np.concatenate(az_all)
    el_vals = np.concatenate(el_all)
    spectra = np.concatenate(spec_all, axis=0)

    az0 = cfg.get("az_start", AZ_START)
    az1 = cfg.get("az_stop",  AZ_STOP)
    el0 = cfg.get("el_start", EL_START)
    el1 = cfg.get("el_stop",  EL_STOP)

    # 4 output pixels per degree in both axes
    img_w = max(200, int((az1 - az0) * 4))
    img_h = max(50,  int((el1 - el0) * 4))
    az_grid = np.linspace(az0, az1, img_w)
    el_grid = np.linspace(el0, el1, img_h)
    AZ_G, EL_G = np.meshgrid(az_grid, el_grid)
    points  = np.column_stack([az_vals, el_vals])

    def to_image(vals):
        img = griddata(points, vals, (AZ_G, EL_G), method="linear")
        if np.any(np.isnan(img)):
            img_nn = griddata(points, vals, (AZ_G, EL_G), method="nearest")
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

    n_bins   = spectra.shape[1]
    integ    = spectra.mean(axis=1)
    n_maps   = min(8, n_bins)
    bin_idxs = np.linspace(0, n_bins - 1, n_maps, dtype=int)

    ncols = 3
    nrows = (1 + n_maps + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(6 * ncols, 4 * nrows))
    axes = list(axes.flat)

    image_map(axes[0], integ, "Integrated (all bins)", "dBFS (mean)")
    for i, bidx in enumerate(bin_idxs):
        image_map(axes[i + 1], spectra[:, bidx],
                  f"{freqs_mhz[bidx]:.2f} MHz", "dBFS")
    for ax in axes[1 + n_maps:]:
        ax.set_visible(False)

    n_total    = len(az_vals)
    az_vel     = cfg.get("az_scan_vel",  AZ_SCAN_VEL)
    samp_hz    = cfg.get("sample_hz",    SAMPLE_HZ)
    el_step    = cfg.get("el_step_deg",  EL_STEP_DEG)
    az_res     = az_vel / samp_hz
    fig.suptitle(
        f"Sky fly-scan-H,10.75ghzlo — {os.path.basename(run_dir)}\n"
        f"Center {center_hz/1e6:.4f} MHz   {n_total} samples   "
        f"res: {az_res:.2f}° az × {el_step:.2f}° el",
        fontsize=10)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    print(f"Plot saved → {out_png}  ({n_total} samples, {len(files)} rows)")


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Sky fly-scan: continuous-motion SDR survey")
    parser.add_argument("--port",       default=SERIAL_PORT)
    parser.add_argument("--az-start",   type=float, default=AZ_START,    metavar="DEG")
    parser.add_argument("--az-stop",    type=float, default=AZ_STOP,     metavar="DEG")
    parser.add_argument("--el-start",   type=float, default=EL_START,    metavar="DEG")
    parser.add_argument("--el-stop",    type=float, default=EL_STOP,     metavar="DEG")
    parser.add_argument("--el-step",    type=float, default=EL_STEP_DEG, metavar="DEG",
                        help="Elevation step between rows in deg (default %(default)s)")
    parser.add_argument("--az-vel",     type=float, default=AZ_SCAN_VEL, metavar="DEG_S",
                        help="Azimuth scan velocity deg/s (default %(default)s)")
    parser.add_argument("--sample-hz",  type=float, default=SAMPLE_HZ,
                        help="Samples per second during each row (default %(default)s)")
    parser.add_argument("--freq",       type=float, default=CENTER_FREQ / 1e6,
                        metavar="MHZ",
                        help="SDR center frequency MHz (default %(default)s)")
    parser.add_argument("--plot-only",  metavar="DIR",
                        help="Re-plot an existing run directory and exit")
    parser.add_argument("--debug",      action="store_true")
    args = parser.parse_args()

    global DEBUG
    DEBUG = args.debug

    if args.plot_only:
        plot_results(args.plot_only,
                     os.path.join(args.plot_only, "sky_fly_scan.png"))
        return

    center_hz = args.freq * 1e6
    els       = np.arange(args.el_start, args.el_stop + args.el_step * 0.01,
                          args.el_step)
    az_range  = abs(args.az_stop - args.az_start)
    row_s     = az_range / args.az_vel
    samp_row  = int(row_s * args.sample_hz)
    az_res    = args.az_vel / args.sample_hz

    run_dir = f"scan_fly_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    os.makedirs(run_dir, exist_ok=True)

    with open(os.path.join(run_dir, "scan_config.json"), "w") as f:
        json.dump({
            "az_start":       args.az_start,  "az_stop":      args.az_stop,
            "el_start":       args.el_start,  "el_stop":      args.el_stop,
            "el_step_deg":    args.el_step,
            "az_scan_vel":    args.az_vel,
            "sample_hz":      args.sample_hz,
            "center_freq_hz": center_hz,
            "sample_rate_hz": SAMPLE_RATE,
            "fft_size":       FFT_SIZE,
            "averages":       AVERAGES,
            "gain_db":        SOAPY_GAIN,
        }, f, indent=2)

    print(f"Fly-scan  : {len(els)} rows  "
          f"az {args.az_start:.0f}–{args.az_stop:.0f}°  "
          f"el {args.el_start:.0f}–{args.el_stop:.0f}° step {args.el_step:.1f}°")
    print(f"SDR       : {center_hz/1e6:.4f} MHz  {SAMPLE_RATE/1e6:.0f} MS/s  "
          f"{AVERAGES}× avg  ({AVERAGES*FFT_SIZE/SAMPLE_RATE*1000:.2f} ms/spectrum)")
    print(f"Scan      : {args.az_vel} °/s  {args.sample_hz:.0f} Hz  "
          f"→ {az_res:.2f} °/sample az  {args.el_step:.2f} °/row el  "
          f"≈{samp_row} samples/row")
    print(f"Data      : {run_dir}/")

    sdr, rx_stream, spec_stream = open_sdr(center_hz)
    print("SDR ready.")

    rows_done  = 0
    interrupted = False
    try:
        with serial.Serial(args.port, BAUD, timeout=0.5) as ser:
            time.sleep(0.5)
            set_max_velocity(ser, AZ_RETURN_VEL, EL_RETURN_VEL)

            try:
                for row_idx, el in enumerate(els):
                    fname = row_filename(run_dir, el)
                    if os.path.exists(fname):
                        rows_done += 1
                        print(f"[{row_idx+1}/{len(els)}] el={el:.2f}° — skip")
                        continue

                    print(f"[{row_idx+1}/{len(els)}] el={el:.2f}°")
                    direction  = 1 if row_idx % 2 == 0 else -1
                    az_start_r = args.az_start if direction > 0 else args.az_stop
                    az_end_r   = args.az_stop  if direction > 0 else args.az_start

                    az_vals, el_vals, spectra = sweep_row(
                        ser, spec_stream, el,
                        az_start_r, az_end_r, args.az_vel, args.sample_hz)

                    if az_vals:
                        save_row(fname, az_vals, el_vals, spectra)
                        rows_done += 1

            except KeyboardInterrupt:
                interrupted = True
                print(f"\nInterrupted — {rows_done}/{len(els)} rows saved.")

            print("Closing dish ...")
            send_cmd(ser, "c\n")

    finally:
        close_sdr(sdr, rx_stream, spec_stream)

    if rows_done > 0:
        plot_results(run_dir, os.path.join(run_dir, "sky_fly_scan.png"))


if __name__ == "__main__":
    main()
