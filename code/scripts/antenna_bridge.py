#!/usr/bin/env python3

"""
	This file is part of OpenR3flector project.

	Copyright (C) 2026 vantalane <mete@kestech.net>
	SPDX-License-Identifier: GPL-3.0-or-later

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

Antenna Controller Bridge
Translates Hamlib rotctld TCP protocol (used by GPredict / KStars)
into the custom serial format  "a<az>,<el>"  expected by your device.

Usage:
    python antenna_bridge.py --port /dev/ttyUSB0 --tcp-port 4533

GPredict:  Interfaces -> Rotator -> Add  ->  Host: localhost  Port: 4533
KStars:    Set rotator driver to "Custom" pointing at localhost:4533
"""

import socket
import serial
import threading
import argparse
import logging
import time

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
log = logging.getLogger("antenna_bridge")

# ── Shared rotator state ───────────────────────────────────────────────────────

class RotatorState:
    def __init__(self):
        self.az = 0.0
        self.el = 0.0
        self.lock = threading.Lock()

    def set(self, az: float, el: float):
        with self.lock:
            self.az = az
            self.el = el

    def get(self):
        with self.lock:
            return self.az, self.el


# ── Serial writer ──────────────────────────────────────────────────────────────

def open_serial(port: str, baud: int = 9600) -> serial.Serial:
    ser = serial.Serial(port, baud, timeout=1)
    log.info(f"Opened serial port {port} at {baud} baud")
    return ser


def send_to_device(ser: serial.Serial, az: float, el: float):
    """Send  a<az>,<el>\n  to the antenna controller."""
    cmd = f"g{az:.4f},{el:.4f}\n"
    ser.write(cmd.encode())
    log.debug(f"Serial TX: {cmd.strip()}")


def handle_client(conn: socket.socket, addr, state: RotatorState, ser: serial.Serial):
    log.info(f"Client connected: {addr}")
    buf = ""
    try:
        while True:
            data = conn.recv(256)
            if not data:
                break
            buf += data.decode(errors="replace")

            # Process every newline-terminated command in the buffer
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                response = process_command(line, state, ser)
                if response is not None:
                    conn.sendall(response.encode())

    except (ConnectionResetError, BrokenPipeError):
        pass
    finally:
        conn.close()
        log.info(f"Client disconnected: {addr}")


def process_command(line: str, state: RotatorState, ser: serial.Serial) -> str | None:
    log.debug(f"CMD: {line!r}")

    # Normalise extended-format commands  (\set_pos → P, \get_pos → p …)
    if line.startswith("\\") or line.startswith("+"):
        line = line.lstrip("\\+").strip()
        if line.startswith("set_pos"):
            parts = line.split()
            line = "P " + " ".join(parts[1:])
        elif line.startswith("get_pos"):
            line = "p"
        elif line.startswith("stop"):
            line = "S"

    cmd = line.split()
    if not cmd:
        return None

    verb = cmd[0]

    # ── get position ──────────────────────────────────────────────────────────
    if verb == "p":
        az, el = state.get()
        return f"{az:.2f}\n{el:.2f}\n"

    # ── set position ──────────────────────────────────────────────────────────
    elif verb == "P":
        if len(cmd) < 3:
            return "RPRT -1\n"
        try:
            az = float(cmd[1])
            el = float(cmd[2])
        except ValueError:
            return "RPRT -1\n"

        # Clamp to safe ranges
        az = max(0.0, min(360.0, az))
        el = max(0.0, min(90.0, el))

        state.set(az, el)
        send_to_device(ser, az, el)
        log.info(f"Set position → AZ {az:.2f}°  EL {el:.2f}°")
        return "RPRT 0\n"

    # ── stop ──────────────────────────────────────────────────────────────────
    elif verb == "S":
        log.info("Stop command received")
        # Send current position again as a 'hold' – adapt if your device has a stop cmd
        az, el = state.get()
        send_to_device(ser, az, el)
        return "RPRT 0\n"

    # ── quit ──────────────────────────────────────────────────────────────────
    elif verb in ("q", "Q"):
        return None   # caller will close socket

    else:
        log.debug(f"Unknown command: {verb!r}")
        return "RPRT -1\n"


# ── TCP server ─────────────────────────────────────────────────────────────────

def run_server(tcp_host: str, tcp_port: int, state: RotatorState, ser: serial.Serial):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((tcp_host, tcp_port))
    server.listen(5)
    log.info(f"Listening on {tcp_host}:{tcp_port}  (rotctld protocol)")

    try:
        while True:
            conn, addr = server.accept()
            t = threading.Thread(
                target=handle_client,
                args=(conn, addr, state, ser),
                daemon=True
            )
            t.start()
    except KeyboardInterrupt:
        log.info("Shutting down.")
    finally:
        server.close()


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="GPredict/KStars → Serial antenna bridge")
    parser.add_argument("--serial-port", default="/dev/ttyUSB0",
                        help="Serial port of your antenna controller (default: /dev/ttyUSB0)")
    parser.add_argument("--baud",        default=9600, type=int,
                        help="Baud rate (default: 9600)")
    parser.add_argument("--tcp-host",    default="0.0.0.0",
                        help="TCP bind address (default: 0.0.0.0)")
    parser.add_argument("--tcp-port",    default=4533, type=int,
                        help="TCP port to listen on (default: 4533)")
    parser.add_argument("--debug",       action="store_true",
                        help="Enable debug logging")
    args = parser.parse_args()

    if args.debug:
        log.setLevel(logging.DEBUG)

    state = RotatorState()
    ser   = open_serial(args.serial_port, args.baud)

    run_server(args.tcp_host, args.tcp_port, state, ser)


if __name__ == "__main__":
    main()