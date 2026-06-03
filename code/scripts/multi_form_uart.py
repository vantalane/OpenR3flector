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
Configurable smooth circular / figure-8 setpoint sender over UART.

Edit the VARIABLES section below to configure behavior, then run the script.
"""

import time
import math
import signal
import sys
from collections import deque

try:
    import serial
except Exception as e:
    print("pyserial is required: pip install pyserial")
    raise

# -----------------------
# VARIABLES (edit these)
# -----------------------
PORT = "/dev/ttyUSB1"
BAUD = 9600

# Motion pattern: "circle" or "figure8"
PATTERN = "circle"

# Center setpoints (units = encoder counts or robot units)
CENTER_PITCH = 700.0
CENTER_YAW = 150.0

# Diameter (same units as centers)
DIAMETER = 200.0

# Rotations per second (positive -> CCW). For figure-8 this controls base frequency.
ROTATIONS_PER_SEC = 0.20  # 0.20 -> one rotation in 5s for circle

# Output (serial) rate in Hz
OUTPUT_HZ = 10.0

# Smoothness: apply an exponential low-pass filter to sent values.
# 0.0 = no filtering (send raw setpoints). Higher -> smoother but more lag.
SMOOTHING_ALPHA = 0.0

# Optional amplitude tapering to avoid sudden start (seconds)
RAMP_UP_SECONDS = 1.0

# Optional integer clipping bounds (None = no clip)
MIN_PITCH = None
MAX_PITCH = None
MIN_YAW = None
MAX_YAW = None

# Send format (keeps original device protocol)
SEND_PITCH_FMT = "p{:+d}\n"   # use integer with sign if helpful
SEND_YAW_FMT   = "y{:+d}\n"
# -----------------------
# End VARIABLES
# -----------------------

period = 1.0 / float(OUTPUT_HZ)
radius = DIAMETER / 2.0
omega = 2.0 * math.pi * float(ROTATIONS_PER_SEC)  # radians/sec
t0 = time.time()
running = True

def clipped_int(x, mn, mx):
    if mn is not None and x < mn:
        x = mn
    if mx is not None and x > mx:
        x = mx
    return int(round(x))

# signal handler to stop cleanly
def _handle_sigint(signum, frame):
    global running
    running = False

signal.signal(signal.SIGINT, _handle_sigint)
signal.signal(signal.SIGTERM, _handle_sigint)

# open serial
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except Exception as e:
    print(f"Failed to open serial port {PORT} at {BAUD}: {e}")
    sys.exit(1)

# smoothing state
last_p = CENTER_PITCH
last_y = CENTER_YAW

# optional ramp helper
def ramp_scale(elapsed, ramp_seconds):
    if ramp_seconds <= 0.0:
        return 1.0
    return min(1.0, max(0.0, elapsed / ramp_seconds))

try:
    while running:
        now = time.time()
        t = now - t0
        # scaled time for ramping
        ramp = ramp_scale(t, RAMP_UP_SECONDS)

        # base angle
        theta = omega * t

        if PATTERN.lower() == "circle":
            # circle: x = r * cos(theta), y = r * sin(theta)
            dx = radius * math.cos(theta)
            dy = radius * math.sin(theta)
        else:
            # figure-8: lemniscate-like / Lissajous. This variant gives one loop left, one loop right per cycle.
            # Use x = r * sin(theta), y = r * sin(2*theta)/2 to keep amplitude comparable.
            dx = radius * math.sin(theta)
            dy = (radius * math.sin(2.0 * theta)) * 0.5

        # apply ramping (start from center and grow amplitude smoothly)
        dx *= ramp
        dy *= ramp

        p_raw = CENTER_PITCH + dy   # map dy -> pitch (keeps mapping similar to original code)
        y_raw = CENTER_YAW   + dx   # map dx -> yaw

        # smoothing (exponential low-pass)
        alpha = SMOOTHING_ALPHA
        p_smooth = (alpha * p_raw) + ((1.0 - alpha) * last_p)
        y_smooth = (alpha * y_raw) + ((1.0 - alpha) * last_y)
        last_p, last_y = p_smooth, y_smooth

        # clipping & integer conversion
        p_val = clipped_int(p_smooth, MIN_PITCH, MAX_PITCH)
        y_val = clipped_int(y_smooth, MIN_YAW, MAX_YAW)

        # write to serial (two separate commands as original)
        try:
            ser.write(SEND_PITCH_FMT.format(p_val).encode())
            ser.write(SEND_YAW_FMT.format(y_val).encode())
        except Exception:
            # swallow write errors but continue trying until stopped
            pass

        # maintain loop rate precisely
        next_time = now + period
        sleep_time = next_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            # if behind, yield briefly to avoid tight spin
            time.sleep(0.001)

except Exception:
    # allow traceback for unexpected errors
    raise
finally:
    try:
        ser.close()
    except Exception:
        pass
