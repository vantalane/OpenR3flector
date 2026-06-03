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

# simple circular setpoint sender over UART at 20Hz
# usage: python3 circular_uart.py /dev/ttyUSB0 115200 700 150 200
# args: serial_port baud center_pitch center_yaw diameter_counts

import sys
import time
import math
import serial

# ./circular_uart.py /dev/ttyUSB1 9600 750 300 200
if len(sys.argv) != 6:
    print("Usage: circular_uart.py <port> <baud> <center_pitch> <center_yaw> <diameter>")
    sys.exit(1)

port = sys.argv[1]
baud = int(sys.argv[2])
center_pitch = float(sys.argv[3])
center_yaw = float(sys.argv[4])
diameter = float(sys.argv[5])

ser = serial.Serial(port, baud, timeout=1)
freq = 10.0
period = 1.0 / freq
radius = diameter / 2.0
t0 = time.time()

try:
    while True:
        t = time.time() - t0
        theta = 2 * math.pi * (t * 0.05)  # 0.2 rotations/sec -> one rotation every 5s (adjustable)
        p_val = int(round(center_pitch + radius * math.sin(theta)))
        y_val = int(round(center_yaw   + radius * math.cos(theta)))

        ser.write(f"p{p_val}\n".encode())
        # small gap is fine; commands are separate
        ser.write(f"y{y_val}\n".encode())

        # maintain 20Hz loop
        time.sleep(period)
except KeyboardInterrupt:
    pass
finally:
    ser.close()
