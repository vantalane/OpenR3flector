"""
	This file is part of OpenR3flector project.

	Copyright (C) 2026 vantalane <mete@kestech.net>
	SPDX-License-Identifier: GPL-3.0-or-later

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.


antenna_monitor.py

Real-time monitor and controller for the R3 opencontrol antenna driver.
Requires: pyserial, matplotlib
    pip install pyserial matplotlib
"""

import serial
import serial.tools.list_ports
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# ── Config ────────────────────────────────────────────────────────────────────
BAUD_RATE     = 115200
POLL_HZ       = 50          # target poll rate for motion data
PLOT_WINDOW_S = 10          # seconds of history shown in plots
MAX_POINTS    = POLL_HZ * PLOT_WINDOW_S

# ── Commands (must match firmware uart_comms.h) ───────────────────────────────
CMD_HOME         = 'h'
CMD_HOME_UNSAFE  = 'u'
CMD_GOTO         = 'g'
CMD_SYNC         = 'w'
CMD_TRACK        = 't'
CMD_PARK         = 'c'
CMD_UNPARK       = 'o'
CMD_ABORT        = 'q'
CMD_ESTOP        = 'e'
CMD_GET_STATUS   = 's'
CMD_GET_MOTION   = 'm'
CMD_SET_ACC      = 'a'
CMD_SET_VEL      = 'v'
CMD_HW_RESET     = 'r'
CMD_CHIRP        = 'j'
CMD_DEBUG        = 'd'
CMD_LNBSET       = 'p'
CMD_GET_PLAN     = 'l'

DEV_STATES = {
    0: "UNINIT",
    1: "HOMING",
    2: "CLOSED",
    3: "OPENING",
    4: "OPEN",
    5: "CLOSING",
    6: "FAULT",
}

# ── Shared data (written by serial thread, read by GUI/plot) ──────────────────
class DeviceData:
    def __init__(self):
        self.lock = threading.Lock()
        # status fields
        self.device_state      = 0
        self.max_az_acc        = 0.0
        self.max_alt_acc       = 0.0
        self.max_az_vel        = 0.0
        self.max_alt_vel       = 0.0
        self.az_body_offset    = 0.0
        self.alt_body_offset   = 0.0
        self.az_0_tilt         = 0.0
        self.az_90_tilt        = 0.0
        self.calib_bitmap      = 0
        # motion — body frame
        self.az_pos            = 0.0
        self.az_setpoint       = 0.0
        self.alt_pos           = 0.0
        self.alt_setpoint      = 0.0
        # motion — world frame
        self.world_az_pos      = 0.0
        self.world_alt_pos     = 0.0
        self.world_az_setpoint = 0.0
        self.world_alt_setpoint= 0.0
        # motion — dynamics
        self.az_vel            = 0.0
        self.alt_vel           = 0.0
        self.az_accel          = 0.0
        self.alt_accel         = 0.0
        # trajectory planner output (what the motor PID tracks)
        self.plan_az_pos       = 0.0
        self.plan_alt_pos      = 0.0
        self.plan_az_vel       = 0.0
        self.plan_alt_vel      = 0.0
        # time-series deques for plotting
        n = MAX_POINTS
        self.t                = deque(maxlen=n)
        self.d_az_pos         = deque(maxlen=n)
        self.d_alt_pos        = deque(maxlen=n)
        self.d_az_set         = deque(maxlen=n)
        self.d_alt_set        = deque(maxlen=n)
        self.d_world_az_pos   = deque(maxlen=n)
        self.d_world_alt_pos  = deque(maxlen=n)
        self.d_world_az_set   = deque(maxlen=n)
        self.d_world_alt_set  = deque(maxlen=n)
        self.d_az_vel         = deque(maxlen=n)
        self.d_alt_vel        = deque(maxlen=n)
        self.d_az_accel       = deque(maxlen=n)
        self.d_alt_accel      = deque(maxlen=n)
        self.t_plan           = deque(maxlen=n)
        self.d_plan_az_pos    = deque(maxlen=n)
        self.d_plan_alt_pos   = deque(maxlen=n)
        self.d_plan_az_vel    = deque(maxlen=n)
        self.d_plan_alt_vel   = deque(maxlen=n)
        self.t0               = time.time()
        # console log
        self.log_lines        = deque(maxlen=200)
        self.connected        = False


data = DeviceData()
ser       = None              # serial.Serial instance, set on connect
ser_wlock = threading.Lock()  # serialises all writes


def _poll_cmd(s, cmd):
    """Write poll command. Must be called with ser_wlock held."""
    s.write(f"{cmd}\n".encode())


def _read_csv(s, n_fields, max_tries=6):
    """Read lines until one has exactly n_fields comma-separated numeric values.
    Lines with a different field count are logged as [fw] and skipped — this
    prevents a motion/plan response from being mis-parsed as a status response
    (and vice-versa) when two responses arrive back-to-back.
    Returns (line, parts) or ('', []) when no valid line arrived."""
    for _ in range(max_tries):
        raw = s.readline()
        if not raw:       # readline timeout — no more data coming
            break
        line = raw.decode(errors='replace').strip()
        if not line:
            continue
        parts = line.split(',')
        if len(parts) == n_fields:
            try:
                [float(p) for p in parts]
                return line, parts
            except ValueError:
                pass
        with data.lock:
            data.log_lines.append(f"[fw] {line}")
    return '', []


# ── Serial thread ─────────────────────────────────────────────────────────────
def serial_thread():
    """Polls device at POLL_HZ and stuffs results into data."""
    interval = 1.0 / POLL_HZ
    while True:
        s = ser  # snapshot — avoids race if main thread sets ser=None
        if s is None or not s.is_open:
            time.sleep(0.1)
            continue

        try:
            # -- status (config / limits / calib) --
            # firmware: state,az_acc,alt_acc,az_vel,alt_vel,az_off,alt_off,az0_tilt,az90_tilt,calib_bitmap
            with ser_wlock:
                _poll_cmd(s, CMD_GET_STATUS)
            _, parts = _read_csv(s, 10)  # status: exactly 10 fields
            if parts:
                with data.lock:
                    data.device_state    = int(float(parts[0]))
                    data.max_az_acc      = float(parts[1])
                    data.max_alt_acc     = float(parts[2])
                    data.max_az_vel      = float(parts[3])
                    data.max_alt_vel     = float(parts[4])
                    data.az_body_offset  = float(parts[5])
                    data.alt_body_offset = float(parts[6])
                    data.az_0_tilt       = float(parts[7])
                    data.az_90_tilt      = float(parts[8])
                    data.calib_bitmap    = int(float(parts[9]))

            # -- motion (fast data) --
            # firmware: body_az,body_alt,body_az_set,body_alt_set,
            #           world_az,world_alt,world_az_set,world_alt_set,
            #           az_vel,alt_vel,az_accel,alt_accel
            with ser_wlock:
                _poll_cmd(s, CMD_GET_MOTION)
            _, parts = _read_csv(s, 12)  # motion: exactly 12 fields
            if parts:
                now = time.time() - data.t0
                with data.lock:
                    data.az_pos             = float(parts[0])
                    data.alt_pos            = float(parts[1])
                    data.az_setpoint        = float(parts[2])
                    data.alt_setpoint       = float(parts[3])
                    data.world_az_pos       = float(parts[4])
                    data.world_alt_pos      = float(parts[5])
                    data.world_az_setpoint  = float(parts[6])
                    data.world_alt_setpoint = float(parts[7])
                    data.az_vel             = float(parts[8])
                    data.alt_vel            = float(parts[9])
                    data.az_accel           = float(parts[10])
                    data.alt_accel          = float(parts[11])
                    data.t.append(now)
                    data.d_az_pos.append(data.az_pos)
                    data.d_alt_pos.append(data.alt_pos)
                    data.d_az_set.append(data.az_setpoint)
                    data.d_alt_set.append(data.alt_setpoint)
                    data.d_world_az_pos.append(data.world_az_pos)
                    data.d_world_alt_pos.append(data.world_alt_pos)
                    data.d_world_az_set.append(data.world_az_setpoint)
                    data.d_world_alt_set.append(data.world_alt_setpoint)
                    data.d_az_vel.append(data.az_vel)
                    data.d_alt_vel.append(data.alt_vel)
                    data.d_az_accel.append(data.az_accel)
                    data.d_alt_accel.append(data.alt_accel)

            # -- trajectory planner (profiled pos/vel the motor PID tracks) --
            # firmware: prof_alt_pos, prof_az_pos, prof_alt_vel, prof_az_vel  (deg / deg/s)
            with ser_wlock:
                _poll_cmd(s, CMD_GET_PLAN)
            _, parts = _read_csv(s, 4)   # plan: exactly 4 fields
            if parts:
                now_plan = time.time() - data.t0
                with data.lock:
                    data.plan_alt_pos = float(parts[0])
                    data.plan_az_pos  = float(parts[1])
                    data.plan_alt_vel = float(parts[2])
                    data.plan_az_vel  = float(parts[3])
                    data.t_plan.append(now_plan)
                    data.d_plan_az_pos.append(data.plan_az_pos)
                    data.d_plan_alt_pos.append(data.plan_alt_pos)
                    data.d_plan_az_vel.append(data.plan_az_vel)
                    data.d_plan_alt_vel.append(data.plan_alt_vel)

        except Exception as e:
            with data.lock:
                data.log_lines.append(f"[err] serial: {e}")

        time.sleep(interval)


def send_cmd(cmd_str):
    """Thread-safe command send + log."""
    s = ser
    if s is None or not s.is_open:
        with data.lock:
            data.log_lines.append("[err] not connected")
        return
    try:
        with ser_wlock:
            s.write(cmd_str.encode())
        with data.lock:
            data.log_lines.append(f"[tx] {cmd_str.strip()}")
    except Exception as e:
        with data.lock:
            data.log_lines.append(f"[err] send: {e}")


# ── GUI ───────────────────────────────────────────────────────────────────────
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("R3 opencontrol monitor")
        self.resizable(True, True)
        self._build_ui()
        self._limits_synced = False
        self._start_plots()
        self._tick()   # periodic GUI refresh

    # ── layout ───────────────────────────────────────────────────────────────
    def _build_ui(self):
        # top bar: port selector + connect
        top = ttk.Frame(self, padding=6)
        top.pack(fill='x')

        ttk.Label(top, text="Port:").pack(side='left')
        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(top, textvariable=self.port_var, width=18)
        self.port_cb['values'] = self._list_ports()
        if self.port_cb['values']:
            self.port_var.set(self.port_cb['values'][0])
        self.port_cb.pack(side='left', padx=4)

        ttk.Button(top, text="Refresh ports",
                   command=self._refresh_ports).pack(side='left', padx=2)
        self.connect_btn = ttk.Button(top, text="Connect",
                                      command=self._toggle_connect)
        self.connect_btn.pack(side='left', padx=6)

        self.state_lbl = ttk.Label(top, text="● disconnected",
                                   foreground='gray')
        self.state_lbl.pack(side='left', padx=8)

        # main area: left=controls, right=live values + log
        main = ttk.Frame(self)
        main.pack(fill='both', expand=True, padx=6, pady=4)

        left  = ttk.Frame(main)
        left.pack(side='left', fill='y', padx=(0,8))
        right = ttk.Frame(main)
        right.pack(side='left', fill='both', expand=True)

        self._build_controls(left)
        self._build_readout(right)

    def _build_controls(self, parent):
        # ── GOTO ─────────────────────────────────────────────────────────────
        goto_frame = ttk.LabelFrame(parent, text="GOTO", padding=8)
        goto_frame.pack(fill='x', pady=4)

        ttk.Label(goto_frame, text="Azimuth (°)").grid(row=0, column=0, sticky='w')
        self.az_var = tk.StringVar(value="0.0")
        ttk.Entry(goto_frame, textvariable=self.az_var, width=10).grid(
            row=0, column=1, padx=4, pady=2)

        ttk.Label(goto_frame, text="Elevation (°)").grid(row=1, column=0, sticky='w')
        self.el_var = tk.StringVar(value="0.0")
        ttk.Entry(goto_frame, textvariable=self.el_var, width=10).grid(
            row=1, column=1, padx=4, pady=2)

        ttk.Button(goto_frame, text="Send GOTO",
                   command=self._cmd_goto).grid(row=2, column=0,
                   columnspan=2, pady=4, sticky='ew')

        # ── motion limits ────────────────────────────────────────────────────
        lim_frame = ttk.LabelFrame(parent, text="Motion limits", padding=8)
        lim_frame.pack(fill='x', pady=4)

        ttk.Label(lim_frame, text="Max Az vel (°/s)").grid(row=0, column=0, sticky='w')
        self.max_az_vel_var = tk.StringVar(value="20.0")
        ttk.Entry(lim_frame, textvariable=self.max_az_vel_var,
                  width=8).grid(row=0, column=1, padx=4, pady=2)

        ttk.Label(lim_frame, text="Max Alt vel (°/s)").grid(row=1, column=0, sticky='w')
        self.max_alt_vel_var = tk.StringVar(value="20.0")
        ttk.Entry(lim_frame, textvariable=self.max_alt_vel_var,
                  width=8).grid(row=1, column=1, padx=4, pady=2)

        ttk.Label(lim_frame, text="Max Az acc (°/s²)").grid(row=2, column=0, sticky='w')
        self.max_az_acc_var = tk.StringVar(value="12.0")
        ttk.Entry(lim_frame, textvariable=self.max_az_acc_var,
                  width=8).grid(row=2, column=1, padx=4, pady=2)

        ttk.Label(lim_frame, text="Max Alt acc (°/s²)").grid(row=3, column=0, sticky='w')
        self.max_alt_acc_var = tk.StringVar(value="12.0")
        ttk.Entry(lim_frame, textvariable=self.max_alt_acc_var,
                  width=8).grid(row=3, column=1, padx=4, pady=2)

        ttk.Button(lim_frame, text="Apply velocity",
                   command=self._cmd_set_vel).grid(row=4, column=0,
                   columnspan=2, pady=2, sticky='ew')
        ttk.Button(lim_frame, text="Apply accel",
                   command=self._cmd_set_acc).grid(row=5, column=0,
                   columnspan=2, pady=2, sticky='ew')

        # ── SYNC ─────────────────────────────────────────────────────────────
        sync_frame = ttk.LabelFrame(parent, text="SYNC", padding=8)
        sync_frame.pack(fill='x', pady=4)

        ttk.Label(sync_frame, text="Sub-cmd (0=reset,1-3=pt)").grid(
            row=0, column=0, sticky='w')
        self.sync_sub_var = tk.StringVar(value="1")
        ttk.Entry(sync_frame, textvariable=self.sync_sub_var, width=4).grid(
            row=0, column=1, padx=4, pady=2)

        ttk.Label(sync_frame, text="World Az (°)").grid(row=1, column=0, sticky='w')
        self.sync_az_var = tk.StringVar(value="0.0")
        ttk.Entry(sync_frame, textvariable=self.sync_az_var, width=10).grid(
            row=1, column=1, padx=4, pady=2)

        ttk.Label(sync_frame, text="World Alt (°)").grid(row=2, column=0, sticky='w')
        self.sync_alt_var = tk.StringVar(value="0.0")
        ttk.Entry(sync_frame, textvariable=self.sync_alt_var, width=10).grid(
            row=2, column=1, padx=4, pady=2)

        ttk.Button(sync_frame, text="Send SYNC",
                   command=self._cmd_sync).grid(row=3, column=0,
                   columnspan=2, pady=4, sticky='ew')

        # ── TRACK ────────────────────────────────────────────────────────────
        track_frame = ttk.LabelFrame(parent, text="TRACK", padding=8)
        track_frame.pack(fill='x', pady=4)

        ttk.Label(track_frame, text="Start Az (°)").grid(row=0, column=0, sticky='w')
        self.track_az_var = tk.StringVar(value="0.0")
        ttk.Entry(track_frame, textvariable=self.track_az_var, width=10).grid(
            row=0, column=1, padx=4, pady=2)

        ttk.Label(track_frame, text="Start Alt (°)").grid(row=1, column=0, sticky='w')
        self.track_alt_var = tk.StringVar(value="0.0")
        ttk.Entry(track_frame, textvariable=self.track_alt_var, width=10).grid(
            row=1, column=1, padx=4, pady=2)

        ttk.Label(track_frame, text="Az rate (°/s)").grid(row=2, column=0, sticky='w')
        self.track_az_rate_var = tk.StringVar(value="0.0")
        ttk.Entry(track_frame, textvariable=self.track_az_rate_var, width=10).grid(
            row=2, column=1, padx=4, pady=2)

        ttk.Label(track_frame, text="Alt rate (°/s)").grid(row=3, column=0, sticky='w')
        self.track_alt_rate_var = tk.StringVar(value="0.0")
        ttk.Entry(track_frame, textvariable=self.track_alt_rate_var, width=10).grid(
            row=3, column=1, padx=4, pady=2)

        ttk.Button(track_frame, text="Send TRACK",
                   command=self._cmd_track).grid(row=4, column=0,
                   columnspan=2, pady=4, sticky='ew')

        # ── LNB control ──────────────────────────────────────────────────────
        lnb_frame = ttk.LabelFrame(parent, text="LNB control", padding=8)
        lnb_frame.pack(fill='x', pady=4)

        ttk.Label(lnb_frame, text="Polarization").grid(row=0, column=0, sticky='w')
        self.lnb_pol_var = tk.StringVar(value="Off")
        lnb_pol_cb = ttk.Combobox(lnb_frame, textvariable=self.lnb_pol_var,
                                   values=["Off", "Vertical", "Horizontal"],
                                   width=10, state='readonly')
        lnb_pol_cb.grid(row=0, column=1, padx=4, pady=2)

        ttk.Label(lnb_frame, text="LO band").grid(row=1, column=0, sticky='w')
        self.lnb_band_var = tk.StringVar(value="Low (10.7-11.7 GHz)")
        lnb_band_cb = ttk.Combobox(lnb_frame, textvariable=self.lnb_band_var,
                                    values=["Low (10.7-11.7 GHz)", "High (11.7-12.75 GHz)"],
                                    width=18, state='readonly')
        lnb_band_cb.grid(row=1, column=1, padx=4, pady=2)

        ttk.Button(lnb_frame, text="Apply LNB",
                   command=self._cmd_lnb).grid(row=2, column=0,
                   columnspan=2, pady=4, sticky='ew')

        # ── device actions ───────────────────────────────────────────────────
        act_frame = ttk.LabelFrame(parent, text="Device actions", padding=8)
        act_frame.pack(fill='x', pady=4)

        btn_defs = [
            ("Home",         self._cmd_home),
            ("Home (unsafe)",self._cmd_home_unsafe),
            ("Open",         self._cmd_open),
            ("Close/Park",   self._cmd_close),
            ("Abort",        self._cmd_abort),
            ("Debug info",   self._cmd_debug),
            ("HW Reset",     self._cmd_hw_reset),
        ]
        for i, (label, cmd) in enumerate(btn_defs):
            btn = ttk.Button(act_frame, text=label, command=cmd)
            btn.grid(row=i, column=0, sticky='ew', pady=2)

        estop_btn = tk.Button(act_frame, text="E-STOP",
                              command=self._cmd_estop,
                              bg='red', fg='white',
                              font=('TkDefaultFont', 10, 'bold'),
                              relief='raised', pady=4)
        estop_btn.grid(row=len(btn_defs), column=0, sticky='ew', pady=6)

    def _build_readout(self, parent):
        # live value labels
        val_frame = ttk.LabelFrame(parent, text="Live values", padding=8)
        val_frame.pack(fill='x', pady=4)

        labels = [
            ("Device state",        'lbl_state'),
            ("Body Az pos (°)",     'lbl_az_pos'),
            ("Body Az set (°)",     'lbl_az_set'),
            ("Body Alt pos (°)",    'lbl_alt_pos'),
            ("Body Alt set (°)",    'lbl_alt_set'),
            ("World Az pos (°)",    'lbl_world_az_pos'),
            ("World Az set (°)",    'lbl_world_az_set'),
            ("World Alt pos (°)",   'lbl_world_alt_pos'),
            ("World Alt set (°)",   'lbl_world_alt_set'),
            ("Az vel (°/s)",        'lbl_az_vel'),
            ("Alt vel (°/s)",       'lbl_alt_vel'),
            ("Az accel (°/s²)",     'lbl_az_accel'),
            ("Alt accel (°/s²)",    'lbl_alt_accel'),
            ("Max Az vel",          'lbl_mxazv'),
            ("Max Alt vel",         'lbl_mxaltv'),
            ("Max Az acc",          'lbl_mxaza'),
            ("Max Alt acc",         'lbl_mxalta'),
            ("Az body offset",      'lbl_az_off'),
            ("Alt body offset",     'lbl_alt_off'),
            ("Az tilt @0°",         'lbl_az0_tilt'),
            ("Az tilt @90°",        'lbl_az90_tilt'),
            ("Calib bitmap",        'lbl_calib_bm'),
        ]

        cols = 2
        for i, (text, attr) in enumerate(labels):
            r, c = divmod(i, cols)
            ttk.Label(val_frame, text=text+":").grid(
                row=r, column=c*2, sticky='w', padx=(0,4), pady=1)
            lbl = ttk.Label(val_frame, text="—", width=12, anchor='e')
            lbl.grid(row=r, column=c*2+1, sticky='e', pady=1)
            setattr(self, attr, lbl)

        # console log
        log_frame = ttk.LabelFrame(parent, text="Console", padding=4)
        log_frame.pack(fill='both', expand=True, pady=4)

        self.log_text = tk.Text(log_frame, height=10, font=('Courier', 9),
                                state='disabled', wrap='word')
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text['yscrollcommand'] = scroll.set
        self.log_text.pack(side='left', fill='both', expand=True)
        scroll.pack(side='right', fill='y')
        self._log_seen = 0

    # ── plots ─────────────────────────────────────────────────────────────────
    def _start_plots(self):
        self.fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
        self.fig.suptitle("R3 opencontrol — live telemetry", fontsize=11)
        self.ax_pos, self.ax_vel, self.ax_acc = axes

        self.ax_pos.set_ylabel("Position (°)")
        self.ax_vel.set_ylabel("Velocity (°/s)")
        self.ax_acc.set_ylabel("Accel (°/s²)")
        self.ax_acc.set_xlabel("Time (s)")

        for ax in axes:
            ax.grid(True, alpha=0.3)

        # position subplot — solid=body frame, dashed=world frame
        self.line_az_pos,       = self.ax_pos.plot([], [], label='Body Az',       color='#378ADD')
        self.line_alt_pos,      = self.ax_pos.plot([], [], label='Body Alt',      color='#1D9E75')
        self.line_az_set,       = self.ax_pos.plot([], [], '--', label='Body Az set',  color='#378ADD', alpha=0.5)
        self.line_alt_set,      = self.ax_pos.plot([], [], '--', label='Body Alt set', color='#1D9E75', alpha=0.5)
        self.line_world_az_pos, = self.ax_pos.plot([], [], label='World Az',  color='#E06C3A')
        self.line_world_alt_pos,= self.ax_pos.plot([], [], label='World Alt', color='#C040B0')
        self.line_world_az_set, = self.ax_pos.plot([], [], '--', label='World Az set',  color='#E06C3A', alpha=0.5)
        self.line_world_alt_set,= self.ax_pos.plot([], [], '--', label='World Alt set', color='#C040B0', alpha=0.5)
        # dotted = trajectory planner profiled position (what the motor PID tracks)
        self.line_plan_az_pos,  = self.ax_pos.plot([], [], ':', lw=2, label='Plan Az',  color='#378ADD')
        self.line_plan_alt_pos, = self.ax_pos.plot([], [], ':', lw=2, label='Plan Alt', color='#1D9E75')
        self.ax_pos.legend(fontsize=8, loc='upper left')

        # velocity subplot — solid=measured, dotted=planner
        self.line_az_vel,       = self.ax_vel.plot([], [], label='Az vel',  color='#378ADD')
        self.line_alt_vel,      = self.ax_vel.plot([], [], label='Alt vel', color='#1D9E75')
        self.line_plan_az_vel,  = self.ax_vel.plot([], [], ':', lw=2, label='Plan Az vel',  color='#378ADD')
        self.line_plan_alt_vel, = self.ax_vel.plot([], [], ':', lw=2, label='Plan Alt vel', color='#1D9E75')
        self.ax_vel.legend(fontsize=8, loc='upper left')

        # acceleration subplot
        self.line_az_acc,  = self.ax_acc.plot([], [], label='Az accel',  color='#378ADD')
        self.line_alt_acc, = self.ax_acc.plot([], [], label='Alt accel', color='#1D9E75')
        self.ax_acc.legend(fontsize=8, loc='upper left')

        self.fig.tight_layout()
        self.anim = animation.FuncAnimation(
            self.fig, self._update_plots, interval=50, blit=True, cache_frame_data=False)
        plt.show(block=False)

    def _update_plots(self, frame):
        with data.lock:
            t          = list(data.t)
            azp        = list(data.d_az_pos)
            altp       = list(data.d_alt_pos)
            azs        = list(data.d_az_set)
            alts       = list(data.d_alt_set)
            wazp       = list(data.d_world_az_pos)
            waltp      = list(data.d_world_alt_pos)
            wazs       = list(data.d_world_az_set)
            walts      = list(data.d_world_alt_set)
            azv        = list(data.d_az_vel)
            altv       = list(data.d_alt_vel)
            aza        = list(data.d_az_accel)
            alta       = list(data.d_alt_accel)
            tp         = list(data.t_plan)
            plazp      = list(data.d_plan_az_pos)
            paltp      = list(data.d_plan_alt_pos)
            plazv      = list(data.d_plan_az_vel)
            paltv      = list(data.d_plan_alt_vel)

        all_lines = (self.line_az_pos, self.line_alt_pos,
                     self.line_az_set, self.line_alt_set,
                     self.line_world_az_pos, self.line_world_alt_pos,
                     self.line_world_az_set, self.line_world_alt_set,
                     self.line_az_vel, self.line_alt_vel,
                     self.line_plan_az_pos, self.line_plan_alt_pos,
                     self.line_plan_az_vel, self.line_plan_alt_vel,
                     self.line_az_acc, self.line_alt_acc)

        if not t:
            return all_lines

        # sliding x window — anchor to the latest of motion or plan timestamps
        t_end   = max(t[-1], tp[-1] if tp else t[-1])
        t_start = max(min(t[0], tp[0] if tp else t[0]), t_end - PLOT_WINDOW_S)

        for ax in (self.ax_pos, self.ax_vel, self.ax_acc):
            ax.set_xlim(t_start, t_end + 0.5)

        self.line_az_pos.set_data(t, azp)
        self.line_alt_pos.set_data(t, altp)
        self.line_az_set.set_data(t, azs)
        self.line_alt_set.set_data(t, alts)
        self.line_world_az_pos.set_data(t, wazp)
        self.line_world_alt_pos.set_data(t, waltp)
        self.line_world_az_set.set_data(t, wazs)
        self.line_world_alt_set.set_data(t, walts)
        self.line_az_vel.set_data(t, azv)
        self.line_alt_vel.set_data(t, altv)
        self.line_az_acc.set_data(t, aza)
        self.line_alt_acc.set_data(t, alta)
        self.line_plan_az_pos.set_data(tp, plazp)
        self.line_plan_alt_pos.set_data(tp, paltp)
        self.line_plan_az_vel.set_data(tp, plazv)
        self.line_plan_alt_vel.set_data(tp, paltv)

        for ax in (self.ax_pos, self.ax_vel, self.ax_acc):
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        return all_lines

    # ── periodic GUI refresh ──────────────────────────────────────────────────
    def _tick(self):
        with data.lock:
            ds           = data.device_state
            az_pos       = data.az_pos
            az_set       = data.az_setpoint
            alt_pos      = data.alt_pos
            alt_set      = data.alt_setpoint
            w_az_pos     = data.world_az_pos
            w_az_set     = data.world_az_setpoint
            w_alt_pos    = data.world_alt_pos
            w_alt_set    = data.world_alt_setpoint
            az_vel       = data.az_vel
            alt_vel      = data.alt_vel
            az_accel     = data.az_accel
            alt_accel    = data.alt_accel
            mx_az_v      = data.max_az_vel
            mx_alt_v     = data.max_alt_vel
            mx_az_a      = data.max_az_acc
            mx_alt_a     = data.max_alt_acc
            az_off       = data.az_body_offset
            alt_off      = data.alt_body_offset
            az0_tilt     = data.az_0_tilt
            az90_tilt    = data.az_90_tilt
            calib_bm     = data.calib_bitmap
            log_lines    = list(data.log_lines)

        state_str = DEV_STATES.get(ds, f"?({ds})")
        fault      = ds == 6

        self.lbl_state.config(
            text=state_str,
            foreground='red' if fault else 'green' if ds == 4 else 'orange')

        self.lbl_az_pos.config(text=f"{az_pos:.2f}°")
        self.lbl_az_set.config(text=f"{az_set:.2f}°")
        self.lbl_alt_pos.config(text=f"{alt_pos:.2f}°")
        self.lbl_alt_set.config(text=f"{alt_set:.2f}°")
        self.lbl_world_az_pos.config(text=f"{w_az_pos:.2f}°")
        self.lbl_world_az_set.config(text=f"{w_az_set:.2f}°")
        self.lbl_world_alt_pos.config(text=f"{w_alt_pos:.2f}°")
        self.lbl_world_alt_set.config(text=f"{w_alt_set:.2f}°")
        self.lbl_az_vel.config(text=f"{az_vel:.3f}")
        self.lbl_alt_vel.config(text=f"{alt_vel:.3f}")
        self.lbl_az_accel.config(text=f"{az_accel:.3f}")
        self.lbl_alt_accel.config(text=f"{alt_accel:.3f}")
        self.lbl_mxazv.config(text=f"{mx_az_v:.2f}")
        self.lbl_mxaltv.config(text=f"{mx_alt_v:.2f}")
        self.lbl_mxaza.config(text=f"{mx_az_a:.2f}")
        self.lbl_mxalta.config(text=f"{mx_alt_a:.2f}")
        self.lbl_az_off.config(text=f"{az_off:.2f}°")
        self.lbl_alt_off.config(text=f"{alt_off:.2f}°")
        self.lbl_az0_tilt.config(text=f"{az0_tilt:.2f}°")
        self.lbl_az90_tilt.config(text=f"{az90_tilt:.2f}°")
        self.lbl_calib_bm.config(text=f"0b{calib_bm:03b}")

        # sync limit fields from device on first non-zero read
        if not self._limits_synced and mx_az_v > 0:
            self._limits_synced = True
            self.max_az_vel_var.set(f"{mx_az_v:.2f}")
            self.max_alt_vel_var.set(f"{mx_alt_v:.2f}")
            self.max_az_acc_var.set(f"{mx_az_a:.2f}")
            self.max_alt_acc_var.set(f"{mx_alt_a:.2f}")

        # update log
        new_lines = log_lines[self._log_seen:]
        if new_lines:
            self._log_seen = len(log_lines)
            self.log_text.config(state='normal')
            for line in new_lines:
                self.log_text.insert('end', line + '\n')
            self.log_text.see('end')
            self.log_text.config(state='disabled')

        self.after(50, self._tick)   # 20 Hz GUI refresh

    # ── connection ────────────────────────────────────────────────────────────
    def _list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def _refresh_ports(self):
        ports = self._list_ports()
        self.port_cb['values'] = ports
        if ports:
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        global ser
        if ser and ser.is_open:
            ser.close()
            ser = None
            self._limits_synced = True
            self.connect_btn.config(text="Connect")
            self.state_lbl.config(text="● disconnected", foreground='gray')
            with data.lock:
                data.log_lines.append("[info] disconnected")
        else:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Error", "No port selected")
                return
            try:
                ser = serial.Serial(port, BAUD_RATE, timeout=0.05)
                ser.reset_input_buffer()  # clear firmware startup messages
                self.connect_btn.config(text="Disconnect")
                self.state_lbl.config(text="● connected", foreground='green')
                with data.lock:
                    data.log_lines.append(f"[info] connected to {port}")
                # start serial thread once
                if not hasattr(self, '_serial_thread_started'):
                    t = threading.Thread(target=serial_thread, daemon=True)
                    t.start()
                    self._serial_thread_started = True
            except Exception as e:
                messagebox.showerror("Connection failed", str(e))

    # ── command helpers ───────────────────────────────────────────────────────
    def _cmd_goto(self):
        try:
            az = float(self.az_var.get())
            el = float(self.el_var.get())
        except ValueError:
            messagebox.showerror("Error", "Az/El must be numbers")
            return
        send_cmd(f"{CMD_GOTO}{az:.4f},{el:.4f}\n")

    def _cmd_set_vel(self):
        try:
            az_v  = float(self.max_az_vel_var.get())
            alt_v = float(self.max_alt_vel_var.get())
        except ValueError:
            messagebox.showerror("Error", "Velocity values must be numbers")
            return
        send_cmd(f"{CMD_SET_VEL}{az_v:.4f},{alt_v:.4f}\n")

    def _cmd_set_acc(self):
        try:
            az_a  = float(self.max_az_acc_var.get())
            alt_a = float(self.max_alt_acc_var.get())
        except ValueError:
            messagebox.showerror("Error", "Accel values must be numbers")
            return
        send_cmd(f"{CMD_SET_ACC}{az_a:.4f},{alt_a:.4f}\n")

    def _cmd_sync(self):
        try:
            sub = int(self.sync_sub_var.get())
            az  = float(self.sync_az_var.get())
            alt = float(self.sync_alt_var.get())
        except ValueError:
            messagebox.showerror("Error", "SYNC fields must be numbers")
            return
        send_cmd(f"{CMD_SYNC}{sub},{az:.4f},{alt:.4f}\n")

    def _cmd_track(self):
        try:
            az      = float(self.track_az_var.get())
            alt     = float(self.track_alt_var.get())
            az_rate = float(self.track_az_rate_var.get())
            alt_rate= float(self.track_alt_rate_var.get())
        except ValueError:
            messagebox.showerror("Error", "TRACK fields must be numbers")
            return
        send_cmd(f"{CMD_TRACK}{az:.4f},{alt:.4f},{az_rate:.4f},{alt_rate:.4f}\n")

    def _cmd_lnb(self):
        pol_map  = {"Off": 0, "Vertical": 1, "Horizontal": 2}
        band_map = {"Low (10.7-11.7 GHz)": 0, "High (11.7-12.75 GHz)": 1}
        pol  = pol_map.get(self.lnb_pol_var.get(), 0)
        band = band_map.get(self.lnb_band_var.get(), 0)
        send_cmd(f"{CMD_LNBSET}{pol},{band}\n")

    def _cmd_debug(self):
        send_cmd(f"{CMD_DEBUG}\n")

    def _cmd_home(self):
        send_cmd(f"{CMD_HOME}\n")

    def _cmd_home_unsafe(self):
        if messagebox.askyesno("Confirm", "Run unsafe home sequence?"):
            send_cmd(f"{CMD_HOME_UNSAFE}\n")

    def _cmd_open(self):
        send_cmd(f"{CMD_UNPARK}\n")

    def _cmd_close(self):
        send_cmd(f"{CMD_PARK}\n")

    def _cmd_abort(self):
        send_cmd(f"{CMD_ABORT}\n")

    def _cmd_estop(self):
        send_cmd(f"{CMD_ESTOP}\n")

    def _cmd_hw_reset(self):
        if messagebox.askyesno("Confirm", "Hardware reset the device?"):
            send_cmd(f"{CMD_HW_RESET}\n")


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = App()
    app.mainloop()
