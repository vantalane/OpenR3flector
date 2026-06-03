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
INDI Driver – Antenna Rotator as Full EKOS Mount
================================================
Designed to be launched by indiserver from KStars/EKOS.
Now properly declares itself as a TELESCOPE_INTERFACE (0x0001)
with all standard mount properties so it appears under Mounts
and supports slewing/tracking celestial objects.

Hardware command: "a<az.2f>,<el.2f>"
"""

import sys
import serial
import threading
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone

# ── Configuration ─────────────────────────────────────────────────────
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 9600
DEVICE_NAME = "Antenna Rotator"

# ── Global state ──────────────────────────────────────────────────────
state = {
    "az": 0.0,
    "el": 0.0,
    "ra": 0.0,
    "dec": 0.0,
    "port": DEFAULT_PORT,
    "baud": DEFAULT_BAUD,
    "connected": False,
    "tracking": False,
    "coord_mode": "SLEW",
    "ser": None,
}
lock = threading.Lock()

# ── Helpers ───────────────────────────────────────────────────────────
def ts():
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S")

def send(xml: str):
    sys.stdout.write(xml + "\n")
    sys.stdout.flush()

def log(msg: str):
    send(f'<message device="{DEVICE_NAME}" timestamp="{ts()}" message="{msg}"/>')

# ── Property Definitions (critical for mount recognition) ─────────────
def define_all_properties():
    d = DEVICE_NAME

    # Connection
    send(f"""<defSwitchVector device="{d}" name="CONNECTION" label="Connection" group="Main Control"
    state="Idle" perm="rw" rule="OneOfMany" timestamp="{ts()}">
      <defSwitch name="CONNECT" label="Connect">Off</defSwitch>
      <defSwitch name="DISCONNECT" label="Disconnect">On</defSwitch>
    </defSwitchVector>""")

    # Options
    send(f"""<defTextVector device="{d}" name="DEVICE_PORT" label="Serial Port" group="Options"
    state="Idle" perm="rw" timestamp="{ts()}">
      <defText name="PORT" label="Port">{state["port"]}</defText>
    </defTextVector>""")

    send(f"""<defTextVector device="{d}" name="DEVICE_BAUD_RATE" label="Baud Rate" group="Options"
    state="Idle" perm="rw" timestamp="{ts()}">
      <defText name="BAUD" label="Baud">{state["baud"]}</defText>
    </defTextVector>""")

    # Core Mount Properties
    send(f"""<defNumberVector device="{d}" name="HORIZONTAL_COORD" label="Horizontal Coords" group="Main Control"
    state="Idle" perm="rw" timestamp="{ts()}">
      <defNumber name="AZ" label="Azimuth (deg)" format="%.2f" min="0" max="360" step="0.1">{state["az"]:.2f}</defNumber>
      <defNumber name="ALT" label="Altitude (deg)" format="%.2f" min="-10" max="90" step="0.1">{state["el"]:.2f}</defNumber>
    </defNumberVector>""")

    # Added for full mount recognition and RA/DEC support from KStars
    send(f"""<defNumberVector device="{d}" name="EQUATORIAL_EOD_COORD" label="Equatorial Coords" group="Main Control"
    state="Idle" perm="rw" timestamp="{ts()}">
      <defNumber name="RA" label="RA (hh:mm:ss)" format="%010.6m" min="0" max="24" step="0.0001">0</defNumber>
      <defNumber name="DEC" label="DEC (dd:mm:ss)" format="%010.6m" min="-90" max="90" step="0.0001">0</defNumber>
    </defNumberVector>""")

    # Critical for EKOS to treat this as a real mount
    send(f"""<defSwitchVector device="{d}" name="ON_COORD_SET" label="On Coord Set" group="Main Control"
    state="Idle" perm="rw" rule="OneOfMany" timestamp="{ts()}">
      <defSwitch name="SLEW" label="Slew">On</defSwitch>
      <defSwitch name="TRACK" label="Track">Off</defSwitch>
      <defSwitch name="SYNC" label="Sync">Off</defSwitch>
    </defSwitchVector>""")

    send(f"""<defSwitchVector device="{d}" name="TELESCOPE_TRACK_STATE" label="Tracking" group="Main Control"
    state="Idle" perm="rw" rule="OneOfMany" timestamp="{ts()}">
      <defSwitch name="TRACK_ON" label="On">Off</defSwitch>
      <defSwitch name="TRACK_OFF" label="Off">On</defSwitch>
    </defSwitchVector>""")

    send(f"""<defSwitchVector device="{d}" name="TELESCOPE_ABORT_MOTION" label="Abort Motion" group="Main Control"
    state="Idle" perm="rw" rule="AtMostOne" timestamp="{ts()}">
      <defSwitch name="ABORT" label="Abort">Off</defSwitch>
    </defSwitchVector>""")

    send(f"""<defSwitchVector device="{d}" name="TELESCOPE_PARK" label="Park" group="Main Control"
    state="Idle" perm="rw" rule="OneOfMany" timestamp="{ts()}">
      <defSwitch name="PARK" label="Park">Off</defSwitch>
      <defSwitch name="UNPARK" label="Unpark">On</defSwitch>
    </defSwitchVector>""")

    # Telescope Info + Location (helps EKOS/KStars)
    send(f"""<defNumberVector device="{d}" name="TELESCOPE_INFO" label="Telescope Information" group="Options"
    state="Idle" perm="rw" timestamp="{ts()}">
      <defNumber name="TELESCOPE_APERTURE" label="Aperture (mm)" format="%.f" min="10" max="10000" step="1">100</defNumber>
      <defNumber name="TELESCOPE_FOCAL_LENGTH" label="Focal Length (mm)" format="%.f" min="10" max="10000" step="1">0</defNumber>
      <defNumber name="GUIDE_RATE" label="Guide Rate" format="%.2f" min="0" max="1" step="0.01">0.5</defNumber>
    </defNumberVector>""")

    send(f"""<defNumberVector device="{d}" name="GEOGRAPHIC_COORD" label="Location" group="Options"
    state="Idle" perm="rw" timestamp="{ts()}">
      <defNumber name="LAT" label="Latitude" format="%.6f" min="-90" max="90" step="0.0001">51.5</defNumber>
      <defNumber name="LONG" label="Longitude" format="%.6f" min="-180" max="180" step="0.0001">0.0</defNumber>
      <defNumber name="ELEV" label="Elevation (m)" format="%.f" min="0" max="5000" step="1">10</defNumber>
    </defNumberVector>""")

    # Driver info – explicitly declares TELESCOPE interface
    send(f"""<defTextVector device="{d}" name="DRIVER_INFO" label="Driver Info" group="Options"
    state="Idle" perm="ro" timestamp="{ts()}">
      <defText name="DRIVER_NAME" label="Name">{DEVICE_NAME}</defText>
      <defText name="DRIVER_EXEC" label="Exec">indi_antenna_rotator.py</defText>
      <defText name="DRIVER_VERSION" label="Version">1.2</defText>
      <defText name="DRIVER_INTERFACE" label="Interface">0x0001</defText>
    </defTextVector>""")


# ── Serial & Movement ─────────────────────────────────────────────────
def serial_connect() -> bool:
    with lock:
        p, b = state["port"], int(state["baud"])
    try:
        ser = serial.Serial(p, b, timeout=1)
        with lock:
            state["ser"] = ser
            state["connected"] = True
        log(f"Connected to {p}")
        return True
    except Exception as e:
        log(f"Failed to open serial: {e}")
        return False

def serial_disconnect():
    with lock:
        ser = state["ser"]
        state["ser"] = None
        state["connected"] = False
    if ser:
        try:
            ser.close()
        except Exception:
            pass
    log("Serial disconnected")

def move_to(az: float, el: float):
    az = max(0.0, min(360.0, az))
    el = max(0.0, min(90.0, el))
    with lock:
        state["az"] = az
        state["el"] = el
        ser = state["ser"]
    if ser:
        cmd = f"a{az:.2f},{el:.2f}\n"
        try:
            ser.write(cmd.encode('utf-8'))
            ser.flush()
        except Exception as e:
            log(f"Serial write failed: {e}")


# ── Reporting Functions ───────────────────────────────────────────────
def report_connection(connected: bool):
    c = "On" if connected else "Off"
    d = "Off" if connected else "On"
    st = "Ok" if connected else "Idle"
    send(f"""<setSwitchVector device="{DEVICE_NAME}" name="CONNECTION" state="{st}" timestamp="{ts()}">
      <oneSwitch name="CONNECT">{c}</oneSwitch>
      <oneSwitch name="DISCONNECT">{d}</oneSwitch>
    </setSwitchVector>""")

def report_coords():
    with lock:
        az, el = state["az"], state["el"]
    send(f"""<setNumberVector device="{DEVICE_NAME}" name="HORIZONTAL_COORD" state="Ok" timestamp="{ts()}">
      <oneNumber name="AZ">{az:.2f}</oneNumber>
      <oneNumber name="ALT">{el:.2f}</oneNumber>
    </setNumberVector>""")

def report_eq_coords():
    with lock:
        ra, dec = state["ra"], state["dec"]
    send(f"""<setNumberVector device="{DEVICE_NAME}" name="EQUATORIAL_EOD_COORD" state="Ok" timestamp="{ts()}">
      <oneNumber name="RA">{ra:.6f}</oneNumber>
      <oneNumber name="DEC">{dec:.6f}</oneNumber>
    </setNumberVector>""")

def report_on_coord_set():
    with lock:
        m = state["coord_mode"]
    send(f"""<setSwitchVector device="{DEVICE_NAME}" name="ON_COORD_SET" state="Ok" timestamp="{ts()}">
      <oneSwitch name="SLEW">{"On" if m == "SLEW" else "Off"}</oneSwitch>
      <oneSwitch name="TRACK">{"On" if m == "TRACK" else "Off"}</oneSwitch>
      <oneSwitch name="SYNC">{"On" if m == "SYNC" else "Off"}</oneSwitch>
    </setSwitchVector>""")

def report_track_state():
    with lock:
        on = "On" if state["tracking"] else "Off"
        off = "Off" if state["tracking"] else "On"
    send(f"""<setSwitchVector device="{DEVICE_NAME}" name="TELESCOPE_TRACK_STATE" state="Ok" timestamp="{ts()}">
      <oneSwitch name="TRACK_ON">{on}</oneSwitch>
      <oneSwitch name="TRACK_OFF">{off}</oneSwitch>
    </setSwitchVector>""")


# ── Handlers ──────────────────────────────────────────────────────────
def handle_new_switch(name: str, switches: dict):
    if name == "CONNECTION":
        if switches.get("CONNECT") == "On":
            if serial_connect():
                report_connection(True)
                report_coords()
        elif switches.get("DISCONNECT") == "On":
            serial_disconnect()
            report_connection(False)

    elif name == "ON_COORD_SET":
        for k, v in switches.items():
            if v == "On":
                with lock:
                    state["coord_mode"] = k
        report_on_coord_set()
        log(f"Mode changed to {state['coord_mode']}")

    elif name == "TELESCOPE_TRACK_STATE":
        with lock:
            state["tracking"] = (switches.get("TRACK_ON") == "On")
        report_track_state()
        log(f"Tracking {'enabled' if state['tracking'] else 'disabled'}")
        if state["tracking"]:
            threading.Thread(target=tracking_loop, daemon=True).start()

    elif name == "TELESCOPE_ABORT_MOTION":
        if switches.get("ABORT") == "On":
            log("Motion aborted")
            with lock:
                state["tracking"] = False
            report_coords()

    elif name == "TELESCOPE_PARK":
        if switches.get("PARK") == "On":
            log("Parking antenna (moving to 0,0)")
            move_to(0.0, 0.0)
            report_coords()
        elif switches.get("UNPARK") == "On":
            log("Unparked")


def handle_new_number(name: str, numbers: dict):
    if name == "HORIZONTAL_COORD":
        az = float(numbers.get("AZ", state["az"]))
        el = float(numbers.get("ALT", state["el"]))
        log(f"Slewing to Az={az:.2f}° El={el:.2f}°")
        move_to(az, el)
        report_coords()

    elif name == "EQUATORIAL_EOD_COORD":
        # Placeholder – full conversion needs astropy + location/time
        ra = float(numbers.get("RA", state["ra"]))
        dec = float(numbers.get("DEC", state["dec"]))
        with lock:
            state["ra"], state["dec"] = ra, dec
        log(f"Received RA/Dec command ({ra:.4f}, {dec:.2f}). Mapping to current AltAz (full conversion TODO).")
        # For now just report back and move to last known AltAz
        move_to(state["az"], state["el"])
        report_eq_coords()
        report_coords()

    elif name == "TELESCOPE_INFO" or name == "GEOGRAPHIC_COORD":
        # Just acknowledge – values stored if you want to expand later
        send(f"""<setNumberVector device="{DEVICE_NAME}" name="{name}" state="Ok" timestamp="{ts()}">
          <oneNumber name="{list(numbers.keys())[0] if numbers else 'LAT'}">0</oneNumber>
        </setNumberVector>""")


def handle_new_text(name: str, texts: dict):
    if name == "DEVICE_PORT":
        with lock:
            state["port"] = texts.get("PORT", state["port"])
        send(f'<setTextVector device="{DEVICE_NAME}" name="DEVICE_PORT" state="Ok" timestamp="{ts()}"><oneText name="PORT">{state["port"]}</oneText></setTextVector>')
    elif name == "DEVICE_BAUD_RATE":
        with lock:
            state["baud"] = texts.get("BAUD", state["baud"])
        send(f'<setTextVector device="{DEVICE_NAME}" name="DEVICE_BAUD_RATE" state="Ok" timestamp="{ts()}"><oneText name="BAUD">{state["baud"]}</oneText></setTextVector>')


def tracking_loop():
    """Simple tracking simulation. In a real implementation you would convert current target RA/Dec to AltAz every few seconds."""
    while True:
        with lock:
            if not state["tracking"] or not state["connected"]:
                break
            # TODO: Use location + time + astropy.coordinates to compute current AltAz from stored RA/Dec
            # For now we just keep sending the last position every 5s
        report_coords()
        time.sleep(5.0)


# ── XML Parser & Dispatcher ───────────────────────────────────────────
class IndiParser:
    def __init__(self):
        self.buf = ""
        self.depth = 0

    def feed(self, chunk: str):
        results = []
        for ch in chunk:
            self.buf += ch
            if ch == '<':
                continue
            if ch == '>':
                tag_part = self.buf.rsplit('<', 1)[-1].rstrip('>').strip()
                if tag_part.startswith('/'):
                    self.depth -= 1
                elif not tag_part.endswith('/'):
                    self.depth += 1
                if self.depth <= 0 and self.buf.strip():
                    results.append(self.buf.strip())
                    self.buf = ""
                    self.depth = 0
        return results


def parse_element(xml_str: str):
    try:
        return ET.fromstring(xml_str)
    except ET.ParseError:
        return None


def dispatch(elem):
    if elem is None:
        return
    tag = elem.tag
    if tag == "getProperties":
        define_all_properties()
    elif tag == "newSwitchVector":
        name = elem.attrib.get("name", "")
        switches = {child.attrib["name"]: child.text.strip() for child in elem if child.tag == "oneSwitch"}
        handle_new_switch(name, switches)
    elif tag == "newNumberVector":
        name = elem.attrib.get("name", "")
        numbers = {child.attrib["name"]: child.text.strip() for child in elem if child.tag == "oneNumber"}
        handle_new_number(name, numbers)
    elif tag == "newTextVector":
        name = elem.attrib.get("name", "")
        texts = {child.attrib["name"]: child.text.strip() for child in elem if child.tag == "oneText"}
        handle_new_text(name, texts)


# ── Main ──────────────────────────────────────────────────────────────
def main():
    parser = IndiParser()
    sys.stdin = open(sys.stdin.fileno(), "r", encoding="utf-8", errors="replace", buffering=1)
    sys.stdout = open(sys.stdout.fileno(), "w", encoding="utf-8", buffering=1)

    log("Antenna Rotator INDI driver (v1.2) started. Waiting for EKOS/KStars...")

    for line in sys.stdin:
        for xml in parser.feed(line):
            elem = parse_element(xml)
            dispatch(elem)


if __name__ == "__main__":
    main()