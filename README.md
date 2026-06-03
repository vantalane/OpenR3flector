# OpenR3flector

OpenR3flector is an open-source firmware, driver, and tooling suite that repurposes the **Satenne R3 Premium** automatic satellite TV dish as a general-purpose azimuth-elevation mount controllable via the INDI protocol.

The project replaces the original proprietary controller firmware with custom firmware running on the onboard STM32 microcontroller, and provides an INDI driver that exposes the dish as a telescope mount to planetarium software such as KStars.

This is independent reverse engineering and hobby research work. Not affiliated with Satenne, Vadac BV, or any related entity.

Intended usecase of the repurposed hardware using this project include:

* Radio astronomy
* RF / SDR experimentation
* Satellite tracking for AHRPT and other science downlink reception 

---

## Project contents

* Custom STM32 firmware for controlling dish motion and peripherals.
* [INDI](https://indilib.org/) driver capable of tracking celestial objects and tracking satellites from TLE's.
* Scripts to plot RF sources using an external SDR.
* Scripts to control dish using Gpredict and other hamlib compatible stuff.

## Implemented features

* UART interface for motor control and status reading.
* Device body to world orientation calibration/alignment.
* Polarization and local oscillator controls of the LNB.
* Configurable motion acceleration and velocity.
* Dish control as a telescope in INDI compatible planetarium apps. (Kstars, Skychart etc.) 
* Scripts to operate the dish to track satellites.

## WIP

* Figuring out how the SERIT RF-frontend (STV0903 IC) NIM works.
* Writing MPEG-2 stream and metadata decoder in VHDL for the Spartan 3A.

## Long(er) term goals

* Decode free-to-air satellite TV broadcasts similar to MiniTiouner project using the FPGA on the board.
* Support for similar satellite controllers. (many are likely to contain similar structure and hardware).
* Once satellite TV metadata can be decoded using the FPGA, implement an automatic localisation function/operration to 
determine body orientation and position in the world. Such that manual alignment becomes unnecessary.

---

## Background

The Satenne R3 Premium is a motorized satellite TV dish system designed for caravans/RV's. The original system used a controller board to locate and point to geostationary TV satellites, controlled by a (special?) TV. This project (almost) fully reverse-engineered that controller board and reimplemented control of the dish as a general-purpose alt-az mount.

---

## Repository Structure

```
satenne_r3_opencontrol/
├── code/
│   ├── openr3flector_code/         # STM32 firmware (GPL-3.0-or-later)
│   ├── indi_openr3flector_driver/  # INDI telescope driver (LGPL-2.0 / GPL-3.0-or-later)
│   ├── scripts/                    # Supporting Python scripts (GPL-3.0-or-later)
│   ├── fpga_mpeg_decoder/          # FPGA code (GPL-3.0-or-later)
│   ├── indi/                       # INDI library submodule (LGPL-2.0+)
│   ├── sgp4/                       # SGP4 orbital mechanics submodule
│   └── STM32CubeF4/                # STM32 HAL submodule (BSD-3-Clause)
├── notes/                          # Reverse engineering notes and pinout documentation
├── media/                          # PCB photographs
├── private/                        # Internal reference submodule — inaccessible to the public,
│                                   # not required to build or use this project
├── LICENSE                         # GPL-3.0-or-later
├── LICENSE-DOCS                    # CC-BY-SA 4.0
└── THIRD_PARTY.md                  # Third-party dependency listing
```

---

## Building and Using

Each component has its own README with build instructions:

* Firmware: [`code/openr3flector_code/README.md`](code/openr3flector_code/README.md)
* INDI driver: [`code/indi_openr3flector_driver/README.md`](code/indi_openr3flector_driver/README.md)

### Hardware required

* ST-link for STM32 programming.
* Satenne R3 Premium dish and its controller board.

<!-- TODO fill in useful hw such as rs485 to USB converter and rj12 cable -->
---

## Licensing

All source code, firmware, and scripts authored by vantalane are licensed under the
**GNU General Public License v3.0 or later (GPL-3.0-or-later)**.
See [ `LICENSE` ](LICENSE).

All notes, pinout documentation, reverse engineering notes, and non-code content authored
by vantalane are licensed under **Creative Commons Attribution-ShareAlike 4.0 International
(CC-BY-SA 4.0)**. See [ `LICENSE-DOCS` ](LICENSE-DOCS).

Third-party dependencies carry their own licenses. See [ `THIRD_PARTY.md` ](THIRD_PARTY.md).

---

## Dependencies

See [ `THIRD_PARTY.md` ](THIRD_PARTY.md) for the full listing of submodule dependencies, 
their upstream sources, and their licenses.

---

## Disclaimer

This project is independent hobby and research work. Not affiliated with Satenne, Vadac BV, 
or any related entity. The author is the lawful owner of the physical hardware used.

All content is provided as-is, without warranty of any kind. Use at your own risk.

---

## Author

vantalane / Kestech

mete (at) kestech (dot) net
