# Cyberuler: A Self-Playing Ruler

[English](README.md) | [中文](README_zh.md)

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)
[![Hardware: ESP32-S3](https://img.shields.io/badge/Hardware-ESP32--S3-blue.svg)]()

![Cyberuler](picture/real_Isometric.jpg)

> An automated musical instrument that turns an ordinary steel ruler into a playable synthesizer — controlled by stepper motors, servos, electromagnets, and MIDI.

## Contents

- ✨ [Features](#features)
- 🎵 [A Note on Timbre](#a-note-on-timbre)
- 📂 [Repository Structure](#repository-structure)
- 🛠️ [Hardware at a Glance](#hardware-at-a-glance)
- 🚀 [Getting Started](#getting-started)
  - [Build Prerequisites](#build-prerequisites)
  - [Serial Terminal Commands](#serial-terminal-commands)
- 🗺️ [Roadmap](#roadmap-the-stationery-band)
- 👤 [Author](#author)
- 📄 [License](#license)

## 🎥 Demo

[![Watch on Bilibili](https://img.shields.io/badge/Bilibili-Demo-00A1D6?style=flat&logo=bilibili)](https://www.bilibili.com/video/BV1fZ9hBnEL7)

<a id="features"></a>
## ✨ Features

- **Pitch Control:** A stepper motor with a lead screw slides the ruler to change the effective vibrating length, producing different pitches.
- **Plucking Mechanism:** A high-speed servo strums the ruler. Four electromagnets (H-bridge driven, two top and two bottom) clamp and release the ruler for clean note articulation.
- **Acoustic Calibration:** An INMP441 I2S microphone captures the ruler's real acoustic response. Fast Fourier Transform (FFT) analysis builds a frequency-to-position mapping — no manual tuning required.
- **NLS Curve Fitting:** Uses Levenberg-Marquardt nonlinear least squares to fit measured frequency data to a `f = k/(pos + a)² + b` model for accurate pitch interpolation across the full range.
- **USB MIDI Input:** Connects as a USB MIDI device via TinyUSB. Responds to Note On/Off messages with velocity sensitivity — play it live from any MIDI keyboard or DAW.
- **USB Microphone:** The I2S microphone is also exposed as a USB Audio (UAC 2.0) input — the ruler doubles as a USB mic for your computer.
- **Serial Terminal REPL:** Built-in command-line interface with line editing, ANSI arrow-key navigation, and command history. Full control over motors, calibration, and diagnostics.

<a id="a-note-on-timbre"></a>
## 🎵 A Note on Timbre

A vibrating string (guitar, piano) follows the wave equation — its overtones are integer multiples of the fundamental, producing a familiar harmonic sound. A steel ruler, however, is a cantilever beam governed by the Euler-Bernoulli beam equation. Under one-end-fixed, one-end-free boundary conditions, its natural frequencies are:

$$\omega_n = \beta_n^2 \sqrt{\frac{EI}{\rho A L^4}}$$

where $\beta_n$ are roots of $1 + \cos\beta \cosh\beta = 0$. The first overtone is roughly **6.26×** the fundamental, and the second is **17.55×** — neither is an integer multiple.

This means the ruler's overtones are inherently inharmonic. Calibration ensures the fundamental frequency is pitch-accurate, but the overtone structure is a physical given — the instrument will always carry a raw, slightly "imperfect" timbre. That is not a flaw; it is the character of the instrument.

<a id="repository-structure"></a>
## 📂 Repository Structure

| Directory | Description |
|---|---|
| `v1/code/` | ESP-IDF firmware (C) — motor control, FFT, USB MIDI, REPL terminal |
| `v1/model_A_1.2/` | SolidWorks 3D models + printable STL files |
| `v1/model_legacy/` | Archived older model versions (A_1.0, A_1.1, B) |
| `v1/pcb/` | PCB design files (STEP, Gerber), [OSHWHub project](https://oshwhub.com/stccff/self_play_ruler) |
| `v1/bom_zh.xlsx` | Bill of materials |
| `prototype/` | Early prototype — firmware, Python host scripts, 3D models |
| `tools/` | Auxiliary tools and drivers |
| `picture/` | Project photos, renders, and diagrams |

<a id="hardware-at-a-glance"></a>
## 🛠️ Hardware at a Glance

- **MCU:** ESP32-S3 DevKitC
- **PCB:** Custom mainboard (fabricated by JLCPCB) — integrates power, motor drivers, microphone, USB MIDI, and USB Audio. Gerber files in `v1/pcb/`, [OSHWHub project page](https://oshwhub.com/stccff/self_play_ruler)
- **Stepper Driver:** DRV8825 module (A4988 optional)
- **Electromagnet Driver:** L9110S module
- **Voltage Regulator:** MP1845EN module (Mini360 optional)
- **Stepper Motor:** Lead-screw stepper salvaged from a CD-ROM drive
- **Servo:** SG90 (strumming)
- **Electromagnets:** x4 — 2 top, 2 bottom
- **Audio:** INMP441 I2S microphone
- **Power:** 12V DC barrel jack (external power supply required)
- **Chassis:** 3D-printed parts + 150mm steel ruler

![Exploded view](picture/sw_explode.png)

![Custom PCB](picture/pcb.jpg)

See `v1/bom_zh.xlsx` for the detailed bill of materials.

> ⚠️ **Important Notes:**
> - The ESP32-S3 DevKitC is powered via USB. The 12V DC input supplies the motors and drivers only — the device will not function without USB connected.
> - If the **SGM2521YS8** power-switch IC is not populated on the PCB, do **not** disconnect USB while 12V power remains connected. Doing so cuts power to the ESP32, leaving the DRV8825 control pins floating and the stepper motor in an abnormal state that causes overheating.

<a id="getting-started"></a>
## 🚀 Getting Started

1. **Print the parts:** STL files are in `v1/model_A_1.2/output/`.
2. **Assemble the hardware:** Mount motors, ruler, electromagnets, and PCB onto the printed chassis.
3. **Flash the firmware:** Build and upload the ESP-IDF project in `v1/code/` to your ESP32-S3.
4. **Calibrate:** Press the BOOT button on the dev board (single-click triggers auto-calibration), or run the `ftinit` command via the serial terminal. The microphone listens as the ruler is plucked across its range to build the pitch map.

![BOOT button for calibration](picture/button_info.png)
5. **Play:** Connect a USB MIDI keyboard, or use the serial terminal (`p` command) to play notes directly.

### Build Prerequisites

- **ESP-IDF** v5.0 or later (target: ESP32-S3)
- Dependencies (auto-resolved by IDF Component Manager):
  - `espressif/esp-dsp` — FFT library
  - `espressif/tinyusb` — USB MIDI device stack
  - `espressif/button` — GPIO button driver
  - `espressif/led_strip` — WS2812 RGB LED driver

```bash
cd v1/code
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash
```

### Serial Terminal Commands

Connect via UART (115200 baud) to access the REPL. Key commands:

| Command | Description |
|---|---|
| `help` | List all available commands |
| `ftinit` | Rebuild frequency calibration table |
| `p <note>` | Play a note by numbered musical notation (e.g. `p 1.`) |
| `midivelocity <0\|1>` | Toggle MIDI velocity sensitivity |
| `compiletime` | Print firmware build timestamp |

<a id="roadmap-the-stationery-band"></a>
## 🗺️ Roadmap: The Stationery Band

The Self-Playing Ruler is the first instrument in a larger vision — a full desk-sized automated orchestra built from everyday stationery.

![Stationery Band](picture/stationery_band.png)

- [x] **The Guitar/Bass:** Cyberuler (This project)
- [ ] **The Flute:** Automated Wind Pen
- [ ] **The Drums:** Percussive Pen and Desk Tappers
- [ ] **The Conductor:** Central sync hub for all instruments

<a id="author"></a>
## 👤 Author

**stccff** — [github.com/stccff](https://github.com/stccff) — [913602792@qq.com](mailto:913602792@qq.com)

Feedback, questions, and build stories are always welcome. If you make your own, I'd love to see it.

<a id="license"></a>
## 📄 License

This project is licensed under [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/) — you are free to share and adapt for non-commercial purposes, with attribution and under the same license.

See the [LICENSE](LICENSE) file for details.
