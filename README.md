# OpenGrip‑FFB

> vJoy Force‑Feedback → ODrive controller + sleek PySide6 GUI for DIY direct‑drive wheels (and future flight‑sticks)

OpenGrip‑FFB bridges Windows games’ vJoy force‑feedback (FFB) with an ODrive‑powered direct‑drive motor. It contains a high‑rate async controller and a polished Qt (PySide6) desktop app with profiles, effect multipliers, live status, and persistence.

---

## Features

* **Real‑time controller** that reads game FFB, mixes effects, and writes **torque/current** to ODrive at high frequency (configurable loop rates).
* **Steering angle mapping** with configurable **lock‑to‑lock degrees** and optional **angle print** for debugging.
* **User center**: hotkeys to **re‑center (C)** and **zero center (Z)** at runtime.
* **Global torque polarity** (+/‑) with a single toggle (**I**) to address sign inversions quickly.
* **vJoy X‑axis output** updated from the wheel’s true position so games stay in sync.
* **PySide6 GUI** with Start/Stop toggle, master/effect multipliers, profiles, scrollable tabs, status bar, and QSettings persistence.
* **Future**: Flight‑stick tab scaffolded.

---

## Repository Layout

```
ffb_gui.py                 # PySide6 desktop app (tabs: Main, Effects, Flight Stick, About)
ffb_odrive_controller.py   # Async ODrive controller + vJoy position output
ffb_sniffer.py             # CLI entrypoint (parses args, wires monitor+controller)
```

> The project also expects a `vjoy_ffb_monitor` module that captures and mixes vJoy FFB effects (part of the project in the repo).

---

## Requirements

* **OS**: Windows 10/11 (vJoy + Windows hotkeys).
* **Python**: 3.9+ recommended.
* **Drivers / SDKs**:

  * **vJoy** driver and **vJoyInterface.dll** available on PATH for axis output.
  * **ODrive** configured and reachable via USB.
* **Python packages**:

  * `PySide6` (GUI)
  * `odrive` (motor control)
  * project module `vjoy_ffb_monitor` (FFB capture/mix)

Install the basics:

```bash
pip install PySide6 odrive
```

Install and configure **vJoy** from its official installer, then create a vJoy device (default ID: 1).

> ⚠️ **Safety**: Start with conservative current/torque limits. Secure the wheel, keep fingers clear, and have an e‑stop.

---

## Quick Start

### 1) Connect hardware

* ODrive + BLDC motor + encoder wired and calibrated.
* vJoy installed with at least one virtual device (ID **1** by default).

### 2) Run the GUI

```bash
python ffb_gui.py
```

**Main tab**

* **Axis** selector and **Re‑Center / Zero Center / Calibrate Axis** actions.
* **Max Strength (Nm)**, **Lock‑to‑Lock (°)**, **Current Limit (A)**, **Force Hz**, **Position Hz**.
* Wheel visualization with center/end‑stop ticks.

**Effects tab**

* **Master multiplier** and per‑effect **sliders/spinboxes** with optional **Invert**.
* **Reset All** returns to defaults.

**Profiles**

* Editable combo + **Save/Load/Save As/Delete** with persistence (QSettings + JSON profiles).

**About**

* Project link + author + support button.

### 3) Run the Controller from CLI

You can also run the controller headless:

```bash
python ffb_sniffer.py \
  --vjoy-id 1 \
  --odrive-axis 1 \
  --gain 1.0 \
  --lock-to-lock-deg 900 \
  --force-hz 800 --pos-hz 400
```

Common options:

* **FFB shaping**:

  * `--strength Constant=0.8 Spring=1.2 All=1.5`
  * `--invert Constant Spring`
  * `--ffb-max-nm 2.0` (maps USB 10000 → Nm)
* **Diagnostics**: `--print-angle`, `--debug-ffb`, `--debug-force`
* **Mode**: `--mode auto|torque|current`
* **Test sine**: `--test-sine-amp 1.0 --test-sine-hz 1.0`
* **Hotkeys** (Windows console): `C` (recenter), `Z` (clear center), `I` (invert torque polarity)

Stop with **Ctrl+C**.

---

## How It Works

1. **VJoyFfbMonitor** captures USB FFB effects (Constant, Spring, Damper, etc.), applies your multipliers/inversions, and produces a net torque command.
2. **ODriveController** runs two high‑rate async loops:

   * **Position reader** updates the steering angle/turns and writes vJoy X‑axis (so the game sees correct position).
   * **Force writer** sends torque (Nm) or current (A) to ODrive at **force_hz**.
3. If torque mode seems ineffective (Iq ≈ 0), it can **fallback to current control** automatically.

---

## Troubleshooting

* **“Could not load vJoyInterface.dll” / “Could not acquire vJoy device”**

  * Ensure vJoy is installed, `vJoyInterface.dll` is on PATH, and the device ID exists/enabled.
* **No angle updates or wrong center**

  * Use **C** to recenter at the current position, or **Z** to clear the user offset.
* **Forces push the wrong way**

  * Tap **I** (invert torque polarity) or set the initial polarity in code/args.
* **ODrive not entering CLOSED_LOOP_CONTROL**

  * Verify wiring, encoder calibration, current limits, and watchdog.

---

## Acknowledgements

* Author: **Lucas Coraça Silva**
* GitHub: **github.com/LucasCoraca/OpenGrip‑FFB**
* Built with **Python**, **PySide6/Qt**, **vJoy**, and **ODrive**.

---

## Support

If this project helps you, consider supporting its development on Patreon:

[![Support on Patreon](https://img.shields.io/badge/Support-Patreon-orange.svg)](https://www.patreon.com/lucascoraca)

---

## License (OpenGrip‑FFB)

**GNU GPL v3.0 or later** (GPL‑3.0‑or‑later).

SPDX‑License‑Identifier: `GPL-3.0-or-later`

You are free to use, study, modify, and redistribute this software under the terms of the GNU General Public License. If you convey modified versions or binaries, you must also provide the corresponding source code under the same license and include the full GPL text. Place the license text in a `LICENSE` (or `COPYING`) file at the repo root.

**Obligations summary** (non‑lawyerly):

* Keep this project under GPL‑3.0‑or‑later when redistributing.
* Include a copy of the GPL with any distribution.
* Provide source (or a written offer) for any binaries you ship that include or link this code.

> MIT‑licensed dependencies listed below are **GPL‑compatible**; keep their copyright notices and license texts.

---

## Third‑party Licenses & References

**ODrive**

* Project: [https://github.com/odriverobotics/ODrive](https://github.com/odriverobotics/ODrive)
* License: **MIT** (see the repo’s `LICENSE.md`).
* Docs: [https://docs.odriverobotics.com/](https://docs.odriverobotics.com/)

**vJoy**

* Project: [https://github.com/shauleiz/vJoy](https://github.com/shauleiz/vJoy) (original)
* License: **MIT** (see the repo’s `LICENSE.txt`).
* Website: [http://vjoystick.sourceforge.net/](http://vjoystick.sourceforge.net/)

**Redistribution notes**

* You may bundle `vJoyInterface.dll` with your app. The vJoy SDK is MIT‑licensed; include its license text.
* Always preserve upstream copyright and license notices in your distributions (e.g., ship a `third_party_licenses/` folder or aggregate them in your About box).

---

## Acknowledgements

* Author: **Lucas Coraça Silva**
* GitHub: **github.com/LucasCoraca/OpenGrip‑FFB**
* Built with **Python**, **PySide6/Qt**, **vJoy**, and **ODrive**.
