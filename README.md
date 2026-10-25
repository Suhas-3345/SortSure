# SortSure

**A confidence-aware, multimodal edge-AI waste segregation station for cafeterias and food courts.**
Vishwakarma Awards 2026-27 | Theme: Sustainable Cities | Sub-theme: Waste Management & Circular Economy Hardware

Team **BinaryBin**: H.C. Chinmay, B. Venkata Suhas, Diya Chinnappa, Atharv Somani, Komal Kumari

> **Design principle:** when the system is not sure what an item is, it protects the recyclable stream by sending the item to residual instead of guessing. Every physical action is verified before the next one starts.

## Project status (read this first)

| Part | Status |
|---|---|
| Control logic (state machine, confidence policy, interlocks, fault handling) | Written; **simulated in Wokwi** (`wokwi/`) |
| ESP32 real-hardware firmware (`firmware/`) | Written; **not yet run on hardware** |
| Raspberry Pi vision node (`pi/`) | Written; **not yet run on hardware**; **no trained model yet**, so every item routes to residual |
| Mechanical CAD | [ADD: status / link] |
| KiCad schematics | [ADD: status / link] |
| Dataset and model validation | Not started; plan in section "Validation plan" |

Confidence thresholds, mass and height limits, step counts and current limits in the code are **placeholders**, to be tuned from measured data.

## How it works

1. An item enters through the intake (beam-break). The intake flap closes.
2. The ESP32 reads mass (HX711 load cell), height (VL53L1X), and metal (inductive sensor) and turns on the LED ring.
3. The ESP32 sends `REQ,mass_g=..,metal=..,height_mm=..` to the Raspberry Pi over UART.
4. The Pi captures an image, checks image quality, runs the classifier and replies `RES,<CLASS>,<confidence>` with CLASS in `ORG PET CAN PAP OTH`.
5. The ESP32 applies the policy:

| Condition | Action |
|---|---|
| High confidence and sensors agree | Route to the predicted stream (organic or dry) |
| Mid confidence | Ask for one more view, then reassess; still unsure goes to residual |
| Low confidence, class `OTH`, or no reply (8 s) | Residual |
| Sensor contradiction (e.g. `PET` with metal detected) | Residual, event logged |
| Item outside mass or height range | Residual |
| Invalid sensor or Pi unreachable (3 timeouts) | Degraded mode: residual only |

6. The chute rotates to the target outlet. **The trapdoor opens only after the index sensor confirms alignment.**
7. A drop beam confirms the item passed. One recovery pulse is allowed, then maintenance lockout.
8. The chute returns home and the home sensor confirms no steps were lost.

The Raspberry Pi only classifies. It cannot move any actuator.

## Repository layout

```
wokwi/              Wokwi simulation (sketch.ino, diagram.json, libraries.txt)
firmware/SortSure_esp32/SortSure_esp32.ino    ESP32 real-hardware firmware
pi/pi_vision.py     Raspberry Pi vision node and dataset collector
cad/                [ADD] CAD files (.STEP and native)
kicad/              [ADD] KiCad project
docs/               [ADD] photos, test logs, screenshots
```

## Run the simulation (no hardware needed)

1. Open wokwi.com, create a new **ESP32** project.
2. Paste `wokwi/sketch.ino`, `wokwi/diagram.json` and `wokwi/libraries.txt` into the matching tabs.
3. Press play and open the Serial Monitor.

Stand-ins: potentiometers for load cell, ToF and motor current; buttons for beam, metal, drop, home and hand-interlock sensors; two potentiometers act as the Raspberry Pi's class and confidence output (Wokwi cannot run a Raspberry Pi). Typing `RES,PET,0.93` in the Serial Monitor also works.

Demo cases: confident dry item; sensor conflict (hold METAL then `RES,PET,0.95`); mid confidence (`RES,PAP,0.70`); unknown item (`RES,OTH,0.9`); Pi timeout (send nothing for 8 s); hand interlock (hold HAND during motion); jam (current knob to maximum); failed drop (don't press DROP).

## Real hardware

### ESP32 pin map (DevKit 38-pin)

| Function | GPIO | Notes |
|---|---|---|
| A4988 STEP / DIR / EN | 18 / 19 / 23 | EN goes through the interlock gate; external 10k pull-up so the driver is disabled at boot |
| Trapdoor servo | 13 | Servo powered from the 5 V rail, common ground |
| HX711 DT / SCK | 4 / 5 | |
| I2C SDA / SCL | 21 / 22 | VL53L1X (0x29), INA219 (0x40) |
| Pi UART RX2 / TX2 | 16 / 17 | 3.3 V both sides |
| Intake beam | 27 | LOW = item entered |
| Inductive sensor (via optocoupler) | 26 | LOW = metal; 10k pull-up |
| Drop beam | 25 | LOW = item passed |
| Home / organic index | 33 | LOW = aligned |
| Dry index | 35 | Input-only; external 10k pull-up |
| Residual index | 39 | Input-only; external 10k pull-up |
| Hand / second-item beam | 32 | LOW = blocked |
| Cover switch | 14 | Switch to GND when closed; HIGH (open or broken wire) = unsafe |
| LED ring MOSFET gate | 15 | 10k gate pull-down |
| Status LED | 2 | |

### Raspberry Pi wiring

- Pi GPIO14 TXD (pin 8) to ESP32 GPIO16; Pi GPIO15 RXD (pin 10) to ESP32 GPIO17.
- Join grounds (Pi pin 6, 9 or 14 to ESP32 GND).
- Camera on the CSI connector. Power the Pi from its own 5 V / 3 A supply.
- Enable the serial hardware and disable the serial login shell in `raspi-config`.

### Flash the ESP32

Arduino IDE, board **ESP32 Dev Module**. Install libraries: `HX711` (bogde), `VL53L1X` (Pololu), `Adafruit INA219`, `ESP32Servo`. Open `firmware/SortSure_esp32/SortSure_esp32.ino`.

### Pi setup

```
sudo apt install -y python3-picamera2 python3-serial python3-pil python3-numpy
pip install tflite-runtime --break-system-packages    # or: pip install ai-edge-litert
python3 pi/pi_vision.py run
```

Place `model.tflite` and `labels.txt` (classes in model output order, using exactly `ORG PET CAN PAP OTH`) next to `pi_vision.py`. Without a model every item goes to residual.

### Bring-up order (do not skip ahead)

1. Power with the stepper supply off. Send `STATUS` over USB serial (115200) to see which sensors respond.
2. Load cell: `TARE` with an empty tray, add a known weight, send `CAL,<grams>`.
3. Height: `ZERO` with an empty tray.
4. Stepper alone with the chute disconnected. Set `DIR_FORWARD_LEVEL` and `POS_STEPS` so each index sensor triggers at its outlet.
5. Servo trapdoor angles (`DOOR_OPEN`, `DOOR_CLOSED`).
6. Full cycle with typed replies (`RES,PET,0.93`), then with the Pi stub.
7. Add the real vision model last.

USB maintenance commands: `STATUS`, `TARE`, `CAL,<g>`, `ZERO`, `R` (reset from lockout).

### Message protocol (UART, 115200 8N1, newline-terminated)

| Direction | Message |
|---|---|
| ESP32 to Pi | `REQ,mass_g=<g>,metal=<0/1>,height_mm=<mm>` |
| Pi to ESP32 | `RES,<ORG\|PET\|CAN\|PAP\|OTH>,<0..1>` |
| ESP32 to Pi/USB | `DASH,{json}` every 10 s (counts, faults, sensor health) for the facility dashboard |

## Safety design

- Nothing is released until chute alignment is confirmed by an index sensor.
- A hand or second item at the intake, or an open cover, blocks all actuator movement (software pause and hardware EN gate).
- Failed drop: one recovery pulse, then maintenance lockout with the driver disabled.
- Weak evidence always defaults to residual or a degraded state.
- No crushing or compaction.

**Prototype safety interlock only. This is not a certified functional-safety design.**

## Known limitations

- A stepper driver draws roughly constant current even when stalled, so the current check only catches gross faults. Lost steps and jams are caught by the index sensors, which must be fitted.
- No hardware watchdog yet (ESP32 core 3.x API change); add before any unattended use.
- No Wi-Fi dashboard yet; JSON is sent over serial and a Pi service would forward it.
- Image quality gates (brightness, sharpness) and confidence thresholds are untuned.
- The inductive sensor has a very short range (a few mm); tray must be non-metallic and the sensor mounted close under it.
- Closed containers, free liquids, biomedical waste, e-waste and hazardous items are out of scope.

## Validation plan

- Collect images in the physical sensing chamber (clean, dirty, crushed, transparent, occluded, composite, unknown).
- Split train and test by object, not by image.
- Report confusion matrix, per-class precision and recall, unknown-rejection rate, and **false-accept rate into recycling**.
- Track dry-recyclable stream purity, successful drops and jams per 100 cycles, median cycle time, inference latency and energy per cycle.

## Development notes

[ADD, and keep it truthful: who built what, which tools were used, including any AI assistance for code, images or drafts, and what you have personally tested.]
