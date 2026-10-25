# SortSure

**A Confidence-Aware, Multimodal Edge-AI Waste Segregation Station for Cafeterias and Food Courts**

**Vishwakarma Awards 2026–27** | Theme: Sustainable Cities | Sub-theme: Waste Management & Circular Economy Hardware

**Team BinaryBin:** H.C. Chinmay, B. Venkata Suhas, Diya Chinnappa, Atharv Somani, Komal Kumari

---

## 1. Project Overview

SortSure is a smart waste-segregation prototype designed to support waste sorting in cafeterias and food courts using physical sensors, embedded control, and computer vision.

The system combines an ESP32 microcontroller with a Raspberry Pi vision node. The ESP32 handles sensor acquisition, actuator control, safety interlocks, and fault recovery, while the Raspberry Pi captures images and performs waste classification.

**Core principle:** When the available evidence is uncertain or contradictory, the system defaults to residual waste rather than making an unreliable recycling decision.

**Current Status:** Control logic has been simulated in Wokwi. The ESP32 firmware and Raspberry Pi vision node have been written but have not yet been validated on physical hardware. A trained classification model and full dataset validation are still pending.

---

## 2. Key Features

- **Multimodal Sensing:** Planned integration of weight, height/distance, and metal detection.
- **Vision-Assisted Classification:** Camera-based waste recognition using a Raspberry Pi.
- **Confidence-Aware Routing:** Uncertain, unknown, or contradictory classifications default to residual waste.
- **Safety Interlocks:** Chute alignment must be confirmed before the trapdoor opens.
- **Fault Handling:** Communication timeouts, sensor faults, and failed drops trigger defined fallback behaviour.
- **Dataset Collection:** Supports capturing labelled images for future model training.
- **Serial Diagnostics:** Maintenance commands and structured status messages support testing and debugging.

---

## 3. System Architecture

### ESP32 Control Unit

Responsible for:

- Reading sensor data.
- Controlling the stepper motor and trapdoor servo.
- Managing safety interlocks.
- Communicating with the Raspberry Pi.
- Applying classification and sensor-consistency policies.
- Handling timeouts, failed drops, and recovery procedures.

### Raspberry Pi Vision Unit

Responsible for:

- Capturing images through a compatible camera.
- Checking image brightness and sharpness.
- Running the classification model when available.
- Returning classification labels and confidence scores.
- Collecting labelled images for dataset development.

**Safety principle:** The Raspberry Pi performs classification only and does not directly control actuators.

---

## 4. System Workflow

1. An item is detected at the intake, and the intake mechanism closes.
2. The ESP32 reads the available sensors and activates the illumination ring.
3. Sensor information is sent to the Raspberry Pi through UART.
4. The Raspberry Pi captures an image, evaluates image quality, and attempts classification if a valid model is available.
5. The Raspberry Pi returns a classification label and confidence score.
6. The ESP32 evaluates the classification, sensor consistency, and safety conditions.
7. The chute moves towards the selected outlet.
8. The trapdoor opens only after the corresponding index sensor confirms alignment.
9. A drop sensor checks whether the item has passed. A limited recovery attempt is followed by maintenance lockout if the drop still fails.
10. The mechanism returns to its home position, verified using the home sensor.

### Classification Labels

| Label | Intended Category |
|---|---|
| `ORG` | Organic waste |
| `PET` | PET plastic |
| `CAN` | Can |
| `PAP` | Paper |
| `OTH` | Other, unknown, or residual waste |

The classification model, routing policy, and sensor-consistency rules must be validated against the physical system before deployment.

---

## 5. Repository Structure

```text
SortSure/
├── README.md
├── .gitignore
├── firmware/
│   └── SortSure_esp32/
│       └── SortSure_esp32.ino
└── pi/
    └── pi_vision.py
```

Additional directories may be introduced as development progresses:

```text
wokwi/    Wokwi simulation files
cad/      Mechanical CAD files
kicad/    Schematics and PCB design files
docs/     Photographs, test logs, and validation reports
```

These additional directories should only be included when their corresponding files are available.

---

## 6. Hardware Configuration

### ESP32 Pin Mapping

The firmware is intended for an ESP32 DevKit with a 38-pin layout.

| Function | GPIO |
|---|---|
| Stepper driver STEP / DIR / EN | 18 / 19 / 23 |
| Trapdoor servo | 13 |
| HX711 DT / SCK | 4 / 5 |
| I2C SDA / SCL | 21 / 22 |
| Raspberry Pi UART RX2 / TX2 | 16 / 17 |
| Intake beam | 27 |
| Inductive sensor input | 26 |
| Drop beam | 25 |
| Home / organic index | 33 |
| Dry index | 35 |
| Residual index | 39 |
| Hand / second-item beam | 32 |
| Cover switch | 14 |
| LED-ring MOSFET gate | 15 |
| Status LED | 2 |

Verify these assignments against the current firmware and actual wiring before powering the system. Some GPIO pins are input-only and may require external pull-ups according to the circuit design.

### Raspberry Pi UART Connections

The design uses a 115200-baud serial connection.

- Pi GPIO14 TXD (physical pin 8) → ESP32 GPIO16 (RX2).
- Pi GPIO15 RXD (physical pin 10) ← ESP32 GPIO17 (TX2).
- Connect the Raspberry Pi and ESP32 grounds.
- Use compatible 3.3 V UART logic on both sides.
- Power the Raspberry Pi using its own suitable power supply.

Enable the serial hardware and disable the serial login shell through `raspi-config` before using the UART interface.

---

## 7. Serial Communication Protocol

Messages are newline-terminated and use 115200 baud, 8 data bits, no parity, and 1 stop bit (8N1).

| Direction | Message Format |
|---|---|
| ESP32 → Raspberry Pi | `REQ,mass_g=<g>,metal=<0/1>,height_mm=<mm>` |
| Raspberry Pi → ESP32 | `RES,<ORG\|PET\|CAN\|PAP\|OTH>,<confidence>` |
| ESP32 status output | `DASH,{json}` |

The Raspberry Pi script uses `/dev/serial0` by default.

---

## 8. Software Setup

### ESP32 Firmware

1. Install the Arduino IDE.
2. Install ESP32 board support.
3. Select **ESP32 Dev Module**.
4. Install the required libraries:
   - HX711 (bogde)
   - VL53L1X (Pololu)
   - Adafruit INA219
   - ESP32Servo
5. Open `firmware/SortSure_esp32/SortSure_esp32.ino`.
6. Review the pin assignments and safety interlocks.
7. Compile and test in a controlled setup before connecting actuator power.

### Raspberry Pi Vision Node

The Python script is designed for Raspberry Pi OS with a compatible CSI camera.

Install the required system packages:

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-serial python3-pil python3-numpy
```

Install a TensorFlow Lite-compatible runtime supported by your operating system and Python version. Depending on compatibility, this may be `tflite-runtime` or `ai-edge-litert`.

From the repository root, start the vision node using:

```bash
python3 pi/pi_vision.py run
```

To collect labelled images, run:

```bash
python3 pi/pi_vision.py collect --label PET
```

Replace `PET` with another supported label when collecting a different category. The collection mode saves images under `pi/dataset/<LABEL>/`.

### Classification Model

For inference, place the following files next to `pi_vision.py`:

- `model.tflite` — a trained and compatible model.
- `labels.txt` — one class label per line, in the model's output order, using `ORG`, `PET`, `CAN`, `PAP`, and `OTH`.

**A trained model is not currently included.** Without a usable model, the vision node defaults to the unknown/residual class. The classifier must be trained and validated before real-world classification performance can be claimed.

---

## 9. Recommended Hardware Testing Sequence

1. Power the controller with actuator power disabled and check sensor responses.
2. Tare the load cell with an empty tray and calibrate it using a known weight.
3. Zero the height sensor with an empty tray.
4. Test stepper direction and position with the chute disconnected.
5. Verify that each index sensor detects its intended outlet.
6. Calibrate the trapdoor's open and closed positions.
7. Test a complete cycle using a controlled serial response.
8. Test communication timeouts, sensor contradictions, blocked intake, failed-drop recovery, and lockout behaviour.
9. Integrate the Raspberry Pi communication and camera workflow.
10. Add a trained classification model only after the control and safety behaviour has been verified.

The firmware's maintenance commands described in the project notes include `STATUS`, `TARE`, `CAL,<g>`, `ZERO`, and `R` for resetting from lockout.

---

## 10. Safety Considerations

- An item must not be released until chute alignment is confirmed.
- A blocked intake or open cover should inhibit actuator movement.
- Failed drop detection should trigger the defined recovery attempt and then lockout.
- Uncertain or contradictory classification should default to residual waste.
- The prototype is not intended for crushing or compaction.
- Keep hands clear of moving parts and disconnect power before changing wiring.

**This is a prototype safety concept, not a certified functional-safety system.** The hardware interlock, emergency-stop behaviour, power design, and mechanical safeguards require review and physical validation before unattended operation.

---

## 11. Known Limitations

- The firmware and Raspberry Pi node have not yet been validated on the physical system.
- The trained classification model and model-validation dataset are not yet available.
- Confidence thresholds, sensor limits, step counts, and current limits are placeholders requiring measurement-based tuning.
- Image-quality thresholds need calibration for the actual enclosure and lighting.
- Current sensing alone may not reliably detect a stalled stepper motor.
- A hardware watchdog and complete dashboard-forwarding service are not yet implemented.
- The inductive sensor has limited range and must be mounted appropriately close to the target material.
- Closed containers, free liquids, biomedical waste, e-waste, and hazardous items are outside the current intended scope.

---

## 12. Validation Plan

Before claiming reliable sorting performance:

1. Collect representative images of clean, dirty, crushed, transparent, occluded, composite, and unknown items.
2. Split training and test sets by physical object rather than only by image.
3. Report a confusion matrix, per-class precision and recall, unknown-rejection rate, and false-accept rate into recycling.
4. Measure recyclable-stream purity, successful drops, jams per 100 cycles, median cycle time, inference latency, and energy per cycle.
5. Document test conditions, failures, corrective changes, and remaining risks.

---
