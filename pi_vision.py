#!/usr/bin/env python3
"""SortSure - Raspberry Pi vision node.

Two modes:
  python3 pi_vision.py run                 # serve the ESP32: answer REQ lines with RES,<CLASS>,<conf>
  python3 pi_vision.py collect --label PET # build your dataset: press Enter to save an image to dataset/PET/

Wiring (3.3 V logic both sides, COMMON GROUND between Pi and ESP32):
  Pi GPIO14 TXD (pin 8)  -> ESP32 GPIO16 (RX2)
  Pi GPIO15 RXD (pin 10) <- ESP32 GPIO17 (TX2)
  Pi GND (pin 6/9/14)    -> ESP32 GND
  Camera on the CSI ribbon. Power the Pi from its own 5 V / 3 A supply (USB-C), not from the ESP32.

Setup (Raspberry Pi OS):
  sudo raspi-config  -> Interface Options -> Serial Port: login shell = NO, serial hardware = YES
  sudo apt install -y python3-picamera2 python3-serial python3-pil python3-numpy
  pip install tflite-runtime --break-system-packages     # or: pip install ai-edge-litert (newer Python)

Files next to this script after training:  model.tflite   labels.txt (one class per line, in model output order)
Class names must be exactly: ORG PET CAN PAP OTH

SAFE DEFAULT: if the model is missing, the image is bad, or anything fails, the reply is
RES,OTH,0.00 and the ESP32 sends the item to residual. The Pi can never move an actuator.
"""
import argparse
import csv
import os
import sys
import time

import numpy as np
import serial
from PIL import Image

PORT, BAUD = "/dev/serial0", 115200
HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(HERE, "model.tflite")
LABELS_PATH = os.path.join(HERE, "labels.txt")
LOG_DIR = os.path.join(HERE, "log")
DATASET_DIR = os.path.join(HERE, "dataset")
VALID = {"ORG", "PET", "CAN", "PAP", "OTH"}

# image-quality gates (tune on your own lighting); failing = "camera dirty / dark" -> residual
MIN_BRIGHTNESS, MAX_BRIGHTNESS, MIN_SHARPNESS = 40, 220, 30.0


def open_camera():
    from picamera2 import Picamera2
    cam = Picamera2()
    cam.configure(cam.create_still_configuration(main={"size": (640, 480), "format": "RGB888"}))
    cam.start()
    time.sleep(1.0)
    return cam


def grab(cam):
    """Return an RGB uint8 array. picamera2 'RGB888' is stored B,G,R, so flip channels.
    Use this same function for collecting AND inference so training/inference match."""
    frame = cam.capture_array()
    return np.ascontiguousarray(frame[:, :, ::-1])


def quality(rgb):
    """(ok, brightness, sharpness). Sharpness = variance of a Laplacian computed in numpy."""
    g = rgb.astype(np.float32).mean(axis=2)
    bright = float(g.mean())
    lap = 4 * g[1:-1, 1:-1] - g[:-2, 1:-1] - g[2:, 1:-1] - g[1:-1, :-2] - g[1:-1, 2:]
    sharp = float(lap.var())
    return (MIN_BRIGHTNESS <= bright <= MAX_BRIGHTNESS and sharp >= MIN_SHARPNESS), bright, sharp


class Classifier:
    def __init__(self):
        self.ready = False
        try:
            try:
                from tflite_runtime.interpreter import Interpreter
            except ImportError:
                from ai_edge_litert.interpreter import Interpreter
            self.interp = Interpreter(model_path=MODEL_PATH)
            self.interp.allocate_tensors()
            self.inp = self.interp.get_input_details()[0]
            self.out = self.interp.get_output_details()[0]
            with open(LABELS_PATH) as f:
                self.labels = [l.strip() for l in f if l.strip()]
            bad = [l for l in self.labels if l not in VALID]
            if bad:
                raise ValueError(f"labels.txt has unknown classes: {bad}")
            self.ready = True
            print(f"model loaded, classes={self.labels}", flush=True)
        except Exception as e:
            print(f"MODEL NOT AVAILABLE ({e}); every item will go to residual", flush=True)

    def predict(self, rgb):
        h, w = self.inp["shape"][1], self.inp["shape"][2]
        img = np.asarray(Image.fromarray(rgb).resize((w, h)))
        if self.inp["dtype"] == np.uint8:
            x = img.astype(np.uint8)
        else:
            x = (img.astype(np.float32) / 255.0)          # match your training preprocessing!
        self.interp.set_tensor(self.inp["index"], x[None, ...])
        self.interp.invoke()
        y = self.interp.get_tensor(self.out["index"])[0].astype(np.float32)
        if self.out["dtype"] == np.uint8:                   # dequantize
            scale, zero = self.out["quantization"]
            y = (y - zero) * scale
        if y.min() < 0 or y.sum() < 0.99 or y.sum() > 1.01:  # not probabilities -> softmax
            e = np.exp(y - y.max())
            y = e / e.sum()
        k = int(np.argmax(y))
        return self.labels[k], float(y[k])


def parse_req(line):
    parts = dict(p.split("=", 1) for p in line.split(",")[1:] if "=" in p)
    return float(parts.get("mass_g", 0)), int(parts.get("metal", 0)), float(parts.get("height_mm", 0))


def run():
    os.makedirs(LOG_DIR, exist_ok=True)
    cam = open_camera()
    clf = Classifier()
    ser = serial.Serial(PORT, BAUD, timeout=1)
    log_path = os.path.join(LOG_DIR, "decisions.csv")
    new = not os.path.exists(log_path)
    logf = open(log_path, "a", newline="")
    log = csv.writer(logf)
    if new:
        log.writerow(["time", "mass_g", "metal", "height_mm", "brightness", "sharpness", "class", "conf", "image"])
    print("vision node ready", flush=True)

    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line.startswith("REQ,"):
            continue                                        # DASH lines etc: forward to dashboard here if wanted
        cls, conf, img_name, bright, sharp = "OTH", 0.0, "", 0.0, 0.0
        mass = metal = height = 0
        try:
            mass, metal, height = parse_req(line)
            rgb = grab(cam)
            ok, bright, sharp = quality(rgb)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            img_name = f"{stamp}.jpg"
            Image.fromarray(rgb).save(os.path.join(LOG_DIR, img_name), quality=90)   # keep for validation
            if not ok:
                print(f"bad image (brightness={bright:.0f} sharpness={sharp:.0f}) -> OTH", flush=True)
            elif clf.ready:
                cls, conf = clf.predict(rgb)
        except Exception as e:                              # any failure -> residual
            print(f"error: {e}", flush=True)
            cls, conf = "OTH", 0.0
        ser.write(f"RES,{cls},{conf:.2f}\n".encode())
        log.writerow([time.strftime("%H:%M:%S"), mass, metal, height, f"{bright:.0f}", f"{sharp:.0f}", cls, f"{conf:.2f}", img_name])
        logf.flush()
        print(f"{line} -> {cls} {conf:.2f}", flush=True)


def collect(label):
    label = label.upper()
    if label not in VALID:
        sys.exit(f"label must be one of {sorted(VALID)}")
    out = os.path.join(DATASET_DIR, label)
    os.makedirs(out, exist_ok=True)
    cam = open_camera()
    n = len(os.listdir(out))
    print(f"Collecting '{label}' into {out}. Vary orientation, dirt, crushing, lighting. Enter = capture, q = quit.")
    print("Ring light must be ON during collection (it is only on during a cycle in the firmware).")
    while True:
        if input(f"[{n}] ").strip().lower() == "q":
            break
        rgb = grab(cam)
        ok, bright, sharp = quality(rgb)
        Image.fromarray(rgb).save(os.path.join(out, f"{time.strftime('%Y%m%d_%H%M%S')}_{n}.jpg"), quality=92)
        n += 1
        if not ok:
            print(f"  warning: brightness={bright:.0f} sharpness={sharp:.0f} looks off")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="mode", required=True)
    sub.add_parser("run")
    c = sub.add_parser("collect")
    c.add_argument("--label", required=True)
    a = ap.parse_args()
    run() if a.mode == "run" else collect(a.label)
