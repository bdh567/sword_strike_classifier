# Sword Strike Classifier

Real-time gesture classification of sword strike types using IMU data, deployed on an Arduino Nano 33 BLE Sense. Built with Edge Impulse for data pipeline management, CNN-based model training, and TFLite deployment.

---

## Overview

This project implements an end-to-end TinyML pipeline for classifying distinct sword strike motions (e.g., overhead, lateral, diagonal, idle) from inertial measurement unit (IMU) data. The trained model runs entirely on-device via TFLite Micro, with live inference results streamed over BLE to a browser-based web interface — no cloud dependency at inference time.

**Edge Impulse project:** https://studio.edgeimpulse.com/public/1014843/live

---

## Repository Structure
sword_strike_classifier/

├── data_collect/          # Arduino sketch for IMU data logging over BLE Serial

├── live_inference/        # Arduino sketch for on-device inference + BLE output

├── logger.html            # Browser-based data collection interface (paired with data_collect.ino)

├── inference.html         # Browser-based live inference dashboard (paired with sword_inference.ino)

└── README.md

---

## Pipeline

### 1. Data Collection

`data_collect/` contains an Arduino sketch that streams raw 6-axis IMU data (accelerometer + gyroscope) from the Nano 33 BLE Sense over BLE Serial. `logger.html` connects to the board in-browser, lets you label gesture windows in real time, and exports CSV files for upload to Edge Impulse.

### 2. Model Training (Edge Impulse)

The full training pipeline lives in the public Edge Impulse project linked above. Key configuration:

- Input block: fixed-length time-series windows of raw acc/gyro samples
- Processing block: spectral features (FFT-based frequency domain representation)
- Learning block: DNN trained on labeled strike classes
- Data augmentation: noise injection and axis jitter to improve generalization across users and grip styles
- Export: INT8 quantized TFLite model compiled into a C header for deployment

### 3. On-Device Inference

`live_inference/` runs the exported TFLite Micro model directly on the nRF52840 MCU. Inference results are transmitted over BLE and displayed in real time through `inference.html` in the browser — no USB connection required during use.

---

## Hardware

| Component | Detail |
|-----------|--------|
| Board | Arduino Nano 33 BLE Sense |
| MCU | Nordic nRF52840 (ARM Cortex-M4F, 64 MHz) |
| IMU | LSM9DS1 — 3-axis accelerometer + 3-axis gyroscope |
| SRAM | 256 KB |
| Flash | 1 MB |
| Connectivity | Bluetooth Low Energy (BLE) |

---

## Dependencies

**Arduino libraries:**
- Arduino_LSM9DS1
- ArduinoBLE
- Arduino_TensorFlowLite (TFLite Micro)

**Toolchain:**
- Edge Impulse Studio (model training, DSP pipeline)
- Edge Impulse CLI (optional, for local dataset management)
- Arduino IDE 2.x with the Mbed OS Nano board package

---

## Usage

**Data collection:**
1. Flash `data_collect/data_collect.ino` to the Nano 33 BLE Sense
2. Open `logger.html` in a Web Bluetooth-compatible browser (Chrome recommended)
3. Connect to the board, select a gesture label, and record strike windows
4. Export and upload CSV data to Edge Impulse

**Live inference:**
1. Train and export the model from Edge Impulse as an Arduino library
2. Place the exported library in `live_inference/` and flash `sword_inference.ino`
3. Open `inference.html` in browser, connect over BLE
4. Perform strikes — predicted class and confidence appear in real time

---

## Context

Developed for EE 446: TinyML at the University of Washington. Demonstrates the full embedded ML workflow from IMU data collection through DNN training, INT8 quantization, TFLite Micro deployment, and BLE-based inference streaming on a resource-constrained microcontroller.
