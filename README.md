# robot-dog

A Wi-Fi–controlled quadruped robot running on an ESP32, with an onboard access point and a browser-based control interface — no app or external network required.

![Platform](https://img.shields.io/badge/platform-ESP32-000000?logo=espressif)
![Language](https://img.shields.io/badge/language-C%2B%2B%20(Arduino)-00599C?logo=cplusplus)
![Status](https://img.shields.io/badge/status-working%20prototype-brightgreen)

## Overview

`robot-dog` is a self-contained quadruped controller implemented as a single ESP32 Arduino sketch (`sketch_apr13a.ino`). On boot the ESP32 starts its own Wi-Fi access point and runs an HTTP server, so you connect to the robot directly from a phone or laptop and drive it from a web page — there is no companion app and no router in the loop.

The firmware drives six servos (four legs plus a slider and a rotation joint) through a small gait state machine, and supports per-servo trim calibration that persists in flash so the mechanical zero of each joint can be tuned without re-flashing.

## Features

- **Standalone Wi-Fi access point** — broadcasts its own SSID (`RobotDog-ESP32`); no external network needed.
- **Browser-based control** — an HTTP server on port 80 serves a control page; key presses are sent to a `/cmd?key=` endpoint.
- **Six-servo actuation** — four legs (front-left, front-right, back-left, back-right) plus slider and rotation joints.
- **Gait state machine** — staggered stand-up sequence and a multi-phase walk cycle driven on a non-blocking timer.
- **Smoothed motion** — filtered servo target positions for less jerky movement.
- **Persistent per-servo calibration** — trim offsets (in microseconds) adjustable over Serial and saved to flash via the ESP32 `Preferences` API.

## Tech stack

- **MCU:** ESP32
- **Language:** C++ (Arduino framework)
- **Libraries:** `WiFi`, `WebServer`, `Preferences` (ESP32 Arduino core) and `ESP32Servo`

## How it works

1. The ESP32 boots and brings up a SoftAP (default address `192.168.4.1`).
2. A `WebServer` serves the control UI at `/` and accepts commands at `/cmd?key=<key>`.
3. Incoming commands set a gait action; the main loop advances a timed state machine (stand-up, walk phases) and writes smoothed targets to the six servos.
4. Calibration commands over the Serial monitor adjust each servo's trim (±10 µs steps) and can be saved to / reloaded from flash.

## Getting started

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) (or arduino-cli) with the **ESP32 board package** installed
- The **ESP32Servo** library (Library Manager)
- An ESP32 dev board and six servos wired to the pins defined at the top of the sketch (FL=13, BR=14, FR=25, BL=26, slider=27, rotation=33)

### Flash and run

1. Open `sketch_apr13a.ino` in the Arduino IDE.
2. Select your ESP32 board and port, then upload.
3. On your phone/laptop, join the Wi-Fi network **`RobotDog-ESP32`** (default password is set in the sketch — change it before any real use).
4. Open `http://192.168.4.1/` in a browser and use the on-screen controls.
5. (Optional) Open the Serial monitor to trim servos: `+1`/`-1` … `+6`/`-6` adjust a joint, `p` prints offsets, `s` saves to flash, `d` reloads defaults.

## Status

Working hardware prototype. Core walk gait, stand-up, web control, and calibration are implemented in firmware.

## Roadmap

- TODO: add a bill of materials and wiring diagram
- TODO: document the chassis / 3D-printed parts
- TODO: add turning and additional gaits
- TODO: optional station mode (join an existing Wi-Fi network)

## Hardware & media

- TODO: add photos / a demo GIF of the robot walking
- TODO: add a wiring schematic

## License

TODO: add a license (no license file is currently present in this repo).

## Contact

Bhavya Dosi — [LinkedIn](https://www.linkedin.com/in/bhavya-dosi)
