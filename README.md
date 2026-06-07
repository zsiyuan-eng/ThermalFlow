# ThermalFlow

Edge thermal intelligence for anonymous people-flow counting.

ThermalFlow is an end-to-end MLX90640 project: ESP32 firmware captures 32x24
thermal frames, embedded logic can segment and track human heat signatures, and
Python dashboards visualize the stream while maintaining entry / exit counts.
The repository also includes a polished GitHub Pages site in the project root.

## Live Site

Open `index.html` locally, or publish this repository with GitHub Pages from the
root directory. No build step is required.


## Core Modules

- `firmware/edge_counter` - ESP32 firmware that performs thresholding,
  connected-component blob detection, center tracking, FPS reporting, and
  entry/exit counting on device.
- `firmware/serial_stream` - high-throughput serial transmitter for raw 32x24
  temperature matrices.
- `firmware/wifi_stream` - ESP32 access-point firmware that streams thermal
  frames over TCP.
- `software/thermal_dashboard` - Python dashboards for serial, WiFi, optimized
  rendering, and terminal-only counting.

## Hardware Stack

- ESP32 development board
- MLX90640 32x24 thermal infrared array
- I2C connection between ESP32 and MLX90640
- USB serial or ESP32 WiFi access point for host communication

## Python Setup

```bash
pip install -r requirements.txt
```

Run the optimized serial dashboard:

```bash
python software/thermal_dashboard/thermal_dashboard_fast.py
```

Run the WiFi dashboard:

```bash
python software/thermal_dashboard/thermal_dashboard_wifi.py
```

## Arduino Setup

Open the sketch you want to flash:

- `firmware/edge_counter/edge_counter.ino`
- `firmware/serial_stream/serial_stream.ino`
- `firmware/wifi_stream/wifi_stream.ino`

Install the Adafruit MLX90640 library in Arduino IDE before compiling.

## Publish

```bash
git init
git add .
git commit -m "Polish ThermalFlow project"
```

Push to GitHub, then enable Pages from the repository root.
"# ThermalFlow" 
