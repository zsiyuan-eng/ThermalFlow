# ThermalFlow Architecture

ThermalFlow is split into three layers: sensing firmware, host visualization,
and project presentation.

## 1. Sensing Firmware

The ESP32 reads the MLX90640 over I2C and can operate in three modes:

- `edge_counter` performs embedded thresholding, connected-component detection,
  center extraction, nearest-neighbor tracking, and entry/exit decisions.
- `serial_stream` focuses on raw frame transmission over USB serial.
- `wifi_stream` exposes an ESP32 access point and streams frames over TCP.

## 2. Host Dashboard

The Python dashboard parses either serial or TCP frames and renders the 32x24
thermal matrix as a live heatmap. Tracking overlays, event counters, frame
statistics, and connection status are displayed for demos and debugging.

## 3. Website Layer

The root `index.html`, `styles.css`, and `script.js` form a static GitHub Pages
site. The animated canvas is a front-end simulation of the project output, so the
website works without hardware attached.

## Data Flow

```text
MLX90640
  -> ESP32 I2C frame acquisition
  -> thermal threshold / frame serialization
  -> USB serial or WiFi TCP
  -> Python dashboard
  -> heatmap, tracks, entry/exit counts
```

## Why Thermal Sensing

The system uses low-resolution infrared data instead of camera images. That
makes the demo privacy-friendly while still showing a meaningful sensing,
tracking, and counting pipeline.
