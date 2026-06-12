# ThermalFlow Architecture

Two main parts: ESP32 firmware that does sensing and tracking, and a Python dashboard on the host side.

## 1. Sensing Firmware

The ESP32 reads the MLX90640 over I2C at up to 16 Hz. Three firmware modes:

- `edge_counter` — does the full pipeline on-device: temperature threshold, connected-component detection, center extraction, nearest-neighbor tracking across frames, and entry/exit decisions. Outputs counts + centers over serial.
- `serial_stream` — skips all that, just dumps raw frames as fast as possible over USB serial at 460800 baud. The host handles everything.
- `wifi_stream` — same raw-frame dump, but over TCP. ESP32 creates its own access point so no router needed.

## 2. Host Dashboard

Python reads either serial or TCP frames, parses the 32×24 temperature matrix, runs connected-component analysis and tracking if needed, and renders a live heatmap. The dashboard shows the thermal field, tracking circles with IDs, entry/exit counters, and FPS.

Four scripts covering different use cases:

- `thermal_dashboard_fast.py` — optimized for live demos, minimal latency
- `thermal_dashboard_full.py` — more panels and debug info
- `thermal_dashboard_wifi.py` — TCP client for the WiFi firmware
- `thermal_counter_cli.py` — terminal only, no GUI, good for headless testing

## Data Flow

```
MLX90640
  -> ESP32 I2C frame read
  -> temperature thresholding + serialization (or edge counting)
  -> USB serial or WiFi TCP
  -> Python parser
  -> heatmap render, tracking overlays, entry/exit counts
```

## Why thermal and not camera

Low-resolution IR means no facial features, no identifying information. You can tell there's a person-shaped heat blob crossing a line, but that's it. Makes the demo safe to run anywhere without consent issues.
