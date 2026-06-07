# Thermal Dashboard

Python host applications for ThermalFlow.

## Applications

- `thermal_dashboard_fast.py` - optimized serial dashboard for live demos.
- `thermal_dashboard_full.py` - full serial dashboard with richer panels.
- `thermal_dashboard_wifi.py` - TCP dashboard for the ESP32 WiFi access point.
- `thermal_counter_cli.py` - terminal-only counter for lightweight testing.

## Install

From the repository root:

```bash
pip install -r requirements.txt
```

## Serial Dashboard

1. Flash `firmware/serial_stream/serial_stream.ino` or
   `firmware/edge_counter/edge_counter.ino`.
2. Update `SERIAL_PORT` in the Python script.
3. Run:

```bash
python software/thermal_dashboard/thermal_dashboard_fast.py
```

## WiFi Dashboard

1. Flash `firmware/wifi_stream/wifi_stream.ino`.
2. Connect your computer to `MLX90640_AP`.
3. Run:

```bash
python software/thermal_dashboard/thermal_dashboard_wifi.py
```

The default ESP32 endpoint is `192.168.4.1:8888`.
