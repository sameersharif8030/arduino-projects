# Nimbus-1 — ESP32 Smart Weather Balloon Telemetry System

A high-altitude weather balloon payload built on ESP32 that streams real-time
telemetry to a web dashboard served directly from the board.

## What It Does

- Reads **pressure, temperature** (BMP085), **GPS position** (TinyGPS++),
  and **6-axis motion** (MPU6050 over I²C)
- Hosts a **self-contained web dashboard** on the ESP32 — no internet required
- Serves live sensor data as **JSON** over a REST endpoint
- Dashboard shows charts, a map, and live gauges using Chart.js + Leaflet

## Repository Structure

```
arduino-projects/
├── firmware/
│   ├── WeatherBalloon/         # Main firmware (BMP085 + GPS + WebServer)
│   ├── SmartBalloon_filtered/  # Noise-filtered MPU6050 + JSON API backend
│   ├── SmartBalloon_npm/       # NPM sensor variant
│   └── SmartBalloon_mpu/       # MPU6050 motion-tracking variant
├── dashboard/
│   ├── index.html              # Main telemetry dashboard (Tailwind + Chart.js + Leaflet)
│   └── web_demo.html           # Standalone web demo
└── README.md
```

## Hardware

| Component       | Purpose                |
|-----------------|------------------------|
| ESP32           | Main controller + WiFi |
| BMP085          | Barometric pressure    |
| NEO-6M / similar| GPS                    |
| MPU6050         | Accelerometer + gyro   |

## Quick Start

1. Open any `firmware/<variant>/<variant>.ino` in Arduino IDE
2. Install required libraries (listed at top of each .ino)
3. Set your WiFi credentials in the sketch
4. Upload to ESP32
5. Open the serial monitor for the AP IP, navigate to it in your browser
6. The dashboard loads directly from the board

## Variants

- **WeatherBalloon** — Full telemetry with BMP085 + GPS + MPU6050 and
  embedded HTML dashboard
- **SmartBalloon_filtered** — Focused on clean MPU6050 data with a pure
  JSON API backend (CORS-enabled)
- **SmartBalloon_npm** — Sensor variant using NPM measurements
- **SmartBalloon_mpu** — MPU6050 motion-tracking focused build

## License

MIT
