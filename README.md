# Nimbus-1 — ESP32 Smart Weather Balloon Telemetry System

A high-altitude weather balloon payload built on ESP32 that streams real-time
telemetry to a web dashboard served directly from the board.

## What It Does

- Reads **pressure, temperature** (BMP085/BMP180), **GPS position** (TinyGPS++),
  and **6-axis motion** (MPU6050 over I²C)
- Hosts a **self-contained web dashboard** on the ESP32 — no internet required
- Serves live sensor data as **JSON** over a REST endpoint
- Dashboard shows charts, a map, and live gauges using Chart.js + Leaflet

## Repository Structure

```
├── firmware/
│   ├── WeatherBalloon/              # Main firmware — BMP085 + GPS + MPU6050 + WebServer
│   ├── WeatherBalloon_async/        # AsyncWebServer variant (BMP180 + MPU6050)
│   ├── WeatherBalloon_early/        # Early prototype (AsyncWebServer, simpler)
│   ├── SmartBalloon_filtered/       # Noise-filtered MPU6050 + JSON API backend
│   ├── SmartBalloon_filtered_full/  # Full version with per-sensor noise filtering
│   ├── SmartBalloon_mpu/            # MPU6050 motion-tracking focused
│   ├── SmartBalloon_mpu_updates/    # MPU6050 with startup calibration
│   ├── SmartBalloon_npm/            # NPM sensor variant
│   └── SmartBalloon_test/           # Test build with noise filtering
├── dashboard/
│   ├── index.html                   # Main telemetry dashboard (Tailwind + Chart.js + Leaflet)
│   └── web_demo.html                # Standalone web demo
└── README.md
```

## Hardware

| Component       | Purpose                |
|-----------------|------------------------|
| ESP32           | Main controller + WiFi |
| BMP085 / BMP180 | Barometric pressure    |
| NEO-6M / similar| GPS                    |
| MPU6050         | Accelerometer + gyro   |

## Quick Start

1. Open any `firmware/<variant>/<file>.ino` in Arduino IDE
2. Install required libraries (listed at top of each .ino)
3. Set your WiFi credentials in the sketch
4. Upload to ESP32
5. Open the serial monitor for the AP IP, navigate to it in your browser
6. The dashboard loads directly from the board

## Firmware Variants

| Variant | Server | Sensors | Notes |
|---------|--------|---------|-------|
| WeatherBalloon | WebServer | BMP085 + GPS + MPU6050 | Main build, embedded HTML dashboard |
| WeatherBalloon_async | AsyncWebServer | BMP180 + MPU6050 | Async web server variant |
| WeatherBalloon_early | AsyncWebServer | BMP180 + MPU6050 | Early prototype |
| SmartBalloon_filtered | WebServer | MPU6050 | Noise-filtered, JSON API + CORS |
| SmartBalloon_filtered_full | WebServer | MPU6050 | Full per-sensor noise filtering |
| SmartBalloon_mpu | WebServer | MPU6050 | Motion-tracking focused |
| SmartBalloon_mpu_updates | WebServer | MPU6050 | With startup calibration |
| SmartBalloon_npm | WebServer | NPM sensors | NPM measurement variant |
| SmartBalloon_test | WebServer | MPU6050 | Test build with noise filtering |

## License

MIT
