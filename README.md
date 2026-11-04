# Flight Data Telemetry Node

Telemetry system for a small airplane: an **ESP32-S3** reads sensors, packs the data into a binary packet, and sends it to a **Python ground station** that shows it live on a web dashboard.

The firmware is simulated in [Wokwi](https://wokwi.com), so no hardware is needed to try it.

## Features

- **MPU6050**: roll, pitch and yaw, using a complementary filter and gyro calibration at start-up
- **BMP180**: pressure, temperature and barometric altitude
- **GPS**: currently simulated (NMEA sentences read with TinyGPSPlus). It will be replaced by a real GPS module.
- **Binary packet** (50 bytes) with sync bytes, protocol version, status bits and CRC32
- **Python ground station** with a live web dashboard (`http://localhost:8000`)
- Missing sensors are reported in the status byte and their values are sent as NaN

## Project structure

| Path | Description |
|------|-------------|
| src/sketch.ino | Main firmware: sensors, attitude filter, packet sending |
| src/telemetry_packet.h/.cpp | Packet structure and CRC32 |
| src/fake_gps.h | Simulated GPS module |
| src/dashboard.html | Web dashboard |
| gcs.py | Ground station: receiver and flight simulator |
| diagram.json, wokwi.toml | Wokwi simulation setup |
| platformio.ini | PlatformIO configuration |

## Packet format

sync (0xAA 0x55) | version | status | timestamp | packet_id | latitude | longitude | gps_altitude | bmp_altitude | pressure | temperature | pitch | roll | yaw | crc32

Total size: 50 bytes, sent 20 times per second.

## How to run

Requirements: Python 3 and [PlatformIO](https://platformio.org).

**Demo mode** (no Wokwi needed, fake data made inside the script):

python gcs.py --demo


**Sensors mode** (runs the firmware in Wokwi and reads its real packets):

pip install wokwi-client
pio run
python gcs.py


This mode needs a Wokwi token in the WOKWI_CLI_TOKEN environment variable.

Then open `http://localhost:8000` in your browser.

## Roadmap

- [ ] Replace the simulated GPS with a real GPS module
- [ ] Add a radio link between the plane and the ground station

## Status

Work in progress. I am building this step by step as a learning project.
