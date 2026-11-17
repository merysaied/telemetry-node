# Flight Data Telemetry Node

Telemetry system for a small airplane: an ESP32-S3 reads sensors, packs the data into a binary packet, and sends it to a Python ground station that shows it live on a web dashboard.

The firmware is simulated in [Wokwi](https://wokwi.com), so no hardware is needed to try it. The ground station can also fly the virtual sensors with the **JSBSim** flight simulator (a Cessna 172 with a simple autopilot), so the data looks like a real flight.

## Features

**Firmware (ESP32-S3)**
- **MPU6050**: roll, pitch and yaw, using a complementary filter and gyro calibration at start-up
- **BMP180**: pressure, temperature and barometric altitude
- **GPS**: currently simulated (`fake_gps.h` builds NMEA sentences that are read with TinyGPSPlus). It behaves like a real receiver: cold start with no fix, slow position drift, changing satellite count and HDOP, and rare short fix losses. It will be replaced by a real GPS module.
- Binary packet (50 bytes) with sync bytes, protocol version, status bits and CRC32
- Missing sensors are reported in the status byte and their values are sent as NaN

**Ground station (Python)**
- Packet parser that recovers from corrupted data (a bad CRC only drops one byte, so the next packets are not affected)
- Link statistics: good, bad (CRC) and lost packets (counted from `packet_id`)
- **Three data sources**, selectable at start-up or live from the dashboard (see below)

**Web dashboard** (`http://localhost:8000`)
- Live values: attitude, pressure, temperature, barometric altitude, GPS position, sensor status
- Attitude, heading and altitude instruments
- Live map with the flight track and a "Follow aircraft" option (the map needs internet access for the OpenStreetMap tiles)
- Buttons to switch between data sources without restarting

## Data sources

| Mode | What runs | Sensors | GPS | Needs |
|------|-----------|---------|-----|-------|
| **Demo** | Only the Python script | Fake, generated in `gcs.py` | Fake, generated in `gcs.py` | Python 3 |
| **Hybrid** (default) | Firmware in Wokwi | Driven by a simple flight model in `gcs.py` | Fake GPS inside the firmware | Wokwi token |
| **Flight Simulator** | Firmware in Wokwi + JSBSim | Driven by JSBSim, with realistic sensor errors | JSBSim position + GPS error, replaces the firmware GPS position | Wokwi token, `jsbsim` |

In Flight Simulator mode the Cessna 172 follows a flight plan (heading and altitude changes, see `PLAN` in `jsbsim_flight.py`). Sensor errors can be switched off with `NOISE = False`. They include gyro bias and noise, accelerometer noise, barometer noise, slow weather pressure changes, and slowly drifting GPS error.

Wokwi cannot move a GPS part, so the firmware still produces its own GPS. The ground station replaces the latitude, longitude and GPS altitude with the JSBSim values that match the packet time.

If a mode cannot start (missing token, missing package, ...), the dashboard shows the reason and you can pick another mode.

## Project structure

| Path | Description |
|------|-------------|
| `src/sketch.ino` | Main firmware: sensors, attitude filter, packet sending |
| `src/telemetry_packet.h/.cpp` | Packet structure and CRC32 |
| `src/fake_gps.h` | Simulated GPS module (NMEA output) |
| `src/dashboard.html` | Web dashboard |
| `gcs.py` | Ground station: packet receiver, HTTP server, mode manager, Demo and Hybrid flight models |
| `jsbsim_flight.py` | JSBSim Cessna 172 with autopilot, sensor errors and GPS errors |
| `JSBout172B.csv` | Data log written by JSBSim (generated output, not needed to run the project) |
| `diagram.json`, `wokwi.toml`, `wokwi-project.txt` | Wokwi simulation setup |
| `platformio.ini` | PlatformIO configuration |

## Packet format

```
sync (0xAA 0x55) | version | status | timestamp | packet_id | latitude | longitude | gps_altitude | bmp_altitude | pressure | temperature | pitch | roll | yaw | crc32
```

Total size: 50 bytes, little-endian, no padding. Sent 20 times per second.

Status bits: `0x01` MPU6050 OK, `0x02` BMP180 OK, `0x04` GPS fix.

## How to run

Requirements: Python 3 and [PlatformIO](https://platformio.org/).

### Demo (no Wokwi needed)

```bash
python gcs.py --demo
```

### Hybrid and Flight Simulator (firmware in Wokwi)

```bash
pip install wokwi-client
pio run                                  # build the firmware
export WOKWI_CLI_TOKEN=<your token>      # Windows PowerShell: $env:WOKWI_CLI_TOKEN="<your token>"
python gcs.py                            # Hybrid mode
```

For Flight Simulator mode, also install JSBSim:

```bash
pip install jsbsim
python gcs.py --mode simulator
```

Get a Wokwi token at <https://wokwi.com/dashboard/ci>.

Then open <http://localhost:8000> (it opens automatically unless you use `--no-browser`). You can change the mode at any time with the buttons at the top of the dashboard.

### Options

| Option | Description |
|--------|-------------|
| `--mode demo\|hybrid\|simulator` | Data source at start-up (default: `hybrid`) |
| `--demo` | Same as `--mode demo` |
| `--http-port N` | Dashboard port (default: 8000) |
| `--no-browser` | Do not open the browser automatically |
| `--minutes M` | Simulated minutes to run in Wokwi modes, 0 = until Ctrl+C (default: 5) |
| `--step S` | Simulated seconds between sensor updates (default: 0.5) |

The raw data is also available as JSON at <http://localhost:8000/data>.

## Roadmap

- [x] Flight simulator (JSBSim) with realistic sensor and GPS errors
- [x] Selectable data sources on the dashboard (Demo, Hybrid, Flight Simulator)
- [x] Live map and flight instruments on the dashboard

## Status

Work in progress. I am building this step by step as a learning project.
