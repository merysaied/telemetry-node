"""
gcs.py - Ground Control Station: receiver + flight simulator in ONE file.

Put this file in the project root (next to diagram.json and platformio.ini).

Two modes:
  python gcs.py           SENSORS: start Wokwi in the cloud, drive the virtual MPU6050 and
                          BMP180 with a flight model (data keeps changing), read the packets
                          the ESP32 sends, show them on the dashboard.
  python gcs.py --demo    DEMO: fake data made inside this script, no Wokwi needed.

Both modes serve the dashboard on http://localhost:8000 (JSON at /data).

Extra options:
  --http-port N       dashboard port (default 8000)
  --no-browser        do not open the browser automatically
  --minutes M         (sensors mode) simulated minutes to run, 0 = until Ctrl+C (default 5)
  --step S            (sensors mode) simulated seconds between sensor updates

Sensors mode needs:  pip install wokwi-client   and   set WOKWI_CLI_TOKEN

GPS: latitude, longitude and GPS altitude come from the ESP32 itself (fake GPS module in
the firmware: fake_gps.h + TinyGPSPlus). This script only shows what the packets contain.
"""
import argparse
import asyncio
import json
import math
import os
import struct
import threading
import time
import webbrowser
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

# =============================================================================
# 1. SETTINGS
# =============================================================================
ROOT = Path(__file__).resolve().parent
BUILD = ROOT / ".pio" / "build" / "esp32-s3-devkitc-1"
DIAGRAM = ROOT / "diagram.json"
FIRMWARE = BUILD / "firmware.bin"
ELF = BUILD / "firmware.elf"
DASHBOARD_CANDIDATES = [ROOT / "src" / "dashboard.html", ROOT / "dashboard.html"]

HTTP_PORT = 8000

# Packet layout. MUST match TelemetryPacket in telemetry_packet.h:
#   B sync1, B sync2, B version, B status, I timestamp, H packet_id, 9 floats, I crc32  -> 50 bytes
# The "<" means little-endian with NO padding (same as #pragma pack(1) on the ESP32).
PACKET_FORMAT = "<B B B B I H 9f I"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
SYNC = b"\xaa\x55"
PROTOCOL_VERSION = 1      # must match PROTOCOL_VERSION in telemetry_packet.h

# Positions of two fields inside a decoded packet tuple (same order as PACKET_FORMAT).
IDX_VERSION = 2
IDX_PACKET_ID = 5

# =============================================================================
# 2. SHARED STATE (the dashboard reads this through /data)
# =============================================================================
lock = threading.Lock()      # several threads touch `state`, so every access uses this lock
last_update = [0.0]          # time of the last good packet (list so functions can change it)
state = {
    "connected": False, "demo": False, "good": 0, "bad": 0, "lost": 0,
    "version": 0, "status": 0, "timestamp": 0, "packet_id": 0, "latitude": 0.0, "longitude": 0.0,
    "gps_altitude": 0.0, "bmp_altitude": 0.0, "pressure": 0.0,
    "temperature": 0.0, "pitch": 0.0, "roll": 0.0, "yaw": 0.0,
}


def clean(value):
    """JSON cannot hold NaN/Infinity, so replace them with 0."""
    if isinstance(value, float) and not math.isfinite(value):
        return 0.0
    return value


# =============================================================================
# 3. PACKET PARSING
# =============================================================================
class PacketReader:
    """Rebuilds packets from arbitrary chunks of serial bytes.

    How it works: keep a buffer, look for the sync bytes AA 55, take PACKET_SIZE bytes
    from there and check the CRC. If the CRC is wrong, throw away only ONE byte
    and search again - so one corrupted packet never breaks the next ones.
    """

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0            # number of CRC failures so far

    def feed(self, data):
        self.buf += data
        packets = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                del self.buf[:-1]               # keep a possible lone 0xAA at the end
                break
            if len(self.buf) - i < PACKET_SIZE:
                del self.buf[:i]                # wait for more bytes
                break
            raw = bytes(self.buf[i:i + PACKET_SIZE])
            received = struct.unpack_from("<I", raw, PACKET_SIZE - 4)[0]
            if (zlib.crc32(raw[:PACKET_SIZE - 4]) & 0xFFFFFFFF) == received:
                packets.append(struct.unpack(PACKET_FORMAT, raw))
                del self.buf[:i + PACKET_SIZE]
            else:
                self.bad += 1
                del self.buf[:i + 1]
        return packets


class LinkStats:
    """Counts lost packets using packet_id (a 16-bit counter that wraps at 65535)."""

    def __init__(self):
        self.last_id = None

    def lost_since_last(self, packet_id):
        lost = 0
        if self.last_id is not None:
            gap = (packet_id - self.last_id) & 0xFFFF   # & 0xFFFF handles the wrap-around
            if gap > 1:
                lost = gap - 1
        self.last_id = packet_id
        return lost


def store_packet(p, lost):
    """Copy one decoded packet into the shared state."""
    (_s1, _s2, version, status, timestamp, packet_id, latitude, longitude, gps_altitude,
     bmp_altitude, pressure, temperature, pitch, roll, yaw, _crc) = p
    with lock:
        state.update(
            version=version, status=status,
            timestamp=timestamp, packet_id=packet_id,
            latitude=latitude, longitude=longitude,
            gps_altitude=gps_altitude, bmp_altitude=bmp_altitude,
            pressure=pressure, temperature=temperature,
            pitch=pitch, roll=roll, yaw=yaw,
        )
        state["good"] += 1
        state["lost"] += lost
    last_update[0] = time.time()


def set_bad(count):
    with lock:
        state["bad"] = count


# =============================================================================
# 4. DASHBOARD HTTP SERVER
# =============================================================================
class Handler(BaseHTTPRequestHandler):
    def send_body(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/data":
            with lock:
                payload = {k: clean(v) for k, v in state.items()}
            payload["age"] = (time.time() - last_update[0]) if last_update[0] else None
            self.send_body(200, "application/json", json.dumps(payload).encode())
        elif path in ("/", "/index.html"):
            for candidate in DASHBOARD_CANDIDATES:
                if candidate.exists():
                    self.send_body(200, "text/html; charset=utf-8", candidate.read_bytes())
                    return
            self.send_body(404, "text/plain",
                           b"dashboard.html not found (looked in src/ and the project root)")
        else:
            self.send_body(404, "text/plain", b"Not found")

    def log_message(self, *args):
        pass        # keep the console quiet


def start_http(port, open_browser):
    try:
        server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    except OSError as e:
        raise SystemExit(f"Cannot start web server on port {port}: {e}\n"
                         f"Try: python gcs.py --http-port 8001")
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = f"http://localhost:{port}"
    print(f"Dashboard: {url}")
    if open_browser:
        webbrowser.open(url)
    return server


# =============================================================================
# 5. MODE: DEMO (fake data made inside this script)
# =============================================================================
def demo_worker():
    start = time.time()
    packet_id = 0
    with lock:
        state["connected"] = True
        state["demo"] = True
    while True:
        t = time.time() - start
        packet_id = (packet_id + 1) & 0xFFFF
        alt = 120 + 80 * math.sin(t / 10)
        with lock:
            state.update(
                version=PROTOCOL_VERSION, status=0x07,      # 0x07 = MPU + BMP + GPS all OK
                timestamp=int(t * 1000), packet_id=packet_id,
                latitude=50.1109 + 0.01 * math.sin(t / 20),
                longitude=8.6821 + 0.015 * math.cos(t / 20),
                gps_altitude=alt + 2, bmp_altitude=alt,
                pressure=1013.25 * (1 - alt / 44330.0) ** 5.255,
                temperature=22.5 + math.sin(t / 30),
                roll=30 * math.sin(t / 3), pitch=15 * math.sin(t / 4.5),
                yaw=(t * 12) % 360,
            )
            state["good"] += 1
        last_update[0] = time.time()
        time.sleep(0.1)


# =============================================================================
# 6. MODE: SENSORS (Wokwi cloud + flight model)
# =============================================================================
# ----- flight model parameters (change freely) -----
SPEED = 60.0                                          # m/s (keep equal to SIM_SPEED_MPS in sketch.ino)
ALT_MID, ALT_AMP, ALT_PERIOD = 350.0, 250.0, 120.0    # altitude 100..600 m, 120 s cycle
ROLL_MID, ROLL_AMP, ROLL_PERIOD = 15.0, 8.0, 17.0     # bank 7..23 deg
G = 9.81
BOOT_SECONDS = 4.0      # let the firmware boot before touching the sensors


def flight_model(t):
    """Return the aircraft state at simulation time t (seconds)."""
    w = 2.0 * math.pi / ALT_PERIOD
    alt = ALT_MID - ALT_AMP * math.cos(w * t)
    climb = ALT_AMP * w * math.sin(w * t)                       # vertical speed, m/s
    roll = ROLL_MID + ROLL_AMP * math.sin(2.0 * math.pi * t / ROLL_PERIOD)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, climb / SPEED))))
    turn_dps = math.degrees(G / SPEED * math.tan(math.radians(roll)))   # coordinated turn

    # How fast roll and pitch change (deg/s) = what the gyro X and Y axes would measure.
    # The firmware filter integrates these and uses the accelerometer to correct them.
    roll_rate = ROLL_AMP * (2.0 * math.pi / ROLL_PERIOD) * math.cos(2.0 * math.pi * t / ROLL_PERIOD)
    climb_rate = ALT_AMP * w * w * math.cos(w * t)                      # d(climb)/dt
    ratio = max(-0.99, min(0.99, climb / SPEED))
    pitch_rate = math.degrees(climb_rate / SPEED / math.sqrt(1.0 - ratio * ratio))
    return {"alt": alt, "roll": roll, "pitch": pitch, "turn_dps": turn_dps,
            "roll_rate": roll_rate, "pitch_rate": pitch_rate}


def accel_from_attitude(roll_deg, pitch_deg):
    """Gravity components (in g) the MPU6050 would measure for this attitude.
    It is the inverse of the firmware formulas:
        roll  = atan2(ay, az)
        pitch = atan2(-ax, sqrt(ay^2 + az^2))
    (Simplification: the extra force felt during a turn is ignored.)"""
    r, p = math.radians(roll_deg), math.radians(pitch_deg)
    return -math.sin(p), math.cos(p) * math.sin(r), math.cos(p) * math.cos(r)


def pressure_pa(alt_m):
    """Standard atmosphere; the firmware inverts this formula to get altitude."""
    pa = 101325.0 * (1.0 - alt_m / 44330.0) ** 5.255
    return max(30000.0, min(110000.0, pa))          # BMP180 working range


def temperature_c(alt_m):
    return max(-40.0, min(85.0, 22.0 - 0.0065 * alt_m))


async def rpc(label, coro, timeout=30.0):
    """Await a Wokwi call, but fail with a clear message instead of hanging silently."""
    try:
        return await asyncio.wait_for(coro, timeout)
    except asyncio.TimeoutError:
        raise RuntimeError(f"Timed out after {timeout:.0f}s while waiting for: {label}")


async def run_sim(args):
    from wokwi_client import GET_TOKEN_URL, WokwiClient     # only needed in --sim mode

    token = os.getenv("WOKWI_CLI_TOKEN")
    if not token:
        raise SystemExit(f"Set WOKWI_CLI_TOKEN first. Get it from {GET_TOKEN_URL}")
    for path in (DIAGRAM, FIRMWARE, ELF):
        if not path.exists():
            raise SystemExit(f"Missing file: {path}")

    reader = PacketReader()
    stats = LinkStats()

    version_warned = [False]

    def on_serial(data):
        """Called by the Wokwi client every time bytes arrive from the ESP32."""
        for p in reader.feed(bytes(data)):
            if p[IDX_VERSION] != PROTOCOL_VERSION:           # layout we do not understand
                if not version_warned[0]:
                    print(f"WARNING: packet version {p[IDX_VERSION]} (expected {PROTOCOL_VERSION}); "
                          f"rebuild the firmware with pio run. Ignoring these packets.")
                    version_warned[0] = True
                continue
            store_packet(p, stats.lost_since_last(p[IDX_PACKET_ID]))
        set_bad(reader.bad)

    client = WokwiClient(token)
    serial_task = None
    try:
        hello = await rpc("connect", client.connect())
        print("Connected, server version:", hello["version"])
        await rpc("upload diagram.json", client.upload_file("diagram.json", DIAGRAM))
        await rpc("upload firmware.bin", client.upload_file("firmware.bin", FIRMWARE))
        await rpc("upload firmware.elf", client.upload_file("firmware.elf", ELF))
        await rpc("start_simulation",
                  client.start_simulation(firmware="firmware.bin", elf="firmware.elf"))
        print("Simulation started.")
        serial_task = client.serial_monitor(on_serial)

        print(f"Waiting for the firmware to boot (simulation time {BOOT_SECONDS:g} s)...")
        await rpc("boot wait", client.wait_until_simulation_time(BOOT_SECONDS), timeout=120.0)
        with lock:
            state["connected"] = True
            got = state["good"]
        print(f"Boot finished: {got} valid packet(s) received so far.")
        if got == 0:
            print("WARNING: no valid packets yet. Check that firmware.bin was built from the "
                  "latest sketch.ino, and that diagram.json contains the parts 'mpu' and 'bmp'.")

        limit = args.minutes * 60.0
        if limit:
            print(f"Running {args.minutes:g} simulated minute(s). Ctrl+C to stop earlier.")
        else:
            print("Running until Ctrl+C.")

        sim_t = BOOT_SECONDS          # simulation clock; model time starts at 0 after boot
        k = 0
        warned = False
        next_report = 0.0
        while not limit or (sim_t - BOOT_SECONDS) <= limit:
            t = sim_t - BOOT_SECONDS
            m = flight_model(t)
            ax, ay, az = accel_from_attitude(m["roll"], m["pitch"])
            controls = [
                ("mpu", "accelX", ax), ("mpu", "accelY", ay), ("mpu", "accelZ", az),
                ("mpu", "rotationX", m["roll_rate"]), ("mpu", "rotationY", m["pitch_rate"]),
                ("mpu", "rotationZ", m["turn_dps"]),
                ("bmp", "pressure", pressure_pa(m["alt"])),
            ]
            if k % 4 == 0:
                controls.append(("bmp", "temperature", temperature_c(m["alt"])))
            for part, name, value in controls:
                try:
                    await rpc(f"set_control {part}.{name}",
                              client.set_control(part, name, round(value, 4)), timeout=30.0)
                except RuntimeError:
                    raise
                except Exception as e:
                    if not warned:
                        print(f"set_control {part}.{name} failed: {e!r}")
                        warned = True
            if k == 0:
                print("First sensor update sent. Dashboard should be live now.")

            if t >= next_report:
                print(f"t={t:5.0f}s  alt={m['alt']:.0f} m  roll={m['roll']:.1f}  "
                      f"pitch={m['pitch']:.1f}  packets={state['good']}  bad={state['bad']}")
                next_report += 10.0
            k += 1
            sim_t += args.step
            await rpc("wait_until_simulation_time",
                      client.wait_until_simulation_time(sim_t), timeout=120.0)
        print("Finished.")
    finally:
        with lock:
            state["connected"] = False
        if serial_task is not None:
            serial_task.cancel()
        try:
            await client.disconnect()
        except Exception:
            pass


# =============================================================================
# 7. MAIN
# =============================================================================
def main():
    ap = argparse.ArgumentParser(description="Telemetry ground station (sensors / demo)")
    ap.add_argument("--demo", action="store_true", help="fake data, no Wokwi needed")
    ap.add_argument("--http-port", type=int, default=HTTP_PORT)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--minutes", type=float, default=5.0,
                    help="(sensors mode) simulated minutes to run, 0 = until Ctrl+C (default 5)")
    ap.add_argument("--step", type=float, default=0.5,
                    help="(sensors mode) seconds of simulation time between sensor updates")
    args = ap.parse_args()

    start_http(args.http_port, not args.no_browser)

    try:
        if args.demo:
            print("[GCS] Demo data")
            threading.Thread(target=demo_worker, daemon=True).start()
            while True:                 # keep the main thread alive until Ctrl+C
                time.sleep(1)
        else:
            asyncio.run(run_sim(args))
    except KeyboardInterrupt:
        print("\n[GCS] Stopped by user.")
    except RuntimeError as e:
        print(f"\nERROR: {e}")


if __name__ == "__main__":
    main()