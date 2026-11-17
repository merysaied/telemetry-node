"""
gcs.py - Ground Control Station: receiver + flight simulator in ONE file.

Put this file in the project root (next to diagram.json and platformio.ini).

Three data sources. You can pick one at start-up with --mode, and switch between them
at any time with the buttons on the dashboard:

  demo        ALL the data is fake, made inside this script. No Wokwi, no JSBSim needed.
  hybrid      The ESP32 firmware runs in Wokwi. The virtual MPU6050 and BMP180 are driven by
              a simple flight model, and the GPS is the fake GPS inside the firmware.
  simulator   Flight Simulator = JSBSim needed: the ESP32 firmware runs in Wokwi, and
              its sensors AND its GPS position come from the JSBSim flight simulator.

    python gcs.py                      starts in "hybrid" mode 
    python gcs.py --mode simulator      starts in "simulator" mode (JSBSim needed)
    python gcs.py --demo               same as --mode demo

The dashboard is on http://localhost:8000 (JSON at /data).

Extra options:
  --http-port N       dashboard port (default 8000)
  --no-browser        do not open the browser automatically
  --minutes M         (hybrid, simulator) simulated minutes to run, 0 = until Ctrl+C (default 5)
  --step S            (hybrid, simulator) simulated seconds between sensor updates

Hybrid and simulator need:  pip install wokwi-client   and   set WOKWI_CLI_TOKEN
Simulator also needs:       pip install jsbsim         and   jsbsim_flight.py next to this file

If a mode cannot start (missing token, missing package ...) the dashboard shows the
reason and you can simply pick another mode.
"""
import argparse
import asyncio
import json
import math
import os
import struct
import threading
import time
import traceback
import webbrowser
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

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

# The data sources the dashboard can choose from (key -> name shown to the user).
MODES = {
    "demo": "Demo",
    "hybrid": "Hybrid",
    "simulator": "Flight Simulator",
}
DEFAULT_MODE = "hybrid"

# Packet layout. MUST match TelemetryPacket in telemetry_packet.h:
#   B sync1, B sync2, B version, B status, I timestamp, H packet_id, 9 floats, I crc32  -> 50 bytes
# The "<" means little-endian with NO padding (same as #pragma pack(1) on the ESP32).
PACKET_FORMAT = "<B B B B I H 9f I"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
SYNC = b"\xaa\x55"
PROTOCOL_VERSION = 1      # must match PROTOCOL_VERSION in telemetry_packet.h

# Positions of the fields inside a decoded packet tuple (same order as PACKET_FORMAT).
IDX_VERSION = 2
IDX_STATUS = 3
IDX_TIMESTAMP = 4
IDX_PACKET_ID = 5
IDX_LAT, IDX_LON, IDX_GPS_ALT = 6, 7, 8

STATUS_MPU_OK = 0x01      # must match telemetry_packet.h
STATUS_BMP_OK = 0x02
STATUS_GPS_FIX = 0x04

# =============================================================================
# 2. SHARED STATE (the dashboard reads this through /data)
# =============================================================================
lock = threading.Lock()      # several threads touch `state`, so every access uses this lock
last_update = [0.0]          # time of the last good packet (list so functions can change it)


def fresh_state():
    return {
        "mode": "", "mode_label": "", "error": "", "finished": False,
        "connected": False, "demo": False, "good": 0, "bad": 0, "lost": 0,
        "version": 0, "status": 0, "timestamp": 0, "packet_id": 0, "latitude": 0.0, "longitude": 0.0,
        "gps_altitude": 0.0, "bmp_altitude": 0.0, "pressure": 0.0,
        "temperature": 0.0, "pitch": 0.0, "roll": 0.0, "yaw": 0.0,
    }


state = fresh_state()


def reset_state(mode):
    """Start a new mode from zero: counters, values and errors of the old mode are dropped."""
    with lock:
        state.clear()
        state.update(fresh_state())
        state.update(mode=mode, mode_label=MODES[mode], demo=(mode == "demo"))
        last_update[0] = 0.0


def set_error(message):
    with lock:
        state["error"] = message


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
    manager = None          # the ModeManager, set in main()

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

    def do_POST(self):
        """POST /mode?m=<demo|simulator|hybrid>  - switch the data source."""
        url = urlparse(self.path)
        if url.path != "/mode":
            self.send_body(404, "text/plain", b"Not found")
            return
        # A web page of another site cannot add this header without our permission,
        # so only our own dashboard can change the mode.
        if self.headers.get("X-GCS") != "1":
            self.send_body(403, "text/plain", b"Forbidden")
            return
        mode = parse_qs(url.query).get("m", [""])[0]
        if self.manager is None or not self.manager.request(mode):
            self.send_body(400, "application/json", json.dumps({"ok": False}).encode())
            return
        self.send_body(200, "application/json", json.dumps({"ok": True, "mode": mode}).encode())

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
def demo_worker(stop):
    start = time.time()
    packet_id = 0
    with lock:
        state["connected"] = True
    while not stop.is_set():
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
        stop.wait(0.1)


# =============================================================================
# 6. FLIGHT MODELS AND SENSOR FORMULAS
# =============================================================================
# ----- simple flight model of the hybrid mode (change freely) -----
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


def load_jsbsim():
    """Import the JSBSim flight class only when a JSBSim mode is chosen, so demo and
    demo and hybrid work even if jsbsim is not installed."""
    try:
        from jsbsim_flight import JsbsimFlight
    except ImportError as e:
        raise RuntimeError(f"JSBSim is not available ({e}). Run: pip install jsbsim "
                           f"- and keep jsbsim_flight.py next to gcs.py")
    return JsbsimFlight


def make_jsbsim_hooks():
    """Flight Simulator mode: returns (flight_model, on_packet) that share ONE JSBSim flight.

    flight_model(t)  drives the virtual MPU6050 / BMP180 in Wokwi with JSBSim data.
    on_packet(p)     Wokwi cannot move a GPS part, so the firmware still has its own fake
                     GPS. When a packet arrives (and the firmware says "GPS fix"), the
                     latitude, longitude and GPS altitude are replaced with the JSBSim
                     position at the same simulation time.
    """
    JsbsimFlight = load_jsbsim()
    box = {"flight": None}

    def model(t):
        if box["flight"] is None:                  # created on first use, after Wokwi has booted
            box["flight"] = JsbsimFlight()
        return box["flight"].state_at(t)

    def on_packet(p):
        if box["flight"] is None or not (p[IDX_STATUS] & STATUS_GPS_FIX):
            return p
        t = p[IDX_TIMESTAMP] / 1000.0 - BOOT_SECONDS      # packet time -> model time
        if t < 0:
            return p
        lat, lon, alt = box["flight"].gps_at(t)
        p = list(p)
        p[IDX_LAT], p[IDX_LON], p[IDX_GPS_ALT] = lat, lon, alt
        return tuple(p)

    return model, on_packet


# =============================================================================
# 7. MODES THAT USE WOKWI (hybrid, simulator)
# =============================================================================
async def rpc(label, coro, timeout=30.0):
    """Await a Wokwi call, but fail with a clear message instead of hanging silently."""
    try:
        return await asyncio.wait_for(coro, timeout)
    except asyncio.TimeoutError:
        raise RuntimeError(f"Timed out after {timeout:.0f}s while waiting for: {label}")


async def run_sim(args, flight_model, on_packet=None):
    """flight_model(t) -> dict with alt, roll, pitch, turn_dps, roll_rate, pitch_rate.
    on_packet(p) -> p   optional hook: receives each valid packet tuple, returns it
                        (possibly changed) before it is stored. Use IDX_* for positions."""
    from wokwi_client import GET_TOKEN_URL, WokwiClient     # only needed in the Wokwi modes

    token = os.getenv("WOKWI_CLI_TOKEN")
    if not token:
        raise RuntimeError(f"Set WOKWI_CLI_TOKEN first. Get it from {GET_TOKEN_URL}")
    for path in (DIAGRAM, FIRMWARE, ELF):
        if not path.exists():
            raise RuntimeError(f"Missing file: {path}  (build the firmware with: pio run)")

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
            lost = stats.lost_since_last(p[IDX_PACKET_ID])
            if on_packet is not None:
                p = on_packet(p)
            store_packet(p, lost)
        set_bad(reader.bad)

    client = WokwiClient(token)
    serial_task = None
    try:
        hello = await rpc("connect", client.connect())
        print("Connected, server version:", hello["version"])
        # firmware.elf is big and the upload can be slow: allow up to 5 minutes
        await rpc("upload diagram.json", client.upload_file("diagram.json", DIAGRAM), timeout=300.0)
        await rpc("upload firmware.bin", client.upload_file("firmware.bin", FIRMWARE), timeout=300.0)
        await rpc("upload firmware.elf", client.upload_file("firmware.elf", ELF), timeout=300.0)
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
            # send all values of this step TOGETHER and wait once (one by one is slow on a
            # slow connection and makes the dashboard show "WAITING FOR DATA")
            results = await asyncio.gather(
                *[rpc(f"set_control {part}.{name}",
                      client.set_control(part, name, round(value, 4)), timeout=30.0)
                  for part, name, value in controls],
                return_exceptions=True)
            for r in results:
                if isinstance(r, RuntimeError):          # timeout: stop with a clear message
                    raise r
                if isinstance(r, Exception) and not warned:
                    print(f"set_control failed: {r!r}")
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
# 8. MODE MANAGER (starts, stops and switches the data source while running)
# =============================================================================
class ModeManager:
    """Runs ONE data source at a time in its own thread. request(mode) is called by the
    dashboard; the old source is stopped completely before the new one starts, so their
    data never mix."""

    def __init__(self, args):
        self.args = args
        self.guard = threading.Lock()       # only one switch at a time
        self.wanted = None                  # the newest mode that was asked for
        self.stop = threading.Event()       # tells the running worker to finish
        self.thread = None
        self.loop = None                    # asyncio loop + task of a Wokwi worker
        self.task = None

    # ---------- called from the dashboard / main ----------
    def request(self, mode):
        if mode not in MODES:
            return False
        self.wanted = mode
        with lock:                          # the dashboard sees the choice at once
            state.update(mode=mode, mode_label=MODES[mode], error="", finished=False,
                         connected=False, demo=(mode == "demo"))
        threading.Thread(target=self._switch, args=(mode,), daemon=True).start()
        return True

    def shutdown(self):
        self.wanted = None
        with self.guard:
            self._stop_current()

    # ---------- internals ----------
    def _switch(self, mode):
        with self.guard:
            if mode != self.wanted:         # a newer request arrived while we waited
                return
            self._stop_current()
            print(f"[GCS] Mode: {MODES[mode]}")
            reset_state(mode)
            self.stop = threading.Event()
            self.thread = threading.Thread(target=self._worker, args=(mode, self.stop), daemon=True)
            self.thread.start()

    def _stop_current(self):
        if self.thread is None:
            return
        self.stop.set()
        if self.loop is not None and self.task is not None:
            try:
                self.loop.call_soon_threadsafe(self.task.cancel)
            except RuntimeError:            # the loop is already closed
                pass
        self.thread.join(timeout=30)
        if self.thread.is_alive():
            print("[GCS] WARNING: the old data source did not stop in 30 s.")
        self.thread = self.loop = self.task = None

    def _worker(self, mode, stop):
        try:
            if mode == "demo":
                demo_worker(stop)
            else:
                self._run_wokwi(mode, stop)
            if not stop.is_set():           # it ended by itself (e.g. --minutes was reached)
                with lock:
                    state["finished"] = True
        except asyncio.CancelledError:
            pass
        except (RuntimeError, SystemExit) as e:            # a clear message for the user
            self._fail(stop, str(e))
        except Exception as e:                             # a bug: show it, keep the details
            traceback.print_exc()
            self._fail(stop, f"{type(e).__name__}: {e}")

    def _run_wokwi(self, mode, stop):
        if mode == "simulator":
            model, on_packet = make_jsbsim_hooks()      # sensors AND GPS from JSBSim
        else:
            model, on_packet = flight_model, None       # simple model + the firmware's own GPS
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)                       # this thread has no loop of its own yet
        task = loop.create_task(run_sim(self.args, model, on_packet))
        self.loop, self.task = loop, task
        if stop.is_set():                                  # stopped before the task existed
            task.cancel()
        try:
            loop.run_until_complete(task)
        except asyncio.CancelledError:
            pass
        finally:
            pending = [t for t in asyncio.all_tasks(loop) if not t.done()]
            for t in pending:
                t.cancel()
            if pending:
                loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            loop.close()

    @staticmethod
    def _fail(stop, message):
        if stop.is_set():                   # we asked it to stop, so this is not an error
            return
        print(f"\n[GCS] ERROR: {message}")
        set_error(message)


# =============================================================================
# 9. MAIN
# =============================================================================
def main(default_mode=DEFAULT_MODE):
    ap = argparse.ArgumentParser(description="Telemetry ground station")
    ap.add_argument("--mode", choices=list(MODES), default=default_mode,
                    help="data source at start-up; it can be changed on the dashboard")
    ap.add_argument("--demo", action="store_true", help="same as --mode demo")
    ap.add_argument("--http-port", type=int, default=HTTP_PORT)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--minutes", type=float, default=5.0,
                    help="(hybrid, simulator) simulated minutes to run, 0 = until Ctrl+C (default 5)")
    ap.add_argument("--step", type=float, default=0.5,
                    help="(hybrid, simulator) seconds of simulation time between sensor updates")
    args = ap.parse_args()
    if args.demo:
        args.mode = "demo"

    manager = ModeManager(args)
    Handler.manager = manager
    start_http(args.http_port, not args.no_browser)
    manager.request(args.mode)

    try:
        while True:                     # keep the main thread alive until Ctrl+C
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[GCS] Stopped by user.")
    finally:
        manager.shutdown()


if __name__ == "__main__":
    main()