"""
jsbsim_flight.py - a Cessna 172 flown by JSBSim with a simple autopilot.

It gives the ground station three things:
  state_at(t)  what the MPU6050 / BMP180 should "feel" at time t (with realistic errors)
  gps_at(t)    the GPS position at time t (true position + GPS error)
  (the true flight is also recorded, so any packet time can be looked up)
"""
import bisect
import math
import random
import jsbsim

# ---------------- flight ----------------
START_LAT, START_LON = 39.7767, 30.5206   # same start as the firmware fake GPS
START_ALT_M = 914.4                        # 3000 ft
START_HEADING_DEG = 0                      # the firmware yaw starts at 0, so we start north

# flight plan: (from time in s, wanted heading in deg, wanted altitude in m)
PLAN = [(0, 0, 914.4), (20, 90, 914.4), (45, 180, 960.0), (70, 270, 960.0)]

# autopilot gains (these worked in the first test)
KP_ROLL, KD_ROLL = 1.2, 0.4
KP_PITCH, KI_PITCH, KD_PITCH = 1.5, 0.5, 0.4
HDG_TO_ROLL = 1.2
ALT_TO_PITCH = 0.003
MAX_ROLL = math.radians(25)
MAX_PITCH = math.radians(8)

# ---------------- sensor errors (set NOISE = False for perfect sensors) ----------------
NOISE = True
GYRO_BIAS_SIGMA = 0.05     # deg/s  constant error of each gyro axis (different every run)
GYRO_NOISE = 0.10          # deg/s  random error on every reading
ANGLE_NOISE = 0.25         # deg    accelerometer noise seen on roll and pitch
BARO_NOISE_M = 0.4         # m      random error of the barometer altitude
WEATHER_AMP_M = 8.0        # m      slow pressure change (weather) shows up as an altitude error
WEATHER_PERIOD_S = 300.0
GPS_SIGMA_M = 2.0          # m      horizontal GPS error (changes slowly)
GPS_ALT_SIGMA_M = 3.0      # m      vertical GPS error
GPS_MEMORY = 0.995         # closer to 1 = slower GPS drift

HIST_DT = 0.05             # record the true flight every 50 ms (the packets are 20 per second)
LOOKAHEAD = 2.5            # s, fly a little ahead so packet times are always inside the record
M_PER_DEG_LAT = 111320.0


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class JsbsimFlight:
    def __init__(self):
        fdm = jsbsim.FGFDMExec(None)
        fdm.set_debug_level(0)
        fdm.load_model("c172x")
        fdm["ic/lat-geod-deg"] = START_LAT
        fdm["ic/long-gc-deg"] = START_LON
        fdm["ic/h-sl-ft"] = START_ALT_M / 0.3048
        fdm["ic/vc-kts"] = 100
        fdm["ic/psi-true-deg"] = START_HEADING_DEG
        fdm.run_ic()
        fdm["propulsion/set-running"] = -1
        fdm["fcs/throttle-cmd-norm"] = 0.7
        fdm["fcs/mixture-cmd-norm"] = 1.0
        try:
            fdm["simulation/do_simple_trim"] = 1
        except Exception as e:
            print("JSBSim trim failed, continuing without it:", e)
        self.fdm = fdm
        self.t0 = fdm.get_sim_time()               # model time starts at 0 here
        self.ail0 = fdm["fcs/aileron-cmd-norm"]
        self.elev0 = fdm["fcs/elevator-cmd-norm"]
        self.pitch_int = 0.0

        self.hist_t = []                           # recorded true flight
        self.hist = []
        self._next_rec = 0.0
        self._record()

        # sensor errors that stay the same for the whole run
        self.gyro_bias = [random.gauss(0, GYRO_BIAS_SIGMA) for _ in range(3)]
        self.weather_phase = random.uniform(0, 2 * math.pi)
        self.gps_err = [0.0, 0.0, 0.0]             # north m, east m, up m

    # ---------- flying ----------
    def _now(self):
        return self.fdm.get_sim_time() - self.t0

    @staticmethod
    def _target(t):
        hdg, alt = PLAN[0][1], PLAN[0][2]
        for start, h, a in PLAN:
            if t >= start:
                hdg, alt = h, a
        return math.radians(hdg), alt

    def _autopilot(self):
        f = self.fdm
        t, dt = self._now(), f.get_delta_t()
        roll, pitch, yaw = f["attitude/roll-rad"], f["attitude/pitch-rad"], f["attitude/psi-rad"]
        alt_m = f["position/h-sl-ft"] * 0.3048
        hdg_t, alt_t = self._target(t)
        roll_t = _clamp(HDG_TO_ROLL * _wrap_pi(hdg_t - yaw), -MAX_ROLL, MAX_ROLL)
        pitch_t = _clamp(ALT_TO_PITCH * (alt_t - alt_m), -MAX_PITCH, MAX_PITCH)
        ail = self.ail0 + KP_ROLL * (roll_t - roll) - KD_ROLL * f["velocities/p-rad_sec"]
        err = pitch_t - pitch
        self.pitch_int = _clamp(self.pitch_int + err * dt, -0.3, 0.3)
        elev = self.elev0 - (KP_PITCH * err + KI_PITCH * self.pitch_int - KD_PITCH * f["velocities/q-rad_sec"])
        f["fcs/aileron-cmd-norm"] = _clamp(ail, -1.0, 1.0)
        f["fcs/elevator-cmd-norm"] = _clamp(elev, -1.0, 1.0)

    def _snapshot(self):
        """The true state of the aircraft right now."""
        f = self.fdm
        phi, theta = f["attitude/roll-rad"], f["attitude/pitch-rad"]
        p, q, r = f["velocities/p-rad_sec"], f["velocities/q-rad_sec"], f["velocities/r-rad_sec"]
        psi_dot = (q * math.sin(phi) + r * math.cos(phi)) / max(0.2, math.cos(theta))
        return (f["position/lat-geod-deg"], f["position/long-gc-deg"], f["position/h-sl-ft"] * 0.3048,
                math.degrees(phi), math.degrees(theta), math.degrees(psi_dot),
                math.degrees(p), math.degrees(q))

    def _record(self):
        t = self._now()
        if t >= self._next_rec:
            self._next_rec = t + HIST_DT
            self.hist_t.append(t)
            self.hist.append(self._snapshot())

    def _advance_to(self, t):
        while self._now() < t:
            self._autopilot()
            self.fdm.run()
            self._record()

    def _truth_at(self, t):
        """True state at model time t (linear interpolation of the record)."""
        i = bisect.bisect_right(self.hist_t, t)
        if i <= 0:
            return self.hist[0]
        if i >= len(self.hist_t):
            return self.hist[-1]
        t0, t1 = self.hist_t[i - 1], self.hist_t[i]
        k = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        a, b = self.hist[i - 1], self.hist[i]
        return tuple(x + k * (y - x) for x, y in zip(a, b))

    # ---------- what the sensors report ----------
    def state_at(self, t):
        """Fly until t, return what the MPU6050 and BMP180 should measure at t."""
        self._advance_to(t + LOOKAHEAD)
        lat, lon, alt, roll, pitch, turn, rrate, prate = self._truth_at(t)
        if NOISE:
            g = random.gauss
            weather = WEATHER_AMP_M * math.sin(2 * math.pi * t / WEATHER_PERIOD_S + self.weather_phase)
            alt += weather + g(0, BARO_NOISE_M)
            roll += g(0, ANGLE_NOISE)
            pitch += g(0, ANGLE_NOISE)
            rrate += self.gyro_bias[0] + g(0, GYRO_NOISE)
            prate += self.gyro_bias[1] + g(0, GYRO_NOISE)
            turn += self.gyro_bias[2] + g(0, GYRO_NOISE)
        return {"alt": alt, "roll": roll, "pitch": pitch,
                "turn_dps": turn, "roll_rate": rrate, "pitch_rate": prate}

    def gps_at(self, t):
        """GPS reading at model time t: true position + a slowly changing GPS error."""
        lat, lon, alt = self._truth_at(t)[:3]
        if NOISE:
            k = math.sqrt(1.0 - GPS_MEMORY ** 2)
            sig = (GPS_SIGMA_M, GPS_SIGMA_M, GPS_ALT_SIGMA_M)
            self.gps_err = [GPS_MEMORY * e + k * s * random.gauss(0, 1) for e, s in zip(self.gps_err, sig)]
            north, east, up = self.gps_err
            lat += north / M_PER_DEG_LAT
            lon += east / (M_PER_DEG_LAT * math.cos(math.radians(lat)))
            alt += up
        return lat, lon, alt