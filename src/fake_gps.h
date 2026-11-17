#pragma once

#include <Arduino.h>
#include <math.h>

// FakeGpsStream - pretends to be the serial port of a GPS module (like NEO-6M).
//
// It builds NMEA sentences (GPGGA, GPRMC) from a position that WE give it, and lets
// TinyGPSPlus read them byte by byte, exactly like a real GPS on Serial1.
//
// To feel like a real receiver it also simulates:
//   - cold start: no fix for the first COLD_START_MS after boot
//   - position noise: slow drift of about 2 m (not random jumps)
//   - speed and course noise
//   - changing number of satellites and HDOP
//   - rare short fix losses (a few seconds)
class FakeGpsStream : public Stream {
 public:
  // intervalMs = time between two GPS updates (200 ms = 5 Hz)
  explicit FakeGpsStream(uint32_t intervalMs = 200) : intervalMs_(intervalMs) {}

  void update(double lat, double lon, float altM, float speedMps, float headingDeg) {
    uint32_t now = millis();
    if (pos_ < len_) return;                                   // old text not read yet
    if (built_ && (now - lastBuildMs_) < intervalMs_) return;  // too early
    built_ = true;
    lastBuildMs_ = now;
    pos_ = 0;
    len_ = 0;

    updateFixState(now);
    updateNoise();

    // --- fake UTC clock that starts at 12:00:00 when the board boots ---
    uint32_t sec = now / 1000;
    unsigned centi = (now % 1000) / 10;
    int hh = (int)((12 + sec / 3600) % 24);
    int mm = (int)((sec / 60) % 60);
    int ss = (int)(sec % 60);
    char timeText[16];
    snprintf(timeText, sizeof(timeText), "%02d%02d%02d.%02u", hh, mm, ss, centi);

    char body[128];

    if (!hasFix_) {
      // No fix: a real GPS still sends sentences, but position fields are empty.
      snprintf(body, sizeof(body), "GPGGA,%s,,,,,0,%02d,,,M,,M,,", timeText, sats_);
      addSentence(body);
      snprintf(body, sizeof(body), "GPRMC,%s,V,,,,,,,010126,,,N", timeText);
      addSentence(body);
      return;
    }

    // --- add the slow position error (meters -> degrees) ---
    double noisyLat = lat + errNorthM_ / 111320.0;
    double noisyLon = lon + errEastM_ / (111320.0 * cos(lat * M_PI / 180.0));

    char latText[16], lonText[16];
    formatCoord(latText, sizeof(latText), noisyLat, true);
    formatCoord(lonText, sizeof(lonText), noisyLon, false);
    char ns = (noisyLat >= 0) ? 'N' : 'S';
    char ew = (noisyLon >= 0) ? 'E' : 'W';

    float speedKnots = fmaxf(0.0f, speedMps + speedErr_) * 1.943844f;   // NMEA speed is in knots
    float course = fmodf(headingDeg + courseErr_, 360.0f);              // NMEA course is 0..360
    if (course < 0) course += 360.0f;

    // GGA = fix data: position, fix quality (1 = GPS fix), satellites, HDOP, altitude
    snprintf(body, sizeof(body), "GPGGA,%s,%s,%c,%s,%c,1,%02d,%.1f,%.1f,M,0.0,M,,",
             timeText, latText, ns, lonText, ew, sats_, hdop_, altM);
    addSentence(body);

    // RMC = recommended minimum: status A (valid), position, speed, course, date
    snprintf(body, sizeof(body), "GPRMC,%s,A,%s,%c,%s,%c,%.2f,%.1f,010126,,,A",
             timeText, latText, ns, lonText, ew, speedKnots, course);
    addSentence(body);
  }

  // ---- the functions every Stream must have: how the reader gets our bytes ----
  int available() override { return (int)(len_ - pos_); }
  int read() override { return (pos_ < len_) ? (uint8_t)buf_[pos_++] : -1; }
  int peek() override { return (pos_ < len_) ? (uint8_t)buf_[pos_] : -1; }
  size_t write(uint8_t) override { return 1; }   // a real GPS can receive commands; we ignore them

 private:
  // ---- tuning: change these to make the GPS better or worse ----
  static const uint32_t COLD_START_MS = 20000;   // no fix for the first 20 s
  static constexpr float POS_SIGMA_M = 2.0f;     // typical position error (meters)
  static constexpr float POS_MEMORY = 0.97f;     // closer to 1 = slower drift
  static constexpr float SPEED_SIGMA = 0.3f;     // speed error (m/s)
  static constexpr float COURSE_SIGMA = 1.5f;    // course error (degrees)
  static const uint32_t DROPOUT_MIN_MS = 3000;   // fix loss lasts 3..6 s
  static const uint32_t DROPOUT_MAX_MS = 6000;
  static const uint32_t DROPOUT_EVERY_MS = 90000; // about one fix loss every 90 s

  static float randUnit() { return random(0, 10001) / 10000.0f; }         // 0..1
  static float randGauss() {                                               // about N(0,1)
    float s = 0;
    for (int i = 0; i < 12; i++) s += randUnit();
    return s - 6.0f;
  }

  // Decides if we have a fix right now: cold start, then rare short losses.
  void updateFixState(uint32_t now) {
    if (now < COLD_START_MS) {
      hasFix_ = false;
      sats_ = 3 + (int)(now / 5000);     // satellites slowly found: 3, 4, 5, 6
      return;
    }
    if (dropoutUntilMs_ != 0 && now >= dropoutUntilMs_) dropoutUntilMs_ = 0;
    if (dropoutUntilMs_ == 0) {
      // chance per update so that a loss happens about once per DROPOUT_EVERY_MS
      float p = (float)intervalMs_ / (float)DROPOUT_EVERY_MS;
      if (randUnit() < p) {
        dropoutUntilMs_ = now + DROPOUT_MIN_MS + random(0, DROPOUT_MAX_MS - DROPOUT_MIN_MS + 1);
      }
    }
    hasFix_ = (dropoutUntilMs_ == 0);
    if (hasFix_) {
      // satellites change slowly between 7 and 11
      if (randUnit() < 0.05f) sats_ += (randUnit() < 0.5f) ? -1 : 1;
      if (sats_ < 7) sats_ = 7;
      if (sats_ > 11) sats_ = 11;
      hdop_ = 0.8f + 0.12f * (11 - sats_) + 0.1f * randUnit();   // fewer satellites = worse HDOP
    }
  }

  // Slow random drift (Gauss-Markov): error changes a little each step, never jumps.
  void updateNoise() {
    float k = sqrtf(1.0f - POS_MEMORY * POS_MEMORY);
    errNorthM_ = POS_MEMORY * errNorthM_ + k * POS_SIGMA_M * randGauss();
    errEastM_  = POS_MEMORY * errEastM_  + k * POS_SIGMA_M * randGauss();
    speedErr_  = SPEED_SIGMA * randGauss();
    courseErr_ = COURSE_SIGMA * randGauss();
  }

  // Converts degrees to NMEA text. Uses whole numbers so rounding can never give "60.0000" minutes.
  static void formatCoord(char *out, size_t n, double value, bool isLat) {
    long long total = llround(fabs(value) * 60.0 * 10000.0);
    int deg = (int)(total / 600000LL);
    int minWhole = (int)((total % 600000LL) / 10000LL);
    int minFrac = (int)(total % 10000LL);
    if (isLat) snprintf(out, n, "%02d%02d.%04d", deg, minWhole, minFrac);
    else       snprintf(out, n, "%03d%02d.%04d", deg, minWhole, minFrac);
  }


  void addSentence(const char *body) {
    uint8_t checksum = 0;
    for (const char *p = body; *p; ++p) checksum ^= (uint8_t)*p;
    int n = snprintf(buf_ + len_, sizeof(buf_) - len_, "$%s*%02X\r\n", body, checksum);
    if (n > 0 && (size_t)n < sizeof(buf_) - len_) len_ += (size_t)n;
  }

  uint32_t intervalMs_;
  uint32_t lastBuildMs_ = 0;
  bool built_ = false;
  char buf_[256];
  size_t len_ = 0;
  size_t pos_ = 0;

  bool hasFix_ = false;
  uint32_t dropoutUntilMs_ = 0;
  int sats_ = 3;
  float hdop_ = 1.0f;
  float errNorthM_ = 0.0f, errEastM_ = 0.0f;
  float speedErr_ = 0.0f, courseErr_ = 0.0f;
};