#pragma once

#include <Arduino.h>
#include <math.h>

// FakeGpsStream - pretends to be the serial port of a GPS module (like NEO-6M).
//
// A real GPS module sends text lines called NMEA sentences, for example:
//     $GPGGA,120000.00,3946.6020,N,03031.2360,E,1,08,0.9,350.0,M,0.0,M,,*5B
// This class builds the same kind of lines from a position that WE give it,
// and lets the rest of the program read them byte by byte with read().
// So the main code (TinyGPSPlus) cannot tell it from a real GPS.
//
// With a real module you would replace this class with Serial1 and change nothing else.
class FakeGpsStream : public Stream {
 public:
  // intervalMs = time between two GPS updates (200 ms = 5 Hz)
  explicit FakeGpsStream(uint32_t intervalMs = 200) : intervalMs_(intervalMs) {}

  // Call this often (every loop). It builds new sentences only when the interval
  // has passed AND the previous sentences were already read.
  void update(double lat, double lon, float altM, float speedMps, float headingDeg) {
    uint32_t now = millis();
    if (pos_ < len_) return;                                   // old text not read yet
    if (built_ && (now - lastBuildMs_) < intervalMs_) return;  // too early
    built_ = true;
    lastBuildMs_ = now;
    pos_ = 0;
    len_ = 0;

    // --- position as text: ddmm.mmmm (latitude) and dddmm.mmmm (longitude) ---
    char latText[16], lonText[16];
    formatCoord(latText, sizeof(latText), lat, true);
    formatCoord(lonText, sizeof(lonText), lon, false);
    char ns = (lat >= 0) ? 'N' : 'S';
    char ew = (lon >= 0) ? 'E' : 'W';

    // --- fake UTC clock that starts at 12:00:00 when the board boots ---
    uint32_t sec = now / 1000;
    unsigned centi = (now % 1000) / 10;
    int hh = (int)((12 + sec / 3600) % 24);
    int mm = (int)((sec / 60) % 60);
    int ss = (int)(sec % 60);
    char timeText[16];
    snprintf(timeText, sizeof(timeText), "%02d%02d%02d.%02u", hh, mm, ss, centi);

    float speedKnots = speedMps * 1.943844f;     // NMEA speed is in knots
    float course = fmodf(headingDeg, 360.0f);    // NMEA course is 0..360 degrees
    if (course < 0) course += 360.0f;

    char body[128];

    // GGA = fix data: position, fix quality (1 = GPS fix), satellites, altitude
    snprintf(body, sizeof(body), "GPGGA,%s,%s,%c,%s,%c,1,08,0.9,%.1f,M,0.0,M,,",
             timeText, latText, ns, lonText, ew, altM);
    addSentence(body);

    // RMC = recommended minimum: status A (valid), position, speed, course, date
    snprintf(body, sizeof(body), "GPRMC,%s,A,%s,%c,%s,%c,%.2f,%.1f,010126,,,A",
             timeText, latText, ns, lonText, ew, speedKnots, course);
    addSentence(body);
  }

  // ---- the three functions every Stream must have: how the reader gets our bytes ----
  int available() override { return (int)(len_ - pos_); }
  int read() override { return (pos_ < len_) ? (uint8_t)buf_[pos_++] : -1; }
  int peek() override { return (pos_ < len_) ? (uint8_t)buf_[pos_] : -1; }
  // Print needs write(); a real GPS can receive commands, ours simply ignores them.
  size_t write(uint8_t) override { return 1; }

 private:
  // Converts degrees to NMEA text. Uses whole numbers (1/10000 of a minute) so that
  // rounding can never produce "60.0000" minutes.
  static void formatCoord(char *out, size_t n, double value, bool isLat) {
    long long total = llround(fabs(value) * 60.0 * 10000.0);
    int deg = (int)(total / 600000LL);
    int minWhole = (int)((total % 600000LL) / 10000LL);
    int minFrac = (int)(total % 10000LL);
    if (isLat) snprintf(out, n, "%02d%02d.%04d", deg, minWhole, minFrac);
    else       snprintf(out, n, "%03d%02d.%04d", deg, minWhole, minFrac);
  }

  // Adds "$<body>*<checksum>\r\n" to the buffer.
  // checksum = XOR of every character between '$' and '*', printed as 2 hex digits.
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
  size_t len_ = 0;   // how many bytes are in the buffer
  size_t pos_ = 0;   // how many bytes were already read
};