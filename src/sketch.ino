#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <math.h>
#include "telemetry_packet.h"
#include "fake_gps.h"

static const float RAD2DEG = 57.2958f;

// ---- sending rate ----
static const unsigned long SEND_INTERVAL_MS = 50;   // one packet every 50 ms = 20 packets per second

// ---- simulated GPS settings (there is no GPS chip in the simulation) ----
static const double START_LAT = 39.7767;          // where the simulated flight starts
static const double START_LON = 30.5206;
static const float SIM_SPEED_MPS = 60.0f;         // ground speed of the simulated aircraft
static const double METERS_PER_DEG_LAT = 111320.0;
static const float GPS_ALT_OFFSET_M = 3.0f;       // GPS altitude differs a little from the barometer
static const float GPS_ALT_NOISE_M = 2.0f;        // random +/- error added to the GPS altitude

// ---- attitude filter settings ----
static const float FILTER_ALPHA = 0.98f;          // 98% trust in the gyro, 2% in the accelerometer
static const int GYRO_CAL_SAMPLES = 200;          // readings averaged at start-up (about 1 second)
static const float GYRO_CAL_MAX_RAD_S = 0.2f;     // average above this = board was moving, ignore it

Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
TelemetryPacket pkt;
uint16_t counter = 0;
unsigned long lastSendMs = 0;     // when the last packet was due
unsigned long lastMicros = 0;     // for dt (time between two measurements)

bool mpuOk = false;               // did the MPU6050 answer at start-up?
bool bmpOk = false;               // did the BMP180 answer at start-up?

float yaw = 0.0f;
float rollDeg = 0.0f;             // filtered roll
float pitchDeg = 0.0f;            // filtered pitch
bool attitudeReady = false;       // false until the first accelerometer angle is known
float gyroBiasX = 0.0f;           // what the gyro reports while the board is still (rad/s)
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

FakeGpsStream gpsPort(200);       // pretend GPS module: sends NMEA text, 5 times per second
TinyGPSPlus gps;                  // reads that text and gives us numbers
double simLat = START_LAT;        // simulated true position
double simLon = START_LON;
float gpsNoise = 0.0f;            // current GPS altitude error
double gpsLat = 0.0;              // last values read from the GPS (0 until the first fix)
double gpsLon = 0.0;
float gpsAlt = 0.0f;

// A gyro that is NOT moving still reports a small constant error (bias). If we integrate
// it, yaw drifts slowly forever. At start-up the board is still, so we average the gyro
// for about a second and subtract that average from every later reading.
void calibrateGyro() {
  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(5);
  }
  float bx = sumX / GYRO_CAL_SAMPLES;
  float by = sumY / GYRO_CAL_SAMPLES;
  float bz = sumZ / GYRO_CAL_SAMPLES;
  if (fabsf(bx) > GYRO_CAL_MAX_RAD_S || fabsf(by) > GYRO_CAL_MAX_RAD_S || fabsf(bz) > GYRO_CAL_MAX_RAD_S) {
    Serial.println("Gyro calibration skipped (board was moving)");
    return;                                  // keep bias = 0
  }
  gyroBiasX = bx;
  gyroBiasY = by;
  gyroBiasZ = bz;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);
  mpuOk = mpu.begin(0x68);
  bmpOk = bmp.begin();
  if (!mpuOk) Serial.println("MPU6050 not found");
  if (!bmpOk) Serial.println("BMP180 not found");
  if (mpuOk) calibrateGyro();                // the board must be still during this second
  lastMicros = micros();
}

void loop() {
  // ---- timing: do the work only when a packet is due (no blocking delay(50)) ----
  unsigned long nowMs = millis();
  if (nowMs - lastSendMs < SEND_INTERVAL_MS) {
    delay(1);                                // not yet: rest 1 ms, then check again
    return;
  }
  lastSendMs += SEND_INTERVAL_MS;            // keep a steady 50 ms rhythm
  if (nowMs - lastSendMs >= SEND_INTERVAL_MS) lastSendMs = nowMs;   // we fell far behind: restart the rhythm

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0f;
  lastMicros = now;

  // ---- read the sensors. A sensor that was not found is NOT read: its values stay NaN ----
  sensors_event_t a = {}, g = {}, t = {};
  if (mpuOk) mpu.getEvent(&a, &g, &t);

  float pressurePa = bmpOk ? bmp.readPressure() : NAN;
  float tempC = bmpOk ? bmp.readTemperature() : NAN;
  float alt = bmpOk ? 44330.0f * (1.0f - powf(pressurePa / 101325.0f, 0.1903f)) : NAN;

  // ---- attitude (only if the MPU6050 exists) ----
  if (mpuOk) {
    // gyro rates in degrees/second, with the start-up bias removed
    float gx = (g.gyro.x - gyroBiasX) * RAD2DEG;
    float gy = (g.gyro.y - gyroBiasY) * RAD2DEG;
    float gz = (g.gyro.z - gyroBiasZ) * RAD2DEG;

    yaw += gz * dt;
    yaw = fmodf(yaw, 360.0f);
    if (yaw < 0) yaw += 360.0f;

    // roll and pitch seen by the accelerometer (good on average, noisy at every instant)
    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;
    float accRoll = atan2f(ay, az) * RAD2DEG;
    float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;

    // Complementary filter: the gyro gives smooth changes (integrate rate x time),
    // the accelerometer slowly pulls the result back so the gyro error cannot grow.
    if (!attitudeReady) {
      rollDeg = accRoll;                     // first sample: start from the accelerometer
      pitchDeg = accPitch;
      attitudeReady = true;
    } else {
      rollDeg = FILTER_ALPHA * (rollDeg + gx * dt) + (1.0f - FILTER_ALPHA) * accRoll;
      pitchDeg = FILTER_ALPHA * (pitchDeg + gy * dt) + (1.0f - FILTER_ALPHA) * accPitch;
    }
  }

  // ---- simulated position: fly at SIM_SPEED_MPS in the direction of yaw ----
  // 1 degree of latitude is ~111320 m; 1 degree of longitude gets shorter away from the equator.
  float headingRad = yaw / RAD2DEG;
  simLat += SIM_SPEED_MPS * cosf(headingRad) * dt / METERS_PER_DEG_LAT;
  simLon += SIM_SPEED_MPS * sinf(headingRad) * dt / (METERS_PER_DEG_LAT * cos(simLat / RAD2DEG));

  // ---- fake GPS: build NMEA text, then read it back with TinyGPSPlus ----
  float simAlt = bmpOk ? alt : 0.0f;         // the fake GPS needs some altitude even if the barometer is missing
  gpsPort.update(simLat, simLon, simAlt + GPS_ALT_OFFSET_M + gpsNoise, SIM_SPEED_MPS, yaw);
  while (gpsPort.available()) gps.encode(gpsPort.read());

  if (gps.location.isUpdated()) {
    gpsLat = gps.location.lat();
    gpsLon = gps.location.lng();
  }
  if (gps.altitude.isUpdated()) {
    gpsAlt = gps.altitude.meters();
    gpsNoise = GPS_ALT_NOISE_M * (random(-1000, 1001) / 1000.0f);   // new error for the next fix
  }

  // ---- fill the packet ----
  pkt.sync1 = 0xAA;
  pkt.sync2 = 0x55;
  pkt.version = PROTOCOL_VERSION;

  // status bits: 1 = working, 0 = missing / error
  pkt.status = 0;
  if (mpuOk) pkt.status |= STATUS_MPU_OK;
  if (bmpOk) pkt.status |= STATUS_BMP_OK;
  if (gps.location.isValid() && gps.location.age() < 2000) pkt.status |= STATUS_GPS_FIX;   // fix younger than 2 s

  pkt.timescounter = millis();
  pkt.packet_id = ++counter;

  pkt.latitude = (float)gpsLat;
  pkt.longitude = (float)gpsLon;
  pkt.gps_altitude = gpsAlt;

  // values of a missing sensor are NaN ("not a number"), never a believable fake number
  pkt.bmp_altitude = alt;
  pkt.pres = pressurePa / 100.0f;
  pkt.temp = tempC;

  pkt.roll = mpuOk ? rollDeg : NAN;
  pkt.pitch = mpuOk ? pitchDeg : NAN;
  pkt.yaw = mpuOk ? yaw : NAN;

  pkt.crc32 = calculatecrc32((const uint8_t *)&pkt,
                             sizeof(TelemetryPacket) - sizeof(uint32_t));

  Serial.write((const uint8_t *)&pkt, sizeof(pkt));
}
