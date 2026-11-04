#pragma once

#include <Arduino.h>

// Protocol version. Increase it every time the packet layout changes,
// so the ground station can refuse packets it does not understand.
static const uint8_t PROTOCOL_VERSION = 1;

// Bits of TelemetryPacket::status  (1 = working, 0 = not working)
static const uint8_t STATUS_MPU_OK  = 0x01;   // bit 0: MPU6050 found at start-up
static const uint8_t STATUS_BMP_OK  = 0x02;   // bit 1: BMP180 found at start-up
static const uint8_t STATUS_GPS_FIX = 0x04;   // bit 2: GPS has a recent valid fix

#pragma pack(push, 1)

struct TelemetryPacket {
  uint8_t sync1;
  uint8_t sync2;

  uint8_t version;      // PROTOCOL_VERSION
  uint8_t status;       // STATUS_* bits OR-ed together

  uint32_t timescounter;
  uint16_t packet_id;

  float latitude;
  float longitude;
  float gps_altitude;

  float bmp_altitude;
  float pres;
  float temp;

  float pitch;
  float roll;
  float yaw;

  uint32_t crc32;
};

#pragma pack(pop)

static_assert(sizeof(TelemetryPacket) == 50, "TelemetryPacket must be 50 bytes");

uint32_t calculatecrc32(const uint8_t *data, size_t length);