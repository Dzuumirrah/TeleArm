#ifndef CONNECTIONS_H
#define CONNECTIONS_H

#include <Arduino.h>
#include "kinematics.h"

// =============================================================================
//  IMU hardware                                   (components.cpp)
// =============================================================================
// Initialise both IMUs on their own SPI bus and apply the settings
// (DLPF 4, gyro +-500 dps, acc +-2 g, magnetometer 100 Hz).
// Returns false if an IMU does not answer. A missing AK8963 only disables magnetometer use.
// (was `void MPUInit()` : it could not report a failure)
bool MPUInit();

// Read both IMUs back to back. Units: g, deg/s, uT (raw AK8963 axes, not calibrated).
// Returns false if a sample is not plausible (disconnected / corrupted SPI read).
bool readIMUs(IMUData &imu9250, IMUData &imu6500);

// true when the AK8963 answered in MPUInit()
bool magnetometerAvailable();

// Bench offset calibration with the library's autoOffsets(): BOTH IMUs lying flat, z up, still.
// Offsets are stored in flash and re-applied by MPUInit().
bool benchCalibrateIMUs();

// =============================================================================
//  Flash storage (NVS)                            (storage.cpp)
// =============================================================================
struct BenchOffsets {
    xyzFloat acc9250, gyr9250;     // library units (raw LSB at +-2 g / +-250 dps)
    xyzFloat acc6500, gyr6500;
};

bool loadCalibration(CalibrationData &c);          // false if missing / corrupt / other version
bool saveCalibration(const CalibrationData &c);    // magic, version and checksum are set here
bool loadBenchOffsets(BenchOffsets &b);
bool saveBenchOffsets(const BenchOffsets &b);
void clearStorage();                               // erase calibration + bench offsets

// =============================================================================
//  ESP-NOW master -> slave                        (networks.cpp)
// =============================================================================
const uint8_t PKT_START = 0xA5;
const uint8_t PKT_TYPE_JOINTS = 1;

// flags
const uint8_t PKT_F_CALIBRATED  = 1 << 0;   // segment calibration valid and anchored
const uint8_t PKT_F_MAG         = 1 << 1;   // 9250 heading uses the magnetometer
const uint8_t PKT_F_LIMIT       = 1 << 2;   // a joint limit clamped a command
const uint8_t PKT_F_SLEW        = 1 << 3;   // the slew-rate limit was active
const uint8_t PKT_F_UNREACHABLE = 1 << 4;   // target outside the robot workspace
const uint8_t PKT_F_IMU_FAULT   = 1 << 5;   // recent IMU read failures (angles are held)

// Fixed 54-byte frame, little endian (ESP32 native), CRC8 (poly 0x07) over bytes 0..52
struct __attribute__((packed)) JointPacket {
    uint8_t  start;        // PKT_START
    uint8_t  type;         // PKT_TYPE_JOINTS
    uint16_t seq;          // increments per packet; gaps = lost packets
    uint32_t t_us;         // master micros() at sampling
    float    joint_deg[3]; // robot joint commands: base, shoulder, elbow  [deg]
    float    q_upper[4];   // upper-arm segment orientation (segment -> P frame), w x y z
    float    q_fore[4];    // forearm segment orientation, w x y z
    uint8_t  flags;        // PKT_F_*
    uint8_t  crc;
};

uint8_t crc8(const uint8_t *data, size_t len);
bool    packetValid(const uint8_t *data, int len);     // length, start byte, CRC

bool ESPNowInit();                                     // false: ESP-NOW could not start
// Fills start/type/seq/crc and sends. Returns false if esp_now_send() rejected it.
bool ESPNowSend(JointPacket &p);
void ESPNowStats(uint32_t &sentOk, uint32_t &sentFail);   // from the send callback

#endif
