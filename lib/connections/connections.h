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
bool MPUInit();

// Read both IMUs back to back. Units: g, deg/s, uT (raw AK8963 axes, not calibrated).
// Returns false if a sample is not plausible (disconnected / corrupted SPI read).
bool readIMUs(IMUData &imu9250, IMUData &imu6500);

// true when the AK8963 answered in MPUInit()
bool magnetometerAvailable();

// What MPUInit() found (reported in the STATUS heartbeat so the slave knows the hardware state even if the boot log was missed).
struct ImuHardwareInfo {
    bool upperOk, foreOk;        // chip answered during MPUInit()
    bool upperIs9250;            // false = an MPU6500 sits in the upper-arm slot
    bool magOk;                  // AK8963 answered
    bool benchLoaded;            // bench offsets found in flash
};
ImuHardwareInfo imuHardwareInfo();

// Bench offset calibration with the library's autoOffsets(): BOTH IMUs lying flat, z up, still.
// Offsets are stored in flash and re-applied by MPUInit().
bool benchCalibrateIMUs();

// Blink the built-in LED at a fixed interval. The LED is used to indicate various states (e.g., calibration, errors).
// @param lastBlinkMs: the last time the LED was toggled (in milliseconds).
// @param on: whether the LED should be on or off.
// @param type: the type of blink (can be used to indicate different states). 0, slow, 1 = normal, 2 = fast
void LEDBlink(bool on, uint32_t lastBlinkMs = 0, uint8_t type = 0);
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

// ---- link state (derived from the unicast MAC ACK reported by the send callback) ----
// "up" until LINK_DOWN_FAILS sends in a row failed; comes back up on the next ACK.
// With a broadcast SLAVE_MAC there is no ACK, so the link always looks up.
bool ESPNowReady();                                    // ESPNowInit() succeeded
bool ESPNowLinkUp();
bool ESPNowTakeLinkUpEvent();                          // true once per down->up (or first) transition
bool ESPNowSendRaw(const uint8_t *data, size_t len);   // no framing added (used by log / status)

// =============================================================================
//  Master -> slave remote monitor: LOG and STATUS frames           (remotelog.cpp)
//
//  Everything the master prints for a human (calibration prompts, init results, errors) is
//  also sent over ESP-NOW so the laptop attached to the SLAVE sees it.
//
//  LOG frame (variable, 11 + len bytes, <= 171):
//    0 start 0xA5 | 1 type 2 | 2..3 seq u16 | 4..7 t_ms u32 | 8 level (bit7 = replayed) | 9 len | 10.. text | crc8
//  STATUS frame (fixed 23 bytes, sent every STATUS_PERIOD_MS, also while no calibration exists):
//    see StatusPacket
//  Both use the same start byte and CRC-8 as JointPacket; the type byte tells them apart.
// =============================================================================
const uint8_t PKT_TYPE_LOG    = 2;
const uint8_t PKT_TYPE_STATUS = 3;

enum LogLevel : uint8_t { LOGLVL_DEBUG = 0, LOGLVL_INFO = 1, LOGLVL_WARN = 2, LOGLVL_ERROR = 3 };
const uint8_t LOG_REPLAY_BIT = 0x80;
const size_t  LOG_MAX_TEXT   = 160;                    // longer messages are truncated
const int     LOG_RING_LINES = 48;                     // queue / history depth (about 8 KB of RAM)
const size_t  LOG_HEADER_LEN = 10;                     // bytes before the text

// STATUS hw bits
const uint8_t HW_UPPER_OK    = 1 << 0;
const uint8_t HW_FORE_OK     = 1 << 1;
const uint8_t HW_UPPER_9250  = 1 << 2;   // 0 = MPU6500 in the upper slot
const uint8_t HW_MAG_OK      = 1 << 3;   // AK8963 answered
const uint8_t HW_CAL_VALID   = 1 << 4;   // both segment calibrations valid
const uint8_t HW_ANCHORED    = 1 << 5;   // heading anchored this power cycle (== transmitting)
const uint8_t HW_MAG_USED    = 1 << 6;   // 9250 heading uses the magnetometer
const uint8_t HW_BENCH_OK    = 1 << 7;   // bench offsets loaded from flash

struct __attribute__((packed)) StatusPacket {
    uint8_t  start;        // PKT_START
    uint8_t  type;         // PKT_TYPE_STATUS
    uint16_t seq;
    uint32_t t_ms;         // master millis()
    uint8_t  state;        // SessionState
    uint8_t  plan;         // calibration plan (0 none, 1 quick, 2 full, 3 mag+full)
    uint8_t  hw;           // HW_*
    uint8_t  flags;        // last PKT_F_* of the joint packet
    uint32_t sentOk;       // ESP-NOW sends that were acknowledged (all frame types)
    uint32_t sentFail;
    uint16_t logDropped;   // log lines lost on the master (queue overflow)
    uint8_t  crc;
};

struct LogFrame {          // decoded LOG frame
    uint16_t seq;
    uint32_t t_ms;
    uint8_t  level;
    bool     replay;
    uint8_t  len;
    char     text[LOG_MAX_TEXT + 1];
};

// ---- master side ----
void logInit();                                        // clear the queue (called by sessionBegin)
// printf-style. Always printed on the USB serial port; queued for ESP-NOW when REMOTE_LOG_ENABLED
// and level >= REMOTE_LOG_MIN_LEVEL. Do NOT include a trailing newline. Never blocks, never sends.
void logMsg(LogLevel level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void logService();                                     // sends queued lines (rate limited); call every loop pass
void logFlush(uint32_t timeoutMs);                     // send everything queued now (before long blocking steps)
uint16_t logDroppedCount();
uint16_t logPendingCount();                            // queued, not sent yet
bool ESPNowSendStatus(StatusPacket &p);                // fills start/type/seq/t_ms-independent fields, counters, crc

// ---- both sides (the slave sketch and the host tests use these) ----
uint8_t packetType(const uint8_t *data, int len);      // PKT_TYPE_* of a frame with valid length/start/CRC, else 0
bool    decodeLogPacket(const uint8_t *data, int len, LogFrame &out);
bool    decodeStatusPacket(const uint8_t *data, int len, StatusPacket &out);
// Text lines for the slave's serial port ("[M 12.345 W] [CAL] ..." / "[M STATUS] ...")
size_t  formatLogLine(const LogFrame &f, char *out, size_t outSize);
size_t  formatStatusLine(const StatusPacket &s, char *out, size_t outSize);

#endif
