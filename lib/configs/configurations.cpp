#include "configurations.h"

// Pins (final) ---------------------------------------------
const int csPin_6500 = 19;
const int mosiPin_6500 = 21;
const int misoPin_6500 = 23;
const int sckPin_6500 = 22;

const int csPin_9250 = 27;
const int mosiPin_9250 = 33;    // previously 12
const int misoPin_9250 = 14;
const int sckPin_9250 = 13;

const int btn_cal = 26;     //changed to pull down   
const int vibration_motor = 25;
const int LED_BUILTIN = 2;  // built-in led pin (for ESP32 DevKitC, GPIO 2)
const int LED_BLINK_INT = 500;  // LED blink interval in milliseconds

const bool useSPI = true;

// Sampling -------------------------------------------------
const int   SAMPLE_RATE_HZ = 200;
const int   SEND_RATE_HZ   = 50;
const float DT_MIN_S = 0.001f;
const float DT_MAX_S = 0.020f;

// Fusion ---------------------------------------------------
const float MADGWICK_BETA_9250 = 0.05f;
const float MADGWICK_BETA_6500 = 0.03f;
const float MADGWICK_BETA_BOOST = 0.4f;
const uint32_t BOOST_AFTER_SEED_MS = 3000;
const bool  USE_MAG_9250 = true;

const bool  USE_HINGE_YAW_CORRECTION = false;   // experimental, see kinematics.h
const float HINGE_YAW_GAIN = 0.002f;
const float HINGE_MIN_OBSERVABILITY = 0.35f;
const float HINGE_MAX_STEP_RAD = 0.002f;

// Calibration ----------------------------------------------
const uint32_t BTN_DEBOUNCE_MS = 30;
const uint32_t BTN_LONG_MS = 2000;
const uint32_t BTN_VERYLONG_MS = 6000;
const uint32_t CAL_COUNTDOWN_MS = 3000;
const uint32_t CAL_HOLD_MS = 2000;
const uint32_t MAG_CAL_MS = 20000;
const float    CAL_STILL_GYRO_DPS = 3.0f;
const float    CAL_SETTLE_TILT_DEG = 1.5f;
const float    QUICK_POSE_TOL_DEG = 10.0f;

// Human arm (in metres; Fariz hands; paper Table 5: upper arm 0.28, forearm 0.25 + hand 0.10 + index 0.085)
const float HUMAN_UPPER_ARM_LEN = 0.30f;
const float HUMAN_ELBOW_TO_TIP_LEN = 0.41f;

// Robot (in metres; placeholders)
const float ROBOT_BASE_HEIGHT = 0.10f;
const float ROBOT_LINK1_LEN = 0.30f;
const float ROBOT_LINK2_LEN = 0.30f;
const float ROBOT_CENTER_X = 0.30f;
const float ROBOT_CENTER_Y = 0.00f;
const float ROBOT_CENTER_Z = 0.10f;
const float MAP_SCALE = 0.5f;
const float JOINT_MIN_DEG[3] = { -90.0f, -30.0f,   0.0f };
const float JOINT_MAX_DEG[3] = {  90.0f, 120.0f, 150.0f };
const float MAX_JOINT_RATE_DPS = 180.0f;

// ESP-NOW --------------------------------------------------
const uint8_t SLAVE_MAC[6] = {0x5C, 0x01, 0x3B, 0x4B, 0x6B, 0x34};
const uint8_t ESPNOW_CHANNEL = 1;

// Remote monitor -------------------------------------------
const bool     REMOTE_LOG_ENABLED = true;
const uint8_t  REMOTE_LOG_MIN_LEVEL = 1;           // INFO and above
const uint32_t REMOTE_LOG_MIN_INTERVAL_MS = 20;
const uint32_t STATUS_PERIOD_MS = 1000;
const int      REMOTE_LOG_REPLAY_LINES = 24;
const int      LINK_DOWN_FAILS = 3;

// Debug ----------------------------------------------------
const bool DEBUG_CSV = true;

// ===================== Slave side =========================
// Pins (Schematic_Slave rev 1.0) ----------------------------
const int SLV_STEP_PIN[3]  = { 23, 18, 16 };    // U2, U3, U4 STEP
const int SLV_DIR_PIN[3]   = { 19, 17,  4 };    // U2, U3, U4 DIR
const int SLV_EN_PIN       = 32;                // A4988_EN (EN# bodge)
const int SLV_LIMIT_PIN[3] = { 25, 26, 27 };    // LS1, LS2, LS3
const int SLV_ESTOP_PIN    = 33;                // E_BTN
const int SLV_I2C_SDA      = 21;                // AS-SDA
const int SLV_I2C_SCL      = 22;                // AS-SCL
const uint32_t SLV_I2C_HZ  = 400000;
const uint8_t TCA9548A_ADDR = 0x70;
const uint8_t SLV_ENC_MUX_CH[3] = { 0, 1, 2 };  // SD0/SC0 -> S1, SD1/SC1 -> S2, SD2/SC2 -> S3

// Drive train (placeholders) --------------------------------
const int   MOTOR_STEPS_PER_REV = 200;
const int   MICROSTEPS = 16;
const float GEAR_RATIO[3] = { 3.0f, 3.0f, 3.0f };                 // 20T motor gear -> 60T joint gear
const float ENC_TURNS_PER_JOINT_TURN[3] = { 3.0f, 3.0f, 3.0f };   // AS5600 behind the motor (motor shaft)
const bool  DIR_INVERT[3] = { false, false, false };

// Homing (placeholders) -------------------------------------
const int    HOME_ORDER[3]          = { 2, 1, 0 };                // elbow, shoulder, base
const int8_t HOME_DIR[3]            = { -1, -1, -1 };
const float  HOME_SWITCH_DEG[3]     = { -100.0f, -40.0f, -10.0f };
const float  HOME_MAX_TRAVEL_DEG[3] = {  220.0f, 190.0f, 190.0f };
const float  READY_DEG[3]           = {    0.0f,  30.0f, 120.0f }; // = robot work centre of cartesian.cpp
const float  HOME_SEEK_DPS   = 15.0f;
const float  HOME_LATCH_DPS  = 3.0f;
const float  PARK_DPS        = 30.0f;
const float  HOME_ACCEL_DPS2 = 100.0f;

// Following the master ---------------------------------------
const float    RUN_MAX_DPS      = 120.0f;
const float    RUN_ACCEL_DPS2   = 300.0f;
const uint32_t LINK_TIMEOUT_MS  = 300;
const uint32_t ESTOP_RELEASE_MS = 1000;

// AS5600 closed loop ----------------------------------------
const float    ENC_RESYNC_DEG    = 0.8f;
const uint32_t ENC_RESYNC_MS     = 200;
const float    ENC_STALL_DEG     = 8.0f;
const float    ENC_CHECK_MIN_DEG = 10.0f;
const float    ENC_RATIO_TOL     = 0.25f;

// PC serial output ------------------------------------------
const bool     PC_OUTPUT_BINARY    = false;
const uint32_t PC_TEXT_PERIOD_MS   = 100;
const uint32_t PC_REPORT_PERIOD_MS = 20;
