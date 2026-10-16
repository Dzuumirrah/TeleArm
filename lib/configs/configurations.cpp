#include "configurations.h"

// Pins (final) ---------------------------------------------
const int csPin_6500 = 19;
const int mosiPin_6500 = 21;
const int misoPin_6500 = 23;
const int sckPin_6500 = 22;

const int csPin_9250 = 27;
const int mosiPin_9250 = 12;
const int misoPin_9250 = 14;
const int sckPin_9250 = 13;

const int btn_cal = 26;
const int vibration_motor = 25;

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

// Human arm (placeholders; paper Table 5: upper arm 0.28, forearm 0.25 + hand 0.10 + index 0.085)
const float HUMAN_UPPER_ARM_LEN = 0.28f;
const float HUMAN_ELBOW_TO_TIP_LEN = 0.43f;

// Robot (placeholders)
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
const uint8_t SLAVE_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
const uint8_t ESPNOW_CHANNEL = 1;

// Debug ----------------------------------------------------
const bool DEBUG_CSV = true;
