#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

#include <stdint.h>

// Master side  =============================================
// MPU6500 (forearm) -- pins FINAL, do not change
extern const int csPin_6500;    // Chip Select Pin
extern const int mosiPin_6500;  // "MOSI" Pin
extern const int misoPin_6500;  // "MISO" Pin
extern const int sckPin_6500;   // SCK Pin

// MPU9250 (upper arm) -- pins FINAL, do not change
extern const int csPin_9250;    // Chip Select Pin
extern const int mosiPin_9250;  // "MOSI" Pin
extern const int misoPin_9250;  // "MISO" Pin
extern const int sckPin_9250;   // SCK Pin

// Others
extern const int btn_cal;           // MPU calibration button (wired to GND, internal pull-up)
extern const int vibration_motor;   // Vibration motor pin (or led pin) -- not used yet

extern const bool useSPI;           // SPI use flag

// Sampling =================================================
extern const int   SAMPLE_RATE_HZ;       // fusion loop rate (DLPF 4, +-500 dps, +-2 g are set in MPUInit)
extern const int   SEND_RATE_HZ;         // ESP-NOW joint packet rate
extern const float DT_MIN_S;             // measured dt is clamped to [DT_MIN_S, DT_MAX_S]
extern const float DT_MAX_S;

// Fusion ===================================================
extern const float MADGWICK_BETA_9250;   // gradient-descent gain, 9-axis (mag) mode
extern const float MADGWICK_BETA_6500;   // gradient-descent gain, 6-axis mode
extern const float MADGWICK_BETA_BOOST;  // fast-convergence gain: calibration poses and the first seconds after a (re)seed
extern const uint32_t BOOST_AFTER_SEED_MS;
extern const bool  USE_MAG_9250;         // use AK8963 for the 9250 heading (north reference)

// Optional continuous yaw correction of the MPU6500 from the elbow-hinge constraint
extern const bool  USE_HINGE_YAW_CORRECTION;
extern const float HINGE_YAW_GAIN;           // fraction of the estimated error corrected per update
extern const float HINGE_MIN_OBSERVABILITY;  // |d| below this -> heading unobservable, no correction
extern const float HINGE_MAX_STEP_RAD;       // max correction per update

// Calibration procedure ====================================
extern const uint32_t BTN_DEBOUNCE_MS;
extern const uint32_t BTN_LONG_MS;       // hold  -> full calibration (pose 1 + pose 2)
extern const uint32_t BTN_VERYLONG_MS;   // hold  -> magnetometer calibration, then full calibration
extern const uint32_t CAL_COUNTDOWN_MS;  // time to get into a pose after pressing
extern const uint32_t CAL_HOLD_MS;       // time the pose is averaged (must stay still)
extern const uint32_t MAG_CAL_MS;        // magnetometer figure-eight duration
extern const float    CAL_STILL_GYRO_DPS;// max gyro std-dev while holding a pose
extern const float    QUICK_POSE_TOL_DEG; // QUICK is refused if a limb is further than this from hanging vertically
extern const float    CAL_SETTLE_TILT_DEG;// max mean angle between filter gravity and accelerometer while holding a pose

// Human arm (metres) -- PLACEHOLDERS, modify later ==========
extern const float HUMAN_UPPER_ARM_LEN;     // shoulder -> elbow
extern const float HUMAN_ELBOW_TO_TIP_LEN;  // elbow -> chosen fingertip, measured along the forearm axis

// Robot (metres, degrees) -- PLACEHOLDERS, modify later =====
extern const float ROBOT_BASE_HEIGHT;       // base plane -> shoulder axis
extern const float ROBOT_LINK1_LEN;         // shoulder axis -> elbow axis
extern const float ROBOT_LINK2_LEN;         // elbow axis -> pen tip
extern const float ROBOT_CENTER_X;          // pen position when the operator holds the work-centre pose
extern const float ROBOT_CENTER_Y;
extern const float ROBOT_CENTER_Z;
extern const float MAP_SCALE;               // robot motion = MAP_SCALE * fingertip motion
extern const float JOINT_MIN_DEG[3];        // base, shoulder, elbow
extern const float JOINT_MAX_DEG[3];
extern const float MAX_JOINT_RATE_DPS;      // slew limit applied before sending

// ESP-NOW ==================================================
extern const uint8_t SLAVE_MAC[6];          // FF:FF:FF:FF:FF:FF = broadcast (dev default)
extern const uint8_t ESPNOW_CHANNEL;

// Debug ====================================================
extern const bool DEBUG_CSV;                // print a CSV status line at 10 Hz on USB serial

#endif
