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
extern const int LED_BUILTIN;         // built-in led pin (for ESP32 DevKitC, GPIO 2) 
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

// Remote monitor (master serial output -> slave over ESP-NOW) ====
extern const bool     REMOTE_LOG_ENABLED;
extern const uint8_t  REMOTE_LOG_MIN_LEVEL;        // 0 debug, 1 info, 2 warn, 3 error
extern const uint32_t REMOTE_LOG_MIN_INTERVAL_MS;  // at most one log frame per interval (keeps the 50 Hz joint stream clear)
extern const uint32_t STATUS_PERIOD_MS;            // STATUS heartbeat
extern const int      REMOTE_LOG_REPLAY_LINES;     // re-sent (marked as replay) when the link comes back
extern const int      LINK_DOWN_FAILS;             // consecutive failed sends before the link counts as down

// Debug ====================================================
extern const bool DEBUG_CSV;                // print a CSV status line at 10 Hz on USB serial
extern const int LED_BLINK_INT;

// Slave side ===============================================
// Joint index everywhere: 0 = base (U2 / A1 / S1), 1 = shoulder (U3 / A2 / S2), 2 = elbow (U4 / A3 / S3).
// Joint angles follow the master's JointPacket convention (see cartesian.cpp); JOINT_MIN/MAX_DEG above are
// the soft limits on both sides.
// Pins -- from Schematic_Slave rev 1.0 (ESP32-DevKitC)
extern const int SLV_STEP_PIN[3];
extern const int SLV_DIR_PIN[3];
extern const int SLV_EN_PIN;                 // A4988 EN# of all three drivers (bodge wire), LOW = enabled
extern const int SLV_LIMIT_PIN[3];           // LOW = switch pressed OR wire broken (C->LS, NC->V, R||C to GND)
extern const int SLV_ESTOP_PIN;              // LOW = pressed OR wire broken (C->3V3, NC->E_BTN, 10k pull-down)
extern const int SLV_I2C_SDA;
extern const int SLV_I2C_SCL;
extern const uint32_t SLV_I2C_HZ;
extern const uint8_t TCA9548A_ADDR;          // A0..A2 tied to GND -> 0x70
extern const uint8_t SLV_ENC_MUX_CH[3];      // TCA9548A channel of each AS5600

// Drive train -- PLACEHOLDERS, check against the real mechanism (docs/slave_mechanism_default.png)
extern const int   MOTOR_STEPS_PER_REV;      // 200 = 1.8 deg stepper
extern const int   MICROSTEPS;               // must match the J3 jumpers (MS1..MS3 all on = 16)
extern const float GEAR_RATIO[3];            // motor turns per joint turn (joint gear teeth / motor gear teeth)
extern const float ENC_TURNS_PER_JOINT_TURN[3]; // = GEAR_RATIO if the AS5600 reads the motor shaft, 1 if it reads the joint shaft
extern const bool  DIR_INVERT[3];            // flip when a positive joint command turns the joint the wrong way

// Homing (runs at boot and after every E_BTN release) -- PLACEHOLDERS
extern const int     HOME_ORDER[3];          // joints are homed one after the other in this order
extern const int8_t  HOME_DIR[3];            // -1 / +1: direction of joint angle in which the switch lies
extern const float   HOME_SWITCH_DEG[3];     // joint angle where the switch releases; must be OUTSIDE JOINT_MIN/MAX_DEG
extern const float   HOME_MAX_TRAVEL_DEG[3]; // give up (fault) if the switch is not found within this travel
extern const float   READY_DEG[3];           // pose each joint moves to right after it is homed
extern const float   HOME_SEEK_DPS;          // joint speed toward the switch
extern const float   HOME_LATCH_DPS;         // slow joint speed away from the switch (defines the zero)
extern const float   PARK_DPS;               // joint speed from the switch to READY_DEG
extern const float   HOME_ACCEL_DPS2;

// Following the master
extern const float    RUN_MAX_DPS;           // joint speed limit while following
extern const float    RUN_ACCEL_DPS2;
extern const uint32_t LINK_TIMEOUT_MS;       // no joint packet for this long -> decelerate and hold
extern const uint32_t ESTOP_RELEASE_MS;      // E_BTN must stay released this long before homing starts

// AS5600 closed loop (step-count correction)
extern const float    ENC_RESYNC_DEG;        // standstill error that triggers a step-counter correction
extern const uint32_t ENC_RESYNC_MS;         // ... when it persists this long
extern const float    ENC_STALL_DEG;         // error that means a stall / big step loss -> fault
extern const float    ENC_CHECK_MIN_DEG;     // minimum park move to verify the encoder after homing
extern const float    ENC_RATIO_TOL;         // |encoder motion / step motion| must be 1 +- this

// PC serial output of the slave
extern const bool     PC_OUTPUT_BINARY;      // false: text lines (serial monitor), true: binary frames (PC software)
extern const uint32_t PC_TEXT_PERIOD_MS;     // text status line period
extern const uint32_t PC_REPORT_PERIOD_MS;   // binary SlaveReport period

#endif
