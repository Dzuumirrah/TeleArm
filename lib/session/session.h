#ifndef SESSION_H
#define SESSION_H

#include <Arduino.h>
#include "kinematics.h"
#include "connections.h"

// =============================================================================
//  Master-side runtime: fixed-rate fusion loop, calibration state machine, joint pipeline
//
//  Pipeline per tick (SAMPLE_RATE_HZ):
//    readIMUs -> applySensorCalibration -> Madgwick (9-axis 9250 / 6-axis 6500)
//    -> segmentOrientation (mount + heading anchoring)  -> Kurpath axis vectors
//    -> fingertip position (P frame) -> robot target -> IK -> limits -> JointPacket (ESP-NOW)
//
//  Calibration (button on btn_cal, active LOW, released = action):
//    short press            QUICK  : pose 1, refresh gyro bias + re-anchor the 6500 heading to the 9250
//    hold >= BTN_LONG_MS    FULL   : pose 1 then pose 2, stores mounting + heading anchoring
//    hold >= BTN_VERYLONG_MS  MAG  : 20 s magnetometer rotation, then FULL
//    power on with button held >= BTN_LONG_MS : BENCH offsets (IMUs flat, z up)
//  Pose 1: arm hanging straight down, elbow straight, still.
//  Pose 2: arm straight forward at shoulder height (horizontal), elbow straight, still.
//  A press during a calibration aborts it. Nothing is transmitted until the arm is calibrated.
// =============================================================================

enum SessionState : uint8_t {
    ST_RUN = 0,
    ST_CAL_COUNTDOWN,     // get into the pose
    ST_CAL_HOLD,          // pose held still, averaging
    ST_MAG_CAL            // rotate the 9250 in all directions
};

bool sessionBegin();      // false if an IMU or ESP-NOW failed to start (the loop still runs and reports)
void sessionStep();       // call from loop() as often as possible
bool IsMPUSynced();       // both segments calibrated and heading-anchored in this power cycle

// introspection (tests / debugging)
SessionState sessionState();
const CalibrationData &sessionCalibration();
void sessionJointCommand(float jointDeg[3]);     // last command that was transmitted (or computed)
joints sessionHumanJoints();                     // Kurpath a, b, d of the last tick
xyzFloat sessionRobotTarget();                   // last pen target [m]

#endif
