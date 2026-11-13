#ifndef SLAVE_SESSION_H
#define SLAVE_SESSION_H

// Slave session: receives the master's frames over ESP-NOW, homes the arm, follows the joint
// commands with the steppers, checks them with the AS5600 encoders, handles E_BTN and the limit
// switches, and reports to the PC on the USB serial port. Called from src/slave/slave.cpp.

#include <Arduino.h>
#include "connections.h"

enum SlaveState : uint8_t { SS_BOOT = 0, SS_HOMING, SS_RUN, SS_ESTOP, SS_FAULT };
enum HomePhase  : uint8_t { HP_LEAVE = 0, HP_SEEK, HP_LATCH, HP_PARK };

bool slaveBegin();          // call once from setup(); false if steppers / ESP-NOW failed to start
void slaveStep();           // call from loop() as often as possible (work runs at 200 Hz)

// ---- introspection (tests / debug)
SlaveState slaveState();
bool  slaveHomed(int j);
bool  slaveEncoderVerified(int j);
float slaveStepDeg(int j);  // joint angle from the step counter
float slaveEncDeg(int j);   // joint angle from the encoder (relative to power-up until homed, NAN without encoder)
float slaveTargetDeg(int j);
uint16_t slaveResyncCount(int j);

// =============================================================================
//  Slave -> PC binary frames (PC_OUTPUT_BINARY = true). Same start byte and CRC-8 as the master frames.
//  In binary mode every valid master frame (JOINTS 1, LOG 2, STATUS 3) is forwarded unchanged,
//  so the PC also gets both segment quaternions from the JointPacket.
// =============================================================================
const uint8_t PKT_TYPE_SLAVE_REPORT = 0x10;
const uint8_t PKT_TYPE_SLAVE_LOG    = 0x11;   // same layout as the master LOG frame

// SlaveReport.flags
const uint8_t SF_HOMED        = 1 << 0;   // all joints homed
const uint8_t SF_LINK_OK      = 1 << 1;   // joint packets arriving
const uint8_t SF_ENC0_OK      = 1 << 2;   // encoder j verified: SF_ENC0_OK << j
const uint8_t SF_MASTER_IMU   = 1 << 5;   // last JointPacket had PKT_F_IMU_FAULT
// SlaveReport.inputs
const uint8_t SI_LIMIT0       = 1 << 0;   // limit switch j active: SI_LIMIT0 << j
const uint8_t SI_ESTOP        = 1 << 3;

struct __attribute__((packed)) SlaveReport {
    uint8_t  start;          // PKT_START
    uint8_t  type;           // PKT_TYPE_SLAVE_REPORT
    uint16_t seq;
    uint32_t t_ms;           // slave millis()
    uint8_t  state;          // SlaveState
    uint8_t  detail;         // while homing: (joint << 4) | HomePhase
    uint8_t  flags;          // SF_*
    uint8_t  inputs;         // SI_* (debounced)
    uint16_t masterSeq;      // seq of the last JointPacket
    float    target_deg[3];  // command being followed (clamped to JOINT_MIN/MAX_DEG)
    float    step_deg[3];    // position from the step counter
    float    enc_deg[3];     // position from the encoders (relative to power-up until homed, NAN if no encoder)
    uint8_t  crc;
};

#endif
