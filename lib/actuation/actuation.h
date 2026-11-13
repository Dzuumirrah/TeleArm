#ifndef ACTUATION_H
#define ACTUATION_H

// Slave hardware layer: A4988 steppers, limit switches, E_BTN, AS5600 encoders behind a TCA9548A.
// Units here are raw (motor microsteps, 12-bit encoder counts). Degrees live in the session layer.
// Joint index: 0 base, 1 shoulder, 2 elbow. Pins and settings: configurations.h ("Slave side").

#include <stdint.h>

// ---- steppers, switches                                         (actuators.cpp)
bool actuatorsInit();                    // pins, step engine; leaves the drivers ENABLED (holding torque)
void driversEnable(bool on);             // EN# on GPIO32
void stepperSetSpeedAccel(int j, uint32_t speedHz, uint32_t accelHz2);  // used by the next move/run call
void stepperMoveTo(int j, int32_t pos);  // absolute target in microsteps (may be called while moving)
void stepperRun(int j, int dir);         // run continuously, dir +1 / -1
void stepperStop(int j);                 // decelerate to a stop
void stepperHalt(int j);                 // stop immediately (no ramp)
bool stepperRunning(int j);
int32_t stepperPos(int j);               // microsteps, positive = positive joint angle
void stepperSetPos(int j, int32_t pos);  // redefine the current position; only call while stopped

bool limitActive(int j);                 // raw: true = pressed or wire broken
bool estopActive();                      // raw: true = pressed or wire broken

// ---- AS5600 encoders                                             (encoders.cpp)
const uint8_t AS5600_MH = 0x08;          // STATUS: magnet too strong
const uint8_t AS5600_ML = 0x10;          // STATUS: magnet too weak
const uint8_t AS5600_MD = 0x20;          // STATUS: magnet detected
const uint8_t ENC_NO_ANSWER = 0xFF;

bool    encodersInit();                  // I2C bus + multiplexer; false if the TCA9548A does not answer
uint8_t encoderStatus(int j);            // AS5600 STATUS bits (MD/ML/MH), ENC_NO_ANSWER on I2C error
bool    encoderRaw(int j, uint16_t &raw);// RAW ANGLE 0..4095, false on I2C error

#endif
