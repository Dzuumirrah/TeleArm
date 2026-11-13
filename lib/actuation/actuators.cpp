#include <Arduino.h>
#include <FastAccelStepper.h>
#include "actuation.h"
#include "configurations.h"

// Step pulses come from the ESP32 hardware (FastAccelStepper: MCPWM/PCNT), so the
// 200 Hz control loop never has to time individual steps.
static FastAccelStepperEngine engine;
static FastAccelStepper *st[3] = { nullptr, nullptr, nullptr };

bool actuatorsInit() {
    digitalWrite(SLV_EN_PIN, HIGH);             // drivers off while the STEP/DIR pins are set up
    pinMode(SLV_EN_PIN, OUTPUT);
    for (int j = 0; j < 3; j++) pinMode(SLV_LIMIT_PIN[j], INPUT);   // external R||C pull-down on the LS line
    pinMode(SLV_ESTOP_PIN, INPUT);                                  // external 10k pull-down (R9)

    engine.init();
    bool ok = true;
    for (int j = 0; j < 3; j++) {
        st[j] = engine.stepperConnectToPin(SLV_STEP_PIN[j]);
        if (!st[j]) { ok = false; continue; }
        st[j]->setDirectionPin(SLV_DIR_PIN[j], !DIR_INVERT[j]);
        st[j]->setAutoEnable(false);
        st[j]->setCurrentPosition(0);
    }
    driversEnable(true);                        // holding torque from now on (also during E-stop)
    return ok;
}

void driversEnable(bool on) { digitalWrite(SLV_EN_PIN, on ? LOW : HIGH); }

void stepperSetSpeedAccel(int j, uint32_t speedHz, uint32_t accelHz2) {
    if (!st[j]) return;
    st[j]->setSpeedInHz(speedHz ? speedHz : 1);
    st[j]->setAcceleration(accelHz2 ? accelHz2 : 1);
}

void stepperMoveTo(int j, int32_t pos) { if (st[j]) st[j]->moveTo(pos); }
void stepperRun(int j, int dir)        { if (st[j]) { if (dir > 0) st[j]->runForward(); else st[j]->runBackward(); } }
void stepperStop(int j)                { if (st[j]) st[j]->stopMove(); }
void stepperHalt(int j)                { if (st[j]) st[j]->forceStop(); }
bool stepperRunning(int j)             { return st[j] && st[j]->isRunning(); }
int32_t stepperPos(int j)              { return st[j] ? st[j]->getCurrentPosition() : 0; }
void stepperSetPos(int j, int32_t pos) { if (st[j]) st[j]->setCurrentPosition(pos); }

bool limitActive(int j) { return digitalRead(SLV_LIMIT_PIN[j]) == LOW; }
bool estopActive()      { return digitalRead(SLV_ESTOP_PIN) == LOW; }
