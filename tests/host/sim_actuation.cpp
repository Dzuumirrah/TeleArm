// Simulated slave hardware for the host tests: replaces lib/actuation (actuators.cpp + encoders.cpp).
// Each joint has a "true" angle driven by the simulated motor; the limit switch and the AS5600 are
// derived from that true angle, so the firmware only sees what real hardware would show it.
#include <math.h>
#include <stdlib.h>
#include "actuation.h"
#include "configurations.h"
#include "sim_slave.h"

SimJoint simJ[3];
bool simEstop = false;

static float spdOf(int j) { return (float)MOTOR_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO[j] / 360.0f; }

void simReset(const float trueDeg[3]) {
    simEstop = false;
    for (int j = 0; j < 3; j++) {
        SimJoint &s = simJ[j];
        s = SimJoint();
        s.trueDeg = trueDeg[j];
        s.encSign = 1; s.encScale = 1.0f; s.encOffset = 1234.0f + 700.0f * j;
        s.encPresent = true; s.switchWorks = true;
        s.hyst = 0.3f;
    }
}

// motor model: trapezoid toward the target / run direction, integer microsteps
void simStep(float dt) {
    for (int j = 0; j < 3; j++) {
        SimJoint &s = simJ[j];
        const float a = (float)s.accel, vmax = (float)s.speed;
        float vWant = 0;
        if (s.mode == SimJoint::MOVE) {
            const float dist = (float)(s.target - s.pos);
            const float vStop = sqrtf(2.0f * a * fabsf(dist));
            vWant = (dist > 0 ? 1 : -1) * fminf(vmax, vStop);
            if (fabsf(dist) < 1.0f && fabsf(s.v) <= a * dt * 2) { s.v = 0; s.mode = SimJoint::IDLE; s.frac = 0; continue; }
        } else if (s.mode == SimJoint::RUN) vWant = s.dir * vmax;
        else if (s.mode == SimJoint::DECEL) {
            vWant = 0;
            if (fabsf(s.v) <= a * dt) { s.v = 0; s.mode = SimJoint::IDLE; s.frac = 0; continue; }
        } else continue;
        const float dv = a * dt;
        if (s.v < vWant) s.v = fminf(vWant, s.v + dv); else s.v = fmaxf(vWant, s.v - dv);
        s.frac += s.v * dt;
        int32_t n = (int32_t)s.frac;               // whole steps this slice
        if (s.mode == SimJoint::MOVE) {            // never overshoot the target
            const int32_t rem = s.target - s.pos;
            if ((rem >= 0 && n > rem) || (rem <= 0 && n < rem)) n = rem;
        }
        s.frac -= (float)n;
        s.pos += n;
        s.trueDeg += (float)n / spdOf(j);
    }
}

void simSlip(int j, int32_t microsteps) { simJ[j].trueDeg += (float)microsteps / spdOf(j); }

// ---- actuation.h
bool actuatorsInit() { driversEnable(true); return true; }
void driversEnable(bool on) { for (int j = 0; j < 3; j++) simJ[j].enabled = on; }
void stepperSetSpeedAccel(int j, uint32_t hz, uint32_t acc) { simJ[j].speed = hz; simJ[j].accel = acc; }
void stepperMoveTo(int j, int32_t pos) {
    simJ[j].target = pos;
    if (pos == simJ[j].pos && simJ[j].mode == SimJoint::IDLE) return;   // like FastAccelStepper: nothing to do
    simJ[j].mode = SimJoint::MOVE; simJ[j].moves++;
}
void stepperRun(int j, int dir) { simJ[j].dir = dir > 0 ? 1 : -1; simJ[j].mode = SimJoint::RUN; }
void stepperStop(int j) { if (simJ[j].mode != SimJoint::IDLE) simJ[j].mode = SimJoint::DECEL; }
void stepperHalt(int j) { simJ[j].mode = SimJoint::IDLE; simJ[j].v = 0; simJ[j].frac = 0; }
bool stepperRunning(int j) { return simJ[j].mode != SimJoint::IDLE; }
int32_t stepperPos(int j) { return simJ[j].pos; }
void stepperSetPos(int j, int32_t p) { simJ[j].pos = p; if (simJ[j].mode == SimJoint::MOVE) simJ[j].mode = SimJoint::IDLE; }

// switch on the HOME_DIR side: presses hyst degrees past HOME_SWITCH_DEG, releases at HOME_SWITCH_DEG (true frame)
bool limitActive(int j) {
    SimJoint &s = simJ[j];
    if (!s.switchWorks) return false;
    const float beyond = (s.trueDeg - HOME_SWITCH_DEG[j]) * HOME_DIR[j];   // > 0: past the release point
    if (s.pressed) { if (beyond < 0) s.pressed = false; }
    else if (beyond >= s.hyst) s.pressed = true;
    return s.pressed;
}
bool estopActive() { return simEstop; }

bool encodersInit() { return true; }
uint8_t encoderStatus(int j) { return simJ[j].encPresent ? AS5600_MD : ENC_NO_ANSWER; }
bool encoderRaw(int j, uint16_t &raw) {
    SimJoint &s = simJ[j];
    if (!s.encPresent) return false;
    const float counts = s.encOffset + s.encSign * s.encScale * s.trueDeg * 4096.0f * ENC_TURNS_PER_JOINT_TURN[j] / 360.0f;
    long c = lroundf(counts) % 4096; if (c < 0) c += 4096;
    raw = (uint16_t)c;
    return true;
}
