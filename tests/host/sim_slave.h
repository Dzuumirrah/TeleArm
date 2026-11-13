#pragma once
#include <stdint.h>

struct SimJoint {
    enum Mode { IDLE, MOVE, RUN, DECEL } mode = IDLE;
    int32_t pos = 0, target = 0;        // step counter as the firmware sees it
    int dir = 1;
    uint32_t speed = 1, accel = 1;
    float v = 0, frac = 0;
    float trueDeg = 0;                  // real joint angle
    bool enabled = false;
    int encSign = 1; float encScale = 1, encOffset = 0; bool encPresent = true;
    bool switchWorks = true, pressed = false; float hyst = 0.3f;
    int moves = 0;
};
extern SimJoint simJ[3];
extern bool simEstop;
void simReset(const float trueDeg[3]);
void simStep(float dt);
void simSlip(int j, int32_t microsteps);   // lost steps: the joint moves without the step counter knowing
