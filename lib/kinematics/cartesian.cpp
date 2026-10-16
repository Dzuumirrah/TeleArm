#include "kinematics.h"
#include "configurations.h"
#include <math.h>

static const float DEG2RADF = 0.017453292519943295f;
static const float RAD2DEGF = 57.29577951308232f;

// Fingertip in P (origin = shoulder). Wrist is assumed straight, so the fingertip lies on the
// forearm axis. The IMU position on the segment does not matter, only the segment orientation.
xyzFloat humanFingertip(const axisVector &up, const axisVector &fo) {
    return up.k * HUMAN_UPPER_ARM_LEN + fo.k * HUMAN_ELBOW_TO_TIP_LEN;
}

// P (X right, Y up, Z back) -> robot (x out/forward, y left, z up).
// Work-centre pose: upper arm hanging, forearm horizontal forward -> pen at ROBOT_CENTER.
xyzFloat mapHumanToRobot(const xyzFloat &tipP) {
    const xyzFloat centerP(0.0f, -HUMAN_UPPER_ARM_LEN, -HUMAN_ELBOW_TO_TIP_LEN);
    const xyzFloat d = tipP - centerP;
    return xyzFloat(ROBOT_CENTER_X - MAP_SCALE * d.z,
                    ROBOT_CENTER_Y - MAP_SCALE * d.x,
                    ROBOT_CENTER_Z + MAP_SCALE * d.y);
}

bool robotInverseKinematics(const xyzFloat &t, float j[3]) {
    const float L1 = ROBOT_LINK1_LEN, L2 = ROBOT_LINK2_LEN;
    const float r = sqrtf(t.x * t.x + t.y * t.y);
    const float zp = t.z - ROBOT_BASE_HEIGHT;
    if (r > 1e-4f) j[0] = atan2f(t.y, t.x) * RAD2DEGF;          // else keep the caller's value

    const float d2 = r * r + zp * zp;
    float c3 = (d2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    bool reachable = true;
    if (c3 > 1.0f)  { c3 = 1.0f;  reachable = false; }          // too far: stretch toward it
    if (c3 < -1.0f) { c3 = -1.0f; reachable = false; }          // too close: fold
    const float th3 = acosf(c3);                                // elbow-up
    const float th2 = atan2f(r, zp) - atan2f(L2 * sinf(th3), L1 + L2 * cosf(th3));
    j[1] = th2 * RAD2DEGF;
    j[2] = th3 * RAD2DEGF;
    return reachable;
}

xyzFloat robotForwardKinematics(const float j[3]) {
    const float th1 = j[0] * DEG2RADF, th2 = j[1] * DEG2RADF, th3 = j[2] * DEG2RADF;
    const float r = ROBOT_LINK1_LEN * sinf(th2) + ROBOT_LINK2_LEN * sinf(th2 + th3);
    const float z = ROBOT_BASE_HEIGHT + ROBOT_LINK1_LEN * cosf(th2) + ROBOT_LINK2_LEN * cosf(th2 + th3);
    return xyzFloat(r * cosf(th1), r * sinf(th1), z);
}

uint8_t limitJointAngles(float j[3], const float prev[3], float dt) {
    uint8_t flags = 0;
    const float maxStep = MAX_JOINT_RATE_DPS * dt;
    for (int i = 0; i < 3; i++) {
        if (!isfinite(j[i])) j[i] = prev[i];
        if (j[i] > JOINT_MAX_DEG[i]) { j[i] = JOINT_MAX_DEG[i]; flags |= 1; }
        if (j[i] < JOINT_MIN_DEG[i]) { j[i] = JOINT_MIN_DEG[i]; flags |= 1; }
        const float dlt = j[i] - prev[i];
        if (dlt > maxStep)       { j[i] = prev[i] + maxStep; flags |= 2; }
        else if (dlt < -maxStep) { j[i] = prev[i] - maxStep; flags |= 2; }
    }
    return flags;
}
