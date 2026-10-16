#include "session.h"
#include "configurations.h"
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------------------------
//  state
// ---------------------------------------------------------------------------------
namespace {

enum Plan : uint8_t { PLAN_NONE, PLAN_QUICK, PLAN_FULL, PLAN_MAG_THEN_FULL };

const int MAG_MAX_SAMPLES = 600;
const uint32_t CAL_TIMEOUT_MS = 25000;           // whole pose sequence must finish within this
const int FAULT_TICKS = 25;                      // consecutive bad reads before it is reported (~125 ms)
const int FAULT_FLAG_TICKS = 100;               // PKT_F_IMU_FAULT stays set this long after a read failure (0.5 s)

struct HoldAcc {
    uint32_t n;
    double   gsum[2][3];
    double   gsq[2][3];
    quaternion qref[2];
    float    qsum[2][4];
    double   tilt[2];            // sum of |angle(filter gravity, accelerometer)| [deg]
};

uint32_t periodUs = 5000;
uint32_t nextTickUs = 0, lastTickUs = 0;
bool     tickInit = false;
uint32_t tickCount = 0;

CalibrationData cal;                    // index 0 = MPU9250 (upper arm), 1 = MPU6500 (forearm)
SensorCal  *scal[2];
SegmentCal *seg[2];
quaternion  g[2];                       // Madgwick quaternions
bool        seeded[2];

IMUData     raw[2], cd[2];
int         badReads = 0;
int         faultCooldown = 0;

SessionState state = ST_RUN;
Plan        plan = PLAN_NONE;
int         poseIdx = 0;                // 1 or 2
uint32_t    stateStartMs = 0, calStartMs = 0;
HoldAcc     acc;
quaternion  g1[2], g2[2];               // averaged pose quaternions
int         lastCountdownPrint = -1;

MagSample   magBuf[MAG_MAX_SAMPLES];
int         magN = 0;

bool        btnRaw = false, btnStable = false;
uint32_t    btnChangeMs = 0, btnPressMs = 0;

bool        wasSynced = false;
uint32_t    boostUntilMs[2] = { 0, 0 };
float       prevJoint[3] = { 0, 0, 0 };
float       jointCmd[3] = { 0, 0, 0 };
joints      humanJ = { 0, 0, 0 };
xyzFloat    robotTarget;
uint8_t     lastFlags = 0;
bool        faultReported = false;

// ---------------------------------------------------------------------------------
void say(const char *s) { Serial.print("[CAL] "); Serial.println(s); }

void resetAcc() { memset(&acc, 0, sizeof(acc)); }

void accumulate() {
    for (int i = 0; i < 2; i++) {
        const double gx = raw[i].gyr.x, gy = raw[i].gyr.y, gz = raw[i].gyr.z;
        acc.gsum[i][0] += gx; acc.gsum[i][1] += gy; acc.gsum[i][2] += gz;
        acc.gsq[i][0] += gx * gx; acc.gsq[i][1] += gy * gy; acc.gsq[i][2] += gz * gz;
        quaternion q = g[i];
        if (acc.n == 0) acc.qref[i] = q;
        const float dot = q.w * acc.qref[i].w + q.x * acc.qref[i].x + q.y * acc.qref[i].y + q.z * acc.qref[i].z;
        const float s = (dot < 0.0f) ? -1.0f : 1.0f;      // q and -q are the same rotation
        {   // does the filter agree with the accelerometer? (gravity in the body frame = 3rd row of R)
            const Mat3 r = rotationFromQuaternion(q);
            const xyzFloat gp(r.m[2][0], r.m[2][1], r.m[2][2]);
            const xyzFloat gm = vunit(cd[i].acc);
            acc.tilt[i] += 57.29577951f * atan2f(vnorm(vcross(gp, gm)), vdot(gp, gm));
        }
        acc.qsum[i][0] += s * q.w; acc.qsum[i][1] += s * q.x; acc.qsum[i][2] += s * q.y; acc.qsum[i][3] += s * q.z;
    }
    acc.n++;
}

quaternion meanQuat(int i) {
    quaternion q = { acc.qsum[i][0], acc.qsum[i][1], acc.qsum[i][2], acc.qsum[i][3] };
    return qNormalize(q);
}

void clearSyncFlags(bool both) {
    seg[1]->anchored = false;
    if (both) seg[0]->anchored = false;
}

void abortCalibration(const char *why) {
    say(why);
    state = ST_RUN;
    plan = PLAN_NONE;
}

void beginPose(int idx, uint32_t nowMs) {
    poseIdx = idx;
    state = ST_CAL_COUNTDOWN;
    stateStartMs = nowMs;
    lastCountdownPrint = -1;
    if (idx == 1) say("POSE 1: arm hanging straight down, elbow straight. Hold still after the countdown.");
    else          say("POSE 2: arm straight forward at shoulder height, elbow straight. Hold still after the countdown.");
}

void startPlan(Plan p, uint32_t nowMs) {
    plan = p;
    calStartMs = nowMs;
    resetAcc();
    if (p == PLAN_MAG_THEN_FULL) {
        if (!magnetometerAvailable()) { abortCalibration("no magnetometer - cannot run magnetometer calibration"); return; }
        state = ST_MAG_CAL;
        stateStartMs = nowMs;
        magN = 0;
        say("MAGNETOMETER: rotate the upper-arm IMU slowly through all orientations (figure-eight) for 20 s.");
        return;
    }
    if (p == PLAN_QUICK && !(seg[0]->valid && seg[1]->valid && seg[0]->anchored)) {
        abortCalibration("QUICK needs a stored full calibration with the magnetometer heading - hold the button 2 s for FULL");
        return;
    }
    beginPose(1, nowMs);
}

void saveAll() {
    cal.calUsedMag = (USE_MAG_9250 && cal.s9250.magValid) ? 1 : 0;
    if (!saveCalibration(cal)) say("WARNING: could not write calibration to flash");
}

// ---- end of a pose hold ------------------------------------------------------------
void finishHold(uint32_t nowMs) {
    bool still = true;
    xyzFloat bias[2];
    for (int i = 0; i < 2; i++) {
        if (!computeGyroBias((int)acc.n, acc.gsum[i], acc.gsq[i], CAL_STILL_GYRO_DPS, bias[i])) still = false;
    }
    if (!still) {                               // moved: average again
        say("not still - hold the pose without moving");
        resetAcc();
        stateStartMs = nowMs;
        return;
    }
    for (int i = 0; i < 2; i++) {
        if (acc.tilt[i] / acc.n > CAL_SETTLE_TILT_DEG) {      // orientation filter has not converged yet
            Serial.printf("[CAL] orientation filter still settling (tilt error %.1f / %.1f deg) - keep holding the pose\n",
                          (float)(acc.tilt[0] / acc.n), (float)(acc.tilt[1] / acc.n));
            resetAcc();
            stateStartMs = nowMs;
            return;
        }
    }

    if (poseIdx == 1) {
        for (int i = 0; i < 2; i++) { g1[i] = meanQuat(i); scal[i]->gyroBias = bias[i]; }
        if (plan == PLAN_QUICK) {
            // the limbs must really hang vertically (checked with the stored mounting; independent of heading)
            const float minDown = cosf(QUICK_POSE_TOL_DEG * 0.017453293f);
            bool hanging = true;
            for (int i = 0; i < 2; i++) {
                const Mat3 r = rotationFromQuaternion(g1[i]);
                const xyzFloat kb(seg[i]->M.m[0][0], seg[i]->M.m[1][0], seg[i]->M.m[2][0]);     // limb axis in sensor coordinates
                if (mat3MulVec(r, kb).z > -minDown) hanging = false;                               // world z is up
            }
            if (!hanging) {
                say("QUICK refused: the arm must hang straight down (upper arm and forearm) - nothing was changed");
                state = ST_RUN; plan = PLAN_NONE;
                return;
            }
            if (reanchorSegment6500(g1[0], *seg[0], g1[1], *seg[1])) {
                saveAll();
                say("QUICK calibration done - forearm heading re-anchored");
            } else {
                clearSyncFlags(false);
                say("QUICK failed: was the arm really hanging straight down? try again");
            }
            state = ST_RUN; plan = PLAN_NONE;
            return;
        }
        beginPose(2, nowMs);
        resetAcc();
        return;
    }

    // pose 2 of FULL
    for (int i = 0; i < 2; i++) g2[i] = meanQuat(i);
    SegmentCal n0 = *seg[0], n1 = *seg[1];
    const bool ok0 = computeSegmentCalibration(g1[0], g2[0], n0);
    const bool ok1 = computeSegmentCalibration(g1[1], g2[1], n1);
    if (ok0 && ok1) {
        *seg[0] = n0; *seg[1] = n1;
        seg[0]->anchored = true; seg[1]->anchored = true;
        saveAll();
        say("FULL calibration done");
    } else {
        say("FULL failed: pose 2 must be an arm raised straight forward (horizontal). try again");
    }
    state = ST_RUN; plan = PLAN_NONE;
}

void finishMagCal(uint32_t nowMs) {
    SensorCal c = cal.s9250;
    if (!computeMagCalibration(magBuf, magN, c)) {
        abortCalibration("magnetometer calibration failed (not enough rotation / inconsistent data) - repeat");
        return;
    }
    cal.s9250 = c;
    cal.seg9250.valid = false; cal.seg9250.anchored = false;     // heading reference changed
    cal.seg6500.valid = false; cal.seg6500.anchored = false;
    seeded[0] = false;                                           // reseed with the new heading
    boostUntilMs[0] = 0;
    saveAll();
    say("magnetometer calibrated");
    plan = PLAN_FULL;
    calStartMs = nowMs;                        // the pose sequence has its own timeout
    resetAcc();
    beginPose(1, nowMs);
}

// ---- button --------------------------------------------------------------------------
void onRelease(uint32_t heldMs, uint32_t nowMs) {
    if (state != ST_RUN) { abortCalibration("aborted by button"); return; }
    if (heldMs >= BTN_VERYLONG_MS)      startPlan(PLAN_MAG_THEN_FULL, nowMs);
    else if (heldMs >= BTN_LONG_MS)     startPlan(PLAN_FULL, nowMs);
    else                                startPlan(PLAN_QUICK, nowMs);
}

void pollButton(uint32_t nowMs) {
    const bool r = (digitalRead(btn_cal) == LOW);
    if (r != btnRaw) { btnRaw = r; btnChangeMs = nowMs; }
    if (nowMs - btnChangeMs >= BTN_DEBOUNCE_MS && btnStable != btnRaw) {
        btnStable = btnRaw;
        if (btnStable) btnPressMs = nowMs;
        else           onRelease(nowMs - btnPressMs, nowMs);
    }
}

// ---- calibration state machine (one tick) ---------------------------------------------------
void calibrationTick(uint32_t nowMs) {
    if (nowMs - calStartMs > CAL_TIMEOUT_MS && state != ST_MAG_CAL) { abortCalibration("timeout"); return; }

    if (state == ST_CAL_COUNTDOWN) {
        const uint32_t el = nowMs - stateStartMs;
        const int left = (int)((CAL_COUNTDOWN_MS - (el < CAL_COUNTDOWN_MS ? el : CAL_COUNTDOWN_MS) + 999) / 1000);
        if (left != lastCountdownPrint) { lastCountdownPrint = left; Serial.printf("[CAL] %d...\n", left); }
        if (el >= CAL_COUNTDOWN_MS) { state = ST_CAL_HOLD; stateStartMs = nowMs; resetAcc(); say("hold still"); }
    } else if (state == ST_CAL_HOLD) {
        accumulate();
        if (nowMs - stateStartMs >= CAL_HOLD_MS) finishHold(nowMs);
    } else if (state == ST_MAG_CAL) {
        if ((tickCount % 10) == 0 && magN < MAG_MAX_SAMPLES && vnorm(raw[0].mag) > 0.0f) {
            MagSample s = { cd[0].acc.x, cd[0].acc.y, cd[0].acc.z, raw[0].mag.x, raw[0].mag.y, raw[0].mag.z };
            magBuf[magN++] = s;
        }
        if (nowMs - stateStartMs >= MAG_CAL_MS || magN >= MAG_MAX_SAMPLES) finishMagCal(nowMs);
    }
}

// ---- run: orientation -> joint commands -----------------------------------------------
void runTick(float dt, uint32_t t_us) {
    const bool synced = IsMPUSynced();
    if (!synced) {
        wasSynced = false;
        if ((tickCount % (uint32_t)(SAMPLE_RATE_HZ * 5)) == 0)
            say("not calibrated: hold the button 2 s (FULL) or press briefly after a reboot (QUICK)");
        return;
    }

    quaternion qU = segmentOrientation(g[0], *seg[0]);
    quaternion qF = segmentOrientation(g[1], *seg[1]);
    axisVector aU = calculateAxisVector(qU), aF = calculateAxisVector(qF);

    if (USE_HINGE_YAW_CORRECTION) {                 // optional continuous heading correction of the 6500
        const float dpsi = hingeYawCorrection(aU, aF);
        if (dpsi != 0.0f) {
            g[1] = alignYawOffset_6500(g[1], dpsi);
            qF = segmentOrientation(g[1], *seg[1]);
            aF = calculateAxisVector(qF);
        }
    }

    humanJ = calculateJointAngles(aU, aF);
    robotTarget = mapHumanToRobot(humanFingertip(aU, aF));

    float j[3] = { prevJoint[0], prevJoint[1], prevJoint[2] };
    const bool reachable = robotInverseKinematics(robotTarget, j);
    if (!wasSynced) { for (int i = 0; i < 3; i++) prevJoint[i] = j[i]; wasSynced = true; }   // no slew from zero
    const uint8_t lim = limitJointAngles(j, prevJoint, dt);
    for (int i = 0; i < 3; i++) { prevJoint[i] = j[i]; jointCmd[i] = j[i]; }

    uint8_t flags = PKT_F_CALIBRATED;
    if (USE_MAG_9250 && cal.s9250.magValid) flags |= PKT_F_MAG;
    if (lim & 1) flags |= PKT_F_LIMIT;
    if (lim & 2) flags |= PKT_F_SLEW;
    if (!reachable) flags |= PKT_F_UNREACHABLE;
    if (faultCooldown > 0) { flags |= PKT_F_IMU_FAULT; faultCooldown--; }
    lastFlags = flags;

    const uint32_t sendEvery = (uint32_t)(SAMPLE_RATE_HZ / SEND_RATE_HZ);
    if ((tickCount % sendEvery) == 0) {
        JointPacket p;
        memset(&p, 0, sizeof(p));
        p.t_us = t_us;
        for (int i = 0; i < 3; i++) p.joint_deg[i] = j[i];
        p.q_upper[0] = qU.w; p.q_upper[1] = qU.x; p.q_upper[2] = qU.y; p.q_upper[3] = qU.z;
        p.q_fore[0] = qF.w;  p.q_fore[1] = qF.x;  p.q_fore[2] = qF.y;  p.q_fore[3] = qF.z;
        p.flags = flags;
        ESPNowSend(p);
    }

    if (DEBUG_CSV && (tickCount % (uint32_t)(SAMPLE_RATE_HZ / 10)) == 0) {
        Serial.printf("%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u\n", (unsigned long)(t_us / 1000UL),
                      humanJ.joint_a, humanJ.joint_b, humanJ.joint_d, j[0], j[1], j[2], (unsigned)flags);
    }
}

}   // namespace

// ---------------------------------------------------------------------------------
//  public
// ---------------------------------------------------------------------------------
bool IsMPUSynced() {
    return seg[0] && cal.seg9250.valid && cal.seg6500.valid && cal.seg9250.anchored && cal.seg6500.anchored;
}

SessionState sessionState() { return state; }
const CalibrationData &sessionCalibration() { return cal; }
void sessionJointCommand(float j[3]) { for (int i = 0; i < 3; i++) j[i] = jointCmd[i]; }
joints sessionHumanJoints() { return humanJ; }
xyzFloat sessionRobotTarget() { return robotTarget; }

bool sessionBegin() {
    // (re)initialise all runtime state
    tickInit = false; tickCount = 0; badReads = 0; faultCooldown = 0; state = ST_RUN; plan = PLAN_NONE;
    seeded[0] = seeded[1] = false; boostUntilMs[0] = boostUntilMs[1] = 0; wasSynced = false; faultReported = false;
    magN = 0; btnRaw = btnStable = false; resetAcc();
    for (int i = 0; i < 3; i++) prevJoint[i] = jointCmd[i] = 0.0f;
    cal = CalibrationData();                     // value-initialised: everything zero
    scal[0] = &cal.s9250; scal[1] = &cal.s6500;
    seg[0] = &cal.seg9250; seg[1] = &cal.seg6500;
    for (int i = 0; i < 2; i++) {
        scal[i]->magScale = xyzFloat(1.0f, 1.0f, 1.0f);
        g[i].w = 1; g[i].x = g[i].y = g[i].z = 0;
    }
    periodUs = 1000000UL / (uint32_t)SAMPLE_RATE_HZ;

    pinMode(btn_cal, INPUT_PULLUP);
    pinMode(vibration_motor, OUTPUT);
    digitalWrite(vibration_motor, LOW);          // vibration feedback postponed

    bool ok = true;
    if (!MPUInit()) ok = false;
    if (!ESPNowInit()) ok = false;

    // bench offsets: power on with the button held
    if (ok && digitalRead(btn_cal) == LOW) {
        const uint32_t t0 = millis();
        while (digitalRead(btn_cal) == LOW && millis() - t0 < BTN_LONG_MS) delay(10);
        if (digitalRead(btn_cal) == LOW) {
            benchCalibrateIMUs();
            // gyro biases stored earlier were relative to the old offsets
            CalibrationData old;
            if (loadCalibration(old)) { old.s9250.gyroBias = xyzFloat(0, 0, 0); old.s6500.gyroBias = xyzFloat(0, 0, 0); saveCalibration(old); }
            while (digitalRead(btn_cal) == LOW) delay(10);
        }
    }

    CalibrationData stored;
    if (loadCalibration(stored)) {
        cal = stored;
        scal[0] = &cal.s9250; scal[1] = &cal.s6500;
        seg[0] = &cal.seg9250; seg[1] = &cal.seg6500;
        // heading anchoring across a reboot: only a magnetometer-referenced 9250 keeps it
        cal.seg9250.anchored = cal.calUsedMag && USE_MAG_9250 && cal.s9250.magValid && cal.seg9250.valid;
        cal.seg6500.anchored = false;
        Serial.println(cal.seg9250.anchored
            ? "[CAL] calibration loaded - press the button briefly with the arm hanging straight down (QUICK)"
            : "[CAL] calibration loaded - heading reference lost: hold the button 2 s (FULL)");
    } else {
        Serial.println("[CAL] no calibration stored: hold the button 2 s (FULL)");
    }
    Serial.printf("[SESSION] %d Hz, send %d Hz, mag %s\n", SAMPLE_RATE_HZ, SEND_RATE_HZ,
                  (USE_MAG_9250 && cal.s9250.magValid && magnetometerAvailable()) ? "on" : "off (6-axis)");
    return ok;
}

void sessionStep() {
    const uint32_t nowUs = micros();
    if (!tickInit) { nextTickUs = nowUs; lastTickUs = nowUs - periodUs; tickInit = true; }
    if ((int32_t)(nowUs - nextTickUs) < 0) return;
    nextTickUs += periodUs;
    if ((int32_t)(nowUs - nextTickUs) > (int32_t)(5 * periodUs)) nextTickUs = nowUs + periodUs;   // fell behind: resync

    float dt = (float)(nowUs - lastTickUs) * 1e-6f;
    lastTickUs = nowUs;
    if (dt < DT_MIN_S) dt = DT_MIN_S;
    if (dt > DT_MAX_S) dt = DT_MAX_S;
    tickCount++;
    const uint32_t nowMs = nowUs / 1000UL;

    pollButton(nowMs);

    if (!readIMUs(raw[0], raw[1])) {
        faultCooldown = FAULT_FLAG_TICKS;
        if (++badReads == FAULT_TICKS && !faultReported) { faultReported = true; Serial.println("[IMU] read failures - transmission stopped (the slave must time out and hold)"); }
        return;                                   // nothing is sent from stale data; the filter state is kept
    }
    if (badReads >= FAULT_TICKS) { Serial.println("[IMU] recovered"); faultReported = false; }
    badReads = 0;

    for (int i = 0; i < 2; i++) {
        cd[i] = applySensorCalibration(raw[i], *scal[i]);
        if (i == 0 && !USE_MAG_9250) cd[i].mag = xyzFloat(0, 0, 0);
        if (!seeded[i]) { g[i] = convertRawToQuaternion(cd[i]); seeded[i] = true; boostUntilMs[i] = nowMs + BOOST_AFTER_SEED_MS; }
        else {
            const bool mag9 = (vnorm(cd[i].mag) > 0.0f);
            // fast convergence while the operator is in a calibration pose (still) and just after a seed
            const bool boost = (state == ST_CAL_COUNTDOWN || state == ST_CAL_HOLD) || (int32_t)(nowMs - boostUntilMs[i]) < 0;
            const float beta = boost ? MADGWICK_BETA_BOOST : (mag9 ? MADGWICK_BETA_9250 : MADGWICK_BETA_6500);
            MadgwickAHRSupdate(cd[i], dt, g[i], beta);
        }
    }

    if (state != ST_RUN) calibrationTick(nowMs);
    else                 runTick(dt, raw[0].time);
}
