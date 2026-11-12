// End-to-end host test: simulated arm + IMUs  ->  real session/connections/kinematics code  ->  ESP-NOW packets.
// Usage:  test_session            (functional tests)
//         test_session drift      (6500 yaw-drift experiment, informational)
#include "testutil.h"
#include "session.h"
#include "connections.h"
#include "sim.h"
#include <string.h>
#include <stdlib.h>
#include <string>
#include <functional>
#include <Preferences.h>

extern std::vector<std::vector<uint8_t> > g_espnowSent;
extern bool g_shimAckOk;
void shimSetMicros(unsigned long v);
void shimSetPin(int p, int v);
extern void (*shimDelayHook)(unsigned long);
unsigned long millis();

// ---------------------------------------------------------------- time, truth, packets
static unsigned long nowUs = 1000000UL;
static Mat3 gUp, gFo;
static size_t consumed = 0;

struct Rec { float t; float j[3]; float e[3]; float tipErrMm; uint8_t flags; uint8_t seq; uint32_t t_us; float hd[3]; float hdT[3]; };
static std::vector<Rec> recs;
static std::vector<LogFrame> monLogs;          // decoded LOG frames the slave would have printed
static std::vector<StatusPacket> monStatus;    // decoded STATUS heartbeats
static std::vector<unsigned long> monLogMs;    // master-side time when each LOG frame was sent
static int badPackets = 0, seqGaps = 0;
static bool haveSeq = false; static uint16_t lastSeq = 0;

static void expectedJoints(const Mat3 &up, const Mat3 &fo, float out[3]) {
    axisVector u = calculateAxisVector(quaternionFromRotation(up)), f = calculateAxisVector(quaternionFromRotation(fo));
    xyzFloat tgt = mapHumanToRobot(humanFingertip(u, f));
    float j[3] = { 0, 0, 0 };
    robotInverseKinematics(tgt, j);
    float prev[3] = { j[0], j[1], j[2] };
    limitJointAngles(j, prev, 1.0f);
    for (int i = 0; i < 3; i++) out[i] = j[i];
}

static void drain() {
    while (consumed < g_espnowSent.size()) {
        const std::vector<uint8_t> &b = g_espnowSent[consumed++];
        const uint8_t ty = packetType(b.data(), (int)b.size());
        if (ty == PKT_TYPE_LOG) { LogFrame f; if (decodeLogPacket(b.data(), (int)b.size(), f)) { monLogs.push_back(f); monLogMs.push_back(millis()); } else badPackets++; continue; }
        if (ty == PKT_TYPE_STATUS) { StatusPacket st; if (decodeStatusPacket(b.data(), (int)b.size(), st)) monStatus.push_back(st); else badPackets++; continue; }
        if (!packetValid(b.data(), (int)b.size()) || b.size() != sizeof(JointPacket)) { badPackets++; continue; }
        JointPacket p; memcpy(&p, b.data(), sizeof(p));
        if (haveSeq && (uint16_t)(p.seq - lastSeq) != 1) seqGaps++;
        haveSeq = true; lastSeq = p.seq;
        Rec r; r.t = (float)nowUs * 1e-6f; r.flags = p.flags; r.seq = p.seq; r.t_us = p.t_us;
        for (int i = 0; i < 3; i++) r.j[i] = p.joint_deg[i];
        expectedJoints(gUp, gFo, r.e);
        xyzFloat a = robotForwardKinematics(r.j), c = robotForwardKinematics(r.e);
        r.tipErrMm = 1000.0f * vnorm(a - c);
        joints hj = sessionHumanJoints();
        axisVector u = calculateAxisVector(quaternionFromRotation(gUp)), f = calculateAxisVector(quaternionFromRotation(gFo));
        joints tj = calculateJointAngles(u, f);
        r.hd[0] = hj.joint_a; r.hd[1] = hj.joint_b; r.hd[2] = hj.joint_d;
        r.hdT[0] = tj.joint_a; r.hdT[1] = tj.joint_b; r.hdT[2] = tj.joint_d;
        recs.push_back(r);
    }
}

static void tick() {
    nowUs += 5000;
    shimSetMicros(nowUs);
    sim.Aup = gUp; sim.Afore = gFo;
    sessionStep();
    drain();
}
static void run(float s) { int n = (int)(s * 200.0f + 0.5f); for (int i = 0; i < n; i++) tick(); }
static bool runUntil(const std::function<bool()> &pred, float maxS) {
    int n = (int)(maxS * 200.0f);
    for (int i = 0; i < n; i++) { if (pred()) return true; tick(); }
    return pred();
}

static Mat3 interp(const Mat3 &a, const Mat3 &b, float s) {
    quaternion qa = quaternionFromRotation(a), qb = quaternionFromRotation(b);
    quaternion qr = qMul(qConj(qa), qb);
    if (qr.w < 0) { qr.w = -qr.w; qr.x = -qr.x; qr.y = -qr.y; qr.z = -qr.z; }
    float vn = sqrtf(qr.x * qr.x + qr.y * qr.y + qr.z * qr.z);
    float ang = 2.0f * atan2f(vn, qr.w);
    xyzFloat ax = vn > 1e-9f ? xyzFloat(qr.x / vn, qr.y / vn, qr.z / vn) : xyzFloat(1, 0, 0);
    quaternion q = { cosf(0.5f * s * ang), ax.x * sinf(0.5f * s * ang), ax.y * sinf(0.5f * s * ang), ax.z * sinf(0.5f * s * ang) };
    return rotationFromQuaternion(qNormalize(qMul(qa, q)));
}
// smooth move of both segments (the operator walks into the pose; the filter sees real motion)
static void glideTo(const Mat3 &up, const Mat3 &fo, float secs) {
    Mat3 u0 = gUp, f0 = gFo;
    int n = (int)(secs * 200.0f);
    for (int i = 1; i <= n; i++) {
        float x = (float)i / n, s = x * x * (3.0f - 2.0f * x);
        gUp = interp(u0, up, s); gFo = interp(f0, fo, s);
        tick();
    }
    gUp = up; gFo = fo;
}

static void trajPose(float t, Mat3 &up, Mat3 &fo) {
    const float w = 6.2831853f;
    float a = 35 + 25 * sinf(w * t / 7.0f), b = 12 + 12 * sinf(w * t / 5.0f + 1.0f), d = 55 + 35 * sinf(w * t / 6.0f + 2.0f);
    float tw = 8 * sinf(w * t / 9.0f), pr = 10 * sinf(w * t / 4.0f);
    up = upperTruth(D2R(a), D2R(b), D2R(tw));
    fo = foreTruth(up, D2R(d), D2R(pr));
}
static float trajT = 0;
// walk into the trajectory start pose and let the filters settle (the truth must not jump)
static void prepTrajectory() {
    Mat3 u, f; trajPose(trajT, u, f);
    glideTo(u, f, 2.0f); run(3.0f);
}
static void runTrajectory(float secs) {
    int n = (int)(secs * 200.0f);
    for (int i = 0; i < n; i++) { trajT += 0.005f; trajPose(trajT, gUp, gFo); tick(); }
}
static void tumble(float secs) {
    int n = (int)(secs * 200.0f); static float t = 0;
    for (int i = 0; i < n; i++) {
        t += 0.005f;
        gUp = mat3Mul(mat3Mul(Rz(0.9f * t), Rx(0.6f * t)), mat3Mul(Ry(0.45f * t), poseN()));
        gFo = gUp; tick();
    }
}

static void pressButton(float secs) { shimSetPin(btn_cal, HIGH); run(secs); shimSetPin(btn_cal, LOW); run(0.1f); }
static void bootDevice() {
    shimSetPin(btn_cal, LOW);
    simStartMotion(nowUs);
    consumed = g_espnowSent.size();
    haveSeq = false;
    sim.failReads = 0;
    sessionBegin();
}

static Mat3 pose1() { return poseN(); }
static Mat3 pose2() { return mat3Mul(Rx(D2R(90)), poseN()); }

// pose-1 / pose-2 sequence after the plan has started (state == COUNTDOWN for pose 1)
static bool fullSequence(const Mat3 &p1, const Mat3 &p2) {
    if (sessionState() != ST_CAL_COUNTDOWN) return false;
    glideTo(p1, p1, 1.5f);
    if (!runUntil([] { return sessionState() == ST_CAL_HOLD; }, 5)) return false;
    if (!runUntil([] { return sessionState() != ST_CAL_HOLD; }, 15)) return false;     // may extend while the filter settles
    if (sessionState() != ST_CAL_COUNTDOWN) return false;
    glideTo(p2, p2, 1.5f);
    if (!runUntil([] { return sessionState() == ST_CAL_HOLD; }, 5)) return false;
    if (!runUntil([] { return sessionState() != ST_CAL_HOLD; }, 15)) return false;
    return sessionState() == ST_RUN;
}

struct Stats { int n; double sumSq[3]; float mx[3]; float tipMax; double tipSum; float hdMax[3]; };
static Stats statsFrom(size_t first, float tMin = 0) {
    Stats s; memset(&s, 0, sizeof(s));
    for (size_t i = first; i < recs.size(); i++) {
        if (recs[i].t < tMin) continue;
        for (int k = 0; k < 3; k++) {
            float e = recs[i].j[k] - recs[i].e[k];
            s.sumSq[k] += e * e; s.mx[k] = fmaxf(s.mx[k], fabsf(e));
            s.hdMax[k] = fmaxf(s.hdMax[k], fabsf(recs[i].hd[k] - recs[i].hdT[k]));
        }
        s.tipMax = fmaxf(s.tipMax, recs[i].tipErrMm); s.tipSum += recs[i].tipErrMm; s.n++;
    }
    return s;
}
static void report(const char *name, const Stats &s) {
    if (!s.n) { printf("  %s: no packets\n", name); return; }
    printf("  %s: %d packets | joint err rms a/b/d %.2f/%.2f/%.2f deg, max %.2f/%.2f/%.2f deg | human-joint max err %.2f/%.2f/%.2f | tip err mean %.1f max %.1f mm\n", name, s.n,
           sqrt(s.sumSq[0] / s.n), sqrt(s.sumSq[1] / s.n), sqrt(s.sumSq[2] / s.n), s.mx[0], s.mx[1], s.mx[2],
           s.hdMax[0], s.hdMax[1], s.hdMax[2], s.tipSum / s.n, s.tipMax);
}

static void setupWorld(bool swap) {
    sim.reset();
    sim.M9 = randomRotation(); sim.M6 = randomRotation();
    sim.heading = 0.7f;
    for (int i = 0; i < 3; i++) { sim.gyroBias9[i] = urand(-0.6f, 0.6f); sim.gyroBias6[i] = urand(-0.6f, 0.6f); }
    sim.hardIron = xyzFloat(nrand(12), nrand(12), nrand(12));
    sim.softIron = xyzFloat(urand(0.9f, 1.1f), urand(0.9f, 1.1f), urand(0.9f, 1.1f));
    sim.akSwap = swap;
    if (getenv("SIM_NODYN")) sim.dynamicAccel = false;                       // debugging aids
    if (getenv("SIM_NONOISE")) sim.gyroNoise = sim.accNoise = sim.magNoise = 0;
    if (getenv("SIM_NOBIAS")) for (int i = 0; i < 3; i++) sim.gyroBias9[i] = sim.gyroBias6[i] = 0;
    trajT = 0;
    gUp = upperTruth(D2R(urand(0, 90)), D2R(urand(0, 40)), D2R(urand(-30, 30)));
    gFo = foreTruth(gUp, D2R(urand(10, 100)), D2R(urand(-20, 20)));
}

// ================================================================================================
static void test_nothing_before_calibration() {
    printf("[no calibration -> nothing is transmitted]\n");
    clearStorage(); setupWorld(false);
    bootDevice();
    run(3);
    CHECK(!IsMPUSynced(), "must not be synced");
    CHECK(g_espnowSent.size() == consumed && recs.empty(), "packets sent without calibration (%d)", (int)recs.size());
    pressButton(0.3f);                                           // QUICK without a full calibration
    CHECK(sessionState() == ST_RUN, "QUICK without calibration must be refused");
    run(1);
    CHECK(recs.empty(), "still nothing");
}

static void test_full_6axis_and_reboot() {
    printf("[FULL calibration without magnetometer calibration (6-axis), then reboot]\n");
    clearStorage(); setupWorld(false); recs.clear();
    bootDevice();
    run(2);
    shimSetPin(btn_cal, HIGH); run(2.3f); shimSetPin(btn_cal, LOW);
    run(0.05f);
    CHECK(sessionState() == ST_CAL_COUNTDOWN, "FULL did not start (state %d)", sessionState());
    bool ok = fullSequence(pose1(), pose2());
    CHECK(ok, "FULL sequence did not finish");
    CHECK(IsMPUSynced(), "not synced after FULL");
    CHECK(!sessionCalibration().calUsedMag, "magnetometer must not be flagged");
    prepTrajectory();
    size_t first = recs.size();
    runTrajectory(30);
    CHECK(recs.size() > first + 1000, "too few packets (%d)", (int)(recs.size() - first));
    Stats s = statsFrom(first, 0);
    report("6-axis, 30 s motion", s);
    CHECK(s.n > 1000 && s.tipMax < 40.0f, "tip error max %.1f mm", s.tipMax);
    CHECK(badPackets == 0, "bad packets %d", badPackets);
    CHECK(seqGaps == 0, "sequence gaps %d", seqGaps);
    float dtMin = 1e9f, dtMax = 0;
    for (size_t i = first + 1; i < recs.size(); i++) { float d = (float)(recs[i].t_us - recs[i - 1].t_us) * 1e-3f; dtMin = fminf(dtMin, d); dtMax = fmaxf(dtMax, d); }
    printf("  packet interval %.1f..%.1f ms (50 Hz = 20 ms)\n", dtMin, dtMax);
    CHECK(dtMin > 19.5f && dtMax < 20.5f, "packet interval");
    for (size_t i = first; i < recs.size(); i++) CHECK(recs[i].flags & PKT_F_CALIBRATED, "calibrated flag");
    size_t nBefore = recs.size();
    bootDevice();
    run(3);
    CHECK(!IsMPUSynced(), "6-axis calibration must not survive a reboot");
    CHECK(recs.size() == nBefore, "transmitting after reboot without anchoring");
}

static void test_mag_flow() {
    printf("[MAG + FULL calibration, trajectory, reboot + QUICK]\n");
    clearStorage(); setupWorld(true); recs.clear();
    bootDevice();
    run(2);
    shimSetPin(btn_cal, HIGH); run(6.3f); shimSetPin(btn_cal, LOW);
    run(0.05f);
    CHECK(sessionState() == ST_MAG_CAL, "MAG cal did not start (state %d)", sessionState());
    for (int i = 0; i < 25 * 200 && sessionState() == ST_MAG_CAL; i++) tumble(0.005f);
    bool ended = sessionState() != ST_MAG_CAL;
    CHECK(ended, "MAG cal did not end");
    CHECK(sessionState() == ST_CAL_COUNTDOWN, "after MAG cal the FULL sequence must follow (state %d)", sessionState());
    const SensorCal &c = sessionCalibration().s9250;
    CHECK(c.magValid && c.magSwapAxes, "axis swap not detected (valid %d swap %d)", c.magValid, c.magSwapAxes);
    CHECK(fabsf(c.magOffset.x - sim.hardIron.x) < 2.0f && fabsf(c.magOffset.y - sim.hardIron.y) < 2.0f && fabsf(c.magOffset.z - sim.hardIron.z) < 2.0f,
          "hard iron estimate %.1f %.1f %.1f vs %.1f %.1f %.1f", c.magOffset.x, c.magOffset.y, c.magOffset.z, sim.hardIron.x, sim.hardIron.y, sim.hardIron.z);
    CHECK(fullSequence(pose1(), pose2()), "FULL after MAG did not finish");
    CHECK(IsMPUSynced(), "not synced");
    CHECK(sessionCalibration().calUsedMag, "mag flag");
    prepTrajectory();
    size_t first = recs.size();
    runTrajectory(30);
    Stats s = statsFrom(first);
    report("9-axis, 30 s motion", s);
    CHECK(s.tipMax < 40.0f, "tip error max %.1f mm", s.tipMax);
    CHECK((recs.back().flags & PKT_F_MAG) != 0, "MAG flag");

    // ---- reboot: mag keeps the 9250 heading, QUICK re-anchors the 6500
    CalibrationData stored; CHECK(loadCalibration(stored), "calibration not in flash");
    size_t nBefore = recs.size();
    gUp = upperTruth(D2R(70), D2R(20), D2R(-10)); gFo = foreTruth(gUp, D2R(60), D2R(5));    // power on in an arbitrary pose
    bootDevice();
    run(1);
    CHECK(sessionCalibration().seg9250.anchored && !sessionCalibration().seg6500.anchored, "anchoring after reboot");
    CHECK(!IsMPUSynced() && recs.size() == nBefore, "must wait for QUICK");
    glideTo(pose1(), pose1(), 2.0f);
    run(0.5f);
    pressButton(0.3f);
    CHECK(sessionState() == ST_CAL_COUNTDOWN, "QUICK did not start (%d)", sessionState());
    CHECK(runUntil([] { return sessionState() == ST_CAL_HOLD; }, 5), "hold");
    CHECK(runUntil([] { return sessionState() == ST_RUN; }, 5), "QUICK did not finish");
    CHECK(IsMPUSynced(), "not synced after QUICK");
    trajT = 0;
    prepTrajectory();
    first = recs.size();
    runTrajectory(30);
    s = statsFrom(first);
    report("after reboot + QUICK", s);
    CHECK(s.tipMax < 40.0f, "tip error after QUICK max %.1f mm", s.tipMax);

    // ---- QUICK with a wrong pose (arm not hanging): refused, nothing changes
    {
        Mat3 Tbefore = sessionCalibration().seg6500.T;
        Mat3 raised = upperTruth(D2R(40), 0, 0);
        glideTo(raised, raised, 2.0f);
        pressButton(0.3f);
        runUntil([] { return sessionState() == ST_RUN; }, 8);
        CHECK(IsMPUSynced(), "a refused QUICK must keep the previous anchoring");
        CHECK(matErr(Tbefore, sessionCalibration().seg6500.T) == 0.0f, "refused QUICK must not change the calibration");
    }
    // ---- QUICK with the humeral twist 10 deg off the pose-1 twist: informational (operator error sensitivity)
    {
        Mat3 tw = upperTruth(0, 0, D2R(10));
        glideTo(tw, tw, 2.0f);
        pressButton(0.3f);
        runUntil([] { return sessionState() == ST_RUN; }, 8);
        Mat3 up = upperTruth(D2R(45), D2R(10), 0), fo = foreTruth(up, D2R(60), 0);
        glideTo(up, fo, 2.0f); run(4.0f);
        joints h = sessionHumanJoints(), t = calculateJointAngles(calculateAxisVector(quaternionFromRotation(up)), calculateAxisVector(quaternionFromRotation(fo)));
        printf("  QUICK done with 10 deg humeral twist error: joint error a%.1f b%.1f d%.1f deg\n", h.joint_a - t.joint_a, h.joint_b - t.joint_b, h.joint_d - t.joint_d);
    }
    // ---- QUICK with only the forearm pronated 10 deg relative to the upper arm
    {
        Mat3 fo10 = upperTruth(0, 0, D2R(10));
        glideTo(pose1(), fo10, 2.0f);
        pressButton(0.3f);
        runUntil([] { return sessionState() == ST_RUN; }, 8);
        Mat3 up = upperTruth(D2R(45), D2R(10), 0), fo = foreTruth(up, D2R(60), 0);
        glideTo(up, fo, 2.0f); run(4.0f);
        joints h = sessionHumanJoints(), t = calculateJointAngles(calculateAxisVector(quaternionFromRotation(up)), calculateAxisVector(quaternionFromRotation(fo)));
        printf("  QUICK done with 10 deg forearm pronation error: joint error a%.1f b%.1f d%.1f deg\n", h.joint_a - t.joint_a, h.joint_b - t.joint_b, h.joint_d - t.joint_d);
    }
    glideTo(pose1(), pose1(), 2.0f);
    pressButton(0.3f);
    runUntil([] { return sessionState() == ST_RUN; }, 8);
    CHECK(IsMPUSynced(), "recovery with the correct pose");
}

static void test_failures() {
    printf("[failure paths]\n");
    clearStorage(); setupWorld(true); recs.clear();
    bootDevice();
    run(1);
    // --- not still during the hold -> averaging restarts, calibration still completes
    shimSetPin(btn_cal, HIGH); run(2.3f); shimSetPin(btn_cal, LOW); run(0.05f);
    glideTo(pose1(), pose1(), 1.5f);
    runUntil([] { return sessionState() == ST_CAL_HOLD; }, 5);
    for (int i = 0; i < 100; i++) { gUp = upperTruth(D2R(5 * sinf(i * 0.3f)), 0, 0); gFo = gUp; tick(); }    // shaking, above CAL_STILL_GYRO_DPS
    CHECK(sessionState() == ST_CAL_HOLD, "should still be waiting for stillness (state %d)", sessionState());
    glideTo(pose1(), pose1(), 0.3f);
    CHECK(runUntil([] { return sessionState() == ST_CAL_COUNTDOWN; }, 6), "pose 1 never accepted after calming down");
    // --- wrong pose 2 (only 30 deg): FULL rejected, state returns to RUN, not synced
    Mat3 w2 = mat3Mul(Rx(D2R(30)), poseN());
    glideTo(w2, w2, 1.5f);
    CHECK(runUntil([] { return sessionState() == ST_CAL_HOLD; }, 5), "hold 2");
    CHECK(runUntil([] { return sessionState() != ST_CAL_HOLD; }, 5), "hold 2 end");
    CHECK(sessionState() == ST_RUN && !IsMPUSynced(), "wrong pose 2 must be rejected (state %d synced %d)", sessionState(), IsMPUSynced());
    // --- abort by button during countdown
    shimSetPin(btn_cal, HIGH); run(2.3f); shimSetPin(btn_cal, LOW); run(0.05f);
    CHECK(sessionState() == ST_CAL_COUNTDOWN, "start");
    run(0.5f); pressButton(0.2f);
    CHECK(sessionState() == ST_RUN, "button must abort calibration");
    // --- a good FULL, then IMU read failure in operation
    shimSetPin(btn_cal, HIGH); run(2.3f); shimSetPin(btn_cal, LOW); run(0.05f);
    CHECK(fullSequence(pose1(), pose2()), "FULL");
    CHECK(IsMPUSynced(), "synced");
    prepTrajectory();
    runTrajectory(5);
    size_t n0 = recs.size();
    sim.failReads = 400;                                          // 2 s of failed reads
    runTrajectory(2.0f);
    CHECK(recs.size() == n0, "packets sent while the IMUs failed (%d)", (int)(recs.size() - n0));
    runTrajectory(1.0f);
    CHECK(recs.size() > n0 + 20, "no packets after recovery");
    bool flagged = false; for (size_t i = n0; i < recs.size(); i++) if (recs[i].flags & PKT_F_IMU_FAULT) flagged = true;
    CHECK(flagged, "IMU_FAULT flag after recovery");
    runTrajectory(10);
    CHECK(!(recs.back().flags & PKT_F_IMU_FAULT), "fault flag must clear");
    // --- corrupted flash is rejected
    CalibrationData good; CHECK(loadCalibration(good), "stored calibration");
    {
        Preferences p; p.begin("telearm", false);
        uint8_t buf[sizeof(CalibrationData) + 8]; size_t n = p.getBytes("cal", buf, sizeof(buf));
        CHECK(n == sizeof(CalibrationData), "stored blob size %d", (int)n);
        buf[n / 2] ^= 0x40; p.putBytes("cal", buf, n); p.end();
    }
    CalibrationData bad; CHECK(!loadCalibration(bad), "corrupted calibration accepted");
    bootDevice(); run(2);
    CHECK(!IsMPUSynced(), "must not be synced with corrupted flash");
    // --- packet integrity
    if (!g_espnowSent.empty()) {
        std::vector<uint8_t> b = g_espnowSent.back(); b[10] ^= 1;
        CHECK(!packetValid(b.data(), (int)b.size()), "CRC must catch a flipped bit");
        CHECK(!packetValid(g_espnowSent.back().data(), 10), "short packet");
    }
    CHECK(sizeof(JointPacket) <= 250, "packet too large for ESP-NOW");
}

static void test_bench_mode() {
    printf("[bench offsets: power on with the button held]\n");
    clearStorage(); setupWorld(false);
    simStartMotion(nowUs);
    shimSetPin(btn_cal, HIGH);
    static unsigned long releaseAt;
    releaseAt = nowUs / 1000 + 4000;
    shimSetMicros(nowUs);
    shimDelayHook = [](unsigned long) { if (millis() >= releaseAt) shimSetPin(btn_cal, LOW); };
    sim.benchCalls = 0;
    sessionBegin();
    shimDelayHook = 0;
    nowUs = millis() * 1000UL + 1000000UL;
    CHECK(sim.benchCalls == 1, "bench calibration calls %d", sim.benchCalls);
    sim.benchCalls = 0;
    shimSetPin(btn_cal, LOW);
    bootDevice();
    CHECK(sim.benchCalls == 0, "no bench calibration without the button");
}

// ---------------------------------------------------------------- remote monitor (master serial -> slave)
static bool logHas(size_t from, const char *needle) {
    for (size_t i = from; i < monLogs.size(); i++) if (strstr(monLogs[i].text, needle)) return true;
    return false;
}
static int logCount(size_t from, const char *needle) {
    int n = 0; for (size_t i = from; i < monLogs.size(); i++) if (strstr(monLogs[i].text, needle)) n++;
    return n;
}

static void test_remote_monitor() {
    printf("[remote monitor: LOG + STATUS frames]\n");
    clearStorage(); setupWorld(true); recs.clear(); monLogs.clear(); monStatus.clear(); monLogMs.clear();
    g_shimAckOk = true;
    badPackets = 0;
    bootDevice();
    run(2.0f);

    // --- boot messages reach the slave
    CHECK(logHas(0, "[NOW] ESP-NOW initialized"), "boot: ESP-NOW line missing");
    CHECK(logHas(0, "[CAL] no calibration stored"), "boot: 'no calibration stored' missing");
    CHECK(logHas(0, "[SESSION]"), "boot: session line missing");
    // STATUS heartbeat while nothing is calibrated
    CHECK(monStatus.size() >= 2 && monStatus.size() <= 3, "status frames in 2 s: %d", (int)monStatus.size());
    if (!monStatus.empty()) {
        const StatusPacket &st = monStatus.back();
        CHECK(st.state == ST_RUN && st.plan == 0, "status state %d plan %d", st.state, st.plan);
        CHECK((st.hw & HW_UPPER_OK) && (st.hw & HW_FORE_OK) && (st.hw & HW_UPPER_9250), "status hw 0x%02X", st.hw);
        CHECK(!(st.hw & HW_ANCHORED), "anchored before calibration");
        char line[300]; formatStatusLine(st, line, sizeof(line));
        CHECK(strstr(line, "state=RUN") && strstr(line, "upper=MPU9250") && strstr(line, "anchored=no"), "status text: %s", line);
    }
    // --- a full calibration: every prompt arrives in order, with level, contiguous seq
    size_t from = monLogs.size();
    shimSetPin(btn_cal, HIGH); run(2.3f); shimSetPin(btn_cal, LOW); run(0.05f);
    CHECK(fullSequence(pose1(), pose2()), "FULL");
    run(1.0f);
    CHECK(logHas(from, "[CAL] POSE 1"), "POSE 1 prompt not forwarded");
    CHECK(logHas(from, "[CAL] 3..."), "countdown 3 not forwarded");
    CHECK(logHas(from, "[CAL] hold still"), "hold prompt not forwarded");
    CHECK(logHas(from, "[CAL] POSE 2"), "POSE 2 prompt not forwarded");
    CHECK(logHas(from, "[CAL] FULL calibration done"), "completion not forwarded");
    {
        int iP1 = -1, iP2 = -1, iDone = -1;
        for (size_t i = from; i < monLogs.size(); i++) {
            if (iP1 < 0 && strstr(monLogs[i].text, "POSE 1")) iP1 = (int)i;
            if (iP2 < 0 && strstr(monLogs[i].text, "POSE 2")) iP2 = (int)i;
            if (iDone < 0 && strstr(monLogs[i].text, "FULL calibration done")) iDone = (int)i;
        }
        CHECK(iP1 >= 0 && iP1 < iP2 && iP2 < iDone, "calibration prompts out of order (%d %d %d)", iP1, iP2, iDone);
    }
    bool seqOk = true, timeOk = true;
    for (size_t i = 1; i < monLogs.size(); i++) {
        if ((uint16_t)(monLogs[i].seq - monLogs[i - 1].seq) != 1) seqOk = false;
        if (monLogs[i].t_ms < monLogs[i - 1].t_ms) timeOk = false;
    }
    CHECK(seqOk, "log sequence gap in a clean link");
    CHECK(timeOk, "log time goes backwards");
    CHECK(logDroppedCount() == 0, "lines dropped in normal operation: %d", logDroppedCount());
    CHECK(monStatus.back().hw & HW_ANCHORED, "status must show anchored after FULL");
    CHECK(monStatus.back().hw & HW_CAL_VALID, "status must show valid calibration");
    CHECK(badPackets == 0, "undecodable frames: %d", badPackets);
    {   // slave-side text
        char line[300]; formatLogLine(monLogs.back(), line, sizeof(line));
        CHECK(line[0] == '[' && line[1] == 'M' && strstr(line, monLogs.back().text), "log text: %s", line);
    }
    // joint stream unaffected: 50 Hz and flags intact
    prepTrajectory(); size_t r0 = recs.size(); runTrajectory(4);
    CHECK(recs.size() - r0 >= 195 && recs.size() - r0 <= 205, "joint packets in 4 s: %d", (int)(recs.size() - r0));

    // --- errors are visible: IMU read failure while calibrated
    from = monLogs.size(); size_t st0 = monStatus.size();
    sim.failReads = 600; runTrajectory(3.2f);
    CHECK(logHas(from, "[IMU] read failures"), "IMU failure not forwarded");
    CHECK(monStatus.size() - st0 >= 2, "status heartbeat stopped while the IMUs failed (%d)", (int)(monStatus.size() - st0));
    bool lvlErr = false; for (size_t i = from; i < monLogs.size(); i++) if (strstr(monLogs[i].text, "[IMU] read failures") && monLogs[i].level == LOGLVL_ERROR) lvlErr = true;
    CHECK(lvlErr, "IMU failure must be ERROR level");
    sim.failReads = 0; runTrajectory(1.0f);
    CHECK(logHas(from, "[IMU] recovered"), "recovery not forwarded");

    // --- debug lines and the CSV are USB only
    from = monLogs.size();
    logMsg(LOGLVL_DEBUG, "debug only on usb"); run(0.3f);
    CHECK(!logHas(from, "debug only"), "DEBUG line was forwarded");
    for (size_t i = 0; i < monLogs.size(); i++) CHECK(!(monLogs[i].text[0] >= '0' && monLogs[i].text[0] <= '9') || strstr(monLogs[i].text, "...") , "csv line in log: %s", monLogs[i].text);

    // --- truncation of an over-long message
    from = monLogs.size();
    { char big[400]; memset(big, 'x', sizeof(big)); big[399] = 0; logMsg(LOGLVL_WARN, "%s", big); run(0.2f); }
    CHECK(monLogs.size() == from + 1 && monLogs.back().len == LOG_MAX_TEXT && strlen(monLogs.back().text) == LOG_MAX_TEXT, "truncation len %d", monLogs.empty() ? -1 : monLogs.back().len);

    // --- burst: rate limit, queue overflow, drop accounting, newest survives
    clearStorage(); setupWorld(true); bootDevice(); run(1.0f);
    from = monLogs.size(); unsigned long t0 = millis();
    for (int i = 0; i < 100; i++) logMsg(LOGLVL_INFO, "burst %d", i);
    CHECK(logPendingCount() == LOG_RING_LINES, "pending %d", logPendingCount());
    run(2.0f);
    CHECK(logDroppedCount() == 100 - LOG_RING_LINES, "dropped %d", logDroppedCount());
    CHECK(logCount(from, "burst ") == LOG_RING_LINES, "burst lines delivered %d", logCount(from, "burst "));
    CHECK(logHas(from, "burst 99") && !logHas(from, "burst 0") && logHas(from, "burst 52") && !logHas(from, "burst 51"), "oldest lines must be the ones dropped");
    {
        unsigned long minGap = 1000000UL; size_t prev = 0; bool have = false;
        for (size_t i = from; i < monLogs.size(); i++) if (strstr(monLogs[i].text, "burst ")) { if (have) { unsigned long gap = monLogMs[i] - monLogMs[prev]; if (gap < minGap) minGap = gap; } prev = i; have = true; }
        CHECK(minGap >= REMOTE_LOG_MIN_INTERVAL_MS, "log frames closer than the rate limit: %lu ms", minGap);
        (void)t0;
    }
    run(1.2f);
    CHECK(monStatus.back().logDropped == 100 - LOG_RING_LINES, "status logDropped %u", monStatus.back().logDropped);

    // --- link loss and recovery: unsent lines are kept, the last lines are replayed and marked
    clearStorage(); setupWorld(true); bootDevice(); run(2.0f);
    from = monLogs.size();
    logMsg(LOGLVL_INFO, "before outage"); run(0.2f);
    g_shimAckOk = false; run(3.5f);                       // 1 Hz heartbeats fail x3 -> link down (idle detection ~3 s)
    CHECK(!ESPNowLinkUp(), "link must be down after repeated failures");
    logMsg(LOGLVL_WARN, "during outage A"); logMsg(LOGLVL_WARN, "during outage B"); run(0.5f);
    CHECK(!logHas(from, "during outage"), "lines delivered while the slave is away");
    CHECK(logPendingCount() >= 2, "lines must stay queued while the link is down (%d)", logPendingCount());
    g_shimAckOk = true; run(2.5f);
    CHECK(ESPNowLinkUp(), "link must come back");
    CHECK(logCount(from, "during outage A") == 1 && logCount(from, "during outage B") == 1, "outage lines must arrive exactly once (%d, %d)", logCount(from, "during outage A"), logCount(from, "during outage B"));
    bool anyReplay = false, replayBefore = false;
    for (size_t i = from; i < monLogs.size(); i++) { if (monLogs[i].replay) { anyReplay = true; if (strstr(monLogs[i].text, "before outage")) replayBefore = true; } }
    CHECK(anyReplay && replayBefore, "replayed history missing/unmarked");
    { char line[300]; for (size_t i = from; i < monLogs.size(); i++) if (monLogs[i].replay) { formatLogLine(monLogs[i], line, sizeof(line)); CHECK(strstr(line, " R]"), "replay marker: %s", line); break; } }

    // --- decoders reject damaged frames
    {
        std::vector<uint8_t> f;
        for (size_t i = g_espnowSent.size(); i-- > 0;) if (packetType(g_espnowSent[i].data(), (int)g_espnowSent[i].size()) == PKT_TYPE_LOG) { f = g_espnowSent[i]; break; }
        LogFrame lf; StatusPacket sp;
        CHECK(!f.empty() && decodeLogPacket(f.data(), (int)f.size(), lf), "sanity decode");
        std::vector<uint8_t> b = f; b[12] ^= 0x10;                  CHECK(!decodeLogPacket(b.data(), (int)b.size(), lf), "flipped text bit accepted");
        b = f; b[9]++;                                              CHECK(!decodeLogPacket(b.data(), (int)b.size(), lf), "wrong len accepted");
        CHECK(!decodeLogPacket(f.data(), (int)f.size() - 1, lf), "truncated frame accepted");
        CHECK(!decodeStatusPacket(f.data(), (int)f.size(), sp), "log decoded as status");
        CHECK(!packetValid(f.data(), (int)f.size()), "log frame passes the JOINT validator (slave would misparse it)");
        CHECK(!decodeLogPacket(f.data(), 3, lf), "tiny frame");
    }
    CHECK(sizeof(StatusPacket) == 23, "status size %d", (int)sizeof(StatusPacket));
    CHECK(LOG_HEADER_LEN + LOG_MAX_TEXT + 1 <= 250, "log frame too large");
    g_shimAckOk = true;
}

int main(int argc, char **argv) {
    if (argc > 1 && std::string(argv[1]) == "probe") {
        clearStorage(); setupWorld(false); recs.clear(); bootDevice(); run(1);
        shimSetPin(btn_cal, HIGH); run(2.3f); shimSetPin(btn_cal, LOW); run(0.05f);
        printf("full ok=%d\n", fullSequence(pose1(), pose2()));
        struct P { float a, b, d; } ps[] = { {0,0,0}, {90,0,0}, {45,0,0}, {0,30,0}, {0,0,60}, {45,20,70} };
        for (int i = 0; i < 6; i++) {
            Mat3 up = upperTruth(D2R(ps[i].a), D2R(ps[i].b), 0), fo = foreTruth(up, D2R(ps[i].d), 0);
            glideTo(up, fo, 2.0f); run(3.0f);
            joints h = sessionHumanJoints();
            printf("truth a%.0f b%.0f d%.0f -> est a%.1f b%.1f d%.1f\n", ps[i].a, ps[i].b, ps[i].d, h.joint_a, h.joint_b, h.joint_d);
        }
        for (int i = 0; i < 40; i++) {
            runTrajectory(0.5f);
            joints h = sessionHumanJoints();
            axisVector u = calculateAxisVector(quaternionFromRotation(gUp)), f = calculateAxisVector(quaternionFromRotation(gFo));
            joints tj = calculateJointAngles(u, f);
            printf("t=%.1f truth a%.1f b%.1f d%.1f | est a%.1f b%.1f d%.1f\n", trajT, tj.joint_a, tj.joint_b, tj.joint_d, h.joint_a, h.joint_b, h.joint_d);
        }
        return 0;
    }
    if (argc > 1 && std::string(argv[1]) == "csv") {          // dummy data set: csv <out.csv>
        const char *outPath = argc > 2 ? argv[2] : "telearm_dummy_packets.csv";
        clearStorage(); setupWorld(true); recs.clear(); bootDevice(); run(1);
        shimSetPin(btn_cal, HIGH); run(6.3f); shimSetPin(btn_cal, LOW); run(0.05f);
        for (int i = 0; i < 25 * 200 && sessionState() == ST_MAG_CAL; i++) tumble(0.005f);
        if (!fullSequence(pose1(), pose2())) { printf("calibration failed\n"); return 1; }
        struct Pz { float a, b, d, tw, pr; const char *name; };
        const Pz poses[] = { { 10, 0, 90, 0, 0, "ready" }, { 75, 10, 15, 5, 0, "reach_forward" }, { 40, 65, 70, -8, 10, "side_raise" }, { 55, -10, 40, 0, -10, "reach_low" } };
        auto mk = [](const Pz &z, Mat3 &u, Mat3 &f) { u = upperTruth(D2R(z.a), D2R(z.b), D2R(z.tw)); f = foreTruth(u, D2R(z.d), D2R(z.pr)); };
        Mat3 u, f; mk(poses[0], u, f); glideTo(u, f, 2.0f); run(3.0f);          // settle in the first pose (not recorded)
        size_t start = g_espnowSent.size();
        struct Seg { int pose; float hold, move; };
        const Seg plan[] = { { 0, 1.0f, 0 }, { 1, 1.2f, 1.6f }, { 2, 1.2f, 1.6f }, { 3, 1.2f, 1.6f }, { 0, 1.0f, 1.8f } };
        for (const Seg &sg : plan) { mk(poses[sg.pose], u, f); if (sg.move > 0) glideTo(u, f, sg.move); run(sg.hold); }
        FILE *fo = fopen(outPath, "w");
        fprintf(fo, "row,seq,t_us,joint0_base_deg,joint1_shoulder_deg,joint2_elbow_deg,qu_w,qu_x,qu_y,qu_z,qf_w,qf_x,qf_y,qf_z,flags,crc\n");
        int row = 0;
        for (size_t i = start; i < g_espnowSent.size() && row < 100; i += 5, row++) {
            JointPacket p; memcpy(&p, g_espnowSent[i].data(), sizeof(p));
            fprintf(fo, "%d,%u,%u,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%u,%u\n", row, p.seq, p.t_us, p.joint_deg[0], p.joint_deg[1], p.joint_deg[2],
                    p.q_upper[0], p.q_upper[1], p.q_upper[2], p.q_upper[3], p.q_fore[0], p.q_fore[1], p.q_fore[2], p.q_fore[3], p.flags, p.crc);
        }
        fclose(fo);
        printf("wrote %d rows from %d packets\n", row, (int)(g_espnowSent.size() - start));
        return 0;
    }
    if (argc > 1 && std::string(argv[1]) == "probe9") {
        clearStorage(); setupWorld(true); recs.clear(); bootDevice(); run(1);
        shimSetPin(btn_cal, HIGH); run(6.3f); shimSetPin(btn_cal, LOW); run(0.05f);
        for (int i = 0; i < 25 * 200 && sessionState() == ST_MAG_CAL; i++) tumble(0.005f);
        printf("full ok=%d\n", fullSequence(pose1(), pose2()));
        { const SensorCal &c = sessionCalibration().s9250; printf("magcal off %.1f %.1f %.1f (true %.1f %.1f %.1f) scale %.3f %.3f %.3f (true inv %.3f %.3f %.3f) swap %d\n", c.magOffset.x, c.magOffset.y, c.magOffset.z, sim.hardIron.x, sim.hardIron.y, sim.hardIron.z, c.magScale.x, c.magScale.y, c.magScale.z, 1/sim.softIron.x, 1/sim.softIron.y, 1/sim.softIron.z, c.magSwapAxes); }
        struct P { float a, b, d; } ps[] = { {0,0,0}, {90,0,0}, {45,0,0}, {0,30,0}, {0,0,60}, {45,20,70}, {-20,0,0} };
        for (int i = 0; i < 7; i++) {
            Mat3 up = upperTruth(D2R(ps[i].a), D2R(ps[i].b), 0), fo = foreTruth(up, D2R(ps[i].d), 0);
            glideTo(up, fo, 2.0f); run(4.0f);
            joints h = sessionHumanJoints();
            printf("static truth a%.0f b%.0f d%.0f -> est a%.1f b%.1f d%.1f\n", ps[i].a, ps[i].b, ps[i].d, h.joint_a, h.joint_b, h.joint_d);
        }
        prepTrajectory();
        for (int i = 0; i < 6; i++) {
            runTrajectory(0.5f);
            joints h = sessionHumanJoints();
            axisVector u = calculateAxisVector(quaternionFromRotation(gUp)), f = calculateAxisVector(quaternionFromRotation(gFo));
            joints tj = calculateJointAngles(u, f);
            printf("t=%.1f truth a%.1f b%.1f d%.1f | est a%.1f b%.1f d%.1f\n", trajT, tj.joint_a, tj.joint_b, tj.joint_d, h.joint_a, h.joint_b, h.joint_d);
        }
        return 0;
    }
    if (argc > 1 && std::string(argv[1]) == "drift") {
        extern void drift_experiment();
        drift_experiment();
        return 0;
    }
    test_nothing_before_calibration();
    test_full_6axis_and_reboot();
    test_mag_flow();
    test_failures();
    test_bench_mode();
    test_remote_monitor();
    printf("\n%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}

// ------------------------------------------------------------------ informational: forearm heading drift
void drift_experiment() {
    printf("[6500 heading drift: gyro bias steps after calibration (hinge correction = %s)]\n", USE_HINGE_YAW_CORRECTION ? "ON" : "OFF");
    for (int trial = 0; trial < 3; trial++) {
        clearStorage(); setupWorld(true); recs.clear(); consumed = g_espnowSent.size();
        bootDevice(); run(1);
        shimSetPin(btn_cal, HIGH); run(6.3f); shimSetPin(btn_cal, LOW); run(0.05f);
        for (int i = 0; i < 25 * 200 && sessionState() == ST_MAG_CAL; i++) tumble(0.005f);
        { const SensorCal &c = sessionCalibration().s9250; printf("  magcal valid %d swap %d (true %d) off %.1f %.1f %.1f (true %.1f %.1f %.1f) scale %.3f %.3f %.3f (true inv %.3f %.3f %.3f)\n", c.magValid, c.magSwapAxes, sim.akSwap, c.magOffset.x, c.magOffset.y, c.magOffset.z, sim.hardIron.x, sim.hardIron.y, sim.hardIron.z, c.magScale.x, c.magScale.y, c.magScale.z, 1/sim.softIron.x, 1/sim.softIron.y, 1/sim.softIron.z); }
        if (!fullSequence(pose1(), pose2())) { printf("  calibration failed\n"); return; }
        // thermal step: the 6500 gyro bias changes by up to 0.3 dps on each axis after the pose-1 estimate
        for (int i = 0; i < 3; i++) sim.gyroBias6[i] += (trial == 0 ? 0.1f : trial == 1 ? 0.2f : 0.4f) * (i == 1 ? -1.0f : 1.0f);
        prepTrajectory();
        for (int blk = 0; blk < 4; blk++) {
            size_t first = recs.size();
            runTrajectory(30);
            Stats s = statsFrom(first);
            char name[64]; snprintf(name, sizeof(name), "bias step %.1f dps, t=%3d..%3d s", trial == 0 ? 0.1f : trial == 1 ? 0.2f : 0.4f, blk * 30, blk * 30 + 30);
            report(name, s);
        }
    }
}
