// Simulated end-to-end test of the slave session: real slave_session.cpp + real packet code
// (networks.cpp / remotelog.cpp) against the simulated steppers, switches and encoders (sim_actuation.cpp).
#include <Arduino.h>
#include <math.h>
#include <vector>
#include "testutil.h"
#include "slave_session.h"
#include "connections.h"
#include "sim_slave.h"
#include "actuation.h"

void shimSetMicros(unsigned long v);
void shimDeliver(const uint8_t *d, int n);

// binary output capture (Serial.write)
static std::vector<uint8_t> g_out;
size_t SerialShim::write(const uint8_t *b, size_t n) { g_out.insert(g_out.end(), b, b + n); return n; }

static unsigned long g_ms = 0;
static uint16_t g_seq = 0;
static bool g_send = false;
static float g_cmd[3] = { 0, 30, 120 };
static uint8_t g_flags = PKT_F_CALIBRATED;

static void sendJoint() {
    JointPacket p = {};
    p.start = PKT_START; p.type = PKT_TYPE_JOINTS; p.seq = g_seq++; p.t_us = g_ms * 1000;
    for (int j = 0; j < 3; j++) p.joint_deg[j] = g_cmd[j];
    p.q_upper[0] = p.q_fore[0] = 1.0f;
    p.flags = g_flags;
    p.crc = crc8((const uint8_t *)&p, sizeof(p) - 1);
    shimDeliver((const uint8_t *)&p, sizeof(p));
}

static void run(float seconds) {
    const unsigned long end = g_ms + (unsigned long)(seconds * 1000.0f);
    while (g_ms < end) {
        g_ms++;
        shimSetMicros(g_ms * 1000UL);
        simStep(0.001f);
        if (g_send && g_ms % 20 == 0) sendJoint();          // 50 Hz like the master
        slaveStep();
    }
}

// run until a state is reached (or timeout); returns true if reached
static bool runUntil(SlaveState s, float maxSeconds) {
    const unsigned long end = g_ms + (unsigned long)(maxSeconds * 1000.0f);
    while (g_ms < end) { run(0.005f); if (slaveState() == s) return true; }
    return false;
}

static void boot(const float trueDeg[3]) {
    simReset(trueDeg);
    g_send = false; g_flags = PKT_F_CALIBRATED;
    g_cmd[0] = 0; g_cmd[1] = 30; g_cmd[2] = 120;
    slaveBegin();
}

static float maxPosErr() {               // firmware step angle vs real joint angle
    float e = 0;
    for (int j = 0; j < 3; j++) e = fmaxf(e, fabsf(slaveStepDeg(j) - simJ[j].trueDeg));
    return e;
}

static void test_homing() {
    printf("[homing at boot]\n");
    const float start[3] = { 40.0f, 70.0f, 45.0f };
    boot(start);
    CHECK(slaveState() == SS_BOOT, "state after begin %d", slaveState());
    CHECK(runUntil(SS_HOMING, 1.0f), "homing did not start");
    CHECK(runUntil(SS_RUN, 90.0f), "homing did not finish, state %d", slaveState());
    for (int j = 0; j < 3; j++) {
        CHECK(slaveHomed(j), "joint %d not homed", j);
        CHECK(slaveEncoderVerified(j), "encoder %d not verified", j);
        CHECK(fabsf(simJ[j].trueDeg - READY_DEG[j]) < 0.5f, "joint %d true %.2f, ready %.2f", j, simJ[j].trueDeg, READY_DEG[j]);
    }
    CHECK(maxPosErr() < 0.4f, "step vs true after homing %.3f deg", maxPosErr());
    CHECK(simJ[0].enabled, "drivers not enabled");
}

static void test_follow_and_link() {
    printf("[follow master, clamp, link timeout]\n");
    g_send = true;
    float maxErr = 0;
    for (int k = 0; k < 400; k++) {                          // 8 s of smooth operator motion
        const float t = k * 0.02f;
        g_cmd[0] = 30.0f * sinf(0.8f * t); g_cmd[1] = 40.0f + 20.0f * sinf(0.5f * t); g_cmd[2] = 100.0f + 25.0f * sinf(0.6f * t);
        run(0.02f);
        if (k > 100) for (int j = 0; j < 3; j++) maxErr = fmaxf(maxErr, fabsf(simJ[j].trueDeg - g_cmd[j]));
    }
    CHECK(slaveState() == SS_RUN, "state %d", slaveState());
    CHECK(maxErr < 3.0f, "tracking error %.2f deg", maxErr);
    run(1.0f);
    for (int j = 0; j < 3; j++) CHECK(fabsf(simJ[j].trueDeg - g_cmd[j]) < 0.3f, "joint %d settles %.2f vs %.2f", j, simJ[j].trueDeg, g_cmd[j]);
    CHECK(maxPosErr() < 0.4f, "step vs true %.3f", maxPosErr());

    // out of range command -> clamped to JOINT_MIN/MAX_DEG
    g_cmd[0] = 170.0f; g_cmd[1] = -80.0f; g_cmd[2] = 100.0f;
    run(3.0f);
    CHECK(fabsf(simJ[0].trueDeg - JOINT_MAX_DEG[0]) < 0.3f, "clamp max: %.2f", simJ[0].trueDeg);
    CHECK(fabsf(simJ[1].trueDeg - JOINT_MIN_DEG[1]) < 0.3f, "clamp min: %.2f", simJ[1].trueDeg);
    CHECK(slaveState() == SS_RUN, "clamped command caused state %d", slaveState());

    // uncalibrated packets are ignored
    g_cmd[0] = 0; g_cmd[1] = 30; g_cmd[2] = 120;
    run(2.0f);
    g_flags = 0; g_cmd[0] = 50; run(1.0f);
    CHECK(fabsf(simJ[0].trueDeg) < 0.3f, "uncalibrated packet moved joint 0 to %.2f", simJ[0].trueDeg);
    g_flags = PKT_F_CALIBRATED; g_cmd[0] = 0;

    // link lost while moving -> decelerate and hold, no fault
    g_cmd[0] = -60; run(0.2f);
    g_send = false;
    run(1.5f);
    const float held = simJ[0].trueDeg;
    run(1.0f);
    CHECK(fabsf(simJ[0].trueDeg - held) < 0.01f, "still moving after link loss");
    CHECK(held > -60.0f + 1.0f, "did not stop early, reached %.2f", held);
    CHECK(slaveState() == SS_RUN, "link loss caused state %d", slaveState());
    g_send = true; run(3.0f);
    CHECK(fabsf(simJ[0].trueDeg + 60.0f) < 0.3f, "did not resume: %.2f", simJ[0].trueDeg);

    // damaged frame is ignored
    JointPacket p = {}; p.start = PKT_START; p.type = PKT_TYPE_JOINTS; p.flags = PKT_F_CALIBRATED; p.joint_deg[0] = 80;
    p.crc = crc8((const uint8_t *)&p, sizeof(p) - 1) ^ 0x5A;
    g_send = false; run(0.1f);
    for (int i = 0; i < 5; i++) { shimDeliver((const uint8_t *)&p, sizeof(p)); run(0.02f); }
    CHECK(fabsf(simJ[0].trueDeg + 60.0f) < 0.3f, "damaged frame moved the joint: %.2f", simJ[0].trueDeg);
    g_send = true;
}

static void test_resync_and_stall() {
    printf("[encoder correction, stall]\n");
    g_cmd[0] = 10; g_cmd[1] = 45; g_cmd[2] = 90;
    run(3.0f);
    const uint16_t r0 = slaveResyncCount(1);
    simSlip(1, 40);                                          // ~1.5 deg lost at standstill
    run(1.0f);
    CHECK(slaveResyncCount(1) == r0 + 1, "resync count %u -> %u", r0, slaveResyncCount(1));
    CHECK(fabsf(simJ[1].trueDeg - g_cmd[1]) < 0.3f, "joint 1 not corrected: %.2f vs %.2f", simJ[1].trueDeg, g_cmd[1]);
    CHECK(slaveState() == SS_RUN, "state %d", slaveState());

    simSlip(2, 400);                                         // ~15 deg: stall
    run(0.1f);
    CHECK(slaveState() == SS_FAULT, "stall not detected, state %d", slaveState());
    for (int j = 0; j < 3; j++) CHECK(!stepperRunning(j), "joint %d still running in FAULT", j);
    run(2.0f);
    CHECK(slaveState() == SS_FAULT, "FAULT left without E_BTN");
}

static void test_estop() {
    printf("[E_BTN: stop, hold, re-home on release]\n");
    // leave the FAULT from the previous test with an E_BTN cycle
    simEstop = true; run(0.05f);
    CHECK(slaveState() == SS_ESTOP, "state %d", slaveState());
    simEstop = false;
    CHECK(runUntil(SS_RUN, 90.0f), "no re-home after release");
    CHECK(maxPosErr() < 0.4f, "step vs true after re-home %.3f", maxPosErr());

    g_cmd[0] = -70; run(0.3f);                               // moving fast
    CHECK(stepperRunning(0), "joint 0 should be moving");
    simEstop = true; run(0.015f);
    CHECK(slaveState() == SS_ESTOP, "E-stop not taken within 15 ms");
    for (int j = 0; j < 3; j++) CHECK(!stepperRunning(j) && simJ[j].v == 0, "joint %d moving after E-stop", j);
    CHECK(simJ[0].enabled, "drivers disabled on E-stop (must keep holding)");
    const float held = simJ[0].trueDeg;
    run(3.0f);                                               // master keeps sending: ignored
    CHECK(fabsf(simJ[0].trueDeg - held) < 1e-4f, "moved during E-stop");
    simEstop = false; run(0.5f);
    simEstop = true; run(0.05f);                             // released < 1 s: still E-stop
    CHECK(slaveState() == SS_ESTOP, "short release started homing");
    simEstop = false; run(0.9f);
    CHECK(slaveState() == SS_ESTOP, "homing started before ESTOP_RELEASE_MS");
    CHECK(runUntil(SS_HOMING, 0.3f), "homing did not start after release");
    CHECK(runUntil(SS_RUN, 90.0f), "homing after E-stop did not finish");
}

static void test_limit_in_run() {
    printf("[limit switch while running]\n");
    simSlip(0, (int32_t)((HOME_SWITCH_DEG[0] - simJ[0].trueDeg - 1.0f) * 26.6667f));   // joint knocked onto its switch
    run(0.05f);
    CHECK(slaveState() == SS_FAULT, "limit in RUN not a fault, state %d", slaveState());
    simEstop = true; run(0.05f); simEstop = false;
    CHECK(runUntil(SS_RUN, 90.0f), "recovery after limit fault failed");
}

static void test_start_on_switch_and_encoder_cases() {
    printf("[start on switch, inverted encoder, wrong encoder ratio, missing encoder]\n");
    const float start[3] = { -101.0f, 10.0f, -11.0f };       // joints 0 and 2 resting on their switches
    simReset(start);
    simJ[0].encSign = -1;                                    // magnet/sensor mounted the other way
    simJ[1].encScale = 2.0f;                                 // e.g. J3 jumpers at 1/8 while MICROSTEPS = 16
    simJ[2].encPresent = false;
    g_send = false;
    slaveBegin();
    CHECK(runUntil(SS_RUN, 120.0f), "homing failed, state %d", slaveState());
    CHECK(slaveEncoderVerified(0), "inverted encoder rejected");
    CHECK(!slaveEncoderVerified(1), "wrong-ratio encoder accepted");
    CHECK(!slaveEncoderVerified(2), "missing encoder accepted");
    CHECK(maxPosErr() < 0.4f, "step vs true %.3f", maxPosErr());
    simSlip(0, 40); run(1.0f);
    CHECK(fabsf(simJ[0].trueDeg - READY_DEG[0]) < 0.3f, "inverted encoder did not correct: %.2f", simJ[0].trueDeg);
}

static void test_switch_not_found() {
    printf("[switch never triggers]\n");
    const float start[3] = { 0.0f, 30.0f, 60.0f };
    simReset(start);
    simJ[2].switchWorks = false;                              // elbow is homed first
    slaveBegin();
    CHECK(runUntil(SS_FAULT, 30.0f), "missing switch not detected, state %d", slaveState());
    CHECK(!slaveHomed(2), "joint homed without a switch");
}

static void test_estop_at_boot() {
    printf("[E_BTN held at boot]\n");
    const float start[3] = { 0.0f, 30.0f, 60.0f };
    simReset(start);
    simEstop = true;
    slaveBegin();
    run(3.0f);
    CHECK(slaveState() == SS_ESTOP, "state %d", slaveState());
    for (int j = 0; j < 3; j++) CHECK(simJ[j].moves == 0 && simJ[j].trueDeg == start[j], "joint %d moved with E_BTN held", j);
    simEstop = false;
    CHECK(runUntil(SS_RUN, 90.0f), "no homing after release");
}

static void test_binary_output() {
    if (!PC_OUTPUT_BINARY) return;
    printf("[binary PC output]\n");
    const float start[3] = { 0.0f, 30.0f, 60.0f };
    boot(start);
    CHECK(runUntil(SS_RUN, 90.0f), "homing");
    g_send = true;
    g_out.clear();
    run(2.0f);
    int reports = 0, joints = 0, logs = 0, bad = 0, linked = 0;
    size_t i = 0;
    while (i < g_out.size()) {
        if (g_out[i] != PKT_START || i + 2 > g_out.size()) { bad++; i++; continue; }
        size_t len = 0;
        const uint8_t t = g_out[i + 1];
        if (t == PKT_TYPE_SLAVE_REPORT) len = sizeof(SlaveReport);
        else if (t == PKT_TYPE_JOINTS) len = sizeof(JointPacket);
        else if ((t == PKT_TYPE_SLAVE_LOG || t == PKT_TYPE_LOG) && i + 10 < g_out.size()) len = LOG_HEADER_LEN + g_out[i + 9] + 1;
        if (len == 0 || i + len > g_out.size() || crc8(&g_out[i], len - 1) != g_out[i + len - 1]) { bad++; i++; continue; }
        if (t == PKT_TYPE_SLAVE_REPORT) {
            SlaveReport r; memcpy(&r, &g_out[i], sizeof(r)); reports++;
            CHECK(r.state == SS_RUN && (r.flags & SF_HOMED), "report state %d flags 0x%02X", r.state, r.flags);
            if (r.flags & SF_LINK_OK) linked++;
        } else if (t == PKT_TYPE_JOINTS) joints++; else logs++;
        i += len;
    }
    CHECK(bad == 0, "%d undecodable bytes in binary output", bad);
    CHECK(linked >= reports - 2, "reports with link ok: %d of %d", linked, reports);
    CHECK(reports >= 95 && reports <= 101, "reports in 2 s: %d", reports);
    CHECK(joints >= 98 && joints <= 101, "forwarded joint frames in 2 s: %d", joints);
}

int main() {
    printf("slave: %d microsteps/deg, base switch at %.0f deg\n", (int)lroundf(MOTOR_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO[0] / 360.0f), HOME_SWITCH_DEG[0]);
    if (PC_OUTPUT_BINARY) { test_binary_output(); }
    else {
        test_homing();
        test_follow_and_link();
        test_resync_and_stall();
        test_estop();
        test_limit_in_run();
        test_start_on_switch_and_encoder_cases();
        test_switch_not_found();
        test_estop_at_boot();
    }
    printf("\n%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
