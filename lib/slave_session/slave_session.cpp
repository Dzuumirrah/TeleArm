#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <stdarg.h>
#include "slave_session.h"
#include "actuation.h"
#include "configurations.h"

namespace {

const uint32_t TICK_US          = 5000;    // 200 Hz control loop
const int      ENC_COUNTS       = 4096;
const int      ENC_MAX_ERR_TICKS = 10;     // 50 ms of failed reads -> encoder dropped until the next homing
const uint8_t  DEBOUNCE_TICKS   = 2;       // 10 ms (switch lines also have an RC filter)
const uint32_t BOOT_SETTLE_MS   = 300;
const float    LATCH_MAX_TRAVEL_DEG = 20.0f;
const uint32_t PARK_TIMEOUT_MS  = 20000;

// ---------------- ESP-NOW receive ring (callback runs in the WiFi task, it only copies)
const int RX_SLOTS = 24;
struct RxSlot { uint8_t len; uint8_t data[LOG_HEADER_LEN + LOG_MAX_TEXT + 1]; };
RxSlot            rx[RX_SLOTS];
volatile uint32_t rxWr = 0, rxRd = 0, rxOverrun = 0, rxBad = 0;

// ---------------- state
SlaveState state = SS_BOOT;
uint32_t   nextTickUs = 0, bootMs = 0;
bool       tickInit = false;

float spd[3];                 // microsteps per joint degree
float cpd[3];                 // encoder counts per joint degree

bool    lsOn[3], esOn;        // debounced inputs
uint8_t lsCnt[3], esCnt;

struct Enc {
    bool     present, primed, verified;
    int8_t   sign;
    uint16_t last;
    int32_t  acc, zeroAcc;    // unwrapped multi-turn counts
    float    zeroDeg;
    uint8_t  errTicks;
    uint32_t errSinceMs;      // standstill error first seen (0 = none)
    uint16_t resyncs;
} enc[3];

bool      homed[3];
int       homeIdx = 0;
HomePhase phase = HP_LEAVE;
bool      phaseMoving = false;
int32_t   phaseStartPos = 0, latchPos[3];
int32_t   latchAcc[3];
uint32_t  phaseStartMs = 0;

float     masterDeg[3] = { NAN, NAN, NAN };
float     target[3];
bool      haveLink = false, linkWasOk = false, newPacket = false;
uint32_t  lastPktMs = 0;
uint16_t  masterSeq = 0;
bool      haveSeq = false;
uint32_t  jointOk = 0, jointLost = 0;
uint8_t   masterFlags = 0;
uint32_t  estopReleaseMs = 0;

uint16_t  reportSeq = 0, slogSeq = 0;
uint32_t  lastReportMs = 0, lastTextMs = 0, lastOverrun = 0;

const char *const STATE_NAME[] = { "BOOT", "HOMING", "RUN", "ESTOP", "FAULT" };
const char *const PHASE_NAME[] = { "LEAVE", "SEEK", "LATCH", "PARK" };
const char *const JOINT_NAME[] = { "base", "shoulder", "elbow" };

// ---------------- output
void slog(uint8_t level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void slog(uint8_t level, const char *fmt, ...) {
    char text[LOG_MAX_TEXT + 1];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)LOG_MAX_TEXT) n = LOG_MAX_TEXT;
    const uint32_t t = millis();
    if (!PC_OUTPUT_BINARY) {
        static const char lv[] = { 'D', 'I', 'W', 'E' };
        Serial.printf("[S %lu.%03lu %c] %s\n", (unsigned long)(t / 1000UL), (unsigned long)(t % 1000UL), lv[level & 3], text);
        return;
    }
    uint8_t f[LOG_HEADER_LEN + LOG_MAX_TEXT + 1];
    f[0] = PKT_START; f[1] = PKT_TYPE_SLAVE_LOG;
    const uint16_t s = slogSeq++;
    memcpy(&f[2], &s, 2); memcpy(&f[4], &t, 4);
    f[8] = level; f[9] = (uint8_t)n;
    memcpy(&f[LOG_HEADER_LEN], text, (size_t)n);
    f[LOG_HEADER_LEN + n] = crc8(f, LOG_HEADER_LEN + n);
    Serial.write(f, LOG_HEADER_LEN + n + 1);
}

// ---------------- helpers
float   clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
float   stepDeg(int j)              { return (float)stepperPos(j) / spd[j]; }
int32_t degToSteps(int j, float d)  { return (int32_t)lroundf(d * spd[j]); }
float   encDeg(int j)               { return enc[j].zeroDeg + enc[j].sign * (float)(enc[j].acc - enc[j].zeroAcc) / cpd[j]; }
bool    encUsable(int j)            { return enc[j].present && enc[j].verified && homed[j]; }
void    setSpeed(int j, float dps, float dps2) {
    stepperSetSpeedAccel(j, (uint32_t)lroundf(dps * spd[j]), (uint32_t)lroundf(dps2 * spd[j]));
}
void haltAll() { for (int j = 0; j < 3; j++) stepperHalt(j); }
void stopAll() { for (int j = 0; j < 3; j++) stepperStop(j); }

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *, const uint8_t *data, int len) {
#else
void onRecv(const uint8_t *, const uint8_t *data, int len) {
#endif
    if (len <= 0 || len > (int)sizeof(rx[0].data) || packetType(data, len) == 0) { rxBad++; return; }
    if (rxWr - rxRd >= (uint32_t)RX_SLOTS) { rxOverrun++; return; }
    RxSlot &s = rx[rxWr % RX_SLOTS];
    s.len = (uint8_t)len;
    memcpy(s.data, data, (size_t)len);
    rxWr++;
}

// ---------------- inputs
void debounce(bool raw, bool &on, uint8_t &cnt) {
    if (raw == on) { cnt = 0; return; }
    if (++cnt >= DEBOUNCE_TICKS) { on = raw; cnt = 0; }
}

void readInputs() {
    for (int j = 0; j < 3; j++) debounce(limitActive(j), lsOn[j], lsCnt[j]);
    debounce(estopActive(), esOn, esCnt);
}

// ---------------- encoders
void probeEncoders() {
    for (int j = 0; j < 3; j++) {
        Enc &e = enc[j];
        const uint8_t s = encoderStatus(j);
        e.primed = false; e.verified = false; e.errTicks = 0; e.errSinceMs = 0;
        e.present = (s != ENC_NO_ANSWER) && (s & AS5600_MD);
        if (s == ENC_NO_ANSWER) slog(LOGLVL_WARN, "encoder %d (%s): no answer on mux channel %u - open loop", j, JOINT_NAME[j], (unsigned)SLV_ENC_MUX_CH[j]);
        else if (!(s & AS5600_MD)) slog(LOGLVL_WARN, "encoder %d (%s): no magnet detected - open loop", j, JOINT_NAME[j]);
        else if (s & (AS5600_ML | AS5600_MH)) slog(LOGLVL_WARN, "encoder %d (%s): magnet too %s - check the gap", j, JOINT_NAME[j], (s & AS5600_ML) ? "weak" : "strong");
    }
}

void readEncoders() {
    for (int j = 0; j < 3; j++) {
        Enc &e = enc[j];
        if (!e.present) continue;
        uint16_t raw;
        if (!encoderRaw(j, raw)) {
            if (++e.errTicks >= ENC_MAX_ERR_TICKS) {
                e.present = false; e.verified = false;
                slog(LOGLVL_WARN, "encoder %d (%s): I2C reads failing - open loop until the next homing", j, JOINT_NAME[j]);
            }
            continue;
        }
        e.errTicks = 0;
        if (!e.primed) { e.last = raw; e.primed = true; continue; }
        int d = (int)raw - (int)e.last;               // shortest way round: the motor turns < 180 deg per 5 ms tick
        if (d > ENC_COUNTS / 2)   d -= ENC_COUNTS;
        if (d < -ENC_COUNTS / 2)  d += ENC_COUNTS;
        e.acc += d;
        e.last = raw;
    }
}

// After homing, compare how far the encoder turned with how far the steps say the joint moved.
void verifyEncoder(int j) {
    Enc &e = enc[j];
    if (!e.present || !e.primed) { slog(LOGLVL_WARN, "joint %d (%s): no encoder - open loop", j, JOINT_NAME[j]); return; }
    const float sTravel = (float)(stepperPos(j) - latchPos[j]) / spd[j];
    const float eTravel = (float)(e.acc - latchAcc[j]) / cpd[j];
    if (fabsf(sTravel) < ENC_CHECK_MIN_DEG) {
        slog(LOGLVL_WARN, "joint %d (%s): park move %.1f deg too short to verify the encoder - open loop", j, JOINT_NAME[j], sTravel);
        return;
    }
    const float r = eTravel / sTravel;
    if (fabsf(fabsf(r) - 1.0f) > ENC_RATIO_TOL) {
        slog(LOGLVL_WARN, "joint %d (%s): encoder moved %.2fx the step motion - ignored. Check MICROSTEPS (J3), "
             "GEAR_RATIO / ENC_TURNS_PER_JOINT_TURN, magnet", j, JOINT_NAME[j], r);
        return;
    }
    e.sign = r > 0 ? 1 : -1;
    e.verified = true;
    slog(LOGLVL_INFO, "joint %d (%s): encoder verified (ratio %.3f)", j, JOINT_NAME[j], r);
}

// ---------------- state changes
void enterFault(const char *why) {
    haltAll();
    state = SS_FAULT;
    slog(LOGLVL_ERROR, "FAULT: %s", why);
    slog(LOGLVL_ERROR, "motors stopped and holding. Press and release E_BTN to re-home.");
}

void enterEstop() {
    haltAll();
    state = SS_ESTOP;
    estopReleaseMs = 0;
    slog(LOGLVL_WARN, "E-STOP: motors stopped and holding. Release E_BTN to re-home.");
}

void beginJoint(uint32_t nowMs) {
    const int j = HOME_ORDER[homeIdx];
    setSpeed(j, HOME_SEEK_DPS, HOME_ACCEL_DPS2);
    phaseStartPos = stepperPos(j);
    phaseStartMs = nowMs;
    phaseMoving = true;
    if (lsOn[j]) { phase = HP_LEAVE; stepperRun(j, -HOME_DIR[j]); }   // already on the switch: get off it first
    else         { phase = HP_SEEK;  stepperRun(j,  HOME_DIR[j]); }
    slog(LOGLVL_INFO, "homing joint %d (%s): %s", j, JOINT_NAME[j], PHASE_NAME[phase]);
}

void startHoming(uint32_t nowMs) {
    stopAll();
    for (int j = 0; j < 3; j++) homed[j] = false;
    probeEncoders();
    state = SS_HOMING;
    homeIdx = 0;
    slog(LOGLVL_INFO, "homing (order %d, %d, %d)", HOME_ORDER[0], HOME_ORDER[1], HOME_ORDER[2]);
    beginJoint(nowMs);
}

void enterRun() {
    state = SS_RUN;
    linkWasOk = false;
    newPacket = false;
    for (int j = 0; j < 3; j++) {
        setSpeed(j, RUN_MAX_DPS, RUN_ACCEL_DPS2);
        target[j] = stepDeg(j);
        enc[j].errSinceMs = 0;
    }
    slog(LOGLVL_INFO, "homing done - following the master");
}

// ---------------- per-state ticks
void homingTick(uint32_t nowMs) {
    const int j = HOME_ORDER[homeIdx];
    char msg[140];
    for (int k = 0; k < 3; k++)
        if (homed[k] && k != j && lsOn[k]) {
            snprintf(msg, sizeof(msg), "joint %d (%s) hit its limit switch while joint %d was homing", k, JOINT_NAME[k], j);
            enterFault(msg); return;
        }
    const float travel = fabsf((float)(stepperPos(j) - phaseStartPos) / spd[j]);

    switch (phase) {
    case HP_LEAVE:
        if (!lsOn[j]) { stepperStop(j); phase = HP_SEEK; phaseMoving = false; return; }
        if (travel > HOME_MAX_TRAVEL_DEG[j]) {
            snprintf(msg, sizeof(msg), "joint %d (%s): switch still active after %.0f deg (LS also reads active when its wire is broken)", j, JOINT_NAME[j], travel);
            enterFault(msg);
        }
        return;
    case HP_SEEK:
        if (!phaseMoving) {
            if (stepperRunning(j)) return;
            phaseStartPos = stepperPos(j);
            stepperRun(j, HOME_DIR[j]);
            phaseMoving = true;
            return;
        }
        if (lsOn[j]) { stepperHalt(j); phase = HP_LATCH; phaseMoving = false; return; }
        if (travel > HOME_MAX_TRAVEL_DEG[j]) {
            snprintf(msg, sizeof(msg), "joint %d (%s): switch not found within %.0f deg - check HOME_DIR / DIR_INVERT and the switch", j, JOINT_NAME[j], travel);
            enterFault(msg);
        }
        return;
    case HP_LATCH:
        if (!phaseMoving) {
            if (stepperRunning(j)) return;
            setSpeed(j, HOME_LATCH_DPS, HOME_ACCEL_DPS2);
            phaseStartPos = stepperPos(j);
            stepperRun(j, -HOME_DIR[j]);
            phaseMoving = true;
            return;
        }
        if (!lsOn[j]) { stepperHalt(j); phase = HP_PARK; phaseMoving = false; return; }
        if (travel > LATCH_MAX_TRAVEL_DEG) {
            snprintf(msg, sizeof(msg), "joint %d (%s): switch does not release after %.0f deg", j, JOINT_NAME[j], travel);
            enterFault(msg);
        }
        return;
    case HP_PARK:
        if (!phaseMoving) {
            if (stepperRunning(j)) return;
            stepperSetPos(j, degToSteps(j, HOME_SWITCH_DEG[j]));        // the release point defines the zero
            enc[j].zeroAcc = enc[j].acc;
            enc[j].zeroDeg = HOME_SWITCH_DEG[j];
            latchPos[j] = stepperPos(j);
            latchAcc[j] = enc[j].acc;
            homed[j] = true;
            setSpeed(j, PARK_DPS, HOME_ACCEL_DPS2);
            stepperMoveTo(j, degToSteps(j, READY_DEG[j]));
            phaseMoving = true;
            phaseStartMs = nowMs;
            slog(LOGLVL_INFO, "joint %d (%s): homed at %.1f deg, moving to %.1f deg", j, JOINT_NAME[j], HOME_SWITCH_DEG[j], READY_DEG[j]);
            return;
        }
        if (lsOn[j]) {
            snprintf(msg, sizeof(msg), "joint %d (%s): switch active again while moving away from it", j, JOINT_NAME[j]);
            enterFault(msg); return;
        }
        if (stepperRunning(j)) {
            if (nowMs - phaseStartMs > PARK_TIMEOUT_MS) {
                snprintf(msg, sizeof(msg), "joint %d (%s): park move did not finish", j, JOINT_NAME[j]);
                enterFault(msg);
            }
            return;
        }
        verifyEncoder(j);
        if (++homeIdx >= 3) enterRun();
        else beginJoint(nowMs);
        return;
    }
}

void closedLoop(uint32_t nowMs, bool linkOk) {
    char msg[140];
    for (int j = 0; j < 3; j++) {
        if (!encUsable(j)) continue;
        Enc &e = enc[j];
        const float err = encDeg(j) - stepDeg(j);
        if (fabsf(err) > ENC_STALL_DEG) {
            snprintf(msg, sizeof(msg), "joint %d (%s): encoder and step count differ by %.1f deg (stall / lost steps)", j, JOINT_NAME[j], err);
            enterFault(msg); return;
        }
        if (fabsf(err) <= ENC_RESYNC_DEG) { e.errSinceMs = 0; continue; }
        if (e.errSinceMs == 0) { e.errSinceMs = nowMs ? nowMs : 1; continue; }
        if (stepperRunning(j) || nowMs - e.errSinceMs < ENC_RESYNC_MS) continue;   // correct only at standstill
        // Steps were lost at standstill or during the last move: the encoder tells where the joint really is.
        const float hold = stepDeg(j);                       // where the step counter thought the joint was
        stepperSetPos(j, degToSteps(j, encDeg(j)));
        stepperMoveTo(j, degToSteps(j, linkOk ? target[j] : hold));
        e.errSinceMs = 0;
        e.resyncs++;
        slog(LOGLVL_WARN, "joint %d (%s): corrected %.2f deg of lost steps from the encoder", j, JOINT_NAME[j], err);
    }
}

void runTick(uint32_t nowMs) {
    char msg[140];
    for (int j = 0; j < 3; j++)
        if (lsOn[j]) {
            snprintf(msg, sizeof(msg), "joint %d (%s) hit its limit switch while running (lost steps, or soft limit beyond the switch)", j, JOINT_NAME[j]);
            enterFault(msg); return;
        }
    const bool linkOk = haveLink && (nowMs - lastPktMs) < LINK_TIMEOUT_MS;
    if (!linkOk && linkWasOk) {
        stopAll();
        slog(LOGLVL_WARN, "no joint data from the master for %lu ms - holding", (unsigned long)LINK_TIMEOUT_MS);
    }
    if (linkOk && newPacket) {
        if (!linkWasOk) slog(LOGLVL_INFO, "joint data from the master - following");
        for (int j = 0; j < 3; j++) {
            target[j] = clampf(masterDeg[j], JOINT_MIN_DEG[j], JOINT_MAX_DEG[j]);
            const int32_t t = degToSteps(j, target[j]);
            if (t != stepperPos(j) || stepperRunning(j)) stepperMoveTo(j, t);
        }
    }
    newPacket = false;
    linkWasOk = linkOk;
    closedLoop(nowMs, linkOk);
}

// ---------------- master frames
void handleJoint(const JointPacket &p, uint32_t nowMs) {
    if (haveSeq) jointLost += (uint16_t)(p.seq - masterSeq - 1);
    masterSeq = p.seq; haveSeq = true; jointOk++;
    masterFlags = p.flags;
    if (!(p.flags & PKT_F_CALIBRATED)) return;
    for (int j = 0; j < 3; j++) if (!isfinite(p.joint_deg[j])) return;
    for (int j = 0; j < 3; j++) masterDeg[j] = p.joint_deg[j];
    lastPktMs = nowMs;
    haveLink = true;
    newPacket = true;
}

void drainRx(uint32_t nowMs) {
    while (rxRd != rxWr) {
        const RxSlot &s = rx[rxRd % RX_SLOTS];
        const uint8_t type = packetType(s.data, s.len);
        if (PC_OUTPUT_BINARY) Serial.write(s.data, s.len);
        if (type == PKT_TYPE_JOINTS) {
            JointPacket p;
            memcpy(&p, s.data, sizeof(p));
            handleJoint(p, nowMs);
        } else if (!PC_OUTPUT_BINARY) {
            char line[300];
            if (type == PKT_TYPE_LOG) {
                LogFrame f;
                if (decodeLogPacket(s.data, s.len, f)) { formatLogLine(f, line, sizeof(line)); Serial.println(line); }
            } else if (type == PKT_TYPE_STATUS) {
                StatusPacket st;
                if (decodeStatusPacket(s.data, s.len, st)) { formatStatusLine(st, line, sizeof(line)); Serial.println(line); }
            }
        }
        rxRd++;
    }
    if (rxOverrun != lastOverrun) {
        slog(LOGLVL_WARN, "%lu frame(s) from the master dropped (slave loop too slow)", (unsigned long)(rxOverrun - lastOverrun));
        lastOverrun = rxOverrun;
    }
}

// ---------------- PC reporting
void report(uint32_t nowMs) {
    const bool linkOk = haveLink && (nowMs - lastPktMs) < LINK_TIMEOUT_MS;
    float e[3];
    for (int j = 0; j < 3; j++) e[j] = (enc[j].present && enc[j].primed) ? encDeg(j) : NAN;   // relative to power-up until homed
    const int hj = (state == SS_HOMING) ? HOME_ORDER[homeIdx] : 0;

    if (!PC_OUTPUT_BINARY) {
        if (nowMs - lastTextMs < PC_TEXT_PERIOD_MS) return;
        lastTextMs = nowMs;
        char where[24] = "";
        if (state == SS_HOMING) snprintf(where, sizeof(where), " j%d %s", hj, PHASE_NAME[phase]);
        Serial.printf("S %s%s cmd=%.1f,%.1f,%.1f pos=%.1f,%.1f,%.1f enc=%.1f,%.1f,%.1f ls=%d%d%d es=%d link=%s mseq=%u ok=%lu lost=%lu bad=%lu\n",
                      STATE_NAME[state], where, target[0], target[1], target[2],
                      stepDeg(0), stepDeg(1), stepDeg(2), e[0], e[1], e[2],
                      lsOn[0], lsOn[1], lsOn[2], esOn, linkOk ? "ok" : "--", (unsigned)masterSeq,
                      (unsigned long)jointOk, (unsigned long)jointLost, (unsigned long)rxBad);
        return;
    }
    if (nowMs - lastReportMs < PC_REPORT_PERIOD_MS) return;
    lastReportMs = nowMs;
    SlaveReport r;
    r.start = PKT_START; r.type = PKT_TYPE_SLAVE_REPORT; r.seq = reportSeq++; r.t_ms = nowMs;
    r.state = state;
    r.detail = (state == SS_HOMING) ? (uint8_t)((hj << 4) | phase) : 0;
    r.flags = (homed[0] && homed[1] && homed[2] ? SF_HOMED : 0) | (linkOk ? SF_LINK_OK : 0)
            | ((masterFlags & PKT_F_IMU_FAULT) ? SF_MASTER_IMU : 0);
    r.inputs = esOn ? SI_ESTOP : 0;
    for (int j = 0; j < 3; j++) {
        if (encUsable(j)) r.flags |= (uint8_t)(SF_ENC0_OK << j);
        if (lsOn[j])      r.inputs |= (uint8_t)(SI_LIMIT0 << j);
        r.target_deg[j] = target[j];
        r.step_deg[j] = stepDeg(j);
        r.enc_deg[j] = e[j];
    }
    r.masterSeq = masterSeq;
    r.crc = crc8((const uint8_t *)&r, sizeof(r) - 1);
    Serial.write((const uint8_t *)&r, sizeof(r));
}

void checkConfig() {
    for (int j = 0; j < 3; j++) {
        if (HOME_SWITCH_DEG[j] >= JOINT_MIN_DEG[j] && HOME_SWITCH_DEG[j] <= JOINT_MAX_DEG[j])
            slog(LOGLVL_WARN, "config: HOME_SWITCH_DEG[%d] lies inside JOINT_MIN/MAX_DEG - normal motion would hit the switch", j);
        if (READY_DEG[j] < JOINT_MIN_DEG[j] || READY_DEG[j] > JOINT_MAX_DEG[j])
            slog(LOGLVL_WARN, "config: READY_DEG[%d] lies outside JOINT_MIN/MAX_DEG", j);
        if ((HOME_SWITCH_DEG[j] - READY_DEG[j]) * HOME_DIR[j] <= 0)
            slog(LOGLVL_WARN, "config: HOME_DIR[%d] does not point from READY_DEG to HOME_SWITCH_DEG", j);
    }
}

void tick(uint32_t nowMs) {
    readInputs();
    readEncoders();
    drainRx(nowMs);

    if (esOn && state != SS_ESTOP) enterEstop();

    switch (state) {
    case SS_BOOT:
        if (nowMs - bootMs >= BOOT_SETTLE_MS) startHoming(nowMs);
        break;
    case SS_HOMING: homingTick(nowMs); break;
    case SS_RUN:    runTick(nowMs); break;
    case SS_ESTOP:
        if (esOn) { estopReleaseMs = 0; break; }
        if (estopReleaseMs == 0) { estopReleaseMs = nowMs ? nowMs : 1; break; }
        if (nowMs - estopReleaseMs >= ESTOP_RELEASE_MS) { slog(LOGLVL_INFO, "E_BTN released"); startHoming(nowMs); }
        break;
    case SS_FAULT: break;                      // only E_BTN (press -> ESTOP -> release -> homing) leaves FAULT
    }
    report(nowMs);
}

} // namespace

bool slaveBegin() {
    rxWr = rxRd = rxOverrun = rxBad = 0;
    state = SS_BOOT; tickInit = false;
    haveLink = linkWasOk = newPacket = haveSeq = false;
    jointOk = jointLost = 0; masterFlags = 0; lastOverrun = 0;
    lastReportMs = lastTextMs = 0;
    for (int j = 0; j < 3; j++) {
        spd[j] = (float)MOTOR_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO[j] / 360.0f;
        cpd[j] = (float)ENC_COUNTS * ENC_TURNS_PER_JOINT_TURN[j] / 360.0f;
        homed[j] = false;
        enc[j] = Enc();
        enc[j].sign = 1;
        masterDeg[j] = NAN;
        target[j] = 0.0f;
    }
    bool ok = true;
    if (!actuatorsInit()) { slog(LOGLVL_ERROR, "stepper init failed"); ok = false; }
    if (!encodersInit()) slog(LOGLVL_WARN, "TCA9548A does not answer at 0x%02X - all joints open loop", (unsigned)TCA9548A_ADDR);
    probeEncoders();                           // also read while waiting in E-stop (bring-up: turn a motor by hand)

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    uint8_t mac[6];
    WiFi.macAddress(mac);
    slog(LOGLVL_INFO, "TeleArm slave, MAC %02X:%02X:%02X:%02X:%02X:%02X (must match SLAVE_MAC), channel %u",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], (unsigned)ESPNOW_CHANNEL);
    if (esp_now_init() != ESP_OK) { slog(LOGLVL_ERROR, "esp_now_init failed"); ok = false; }
    else esp_now_register_recv_cb(onRecv);

    checkConfig();
    for (int j = 0; j < 3; j++) { lsOn[j] = limitActive(j); lsCnt[j] = 0; }
    esOn = estopActive(); esCnt = 0;
    bootMs = millis();
    return ok;
}

void slaveStep() {
    const uint32_t now = micros();
    if (!tickInit) { nextTickUs = now; tickInit = true; }
    if ((int32_t)(now - nextTickUs) < 0) return;
    nextTickUs += TICK_US;
    if ((int32_t)(now - nextTickUs) > (int32_t)(5 * TICK_US)) nextTickUs = now + TICK_US;   // fell behind: resync
    tick(millis());
}

SlaveState slaveState()            { return state; }
bool  slaveHomed(int j)            { return homed[j]; }
bool  slaveEncoderVerified(int j)  { return encUsable(j); }
float slaveStepDeg(int j)          { return stepDeg(j); }
float slaveEncDeg(int j)           { return (enc[j].present && enc[j].primed) ? encDeg(j) : NAN; }
float slaveTargetDeg(int j)        { return target[j]; }
uint16_t slaveResyncCount(int j)   { return enc[j].resyncs; }
