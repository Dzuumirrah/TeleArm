// Remote monitor: master serial output -> LOG frames, plus the shared LOG / STATUS decoders.
// Queue design: one ring of LOG_RING_LINES lines indexed by running counters.
//   wr   = number of lines ever written          rd = next line to transmit
// When the ring is full the oldest unsent line is overwritten (the newest calibration prompt matters most)
// and counted as dropped. The last REMOTE_LOG_REPLAY_LINES lines are re-sent when the link comes back up.
#include "connections.h"
#include "configurations.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace {

struct Entry {
    uint32_t t_ms;
    uint8_t  level;
    uint8_t  len;
    char     text[LOG_MAX_TEXT];
};

Entry    ring[LOG_RING_LINES];
uint32_t wr = 0, rd = 0;
uint32_t replayEnd = 0;          // lines with index < replayEnd are re-sends
uint16_t dropped = 0;
uint16_t logSeq = 0;
uint32_t lastSendMs = 0;
bool     everSent = false;

bool sendOne() {
    if (rd == wr) return false;
    if (wr - rd > (uint32_t)LOG_RING_LINES) {            // overwritten while waiting
        const uint32_t lost = wr - rd - LOG_RING_LINES;
        dropped = (uint16_t)(dropped + lost);
        rd = wr - LOG_RING_LINES;
    }
    const Entry &e = ring[rd % LOG_RING_LINES];
    uint8_t buf[LOG_HEADER_LEN + LOG_MAX_TEXT + 1];
    buf[0] = PKT_START;
    buf[1] = PKT_TYPE_LOG;
    const uint16_t sq = logSeq++;
    memcpy(&buf[2], &sq, 2);
    memcpy(&buf[4], &e.t_ms, 4);
    buf[8] = (uint8_t)(e.level | ((rd < replayEnd) ? LOG_REPLAY_BIT : 0));
    buf[9] = e.len;
    memcpy(&buf[LOG_HEADER_LEN], e.text, e.len);
    const size_t n = LOG_HEADER_LEN + e.len;
    buf[n] = crc8(buf, n);
    ESPNowSendRaw(buf, n + 1);
    rd++;
    return true;
}

void checkLinkEvent() {
    if (!ESPNowTakeLinkUpEvent()) return;
    if (!everSent && rd == 0) return;                    // first link-up and nothing sent yet: plain delivery of the queue
    const uint32_t keep = (uint32_t)REMOTE_LOG_REPLAY_LINES;
    uint32_t from = (wr > keep) ? wr - keep : 0;
    if (wr - from > (uint32_t)LOG_RING_LINES) from = wr - LOG_RING_LINES;
    if (from < rd) { replayEnd = rd; rd = from; }       // resend what may have gone into the void
}

}   // namespace

void logInit() { wr = rd = replayEnd = 0; dropped = 0; logSeq = 0; lastSendMs = 0; everSent = false; }

void logMsg(LogLevel level, const char *fmt, ...) {
    char tmp[LOG_MAX_TEXT + 1];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)LOG_MAX_TEXT) n = (int)LOG_MAX_TEXT;
    Serial.println(tmp);                                 // USB stays a full copy
    if (!REMOTE_LOG_ENABLED || level < REMOTE_LOG_MIN_LEVEL) return;
    Entry &e = ring[wr % LOG_RING_LINES];
    e.t_ms = millis();
    e.level = (uint8_t)level;
    e.len = (uint8_t)n;
    memcpy(e.text, tmp, (size_t)n);
    wr++;
}

void logService() {
    if (!REMOTE_LOG_ENABLED || !ESPNowReady()) return;
    checkLinkEvent();
    if (!ESPNowLinkUp() || rd == wr) return;
    const uint32_t now = millis();
    if (everSent && now - lastSendMs < REMOTE_LOG_MIN_INTERVAL_MS) return;
    if (sendOne()) { lastSendMs = now; everSent = true; }
}

void logFlush(uint32_t timeoutMs) {
    if (!REMOTE_LOG_ENABLED || !ESPNowReady()) return;
    const uint32_t t0 = millis();
    checkLinkEvent();
    while (ESPNowLinkUp() && rd != wr && millis() - t0 < timeoutMs) {
        if (sendOne()) { lastSendMs = millis(); everSent = true; }
        delay(3);                                        // let the radio drain; ESP-NOW has a short TX queue
    }
}

uint16_t logDroppedCount() {
    uint32_t d = dropped;
    if (wr - rd > (uint32_t)LOG_RING_LINES) d += wr - rd - LOG_RING_LINES;   // overwritten but not yet accounted
    return (uint16_t)d;
}
uint16_t logPendingCount() { const uint32_t p = wr - rd; return (uint16_t)(p > (uint32_t)LOG_RING_LINES ? LOG_RING_LINES : p); }

// ---------------------------------------------------------------------------------
//  shared decoders (slave + tests)
// ---------------------------------------------------------------------------------
uint8_t packetType(const uint8_t *d, int len) {
    if (len < 4 || d[0] != PKT_START) return 0;
    if (crc8(d, len - 1) != d[len - 1]) return 0;
    switch (d[1]) {
        case PKT_TYPE_JOINTS: return len == (int)sizeof(JointPacket) ? PKT_TYPE_JOINTS : 0;
        case PKT_TYPE_STATUS: return len == (int)sizeof(StatusPacket) ? PKT_TYPE_STATUS : 0;
        case PKT_TYPE_LOG:
            return (len >= (int)LOG_HEADER_LEN + 1 && d[9] <= LOG_MAX_TEXT && len == (int)LOG_HEADER_LEN + d[9] + 1) ? PKT_TYPE_LOG : 0;
    }
    return 0;
}

bool decodeLogPacket(const uint8_t *d, int len, LogFrame &o) {
    if (packetType(d, len) != PKT_TYPE_LOG) return false;
    memcpy(&o.seq, &d[2], 2);
    memcpy(&o.t_ms, &d[4], 4);
    o.replay = (d[8] & LOG_REPLAY_BIT) != 0;
    o.level = d[8] & (uint8_t)~LOG_REPLAY_BIT;
    o.len = d[9];
    memcpy(o.text, &d[LOG_HEADER_LEN], o.len);
    o.text[o.len] = 0;
    return true;
}

bool decodeStatusPacket(const uint8_t *d, int len, StatusPacket &o) {
    if (packetType(d, len) != PKT_TYPE_STATUS) return false;
    memcpy(&o, d, sizeof(o));
    return true;
}

size_t formatLogLine(const LogFrame &f, char *out, size_t n) {
    static const char lv[] = { 'D', 'I', 'W', 'E' };
    const int w = snprintf(out, n, "[M %lu.%03lu %c%s] %s", (unsigned long)(f.t_ms / 1000UL), (unsigned long)(f.t_ms % 1000UL),
                           lv[f.level & 3], f.replay ? " R" : "", f.text);
    return w < 0 ? 0 : (size_t)w;
}

size_t formatStatusLine(const StatusPacket &s, char *out, size_t n) {
    static const char *const st[] = { "RUN", "CAL_COUNTDOWN", "CAL_HOLD", "MAG_CAL" };
    static const char *const pl[] = { "-", "QUICK", "FULL", "MAG+FULL" };
    const int w = snprintf(out, n,
        "[M STATUS %lu.%03lu] state=%s plan=%s upper=%s fore=%s mag=%s%s cal=%s anchored=%s bench=%s flags=0x%02X tx_ok=%lu tx_fail=%lu log_dropped=%u",
        (unsigned long)(s.t_ms / 1000UL), (unsigned long)(s.t_ms % 1000UL),
        s.state < 4 ? st[s.state] : "?", s.plan < 4 ? pl[s.plan] : "?",
        (s.hw & HW_UPPER_OK) ? ((s.hw & HW_UPPER_9250) ? "MPU9250" : "MPU6500") : "MISSING",
        (s.hw & HW_FORE_OK) ? "MPU6500" : "MISSING",
        (s.hw & HW_MAG_OK) ? "ok" : "none", (s.hw & HW_MAG_USED) ? "(used)" : "",
        (s.hw & HW_CAL_VALID) ? "yes" : "no", (s.hw & HW_ANCHORED) ? "yes" : "no", (s.hw & HW_BENCH_OK) ? "yes" : "no",
        (unsigned)s.flags, (unsigned long)s.sentOk, (unsigned long)s.sentFail, (unsigned)s.logDropped);
    return w < 0 ? 0 : (size_t)w;
}
