#include "connections.h"
#include <Preferences.h>
#include <string.h>

static const uint32_t CAL_MAGIC = 0x544C4D31;   // "TLM1"
static const uint16_t CAL_VERSION = 1;
static const char *NS = "telearm";

static uint32_t checksum(const uint8_t *p, size_t n) {   // FNV-1a
    uint32_t h = 2166136261UL;
    for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619UL; }
    return h;
}

bool saveCalibration(const CalibrationData &in) {
    CalibrationData c = in;
    c.magic = CAL_MAGIC;
    c.version = CAL_VERSION;
    c.checksum = checksum((const uint8_t *)&c, offsetof(CalibrationData, checksum));
    Preferences p;
    if (!p.begin(NS, false)) return false;
    const size_t n = p.putBytes("cal", &c, sizeof(c));
    p.end();
    return n == sizeof(c);
}

bool loadCalibration(CalibrationData &out) {
    Preferences p;
    if (!p.begin(NS, true)) return false;
    CalibrationData c;
    bool ok = (p.getBytesLength("cal") == sizeof(c)) && (p.getBytes("cal", &c, sizeof(c)) == sizeof(c));
    p.end();
    if (!ok) return false;
    if (c.magic != CAL_MAGIC || c.version != CAL_VERSION) return false;
    if (c.checksum != checksum((const uint8_t *)&c, offsetof(CalibrationData, checksum))) return false;
    out = c;
    return true;
}

bool saveBenchOffsets(const BenchOffsets &b) {
    Preferences p;
    if (!p.begin(NS, false)) return false;
    const size_t n = p.putBytes("bench", &b, sizeof(b));
    p.end();
    return n == sizeof(b);
}

bool loadBenchOffsets(BenchOffsets &b) {
    Preferences p;
    if (!p.begin(NS, true)) return false;
    bool ok = (p.getBytesLength("bench") == sizeof(b)) && (p.getBytes("bench", &b, sizeof(b)) == sizeof(b));
    p.end();
    return ok;
}

void clearStorage() {
    Preferences p;
    if (!p.begin(NS, false)) return;
    p.clear();
    p.end();
}
