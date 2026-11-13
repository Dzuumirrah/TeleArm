#include <Arduino.h>
#include <Wire.h>
#include "actuation.h"
#include "configurations.h"

// All three AS5600 have the same address (0x36); the TCA9548A connects one of them at a time.
static const uint8_t AS5600_ADDR    = 0x36;
static const uint8_t REG_STATUS     = 0x0B;
static const uint8_t REG_RAW_ANGLE  = 0x0C;   // 0x0C (high 4 bits) + 0x0D
static int curCh = -1;                         // channel currently selected, -1 = unknown

static bool muxSelect(uint8_t ch) {
    if (curCh == ch) return true;
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write((uint8_t)(1u << ch));
    if (Wire.endTransmission() != 0) { curCh = -1; return false; }
    curCh = ch;
    return true;
}

static bool readRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(AS5600_ADDR, n) != n) return false;
    for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
    return true;
}

bool encodersInit() {
    Wire.begin(SLV_I2C_SDA, SLV_I2C_SCL, SLV_I2C_HZ);
    Wire.setTimeOut(5);                        // ms; a missing device must not stall the 200 Hz loop
    curCh = -1;
    return muxSelect(SLV_ENC_MUX_CH[0]);
}

uint8_t encoderStatus(int j) {
    uint8_t s;
    if (!muxSelect(SLV_ENC_MUX_CH[j]) || !readRegs(REG_STATUS, &s, 1)) return ENC_NO_ANSWER;
    return s & (AS5600_MD | AS5600_ML | AS5600_MH);
}

bool encoderRaw(int j, uint16_t &raw) {
    uint8_t b[2];
    if (!muxSelect(SLV_ENC_MUX_CH[j]) || !readRegs(REG_RAW_ANGLE, b, 2)) return false;
    raw = (uint16_t)(((b[0] & 0x0F) << 8) | b[1]);
    return true;
}
