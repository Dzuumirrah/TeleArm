#pragma once
#include <Arduino.h>
#define HSPI 2
#define VSPI 3
struct SPISettings {
    SPISettings() {}
    SPISettings(unsigned long, int, int) {}
};
struct SPIClass {
    SPIClass(int = VSPI) {}
    void begin(int = -1, int = -1, int = -1, int = -1) {}
    void beginTransaction(SPISettings) {}
    void endTransaction() {}
    uint8_t transfer(uint8_t v) { return v; }
};
extern SPIClass SPI;
