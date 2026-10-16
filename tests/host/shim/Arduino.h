// Minimal Arduino/ESP32 shim: lets the firmware sources be compiled and unit-tested on a PC.
// NOT part of the firmware.
#pragma once
#ifndef ARDUINO
#define ARDUINO 10819
#endif
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#ifndef ESP32
#define ESP32 1
#endif
typedef uint8_t byte;
#ifndef sq
#define sq(x) ((x)*(x))
#endif
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0
#ifndef PI
#define PI 3.14159265358979323846
#endif
#define MSBFIRST 1
#define SPI_MODE0 0
struct String { String(const char * = "") {} };
void shimPinMode(int pin, int mode);
void shimDigitalWrite(int pin, int v);
int shimDigitalRead(int pin);
inline void pinMode(int p, int m) { shimPinMode(p, m); }
inline void digitalWrite(int p, int v) { shimDigitalWrite(p, v); }
inline int digitalRead(int p) { return shimDigitalRead(p); }
void delay(unsigned long ms);
extern void (*shimDelayHook)(unsigned long);
inline void delayMicroseconds(unsigned) {}
unsigned long micros();
unsigned long millis();
struct SerialShim {
    void begin(unsigned long) {}
    void print(const char *s) { printf("%s", s); }
    void print(float f, int d = 2) { printf("%.*f", d, f); }
    void print(int v) { printf("%d", v); }
    void print(unsigned v) { printf("%u", v); }
    void println(const char *s = "") { printf("%s\n", s); }
    void println(float f, int d = 2) { printf("%.*f\n", d, f); }
    void println(int v) { printf("%d\n", v); }
    int printf(const char *fmt, ...) __attribute__((format(printf, 2, 3))) {
        va_list ap; va_start(ap, fmt); int r = vprintf(fmt, ap); va_end(ap); return r; }
    void flush() {}
    int available() { return 0; }
};
extern SerialShim Serial;
