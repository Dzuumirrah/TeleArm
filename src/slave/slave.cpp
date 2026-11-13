// TeleArm slave: ESP-NOW joint commands from the master -> 3 x A4988 / stepper, AS5600 check,
// homing at boot and after E_BTN, limit switches, report to the PC on USB serial (115200).
#include <Arduino.h>
#include "slave_session.h"

void setup() {
    Serial.begin(115200);
    delay(300);
    slaveBegin();
}

void loop() {
    slaveStep();
}
