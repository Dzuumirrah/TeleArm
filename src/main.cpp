#include <Arduino.h>

#include "configurations.h"
#include "connections.h"
#include "kinematics.h"
#include "session.h"

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("TeleArm master");
  sessionBegin();
}

void loop() {
  sessionStep();
}
