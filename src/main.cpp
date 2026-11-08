#include <Arduino.h>

#include "configurations.h"
#include "connections.h"
#include "kinematics.h"
#include "session.h"

const bool USE_BROWNOUT_DET = false;

void setup() {

  if (!USE_BROWNOUT_DET){
    #include "soc/soc.h"
    #include "soc/rtc_cntl_reg.h"
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  }
  Serial.begin(115200);
  delay(300);
  delay(10000);  
  Serial.println("TeleArm master");
  sessionBegin();
}

void loop() {
  sessionStep();
}
