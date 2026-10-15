#include <Arduino.h>
#include <MPU6500_WE.h>

#include "configurations.h"
#include "connections.h"
#include "kinematics.h"

unsigned long now = 0;
MPU6500_WE myMPU6500 = MPU6500_WE(&SPI, csPin_6500, mosiPin_6500, misoPin_6500, sckPin_6500, useSPI);

void MPUInit();
bool IsMPUSynced();

void setup() {
  Serial.begin(115200);
  
  MPUInit();
  
}

void loop() {

  now = micros();   

  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();
  float resultantG = myMPU6500.getResultantG(gValue);

  Serial.print(micros() - now);
  Serial.println("µs");
  
  Serial.println("Acceleration in g (x,y,z):");
  Serial.print(gValue.x);
  Serial.print("   ");
  Serial.print(gValue.y);
  Serial.print("   ");
  Serial.println(gValue.z);
  Serial.print("Resultant g: ");
  Serial.println(resultantG);

  Serial.println("Gyroscope data in degrees/s: ");
  Serial.print(gyr.x);
  Serial.print("   ");
  Serial.print(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);

  Serial.print("Temperature in °C: ");
  Serial.println(temp);
  

  Serial.println("********************************************");

  delay(1000);
}

void MPUInit() {
  
}

bool IsMPUSynced() {
  
}