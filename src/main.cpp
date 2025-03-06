#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <Kalman.h>
#include <kendali.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
  previousTime = millis(); 
  PIDRoll.SetMode(AUTOMATIC);
  PIDPitch.SetMode(AUTOMATIC);
  PIDYaw.SetMode(AUTOMATIC);
}

void loop() {
  ambil_data();
  kendali();

  Serial.print("Roll: "); Serial.print(Roll);
  Serial.print(" | Pitch: "); Serial.print(Pitch);
  Serial.print(" | Yaw: "); Serial.print(Yaw);

  Serial.print(" | U1: "); Serial.print(URoll);
  Serial.print(" | U2: "); Serial.print(UPitch);
  Serial.print(" | U3: "); Serial.println(UYaw);

  delay(1);
}