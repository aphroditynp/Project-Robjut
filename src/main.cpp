#include <Wire.h>
#include <Arduino.h>
#include <akuisisi.h>
#include <math.h>
#include <Kalman.h>


void setup() {
  Wire.begin();
  Serial.begin(9600);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
  previousTime = millis(); 
}

void loop() {
  ambil_data();

  Serial.print("Roll: "); Serial.print(Roll);
  Serial.print(" | Pitch: "); Serial.println(Pitch);

  delay(1);
}