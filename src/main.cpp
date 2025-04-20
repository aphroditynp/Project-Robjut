#include <Wire.h>
#include <akuisisi.h>
#include <Arduino.h>
#include <math.h>
#include <Kalman.h>
#include <Radio.h>


void setup() {
  Wire.begin();
  Serial.begin(9600);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
  previousTime = millis(); 
  remote_setup();
}

void printUSB() {
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print(" | Pitch: "); Serial.print(pitch);
  Serial.print(" | Yaw: "); Serial.print(yaw);
  Serial.print(" | Altitude: "); Serial.print(read_altitude());
  Serial.print(" | Throttle: "); Serial.print(throttle);
  // Serial.print(" | U1: "); Serial.print(u1);
  // Serial.print(" | U2: "); Serial.print(u2);
  // Serial.print(" | U3: "); Serial.print(u3);
  // Serial.print(" | U4: "); Serial.println(u4);
}

void loop() {
  remote_loop();
  ambil_data_imu();
  read_altitude();
  printUSB();
}