#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <Kalman.h>
float Pitch = 0.0, Roll = 0.0; 
float accPitch = 0.0, accRoll = 0.0; 
float AcX= 0.0, AcY = 0.0, AcZ = 0.0;
const int MPU = 0x68;  // Alamat I2C MPU6050
const float alpha = 0.96; 
float dt = 0.01;
unsigned long previousTime, currentTime;

float tesuntukcommit;

Kalman kalmanX;
Kalman kalmanY;

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
  currentTime = millis();
  dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6);

  int16_t Ax = Wire.read() << 8 | Wire.read();
  int16_t Ay = Wire.read() << 8 | Wire.read();
  int16_t Az = Wire.read() << 8 | Wire.read();
  
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6);
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // Ax = AcX / 16384.0;  // Normalisasi (sesuai skala ±2g)
  // Ay = AcY / 16384.0;
  // Az = AcZ / 16384.0;

  float accRoll  = atan2(Ay, sqrt(Ax * Ax + Az * Az)) * 180.0 / PI;
  float accPitch = atan2(Ax, sqrt(Ay * Ay + Az * Az)) * 180.0 / PI;

  Pitch = alpha * (Pitch + (gy / 131.0) * dt) + (1 - alpha) * accPitch;
  Roll = alpha * (Roll + (gx / 131.0) * dt) + (1 - alpha) * accRoll;

  float gyroRateX = gx / 131.0;
  float gyroRateY = gy / 131.0;

  Roll = kalmanX.getAngle(accRoll, gyroRateX, dt);
  Pitch = kalmanY.getAngle(accPitch, gyroRateY, dt);

  Serial.print("Roll: "); Serial.print(Roll);
  Serial.print(" | Pitch: "); Serial.println(Pitch);

  delay(1);
}