#include <Wire.h>
#include <math.h>
#include <Kalman.h>

float Pitch = 0.0, Roll = 0.0; 
float accPitch = 0.0, accRoll = 0.0; 
float AcX= 0.0, AcY = 0.0, AcZ = 0.0;
const int MPU = 0x68;  // Alamat I2C MPU6050
const float alpha = 0.96; 
float dt = 0.01;
unsigned long previousTime, currentTime;

Kalman kalmanX;
Kalman kalmanY;

void ambil_data(){
    currentTime = millis();
  dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6);

  int16_t Ax = (Wire.read() << 8 | Wire.read())/ 16384.0;
  int16_t Ay = (Wire.read() << 8 | Wire.read())/ 16384.0;
  int16_t Az = (Wire.read() << 8 | Wire.read())/ 16384.0;
  
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6);
  int16_t gx = ((Wire.read() << 8) | Wire.read())/ 131.0;
  int16_t gy = ((Wire.read() << 8) | Wire.read())/ 131.0;
  int16_t gz = ((Wire.read() << 8) | Wire.read())/ 131.0;

//   Ax = AcX / 16384.0;  // Normalisasi (sesuai skala ±2g)
//   Ay = AcY / 16384.0;
//   Az = AcZ / 16384.0;

  float accRoll  = atan2(Ay, sqrt(Ax * Ax + Az * Az)) * 180.0 / PI;
  float accPitch = atan2(Ax, sqrt(Ay * Ay + Az * Az)) * 180.0 / PI;

  Pitch = alpha * (Pitch + gy * dt) + (1 - alpha) * accPitch;
  Roll = alpha * (Roll + gx * dt) + (1 - alpha) * accRoll;

  float gyroRateX = gx;
  float gyroRateY = gy;

  Roll = kalmanX.getAngle(accRoll, gyroRateX, dt);
  Pitch = kalmanY.getAngle(accPitch, gyroRateY, dt);
}