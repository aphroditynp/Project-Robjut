#include <Wire.h>
#include <math.h>
#include <Kalman.h>

float Pitch = 0.0, Roll = 0.0, Yaw = 0.0; 
float accPitch = 0.0, accRoll = 0.0; 
float AcX = 0.0, AcY = 0.0, AcZ = 0.0;
const int MPU = 0x68;  // Alamat I2C MPU6050
float dt = 0.01;
unsigned long previousTime, currentTime;

Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;

void ambil_data() {
    currentTime = millis();
    dt = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    // Baca data akselerometer
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6);

    int16_t rawAx = (Wire.read() << 8) | Wire.read();
    int16_t rawAy = (Wire.read() << 8) | Wire.read();
    int16_t rawAz = (Wire.read() << 8) | Wire.read();

    float Ax = rawAx / 16384.0;
    float Ay = rawAy / 16384.0;
    float Az = rawAz / 16384.0;

    // Baca data gyroscope
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6);

    int16_t rawGx = (Wire.read() << 8) | Wire.read();
    int16_t rawGy = (Wire.read() << 8) | Wire.read();
    int16_t rawGz = (Wire.read() << 8) | Wire.read();

    float gx = rawGx / 131.0;
    float gy = rawGy / 131.0;
    float gz = rawGz / 131.0;

    // Hitung sudut dari akselerometer
    accRoll  = atan2(Ay, sqrt(Ax * Ax + Az * Az)) * 180.0 / PI;
    accPitch = atan2(Ax, sqrt(Ay * Ay + Az * Az)) * 180.0 / PI;

    Pitch = 0.98 * (Pitch + (gy) * dt) + (0.02) * accPitch;
    Roll = 0.98 * (Roll + (gx) * dt) + (0.02) * accRoll;

    // // Gunakan Kalman Filter
    // Roll = kalmanX.getAngle(accRoll, gx, dt);
    // Pitch = kalmanY.getAngle(accPitch, gy, dt);

    // // Hitung Yaw dari giroskop (integrasi langsung)
    // Yaw += gz * dt;

    // // Gunakan Kalman Filter untuk Yaw (opsional)
    // Yaw = kalmanZ.getAngle(Yaw, gz, dt);
}