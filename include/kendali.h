#include <akuisisi.h>

float setpoint = 0.0;
float rawroll, rawpitch, rawyaw;
float U1, U2, U3;
float errorRoll, errorPitch, errorYaw;
float KpRoll = 10.0, KpPitch = 10.0, KpYaw = 10.0;
float KiRoll = 0.0, KiPitch = 0.0, KiYaw = 0.0;
float KdRoll = 3.0, KdPitch = 3.0, KdYaw = 3.0;
float integralRoll = 0.0, integralPitch = 0.0, integralYaw = 0.0;
float previousErrorRoll = 0.0, previousErrorPitch = 0.0, previousErrorYaw = 0.0;// Assuming a time step of 0.01 seconds

void kendali (){
    rawroll = Roll;
    rawpitch = Pitch;
    rawyaw = Yaw;
    
    errorRoll = setpoint - rawroll;
    errorPitch = setpoint - rawpitch;
    errorYaw = setpoint - rawyaw;
    
    integralRoll += errorRoll * dt;
    integralPitch += errorPitch * dt;
    integralYaw += errorYaw * dt;
    
    float derivativeRoll = (errorRoll - previousErrorRoll) / dt;
    float derivativePitch = (errorPitch - previousErrorPitch) / dt;
    float derivativeYaw = (errorYaw - previousErrorYaw) / dt;
    
    U1 = KpRoll * errorRoll + KiRoll * integralRoll + KdRoll * derivativeRoll;
    U2 = KpPitch * errorPitch + KiPitch * integralPitch + KdPitch * derivativePitch;
    U3 = KpYaw * errorYaw + KiYaw * integralYaw + KdYaw * derivativeYaw;
    
    previousErrorRoll = errorRoll;
    previousErrorPitch = errorPitch;
    previousErrorYaw = errorYaw;
    
}