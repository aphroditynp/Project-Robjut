#include <akuisisi.h>
#include <PID.h>

 #define PIN_INPUT 0
 #define PIN_OUTPUT 3
 
 //Define Variables we'll be connecting to
 double Setpoint = 0, inputRoll, URoll, inputPitch, UPitch, inputYaw, UYaw;
 double Kproll = 2, Kiroll = 5, Kdroll = 1;
 double Kppitch = 2, Kipitch = 5, Kdpitch = 1;
 double Kpyaw = 2, Kiyaw = 5, Kdyaw = 1;
 
 PID PIDRoll(&inputRoll, &URoll, &Setpoint, Kproll, Kiroll, Kdroll, DIRECT);
 PID PIDPitch(&inputPitch, &UPitch, &Setpoint, Kppitch, Kipitch, Kdpitch, DIRECT);
 PID PIDYaw(&inputYaw, &UYaw, &Setpoint, Kpyaw, Kiyaw, Kdyaw, DIRECT);
 //Specify the links and initial tuning parameters
 void kendali(){
    inputRoll = Roll;
    inputPitch = Pitch;
    inputYaw = Yaw;
    
    PIDRoll.Compute();
    PIDPitch.Compute();
    PIDYaw.Compute();
 }
