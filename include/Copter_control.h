// #include <kendali.h>
// #include <Servo.h>

// #define MOTOR_1_PIN  10  
// #define MOTOR_2_PIN  11
// #define MOTOR_3_PIN  12
// #define MOTOR_4_PIN  13

// Servo motor1, motor2, motor3, motor4;
// float m1_pwm, m2_pwm, m3_pwm, m4_pwm;
// uint16_t throttle;


// void copter_calcOutput(int16_t ch_thr)
// {
//   throttle = ch_thr;
//   int control1 = (int)(omega2[0]*M_CONST);
//   int control2 = (int)(omega2[1]*M_CONST);
//   int control3 = (int)(omega2[2]*M_CONST);
//   int control4 = (int)(omega2[3]*M_CONST);

//   control1 = constrain(control1, -400, 400);
//   control2 = constrain(control2, -400, 400);
//   control3 = constrain(control3, -400, 400);
//   control4 = constrain(control4, -400, 400);
  

//   if(arming){
//     m1_pwm = control1 + throttle ;
//     m2_pwm = control2 + throttle ;
//     m3_pwm = control3 + throttle ;
//     m4_pwm = control4 + throttle ;
//   }
//   else{
//     m1_pwm = 988;
//     m2_pwm = 988;
//     m3_pwm = 988;
//     m4_pwm = 988;
//   }
//     m1_pwm = constrain(m1_pwm, PWM_MIN, PWM_MAX);
//     m2_pwm = constrain(m2_pwm, PWM_MIN, PWM_MAX);
//     m3_pwm = constrain(m3_pwm, PWM_MIN, PWM_MAX);
//     m4_pwm = constrain(m4_pwm, PWM_MIN, PWM_MAX);
// }