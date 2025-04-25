    #ifndef COPTER_CONTROL_H
    #define COPTER_CONTROL_H
    #include <Arduino.h>

    #include "akuisisi.h"
    // #include "Control_Modes.h"
    #include "Copter_config.h"
    #include "Radio.h"
    #include "Ultrasonik.h"
    #include "Control_modes.h"
    // float m1_pwm, m2_pwm, m3_pwm, m4_pwm;
    
    /// matrix invers A F450
    const double A_invers[4][4] = {{292600, 2600600, 0, -6283300},
                                {292600, 0, -2600600, 6283300},
                                {292600, -2600600, 0, -6283300},
                                {292600, 0, 2600600, 6283300}};
    // // wahana putri
    // const double A_invers[4][4] = {{-91600, 172800, 172800, 1471600},
    //                                {-91600, 172800, -172800, -1471600},
    //                                {-91600, -172800, -172800, 1471600},
    //                                {-91600, -172800, 172800, -1471600}};

    // variabel
    float u1, u2, u3, u4;
    float w1, w2, w3, w4;
    float roll_int, pitch_int;
    unsigned long tnow, tbefore;
    float tdelta;
    unsigned long calc_time, last_calc_time;
    float delta_calc_time;
    float alt_ref, heading_now, last_alt, last_heading, alt_now;
    float alt_target, z_velocity;
    float roll_cmd, pitch_cmd, yaw_cmd;
    float min_roll = -35.0f;  // 30
    float max_roll = 35.0f;
    float min_pitch = -35.0f;  // 25
    float max_pitch = 35.0f;
    float min_yaw = -20.0f;
    float max_yaw = 20.0f;
    int PID_max_roll = 400;
    int PID_max_pitch = 400;
    int PID_max_yaw = 400;
    int PID_min_roll = -400;
    int PID_min_pitch = -400;
    int PID_min_yaw = -400;
    // float trim_roll = 3.0f;  // 4.8f;//= 2.71;//3.78; //-40 //(+) bales kanan, (-) bales kiri 5 -2
    // float trim_pitch = 3.5f;  // 10.0f;// 2.40;//2.62; //-30 //25 //22.4 -15.5 -10 -5 // (+) bales maju, (-)bales mundur // 3.5
    // float trim_yaw = -0.015f;   // 0.006 //(+) bales kanan, (-) bales kiri -0.349 -0.360
    float trim_roll = 0.0f;  // 4.8f;//= 2.71;//3.78; //-40 //(+) bales kanan, (-) bales kiri 5 -2
    float trim_pitch = 0.0f;  // 10.0f;// 2.40;//2.62; //-30 //25 //22.4 -15.5 -10 -5 // (+) bales maju, (-)bales mundur // 3.5
    float trim_yaw = 0.0f;   // 0.006 //(+) bales kanan, (-) bales kiri -0.349 -0.360
    float omega2[4];

    //*Mode
    // struct gains {
    //     float k_alt = 2.0f;            // 2
    //     float k_z_velocity = 3.0f;     // 3
    //     float k_roll = 238.9f;         // 324 474 474 234 274 ;245 250 246 238 239.9 //238
    //     float k_pitch = 195.0f;        //182 186 181 179.5 L 180.4 195 200 210
    //     float k_yaw = 960.0f;         // 2400 2500 //950
    //     float k_z_vel = 1.0f;          // 1.0
    //     float k_roll_rate = 5725.8f;   // 5500 5750 5900 5910 5980 82 L 5987 6000 5997 5994
    //     float k_pitch_rate = 3995.8f;  // 3152* 3159jlek 3154.8 3158.8 L 3174.8 3190 3600 3870 3720 3750 3770 3788 3810 3830 3837
    //     float k_yaw_rate = 0.0f;      // 11 2980
    //     float k_i_roll = 0.0f;         //
    //     float k_i_pitch = 0.022f;      // 0.022
    //     float k_i_yaw = 0.05f;         // 0.05
    //     // 15.05: 08.05
    //     int16_t roll_rmt;
    //     int16_t pitch_rmt;
    //     int16_t yaw_rmt;
    // };
    // gains gain;

    struct gains {
        float k_alt             = 2.0;    
        float k_z_velocity      = 3.0;     
        float k_roll            = 3.99;   //241.5f; 324 474 474 234 274 ;245 250 246 238 239.9 //238 241 244.9
        float k_pitch           = 3.99; // 6 //36.0f;   //175.0f;  334 434 484 223 ;182 188 190 193 //182 186 181 179.5 // 180 185.4 178.4 179.8 180.8 179 177.5 174.7 172.9 170.0 176.9 180 183 181.5
        float k_yaw             = 3.99;   // 2400 2500 //950
        float k_z_vel           = 1.0;     
        float k_roll_rate       = 1.1581; //1.7f;        //3415.75f; 5987.8 6500 4500 4100 3790 3650 3610 3624.8 3628.8
        float k_pitch_rate      = 1.1581;  //2.9374f; //2.8712f;  //2733.50f;  // 3174.8 3500 2400 2100 1790 1500 1200 1320 1298 1340 1480 1598 1550 2500 3000.8 3700 3580 3690 3250 3080.8 2860 2840 2788 2800
        float k_yaw_rate        = 1.1581;     // 11 2980
        float k_i_roll          = 0.0;     //
        float k_i_pitch         = 0.0;   // 0.022
        float k_i_yaw           = 0.0;    // 0.05
        // 15.05: 08.05
        int16_t roll_rmt;
        int16_t pitch_rmt;
        int16_t yaw_rmt;
    };
    gains gain;

    void copter_getIntegral(int16_t ch_thr, float roll, float pitch, float yaw) {
        tbefore = tnow;
        tnow = millis();
        tdelta = (tnow - tbefore) / 1000.0;
        roll_int += gain.k_i_roll * roll * tdelta;
        pitch_int += gain.k_i_pitch * pitch * tdelta;
        roll_int = constrain(roll_int, -15, 15);
        pitch_int = constrain(pitch_int, -15, 15);
        // Reset the integrators
        if (ch_thr < 1100) {
            roll_int = 0.0;
            pitch_int = 0.0;
        }
    }
    void copter_ControlFSFB(int16_t ch_r, int16_t ch_p, int16_t ch_y, int16_t ch_thr, float roll, float pitch, float yaw, float gy, float gx, float gz) {
        last_calc_time = calc_time;
        calc_time = micros();
        delta_calc_time = (calc_time - last_calc_time) / 1000000.0;
        heading_now = yaw;
        // distance = read_altitude();
        alt_ref = read_altitude();
        alt_now = read_altitude();
        if (alt_hold && mode_fbwa) {
            if (ch_throttle > 1600) {
                alt_setpoint = read_altitude();
                alt_setpoint += (ch_throttle - 1600) / 20000;
            } else if (ch_throttle < 1400) {
                alt_setpoint = read_altitude();
                alt_setpoint -= (ch_throttle - 1400) / 20000;
            } else {
                alt_setpoint = read_altitude();
            }
            alt_target = read_altitude(); - alt_setpoint;
            z_velocity = (alt_now - last_alt) / delta_calc_time;
        } else if (mode_fbwa && !alt_hold) {
            alt_target = read_altitude(); - alt_ref;
            z_velocity = (read_altitude() - last_alt) / delta_calc_time;
        }
        // yaw_setpoint = heading_now - last_heading;
        yaw_setpoint = delta_yaw;
        if (yaw_setpoint > 180) yaw_setpoint -= 360;
        if (yaw_setpoint < -180) yaw_setpoint += 360;
        copter_getIntegral(ch_thr, roll, pitch, yaw);

        roll_cmd = (map(ch_r - 1500, min_roll_corr, max_roll_corr, min_roll, max_roll));
        pitch_cmd = (map(ch_p - 1500, min_pitch_corr, max_pitch_corr, min_pitch, max_pitch));
        yaw_cmd = (map(ch_y - 1500, min_yaw_corr, max_yaw_corr, min_yaw, max_yaw));  // 0.08
        u1 = 0.0f;                                                                          //(-gain.k_alt*(alt_target/1.000f) + (-gain.k_z_velocity*(z_velocity)/100.0f))/1000000.0f;
        u2 = ((-gain.k_roll * (roll + roll_int - (roll_cmd + trim_roll)) / 10000000.0f) + (-gain.k_roll_rate * (gy) / 10000000.0f));
        u3 = (-gain.k_pitch * ((-pitch) + pitch_int - (pitch_cmd + trim_pitch)) / 10000000.0f) + (-gain.k_pitch_rate * (gx) / 10000000.0f);
        u4 = ((-gain.k_yaw * (yaw_setpoint - (yaw_cmd + trim_yaw)) / 10000000.0f) + (-gain.k_yaw_rate * (gz) / 10000000.0f));
        omega2[0] = (A_invers[0][0] * u1 + A_invers[0][1] * u2 + A_invers[0][2] * u3 + A_invers[0][3] * u4);
        omega2[1] = (A_invers[1][0] * u1 + A_invers[1][1] * u2 + A_invers[1][2] * u3 + A_invers[1][3] * u4);
        omega2[2] = (A_invers[2][0] * u1 + A_invers[2][1] * u2 + A_invers[2][2] * u3 + A_invers[2][3] * u4);
        omega2[3] = (A_invers[3][0] * u1 + A_invers[3][1] * u2 + A_invers[3][2] * u3 + A_invers[3][3] * u4);
        last_heading = heading_now;
        
        // last_alt = alt_setpoint;
    }
    #endif