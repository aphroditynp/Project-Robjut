#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <Kalman.h>
#include <Radio.h>
#include <Ultrasonik.h>
#include <Actuator.h>
#include <akuisisi.h>
#include <Transisition.h>

#define IMU_time 5
#define Ultrasonik_time 10
#define PRINT_time 50
#define CONTROL_time 5
#define RADIO_time 10

TaskHandle_t Task_IMU;
TaskHandle_t Task_Ultrasonik;
TaskHandle_t Task_Print;

void printUSB() {
  // Serial.print("Roll: "); Serial.print(roll);
  // Serial.print(" | Pitch: "); Serial.print(pitch);
  // Serial.print(" | Yaw: "); Serial.print(yaw);
  // Serial.print(" | Altitude: "); Serial.print(read_altitude());
  // Serial.print(" | Throttle: "); Serial.print(throttle);
  // Serial.println("Bagus Ganteng");
  // for (int i = 0; i < 8; i++) {
  //   Serial.print("CH");
  //   Serial.print(i + 1);
  //   Serial.print(": ");
  //   Serial.print(ch_raw[i]);
  //   Serial.print(" ");
  // }
  Serial.print(" | Roll: "); Serial.print(ch_roll);
  Serial.print(" | Pitch: "); Serial.print(ch_pitch);
  Serial.print(" | Yaw: "); Serial.print(ch_yaw);
  Serial.print(" | Throttle: "); Serial.print(ch_throttle);
  Serial.print(" | Motor 1: "); Serial.print(m1_pwm);
  Serial.print(" | Motor 2: "); Serial.print(m2_pwm);
  Serial.print(" | Motor 3: "); Serial.print(m3_pwm);
  Serial.print(" | Motor 4: "); Serial.print(m4_pwm);
  Serial.println();
  // Serial.print(" | U1: "); Serial.print(u1);
  // Serial.print(" | U2: "); Serial.print(u2);
  // Serial.print(" | U3: "); Serial.print(u3);
  // Serial.print(" | U4: "); Serial.println(u4);
}


void updateIMU(void *pvParameters){ 
  for(;;){
      ambil_data_imu();
      vTaskDelay(pdMS_TO_TICKS(IMU_time)); // Reduced delay for more frequent updates
  }
};

void updateUltrasonik(void *pvParameters){
  for(;;){
    read_altitude();
    vTaskDelay(pdMS_TO_TICKS(Ultrasonik_time)); // Reduced delay for more frequent updates
  }
};

void Print_task(void *pvParameters){
  for(;;){
      printUSB();
      // aileron_L.writeMicroseconds();
      vTaskDelay(pdMS_TO_TICKS(PRINT_time)); // Reduced delay for more frequent updates
  }
};

void radio(void *pvParameters){
  for(;;){
      remote_loop();
      vTaskDelay(pdMS_TO_TICKS(RADIO_time)); // Reduced delay for more frequent updates
  }
};

void controlThd(void *pvParameters){
  for(;;){
      Transition_sequence_manual();
      vTaskDelay(pdMS_TO_TICKS(CONTROL_time)); // Reduced delay for more frequent updates
  }
};

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
  previousTime = millis(); 
  remote_setup();
  ultrasonic_setup();
  init_actuator();

  xTaskCreate(updateIMU, "IMU", 2048, NULL, 3, &Task_IMU);
  xTaskCreate(updateUltrasonik, "Ultrasonik", 2048, NULL, 3, &Task_Ultrasonik);
  xTaskCreate(Print_task, "Print", 4096, NULL, 3, &Task_Print);
  xTaskCreate(radio, "Radio", 4096, NULL, 3, NULL);
  xTaskCreate(controlThd, "Control", 4096, NULL, 3, NULL);
}

void loop() {
}