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
TaskHandle_t Task_Radio; // Tambahkan handle untuk task radio

void printUSB() {
  // Read roll, pitch, yaw, and throttle channels
  Serial.print(" | Roll: "); Serial.print(ch_roll);
  Serial.print(" | Pitch: "); Serial.print(ch_pitch);
  Serial.print(" | Yaw: "); Serial.print(ch_yaw);
  Serial.print(" | Throttle: "); Serial.print(ch_throttle);

  for (int i = 0; i < 8; i++) {
    Serial.print("CH"); Serial.print(i); Serial.print(": ");
    Serial.print(channels[i]); Serial.print("\t");
  }

  Serial.println();
}

void updateIMU(void *pvParameters){ 
  for(;;){
      ambil_data_imu();
      vTaskDelay(pdMS_TO_TICKS(IMU_time));
  }
};

void updateUltrasonik(void *pvParameters){
  for(;;){
    read_altitude();
    vTaskDelay(pdMS_TO_TICKS(Ultrasonik_time));
  }
};

void Print_task(void *pvParameters){
  for(;;){
      printUSB();
      vTaskDelay(pdMS_TO_TICKS(PRINT_time));
  }
};

void radio(void *pvParameters){
  for(;;){
      remote_loop();
      vTaskDelay(pdMS_TO_TICKS(RADIO_time));
  }
};

void controlThd(void *pvParameters){
  for(;;){
      Transition_sequence_manual();
      vTaskDelay(pdMS_TO_TICKS(CONTROL_time));
  }
};

void setup() {
  Wire.begin();
  Serial.begin(115200);
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
  xTaskCreate(radio, "Radio", 4096, NULL, 3, &Task_Radio); // Sudah diperbaiki
  xTaskCreate(controlThd, "Control", 4096, NULL, 3, NULL);
}

void loop() {
  // Kosong, semua dikendalikan oleh task
}
