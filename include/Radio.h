#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>

// UART SBUS di ESP32 (gunakan Serial2 dan inverter logic aktif)
HardwareSerial sbusSerial(2);  // RX2: GPIO16, TX2: GPIO17 (TX ga kepake)

uint8_t sbusData[25];
uint16_t ch_raw[16];

bool signal_lost = false;
int16_t ch_roll, ch_pitch, ch_throttle, ch_yaw, ch_mode, ch_mode_backup, ch_vehicle_mode;
bool arming;

bool alt_hold = false;
bool mode_fbwa = false;
bool pos_hold = false;
bool mode_manual = false;
bool mode_fbwa_plane = false;
bool mode_fbwb = false;
bool transition_phase1 = false;
bool transition_phase2 = false;
bool transition_phase3 = false;
bool mode_vtol = false;
bool mode_safety = false;
bool mode_vtol_plane = false;

int mode_now, prev_mode;

float outputScaler(uint16_t ch) {
  return 0.002 * ch - 3;  // Skala dari 172-1811 jadi ~0-1
}

void decodeSBUS() {
  ch_raw[0]  = ((sbusData[1]    | sbusData[2]  << 8)                         & 0x07FF);
  ch_raw[1]  = ((sbusData[2]  >> 3 | sbusData[3]  << 5)                      & 0x07FF);
  ch_raw[2]  = ((sbusData[3]  >> 6 | sbusData[4]  << 2 | sbusData[5] << 10)  & 0x07FF);
  ch_raw[3]  = ((sbusData[5]  >> 1 | sbusData[6]  << 7)                      & 0x07FF);
  ch_raw[4]  = ((sbusData[6]  >> 4 | sbusData[7]  << 4)                      & 0x07FF);
  ch_raw[5]  = ((sbusData[7]  >> 7 | sbusData[8]  << 1 | sbusData[9] << 9)   & 0x07FF);
  ch_raw[6]  = ((sbusData[9]  >> 2 | sbusData[10] << 6)                      & 0x07FF);
  ch_raw[7]  = ((sbusData[10] >> 5 | sbusData[11] << 3)                      & 0x07FF);
  ch_raw[8]  = ((sbusData[12]    | sbusData[13] << 8)                        & 0x07FF);
  ch_raw[9]  = ((sbusData[13] >> 3 | sbusData[14] << 5)                      & 0x07FF);
  ch_raw[10] = ((sbusData[14] >> 6 | sbusData[15] << 2 | sbusData[16] << 10) & 0x07FF);
  ch_raw[11] = ((sbusData[16] >> 1 | sbusData[17] << 7)                      & 0x07FF);
  ch_raw[12] = ((sbusData[17] >> 4 | sbusData[18] << 4)                      & 0x07FF);
  ch_raw[13] = ((sbusData[18] >> 7 | sbusData[19] << 1 | sbusData[20] << 9)  & 0x07FF);
  ch_raw[14] = ((sbusData[20] >> 2 | sbusData[21] << 6)                      & 0x07FF);
  ch_raw[15] = ((sbusData[21] >> 5 | sbusData[22] << 3)                      & 0x07FF);

  signal_lost = sbusData[23] & 0x04;
  arming      = ch_raw[4] > 1500 ? true : false;
}

void remote_setup() {
  sbusSerial.begin(100000, SERIAL_8E2, 16, 17, true); // RX=16, TX=17, inverter logic
  Serial.println("Remote setup...");

  // Tunggu sinyal SBUS
  while (sbusSerial.available() < 25 || sbusSerial.peek() != 0x0F) {
    delay(100);
  }

  Serial.println("Remote ready");
}

void remote_loop() {
  if (sbusSerial.available() >= 25 && sbusSerial.peek() == 0x0F) {
    sbusSerial.readBytes(sbusData, 25);
    decodeSBUS();

    // Scaling dan mapping channel ke fungsi
    ch_roll     = constrain(ch_raw[0] * 0.62652f + 880.27f, 988, 2012);
    ch_pitch    = constrain(ch_raw[1] * 0.62652f + 880.27f, 988, 2012);
    ch_throttle = constrain(ch_raw[2] * 0.62652f + 880.27f, 988, 2012);
    ch_yaw      = constrain(ch_raw[3] * 0.62652f + 880.27f, 988, 2012);

    ch_mode         = ch_raw[5] * 0.62652f + 880.27f;
    ch_mode_backup  = ch_raw[6] * 0.62652f + 880.27f;
    ch_vehicle_mode = ch_raw[7] * 0.62652f + 880.27f;

    // Tentukan mode dari posisi channel
    if (ch_mode_backup <= 1000) {
      mode_now = 1;
    } else if (ch_mode_backup > 1000 && ch_mode_backup <= 1512) {
      mode_now = 2;
    } else {
      mode_now = 3;
    }

    // Debug semua channel
    for (int i = 0; i < 16; i++) {
      Serial.print("CH");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(ch_raw[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}
#endif