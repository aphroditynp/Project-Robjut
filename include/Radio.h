#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include <sbus.h>

// Gunakan UART2 di ESP32
#define SBUS_RX_PIN 16
#define SBUS_TX_PIN 17

HardwareSerial sbusSerial(2);  // UART2
bfs::SbusRx sbus_rx;           // Default constructor (tanpa argumen)
bfs::SbusData data;

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
  return 0.002 * ch - 3;
}

void remote_setup() {
  sbusSerial.begin(100000, SERIAL_8E2, SBUS_RX_PIN, SBUS_TX_PIN);
  sbus_rx.Begin(sbusSerial);  // Inject HardwareSerial via Begin()

  if (sbus_rx.Read()) {
    data = sbus_rx.data();
    arming = data.ch[4] > 1500;
    signal_lost = data.lost_frame;

    if (arming) {
      Serial.println("Please disarm the remote for safety");
      while (arming) {
        if (sbus_rx.Read()) {
          data = sbus_rx.data();
          arming = data.ch[4] > 1500;
          signal_lost = data.lost_frame;
        }
      }
    }

    if (signal_lost) {
      Serial.println("Signal lost, please reconnect");
      while (signal_lost) {
        if (sbus_rx.Read()) {
          data = sbus_rx.data();
          arming = data.ch[4] > 1500;
          signal_lost = data.lost_frame;
        }
      }
    }
  }

  Serial.println("Remote setup complete");
}

void remote_loop() {
  if (sbus_rx.Read()) {
    data = sbus_rx.data();

    ch_roll     = data.ch[0] * 0.62652f + 880.27f;
    ch_pitch    = data.ch[1] * 0.62652f + 880.27f;
    ch_throttle = data.ch[2] * 0.62652f + 880.27f;
    ch_yaw      = data.ch[3] * 0.62652f + 880.27f;

    ch_roll     = constrain(ch_roll, 988, 2012);
    ch_pitch    = constrain(ch_pitch, 988, 2012);
    ch_throttle = constrain(ch_throttle, 988, 2012);
    ch_yaw      = constrain(ch_yaw, 988, 2012);

    arming = data.ch[4] > 1500;
    signal_lost = data.lost_frame;

    ch_mode        = data.ch[5] * 0.62652f + 880.27f;
    ch_mode_backup = data.ch[6] * 0.62652f + 880.27f;
    ch_vehicle_mode = data.ch[7] * 0.62652f + 880.27f;

    for (int i = 0; i < 16; i++) {
      Serial.print(data.ch[i]);
      Serial.print(" ");
    }
    Serial.println();

    if (ch_mode_backup <= 1000) {
      mode_now = 1;
    } else if (ch_mode_backup > 1000 && ch_mode_backup <= 1512) {
      mode_now = 2;
    } else {
      mode_now = 3;
    }
  }
}

#endif
