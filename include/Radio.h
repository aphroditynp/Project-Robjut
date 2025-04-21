#include <HardwareSerial.h>

// Gunakan UART2
HardwareSerial SBUSSerial(2);

// Buffer untuk menerima data SBUS
uint8_t sbusData[25];
uint16_t channels[16];

bool failsafe = false;
bool lostFrame = false;

void radio_setup() {
  // Inisialisasi SBUS Serial: 100000 baud, 8E2, inverted logic
  SBUSSerial.begin(100000, SERIAL_8E2, 16, 17, true);  // RX = GPIO16, TX = GPIO17
}

void decodeSBUS() {
  channels[0]  = ((sbusData[1]    | sbusData[2]  << 8)                         & 0x07FF);
  channels[1]  = ((sbusData[2]  >> 3 | sbusData[3]  << 5)                      & 0x07FF);
  channels[2]  = ((sbusData[3]  >> 6 | sbusData[4]  << 2 | sbusData[5] << 10)  & 0x07FF);
  channels[3]  = ((sbusData[5]  >> 1 | sbusData[6]  << 7)                      & 0x07FF);
  channels[4]  = ((sbusData[6]  >> 4 | sbusData[7]  << 4)                      & 0x07FF);
  channels[5]  = ((sbusData[7]  >> 7 | sbusData[8]  << 1 | sbusData[9] << 9)   & 0x07FF);
  channels[6]  = ((sbusData[9]  >> 2 | sbusData[10] << 6)                      & 0x07FF);
  channels[7]  = ((sbusData[10] >> 5 | sbusData[11] << 3)                      & 0x07FF);
  channels[8]  = ((sbusData[12]    | sbusData[13] << 8)                        & 0x07FF);
  channels[9]  = ((sbusData[13] >> 3 | sbusData[14] << 5)                      & 0x07FF);
  channels[10] = ((sbusData[14] >> 6 | sbusData[15] << 2 | sbusData[16] << 10) & 0x07FF);
  channels[11] = ((sbusData[16] >> 1 | sbusData[17] << 7)                      & 0x07FF);
  channels[12] = ((sbusData[17] >> 4 | sbusData[18] << 4)                      & 0x07FF);
  channels[13] = ((sbusData[18] >> 7 | sbusData[19] << 1 | sbusData[20] << 9)  & 0x07FF);
  channels[14] = ((sbusData[20] >> 2 | sbusData[21] << 6)                      & 0x07FF);
  channels[15] = ((sbusData[21] >> 5 | sbusData[22] << 3)                      & 0x07FF);

  // Flags
  failsafe  = sbusData[23] & 0x08;
  lostFrame = sbusData[23] & 0x04;
}

void printChannels() {
  for (int i = 0; i < 16; i++) {
    Serial.print("CH");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(channels[i]);
    Serial.print("\t");
  }
  Serial.println();

  if (failsafe) Serial.println(">> FAILSAFE MODE <<");
  if (lostFrame) Serial.println(">> FRAME LOST <<");
}

void radio_loop() {
    if (SBUSSerial.available() >= 25) {
      // Sinkronisasi dengan byte pertama SBUS (0x0F)
      if (SBUSSerial.peek() == 0x0F) {
        SBUSSerial.readBytes(sbusData, 25);
  
        if (sbusData[24] == 0x00) {
          decodeSBUS();
          printChannels();
        }
      } else {
        SBUSSerial.read();  // buang byte sampai dapat 0x0F
      }
    }
  }