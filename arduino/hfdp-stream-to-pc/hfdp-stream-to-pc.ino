#include <Arduino.h>
#include <SPI.h>

// Read Honeywell SSC sensor and transfer the data to PC up to 2 kHz rate, in binary format.
// Tested on Wemos D1 Mini (ESP8266)

#define CS_PIN D8

const uint32_t PERIOD_US = 500;       // 2 kHz polling
constexpr uint32_t SPI_HZ = 800000;
SPISettings sensorSPI(SPI_HZ, MSBFIRST, SPI_MODE0);

#pragma pack(push, 1)
struct Packet {
  uint8_t  sync;    // 0xA5
  uint8_t  status;  // 0..3
  uint16_t seq;     // increments each sent sample
  uint16_t raw14;   // 14-bit pressure counts (0..16383 in lower bits)
  uint32_t t_us;    // micros()
};
#pragma pack(pop)

static inline void read2bytes(uint8_t &b1, uint8_t &b2) {
  SPI.beginTransaction(sensorSPI);
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(3);          // >= 2.5 us tHDSS :contentReference[oaicite:2]{index=2}
  b1 = SPI.transfer(0x00);
  b2 = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

void setup() {
  Serial.begin(460800);               // try 460800 or 921600
  delay(50);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();
}

void loop() {
  static uint32_t next_us = 0;
  static uint16_t seq = 0;

  uint32_t now_us = micros();
  if (next_us == 0) next_us = now_us;

  int32_t late = (int32_t)(now_us - next_us);
  if (late >= 0) {
    // don’t bunch reads if we ever get delayed
    if (late > (int32_t)PERIOD_US) next_us = now_us;
    next_us += PERIOD_US;

    uint8_t b1, b2;
    read2bytes(b1, b2);

    uint8_t status = (b1 >> 6) & 0x03;
    uint16_t raw14 = ((uint16_t)(b1 & 0x3F) << 8) | b2;

    // Send only "valid" samples (status==0). If you want *all* polls, remove this if.
    if (status == 0) {
      Packet p;
      p.sync   = 0xA5;
      p.status = status;
      p.seq    = seq++;
      p.raw14  = raw14;
      p.t_us   = micros();

      Serial.write((uint8_t*)&p, sizeof(p));
    }
  } else {
    // precise wait to keep rate stable
    delayMicroseconds((uint32_t)(-late));
  }
}
