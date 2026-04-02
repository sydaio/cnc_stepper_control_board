#include <Wire.h>

// ================== I2C Configuration ==================
#define SLAVE_ADDR 0x10
#define SDA_PIN 21
#define SCL_PIN 22

const uint8_t PACKET_HEADER = 0xAA;

struct __attribute__((packed)) VelocityPacket {
  uint8_t header;
  int16_t v[4];
};

// ================== Test Parameters ==================
unsigned long lastSend = 0;
const int SEND_INTERVAL_MS = 50;   // 20 Hz update rate
int16_t speedValue = 0;
int16_t step = 1;

// ================== Setup ==================
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);  // Start I2C as master
  Serial.println("ESP32 I2C Master Ready");
}

// ================== Main Loop ==================
void loop() {
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();

    // ---- Generate test pattern (ramping speed) ----
    speedValue += step;
    if (speedValue > 10 || speedValue < -10) step = -step;

    VelocityPacket pkt;
    pkt.header = PACKET_HEADER;
    for (int i = 0; i < 4; i++) pkt.v[i] = speedValue;

    sendPacket(pkt);

    Serial.print("Sent speed: ");
    Serial.println(speedValue);
    delay(10);
  }
}

// ================== Send Packet ==================
void sendPacket(const VelocityPacket& pkt) {
  Wire.beginTransmission(SLAVE_ADDR);

  const uint8_t* data = (const uint8_t*)&pkt;
  Wire.write(data, sizeof(VelocityPacket));

  uint8_t result = Wire.endTransmission();
  if (result != 0) {
    Serial.print("I2C Error: ");
    Serial.println(result);
  }
}
