#include <AccelStepper.h>
#include <Wire.h>

// ================== Hardware Configuration ==================
AccelStepper motors[4] = {
  AccelStepper(AccelStepper::DRIVER, 2, 5),   // Front Left
  AccelStepper(AccelStepper::DRIVER, 3, 6),   // Front Right
  AccelStepper(AccelStepper::DRIVER, 4, 7),   // Rear Right
  AccelStepper(AccelStepper::DRIVER, 12, 13)  // Rear Left
};

// ================== Motion Configuration ==================
const float STEPS_PER_REV = 200.0;
const float VELOCITY_SCALE = 1.0;
const float MAX_STEPS = 100000.0;

// ================== I2C Configuration ==================
#define I2C_ADDRESS 0x10
const uint8_t PACKET_HEADER = 0xAA;

struct __attribute__((packed)) VelocityPacket {
  uint8_t header;
  int16_t v[4];
};

volatile VelocityPacket pkt;
volatile bool newPacket = false;

// ================== Watchdog ==================
unsigned long lastPacketTime = 0;
const unsigned long PACKET_TIMEOUT_MS = 100;
bool motorsStopped = false;

// ================== Debug ==================
unsigned long lastDebugPrint = 0;
const unsigned long DEBUG_INTERVAL_MS = 200;
unsigned long loopCounter = 0;

// ================== Setup ==================
void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);

  for (int i = 0; i < 4; i++) {
    motors[i].setMaxSpeed(MAX_STEPS);
    motors[i].setSpeed(0);
  }

  Serial.println("I2C stepper controller ready");
}

// ================== Main Loop ==================
void loop() {
  loopCounter++;

  // ---- Real-time motor stepping ----
  for (int i = 0; i < 4; i++) {
    motors[i].runSpeed();
  }

  // ---- Apply new packet safely ----
  if (newPacket) {
    noInterrupts();
    VelocityPacket localPkt;
    memcpy(&localPkt, (const void*)&pkt, sizeof(VelocityPacket));
    newPacket = false;
    interrupts();

    // Debug received values
    Serial.print("RX: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(localPkt.v[i]);
      Serial.print(" ");
    }
    Serial.println();

    applyVelocities(localPkt.v);
    lastPacketTime = millis();
    motorsStopped = false;
  }

  // ---- Watchdog safety ----
  if (!motorsStopped && millis() - lastPacketTime > PACKET_TIMEOUT_MS) {
    stopMotors();
    Serial.println("⚠️ Watchdog timeout -> motors stopped");
  }

  // ---- Debug loop rate ----
  if (millis() - lastDebugPrint > DEBUG_INTERVAL_MS) {
    lastDebugPrint = millis();
    Serial.print("Loop/s: ");
    Serial.println(loopCounter * (1000 / DEBUG_INTERVAL_MS));
    loopCounter = 0;
  }
}

// ================== I2C Receive Handler ==================
void receiveEvent(int bytesReceived) {
  if (bytesReceived != sizeof(VelocityPacket)) {
    while (Wire.available()) Wire.read(); // flush bad data
    return;
  }

  uint8_t* p = (uint8_t*)&pkt;
  for (int i = 0; i < sizeof(VelocityPacket); i++) {
    if (Wire.available()) {
      p[i] = Wire.read();
    }
  }

  newPacket = true;
}

// ================== Apply Velocities ==================
void applyVelocities(int16_t* v) {
  for (int i = 0; i < 4; i++) {
    float speed = v[i] * STEPS_PER_REV / VELOCITY_SCALE;

    if (speed > MAX_STEPS) speed = MAX_STEPS;
    if (speed < -MAX_STEPS) speed = -MAX_STEPS;

    motors[i].setSpeed(speed);
  }
}

// ================== Safety Stop ==================
void stopMotors() {
  for (int i = 0; i < 4; i++) {
    motors[i].setSpeed(0);
  }
  motorsStopped = true;
}
