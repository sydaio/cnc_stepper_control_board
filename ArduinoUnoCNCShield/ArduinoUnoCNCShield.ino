#include <AccelStepper.h>

// ================== Hardware Configuration ==================

AccelStepper motors[4] = {
  AccelStepper(AccelStepper::DRIVER, 2, 5),   // Front Left
  AccelStepper(AccelStepper::DRIVER, 3, 6),   // Front Right
  AccelStepper(AccelStepper::DRIVER, 4, 7),   // Rear Right
  AccelStepper(AccelStepper::DRIVER, 12, 13)  // Rear Left
};

// ================== Motion Configuration ==================

const float STEPS_PER_REV = 200 * 1.0;
const float VELOCITY_SCALE = 50.0;
const float MAX_STEPS   = 100000.0;

//int counter=0;
//int timer = 0;

// ================== Serial Protocol ==================

const uint8_t PACKET_HEADER = 0xAA;

struct VelocityPacket {
  uint8_t header;
  int16_t v[4];
};

VelocityPacket pkt;

// ================== Watchdog ==================

unsigned long lastPacketTime = 0;
const unsigned long PACKET_TIMEOUT_MS = 100;
bool motorsStopped = false;

// ================== Setup ==================

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    motors[i].setMaxSpeed(MAX_STEPS);
    motors[i].setSpeed(0);
  }

  Serial.println(F("Binary stepper controller ready"));
}

// ================== Main Loop ==================

void loop() {
  // ---- Real-time motor stepping ----
  //counter++;
  //if (millis()-timer > 100)
  //{
  //  Serial.println(counter);
  //  timer=millis();    
  //  counter=0;
  //}


  for (int i = 0; i < 4; i++) {
    motors[i].runSpeed();
  }

  // ---- Serial receive ----
  readBinarySerial();

  // ---- Safety watchdog ----
  if (!motorsStopped && millis() - lastPacketTime > PACKET_TIMEOUT_MS) {
    stopMotors();
  }
}

// ================== Serial Receiver ==================

void readBinarySerial() {
  static uint8_t buffer[sizeof(VelocityPacket)];
  static uint8_t index = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();

    // Sync on header
    if (index == 0 && b != PACKET_HEADER) {
      continue;
    }

    buffer[index++] = b;

    if (index == sizeof(VelocityPacket)) {
      memcpy(&pkt, buffer, sizeof(VelocityPacket));
      index = 0;

      if (pkt.header == PACKET_HEADER) {
        applyVelocities(pkt.v);
        lastPacketTime = millis();
        motorsStopped = false;   // auto-resume
      }
    }
  }
}

// ================== Apply Velocities ==================

void applyVelocities(int16_t* v) {
  for (int i = 0; i < 4; i++) {
    float speed = v[i] * STEPS_PER_REV / VELOCITY_SCALE;

    if (speed >  MAX_STEPS) speed =  MAX_STEPS;
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

