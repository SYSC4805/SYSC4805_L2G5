#include <avr/wdt.h>

// ============================================================
// NANO EVERY SENSOR HUB
// ============================================================

// Side digital IR obstacle sensors.
const int IR_LEFT_PIN  = 12;
const int IR_LEFT_INT  = 2;

const int IR_RIGHT_PIN = 14;
const int IR_RIGHT_INT = 3;

// Front ultrasonic distance sensor.
const int US_TRIG_PIN = 7;
const int US_ECHO_PIN = 8;
const int US_INT = 4;

// Rear analog distance sensor.
const int REAR_ANALOG_PIN = A0;
const int REAR_INT = 5;

// Heartbeat used by Due to detect Nano stalls/resets.
const int HEARTBEAT_PIN = 6;

// ============================================================
// THRESHOLDS / TIMING
// ============================================================

const unsigned long US_TIMEOUT_US = 50000;
const unsigned long US_INTERVAL_US = 60000;
const unsigned long PULSE_WIDTH_US = 100;

const unsigned long HEARTBEAT_INTERVAL_MS = 250;

const float FRONT_NOTIFY_CM = 20.0;

// Calibrate this against the actual rear analog sensor.
const int REAR_BLOCKED_THRESHOLD = 500;

// ============================================================
// ULTRASONIC STATE
// ============================================================

volatile unsigned long echoStart = 0;
volatile unsigned long echoWidth = 0;

volatile bool echoRising = false;
volatile bool echoReady = false;
volatile bool echoWait = false;

unsigned long lastTrigger = 0;
float frontDistanceCm = -1.0;

// ============================================================
// NOTIFICATION PULSES
// ============================================================

unsigned long pulseStart[4] = {0, 0, 0, 0};
bool pulseActive[4] = {false, false, false, false};

const int interruptPins[4] = {
  IR_LEFT_INT,
  IR_RIGHT_INT,
  US_INT,
  REAR_INT
};

// ============================================================
// HEARTBEAT
// ============================================================

unsigned long lastHeartbeatMs = 0;
bool heartbeatLevel = false;

// ============================================================
// HELPERS
// ============================================================

void notifyDue(int index) {
  digitalWrite(interruptPins[index], HIGH);

  pulseStart[index] = micros();
  pulseActive[index] = true;
}

void updateNotifications() {
  unsigned long now = micros();

  for (int i = 0; i < 4; ++i) {
    if (pulseActive[i] &&
        now - pulseStart[i] >= PULSE_WIDTH_US) {

      digitalWrite(interruptPins[i], LOW);
      pulseActive[i] = false;
    }
  }
}

void updateHeartbeat() {
  if (millis() - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = millis();

    heartbeatLevel = !heartbeatLevel;

    digitalWrite(
      HEARTBEAT_PIN,
      heartbeatLevel ? HIGH : LOW
    );
  }
}

void ultrasonicISR() {
  if (!echoWait) return;

  if (digitalRead(US_ECHO_PIN) == HIGH) {
    echoStart = micros();
    echoRising = true;
  } else if (echoRising) {
    echoWidth = micros() - echoStart;

    echoReady = true;
    echoWait = false;
    echoRising = false;
  }
}

void updateUltrasonic() {
  unsigned long now = micros();

  if (echoReady) {
    noInterrupts();

    unsigned long width = echoWidth;
    echoReady = false;

    interrupts();

    frontDistanceCm = width * 0.0343f / 2.0f;
  }

  if (echoWait &&
      now - lastTrigger >= US_TIMEOUT_US) {

    noInterrupts();

    echoWait = false;
    echoRising = false;

    interrupts();

    frontDistanceCm = -1.0;
  }

  if (now - lastTrigger >= US_INTERVAL_US) {
    lastTrigger = now;

    noInterrupts();

    echoWait = true;
    echoRising = false;

    interrupts();

    digitalWrite(US_TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(US_TRIG_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(US_TRIG_PIN, LOW);
  }
}

bool rearObstacleDetected() {
  int raw = analogRead(REAR_ANALOG_PIN);

  return raw >= REAR_BLOCKED_THRESHOLD;
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(9600);

  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_LEFT_INT, OUTPUT);
  digitalWrite(IR_LEFT_INT, LOW);

  pinMode(IR_RIGHT_PIN, INPUT);
  pinMode(IR_RIGHT_INT, OUTPUT);
  digitalWrite(IR_RIGHT_INT, LOW);

  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);

  digitalWrite(US_TRIG_PIN, LOW);

  pinMode(US_INT, OUTPUT);
  digitalWrite(US_INT, LOW);

  pinMode(REAR_ANALOG_PIN, INPUT);

  pinMode(REAR_INT, OUTPUT);
  digitalWrite(REAR_INT, LOW);

  pinMode(HEARTBEAT_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_PIN, LOW);

  attachInterrupt(
    digitalPinToInterrupt(US_ECHO_PIN),
    ultrasonicISR,
    CHANGE
  );

  // 2-second watchdog timeout.
  wdt_enable(WDT_PERIOD_2KCLK_gc);
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  updateUltrasonic();
  updateNotifications();
  updateHeartbeat();

  bool leftBlocked =
      digitalRead(IR_LEFT_PIN) == LOW;

  bool rightBlocked =
      digitalRead(IR_RIGHT_PIN) == LOW;

  if (leftBlocked) {
    notifyDue(0);
  }

  if (rightBlocked) {
    notifyDue(1);
  }

  if (frontDistanceCm >= 0.0f &&
      frontDistanceCm <= FRONT_NOTIFY_CM) {

    notifyDue(2);
  }

  if (rearObstacleDetected()) {
    notifyDue(3);
  }

  wdt_reset();
}