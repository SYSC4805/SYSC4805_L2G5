#include <Wire.h>
#include <VL53L1X.h>
#include <avr/wdt.h>

 
// VERIFIED NANO SENSOR PINS FROM TEAM CODE
 
const int IR_PIN_1 = 12;
const int IR_1_INT = 2;

const int IR_PIN_2 = 14;
const int IR_2_INT = 3;

const int US_TRIG_PIN = 7;
const int US_ECHO_PIN = 8;
const int US_INT = 4;

const int RF_INT = 5;

// Added heartbeat output to let the Due detect a Nano stall/reset.
const int HEARTBEAT_PIN = 6;

 
// SENSOR TIMING
 
const unsigned long US_TIMEOUT_US = 50000;
const unsigned long US_INTERVAL_US = 60000;
const unsigned long PULSE_WIDTH_US = 100;
const unsigned long HEARTBEAT_INTERVAL_MS = 250;

volatile unsigned long echoStart = 0;
volatile unsigned long echoWidth = 0;
volatile bool echoRising = false;
volatile bool echoReady = false;
volatile bool echoWait = false;

unsigned long lastTrigger = 0;
unsigned long lastPrint = 0;
unsigned long lastHeartbeatMs = 0;
bool heartbeatLevel = false;

float ultrasonicDistanceCm = -1.0;

VL53L1X rfSensor;

unsigned long pulseStart[4] = {0, 0, 0, 0};
bool pulseActive[4] = {false, false, false, false};
const int interruptPins[4] = {IR_1_INT, IR_2_INT, US_INT, RF_INT};

void notifyDue(int index) {
  digitalWrite(interruptPins[index], HIGH);
  pulseStart[index] = micros();
  pulseActive[index] = true;
}

void updateNotifications() {
  unsigned long now = micros();

  for (int i = 0; i < 4; ++i) {
    if (pulseActive[i] && now - pulseStart[i] >= PULSE_WIDTH_US) {
      digitalWrite(interruptPins[i], LOW);
      pulseActive[i] = false;
    }
  }
}

void updateHeartbeat() {
  if (millis() - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = millis();
    heartbeatLevel = !heartbeatLevel;
    digitalWrite(HEARTBEAT_PIN, heartbeatLevel ? HIGH : LOW);
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

    ultrasonicDistanceCm = width * 0.0343f / 2.0f;
  }

  if (echoWait && now - lastTrigger >= US_TIMEOUT_US) {
    noInterrupts();
    echoWait = false;
    echoRising = false;
    interrupts();

    ultrasonicDistanceCm = -1.0;
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

void setup() {
  Serial.begin(9600);

  pinMode(IR_PIN_1, INPUT);
  pinMode(IR_1_INT, OUTPUT);
  digitalWrite(IR_1_INT, LOW);

  pinMode(IR_PIN_2, INPUT);
  pinMode(IR_2_INT, OUTPUT);
  digitalWrite(IR_2_INT, LOW);

  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  digitalWrite(US_TRIG_PIN, LOW);

  pinMode(US_INT, OUTPUT);
  digitalWrite(US_INT, LOW);

  pinMode(RF_INT, OUTPUT);
  digitalWrite(RF_INT, LOW);

  pinMode(HEARTBEAT_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(US_ECHO_PIN), ultrasonicISR, CHANGE);

  Wire.begin();
  Wire.setClock(400000);

  rfSensor.setTimeout(500);
  if (!rfSensor.init()) {
    Serial.println("Failed to initialize VL53L1X");
    while (1) {
      // reset will occur.
    }
  }

  rfSensor.setDistanceMode(VL53L1X::Short);
  rfSensor.setMeasurementTimingBudget(30000);
  rfSensor.startContinuous(50);

  // 2-second watchdog, matching the current progress-report criterion.
  wdt_enable(WDT_PERIOD_2KCLK_gc);
}

void loop() {
  updateUltrasonic();
  updateNotifications();
  updateHeartbeat();

  int ir1State = digitalRead(IR_PIN_1);
  int ir2State = digitalRead(IR_PIN_2);

  if (ir1State == 0) {
    notifyDue(0);
  }

  if (ir2State == 0) {
    notifyDue(1);
  }

  // SR-01 / FR-03 close-obstacle signal.
  if (ultrasonicDistanceCm >= 0.0f && ultrasonicDistanceCm <= 10.0f) {
    notifyDue(2);
  }

  if (rfSensor.dataReady()) {
    uint16_t rfDistanceMm = rfSensor.read(false);

    // FR-02 / PR-04: notify for an obstacle at 20 cm or less.
    if (rfDistanceMm <= 200) {
      notifyDue(3);
    }
  }

  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.print("US cm: ");
    Serial.println(ultrasonicDistanceCm);
  }

  // Feed only after a healthy loop iteration.
  wdt_reset();
}
