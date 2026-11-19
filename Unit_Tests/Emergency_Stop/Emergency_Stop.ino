const int ESTOP_PIN = 25;
volatile bool stopped = false;

void stopISR() {
  stopped = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), stopISR, FALLING);
}

void loop() {
  if (stopped) {
    Serial.println("ESTOP");
  }
}
