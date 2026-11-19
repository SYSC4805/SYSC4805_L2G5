// Due-side heartbeat test. Connect the Nano heartbeat output to Due D30.
const int HEARTBEAT_PIN = 30;
bool lastLevel = LOW;
unsigned long lastChange = 0;

void setup() {
  Serial.begin(115200);
  pinMode(HEARTBEAT_PIN, INPUT);
  lastLevel = digitalRead(HEARTBEAT_PIN);
  lastChange = millis();
}

void loop() {
  bool now = digitalRead(HEARTBEAT_PIN);

  if (now != lastLevel) {
    lastLevel = now;
    lastChange = millis();
  }

  if (millis() - lastChange > 2000) {
    Serial.println("WATCHDOG/HEARTBEAT FAULT -> SAFE STOP");
  }
}
