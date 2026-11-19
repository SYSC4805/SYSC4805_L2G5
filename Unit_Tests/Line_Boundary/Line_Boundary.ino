const int LINE_SENSOR_PIN = A0;
int thresholdValue = 2048;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
}

void loop() {
  int raw = analogRead(LINE_SENSOR_PIN);
  Serial.print("raw=");
  Serial.print(raw);
  Serial.print(" boundary=");
  Serial.println(raw >= thresholdValue ? "YES" : "NO");
  delay(50);
}
