const int BATTERY_PIN = A1;
const float ADC_REF_V = 3.3;
const float ADC_MAX = 4095.0;
const float R_TOP = 10000.0;
const float R_BOTTOM = 10000.0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
}

void loop() {
  int raw = analogRead(BATTERY_PIN);
  float adcV = raw / ADC_MAX * ADC_REF_V;
  float batteryV = adcV * (R_TOP + R_BOTTOM) / R_BOTTOM;

  Serial.print("batteryV=");
  Serial.println(batteryV, 3);
  delay(100);
}
