const int R = 22, G = 23, B = 24;

void setRGB(bool r, bool g, bool b) {
  digitalWrite(R, r); digitalWrite(G, g); digitalWrite(B, b);
}

void setup() {
  pinMode(R, OUTPUT); pinMode(G, OUTPUT); pinMode(B, OUTPUT);
}

void loop() {
  setRGB(0,0,1); delay(500); // INITIALIZE - blue
  setRGB(0,1,0); delay(500); // SEARCH - green
  setRGB(1,1,1); delay(500); // PLOW - white
  setRGB(1,1,0); delay(500); // AVOID - yellow
  setRGB(1,0,1); delay(500); // RECOVERY - purple
  setRGB(1,1,0); delay(250); setRGB(0,0,0); delay(250); // low battery flash
  setRGB(1,0,0); delay(250); setRGB(0,0,0); delay(250); // fault flash
}
