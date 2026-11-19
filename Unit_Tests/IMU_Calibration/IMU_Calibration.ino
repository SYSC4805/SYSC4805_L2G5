#include <Wire.h>
#include <LSM6.h>

LSM6 imu;
char report[80];
float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!imu.init()) {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }

  imu.enableDefault();

  long sumX = 0, sumY = 0, sumZ = 0;
  const int samples = 2000;

  for (int i = 0; i < samples; ++i) {
    imu.read();
    sumX += (int16_t)imu.g.x;
    sumY += (int16_t)imu.g.y;
    sumZ += (int16_t)imu.g.z;
    delay(5);
  }

  offsetX = (float)sumX / samples;
  offsetY = (float)sumY / samples;
  offsetZ = (float)sumZ / samples;
}

void loop() {
  imu.read();

  float Gx = (((int16_t)imu.g.x) - offsetX) * 8.75 / 1000.0;
  float Gy = (((int16_t)imu.g.y) - offsetY) * 8.75 / 1000.0;
  float Gz = (((int16_t)imu.g.z) - offsetZ) * 8.75 / 1000.0;

  snprintf(report, sizeof(report),
           "X-axis:%.3f Y-axis:%.3f Z-axis:%.3f",
           Gx, Gy, Gz);

  Serial.println(report);
  delay(500);
}
