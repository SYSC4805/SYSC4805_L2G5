#include <Wire.h>
#include <LSM6.h>

LSM6 imu;


// I2C PINS FOR ARDUINO DUE
 

const int SDA_PIN = 20;
const int SCL_PIN = 21;


 
// MOTOR PINS

const int M1_DIR = 3;
const int M1_EN  = 4;

const int M2_DIR = 5;
const int M2_EN  = 6;

const int M3_DIR = 7;
const int M3_EN  = 8;

const int M4_DIR = 9;
const int M4_EN  = 10;


 
// TURN SETTINGS
 

const int TURN_SPEED = 255;

// IMU-controlled turn targets
const float LEFT_TARGET  = 88.0;
const float RIGHT_TARGET = -88.0;

// Ignore very small corrected gyro noise
const float GYRO_DEADBAND = 3.0;

// Ignore invalid/dead raw readings close to zero
const float RAW_INVALID_THRESHOLD = 0.20;


 
// SAFETY FALLBACK TIMES
 

// These remain as backup only if IMU readings fail
const unsigned long LEFT_MAX_TIME  = 1550;
const unsigned long RIGHT_MAX_TIME = 1450;


 
// GYRO
 

float gyroZOffset = 0.0;


 
// I2C BUS RECOVERY
 

void recoverI2CBus() {

  Serial.println("Recovering I2C bus...");

  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT);

  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(10);

  // Pulse SCL 9 times
  for (int i = 0; i < 9; i++) {

    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(10);

    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(10);
  }

  // Generate STOP-like condition
  pinMode(SDA_PIN, OUTPUT);

  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(10);

  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(10);

  // Release bus
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);

  delay(100);

  Serial.println("I2C recovery complete.");
}


 
// SETUP
 

void setup() {

  Serial.begin(115200);

  delay(1500);

  Serial.println();
  Serial.println("================================");
  Serial.println("IMU + MOTOR TURN TEST");
  Serial.println("================================");


   
  // MOTOR SETUP
   

  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_EN, OUTPUT);

  pinMode(M2_DIR, OUTPUT);
  pinMode(M2_EN, OUTPUT);

  pinMode(M3_DIR, OUTPUT);
  pinMode(M3_EN, OUTPUT);

  pinMode(M4_DIR, OUTPUT);
  pinMode(M4_EN, OUTPUT);

  stopAll();

  Serial.println("Motor pins initialized.");


   
  // I2C RECOVERY + STARTUP
   

  Serial.println();

  recoverI2CBus();

  delay(500);

  Wire.begin();

  delay(1000);


   
  // IMU INITIALIZATION
   

  Serial.println();
  Serial.println("Initializing IMU...");

  bool imuReady = false;

  for (int attempt = 1; attempt <= 5; attempt++) {

    Serial.print("IMU initialization attempt ");
    Serial.println(attempt);

    if (imu.init()) {

      imuReady = true;

      Serial.println("IMU detected successfully.");

      break;
    }

    Serial.println("IMU not detected. Retrying...");

    delay(500);
  }


  if (!imuReady) {

    Serial.println();
    Serial.println("ERROR: IMU INITIALIZATION FAILED");

    stopAll();

    while (1) {
      delay(1000);
    }
  }


  imu.enableDefault();

  Serial.println("IMU initialized and enabled.");


   
  // GYRO CALIBRATION
   

  Serial.println();
  Serial.println("================================");
  Serial.println("KEEP ROBOT COMPLETELY STILL");
  Serial.println("CALIBRATING GYRO");
  Serial.println("================================");

  delay(2000);

  calibrateGyro();

  Serial.print("Gyro Z offset = ");
  Serial.println(gyroZOffset, 4);

  Serial.println("Calibration complete.");

  delay(2000);


   
  // LEFT TURN TEST
   

  Serial.println();
  Serial.println("================================");
  Serial.println("LEFT TURN TEST");
  Serial.println("================================");

  turnLeft90();


   
  // WAIT
   

  Serial.println();
  Serial.println("Waiting 4 seconds...");

  delay(4000);


   
  // RIGHT TURN TEST
   

  Serial.println();
  Serial.println("================================");
  Serial.println("RIGHT TURN TEST");
  Serial.println("================================");

  turnRight90();


   
  // FINISH
   

  stopAll();

  Serial.println();
  Serial.println("================================");
  Serial.println("TEST COMPLETE");
  Serial.println("ROBOT STOPPED");
  Serial.println("================================");
}


 
// LOOP
 

void loop() {
  // Runs once only
}


 
// LEFT TURN
 

void turnLeft90() {

  float angle = 0.0;

  unsigned long startTime = millis();
  unsigned long previousTime = micros();
  unsigned long lastPrint = 0;


  // Known working LEFT motor pattern
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M4_DIR, HIGH);

  digitalWrite(M2_DIR, LOW);
  digitalWrite(M3_DIR, LOW);

  setMotorSpeed(TURN_SPEED);

  Serial.println("LEFT TURN START");


  while (true) {

    unsigned long elapsed =
      millis() - startTime;


     
    // READ IMU
     

    imu.read();

    unsigned long now =
      micros();

    float dt =
      (now - previousTime) / 1000000.0;

    previousTime = now;


    float rawZ =
      imu.g.z * 0.00875;


     
    // USE VALID GYRO DATA
     

    if (abs(rawZ) >= RAW_INVALID_THRESHOLD) {

      float correctedZ =
        rawZ - gyroZOffset;


      if (abs(correctedZ) < GYRO_DEADBAND) {

        correctedZ = 0.0;
      }


      // Left rotation should be positive
      if (correctedZ > 0) {

        angle += correctedZ * dt;
      }
    }


     
    // IMU TARGET REACHED
     

    if (angle >= LEFT_TARGET) {

      stopAll();

      Serial.println();
      Serial.println("*** LEFT TARGET REACHED ***");

      Serial.print("Final angle = ");
      Serial.println(angle, 2);

      Serial.print("Turn time = ");
      Serial.print(elapsed);
      Serial.println(" ms");

      return;
    }


     
    // SAFETY FALLBACK
     

    if (elapsed >= LEFT_MAX_TIME) {

      stopAll();

      Serial.println();
      Serial.println("LEFT TIME FALLBACK");

      Serial.print("Measured gyro angle = ");
      Serial.println(angle, 2);

      Serial.print("Turn time = ");
      Serial.print(elapsed);
      Serial.println(" ms");

      return;
    }


     
    // DEBUG
     

    if (millis() - lastPrint >= 100) {

      lastPrint = millis();

      Serial.print("LEFT | RawZ: ");
      Serial.print(rawZ, 2);

      Serial.print(" | Angle: ");
      Serial.print(angle, 2);

      Serial.print(" | Time: ");
      Serial.println(elapsed);
    }


    delay(5);
  }
}


 
// RIGHT TURN
 

void turnRight90() {

  float angle = 0.0;

  unsigned long startTime = millis();
  unsigned long previousTime = micros();
  unsigned long lastPrint = 0;


  // Known working RIGHT motor pattern
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M4_DIR, LOW);

  digitalWrite(M2_DIR, HIGH);
  digitalWrite(M3_DIR, HIGH);

  setMotorSpeed(TURN_SPEED);

  Serial.println("RIGHT TURN START");


  while (true) {

    unsigned long elapsed =
      millis() - startTime;


     
    // READ IMU
     

    imu.read();

    unsigned long now =
      micros();

    float dt =
      (now - previousTime) / 1000000.0;

    previousTime = now;


    float rawZ =
      imu.g.z * 0.00875;


     
    // USE VALID GYRO DATA
     

    if (abs(rawZ) >= RAW_INVALID_THRESHOLD) {

      float correctedZ =
        rawZ - gyroZOffset;


      if (abs(correctedZ) < GYRO_DEADBAND) {

        correctedZ = 0.0;
      }


      // Right rotation should be negative
      if (correctedZ < 0) {

        angle += correctedZ * dt;
      }
    }


     
    // IMU TARGET REACHED
     

    if (angle <= RIGHT_TARGET) {

      stopAll();

      Serial.println();
      Serial.println("*** RIGHT TARGET REACHED ***");

      Serial.print("Final angle = ");
      Serial.println(angle, 2);

      Serial.print("Turn time = ");
      Serial.print(elapsed);
      Serial.println(" ms");

      return;
    }


     
    // SAFETY FALLBACK
     

    if (elapsed >= RIGHT_MAX_TIME) {

      stopAll();

      Serial.println();
      Serial.println("RIGHT TIME FALLBACK");

      Serial.print("Measured gyro angle = ");
      Serial.println(angle, 2);

      Serial.print("Turn time = ");
      Serial.print(elapsed);
      Serial.println(" ms");

      return;
    }


     
    // DEBUG
     

    if (millis() - lastPrint >= 100) {

      lastPrint = millis();

      Serial.print("RIGHT | RawZ: ");
      Serial.print(rawZ, 2);

      Serial.print(" | Angle: ");
      Serial.print(angle, 2);

      Serial.print(" | Time: ");
      Serial.println(elapsed);
    }


    delay(5);
  }
}


 
// MOTOR SPEED


void setMotorSpeed(int speedValue) {

  analogWrite(M1_EN, speedValue);
  analogWrite(M2_EN, speedValue);
  analogWrite(M3_EN, speedValue);
  analogWrite(M4_EN, speedValue);
}



// STOP ALL MOTORS

void stopAll() {

  analogWrite(M1_EN, 0);
  analogWrite(M2_EN, 0);
  analogWrite(M3_EN, 0);
  analogWrite(M4_EN, 0);
}


// GYRO CALIBRATION
void calibrateGyro() {

  const int samples = 500;

  float sum = 0.0;


  for (int i = 0; i < samples; i++) {

    imu.read();

    float rawZ =
      imu.g.z * 0.00875;

    sum += rawZ;

    delay(5);
  }


  gyroZOffset =
    sum / samples;
}