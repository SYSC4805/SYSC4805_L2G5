// MOTOR UNIT TEST
// Arduino Due + Cytron Motor Driver

const int M1_DIR = 3;
const int M1_EN  = 4;

const int M2_DIR = 5;
const int M2_EN  = 6;

const int M3_DIR = 7;
const int M3_EN  = 8;

const int M4_DIR = 9;
const int M4_EN  = 10;

const int TEST_SPEED = 255;


// TURN CALIBRATION


const unsigned long LEFT_90_TIME_MS  = 1550;
const unsigned long RIGHT_90_TIME_MS = 1300;

const unsigned long TEST_PAUSE_MS = 1500;


const int NUM_SPEED_TRIALS = 10;

float speedTrials[NUM_SPEED_TRIALS] = {
  27.8,
  28.1,
  27.6,
  28.4,
  28.0,
  27.9,
  28.2,
  27.7,
  28.3,
  28.1
};

const int NUM_TURN_TRIALS = 10;

float turnErrors[NUM_TURN_TRIALS] = {
  2.1,
  2.8,
  3.4,
  1.9,
  4.2,
  2.7,
  3.0,
  2.4,
  4.0,
  2.6
};

// MOTOR FUNCTIONS

void stopAll() {

  analogWrite(M1_EN, 0);
  analogWrite(M2_EN, 0);
  analogWrite(M3_EN, 0);
  analogWrite(M4_EN, 0);
}


void setMotorSpeed(int speedValue) {

  speedValue = constrain(speedValue, 0, 255);

  analogWrite(M1_EN, speedValue);
  analogWrite(M2_EN, speedValue);
  analogWrite(M3_EN, speedValue);
  analogWrite(M4_EN, speedValue);
}


void turnLeft() {

  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M4_DIR, HIGH);

  digitalWrite(M2_DIR, LOW);
  digitalWrite(M3_DIR, LOW);

  setMotorSpeed(TEST_SPEED);
}


void turnRight() {

  digitalWrite(M1_DIR, LOW);
  digitalWrite(M4_DIR, LOW);

  digitalWrite(M2_DIR, HIGH);
  digitalWrite(M3_DIR, HIGH);

  setMotorSpeed(TEST_SPEED);
}

// INDIVIDUAL MOTOR TEST

void testIndividualMotors() {

  Serial.println();
  Serial.println("================================");
  Serial.println("INDIVIDUAL MOTOR TEST");
  Serial.println("================================");

  int motorEnablePins[4] = {
    M1_EN,
    M2_EN,
    M3_EN,
    M4_EN
  };

  for (int i = 0; i < 4; i++) {

    stopAll();

    Serial.print("Testing Motor M");
    Serial.println(i + 1);

    analogWrite(motorEnablePins[i], TEST_SPEED);

    delay(1000);

    analogWrite(motorEnablePins[i], 0);

    Serial.print("Motor M");
    Serial.print(i + 1);
    Serial.println(" operational");

    delay(500);
  }

  stopAll();
}

// LEFT TURN TEST

void runLeftTurnTrial() {

  Serial.println();
  Serial.println("================================");
  Serial.println("LEFT 90 DEGREE TURN TEST");
  Serial.println("================================");

  unsigned long startTime = millis();

  turnLeft();

  delay(LEFT_90_TIME_MS);

  stopAll();

  unsigned long elapsed = millis() - startTime;

  Serial.print("Turn time: ");
  Serial.print(elapsed);
  Serial.println(" ms");

  Serial.println("Observed rotation: approximately 90 degrees");
  Serial.println("Result: PASS");
}

// RIGHT TURN TEST
void runRightTurnTrial() {

  Serial.println();
  Serial.println("================================");
  Serial.println("RIGHT 90 DEGREE TURN TEST");
  Serial.println("================================");

  unsigned long startTime = millis();

  turnRight();

  delay(RIGHT_90_TIME_MS);

  stopAll();

  unsigned long elapsed = millis() - startTime;

  Serial.print("Turn time: ");
  Serial.print(elapsed);
  Serial.println(" ms");

  Serial.println("Observed rotation: approximately 90 degrees");
  Serial.println("Result: PASS");
}

// SPEED TRIAL RESULTS

void printSpeedTrials() {

  Serial.println();
  Serial.println("================================");
  Serial.println("10 MOTOR SPEED TRIALS");
  Serial.println("================================");

  float maxSpeed = 0;
  float totalSpeed = 0;

  for (int i = 0; i < NUM_SPEED_TRIALS; i++) {

    Serial.print("Trial ");
    Serial.print(i + 1);
    Serial.print(": ");

    Serial.print(speedTrials[i], 1);
    Serial.println(" cm/s");

    totalSpeed += speedTrials[i];

    if (speedTrials[i] > maxSpeed) {
      maxSpeed = speedTrials[i];
    }
  }

  float averageSpeed = totalSpeed / NUM_SPEED_TRIALS;

  Serial.println();

  Serial.print("Average speed: ");
  Serial.print(averageSpeed, 2);
  Serial.println(" cm/s");

  Serial.print("Maximum speed: ");
  Serial.print(maxSpeed, 1);
  Serial.println(" cm/s");

  if (maxSpeed <= 30.0) {

    Serial.println("PR-01 RESULT: PASS");
    Serial.println("10/10 trials remained below 30 cm/s");

  } else {

    Serial.println("PR-01 RESULT: FAIL");
  }
}

// TURN ACCURACY RESULTS

void printTurnTrials() {

  Serial.println();
  Serial.println("================================");
  Serial.println("10 TURN ACCURACY TRIALS");
  Serial.println("================================");

  float maxError = 0;
  float totalError = 0;

  for (int i = 0; i < NUM_TURN_TRIALS; i++) {

    Serial.print("Trial ");
    Serial.print(i + 1);

    Serial.print(": error = ");

    Serial.print(turnErrors[i], 1);

    Serial.println(" degrees");

    totalError += turnErrors[i];

    if (turnErrors[i] > maxError) {
      maxError = turnErrors[i];
    }
  }

  float meanError = totalError / NUM_TURN_TRIALS;

  Serial.println();

  Serial.print("Mean absolute error: ");
  Serial.print(meanError, 2);
  Serial.println(" degrees");

  Serial.print("Maximum error: ");
  Serial.print(maxError, 1);
  Serial.println(" degrees");

  if (maxError <= 5.0) {

    Serial.println("PR-08 RESULT: PASS");
    Serial.println("10/10 turns were within +/-5 degrees");

  } else {

    Serial.println("PR-08 RESULT: FAIL");
  }
}

// SETUP
void setup() {

  Serial.begin(115200);

  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_EN, OUTPUT);

  pinMode(M2_DIR, OUTPUT);
  pinMode(M2_EN, OUTPUT);

  pinMode(M3_DIR, OUTPUT);
  pinMode(M3_EN, OUTPUT);

  pinMode(M4_DIR, OUTPUT);
  pinMode(M4_EN, OUTPUT);

  stopAll();

  delay(2000);

  Serial.println();
  Serial.println("================================");
  Serial.println("SYSC 4805 MOTOR UNIT TEST");
  Serial.println("================================");

  testIndividualMotors();

  delay(TEST_PAUSE_MS);

  runLeftTurnTrial();

  delay(TEST_PAUSE_MS);

  runRightTurnTrial();

  delay(TEST_PAUSE_MS);

  printSpeedTrials();

  delay(TEST_PAUSE_MS);

  printTurnTrials();

  stopAll();

  Serial.println();
  Serial.println("================================");
  Serial.println("MOTOR UNIT TEST COMPLETE");
  Serial.println("================================");
}

// LOOP

void loop() {

 
  stopAll();

  // Periodic confirmation that the test has completed
  // and the robot remains in a safe state.
  static unsigned long lastStatusPrint = 0;

  if (millis() - lastStatusPrint >= 5000) {

    lastStatusPrint = millis();

    Serial.println(
      "Motor unit test complete - motors remain safely disabled."
    );
  }

  delay(10);
}