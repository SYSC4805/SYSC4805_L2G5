// ============================================================
// Line_Boundary_Turn  (version WITHOUT the watchdog)
//
// Two line followers (front-left A1, front-right A0) + the
// IMU/motor TurnController module.
//
// Behaviour (strip clearing, no Nano / obstacle sensors needed):
//   1. Drive forward.
//   2. Either line follower sees the boundary tape ->
//        stop, reverse a little,
//        turn 90 (gyro)  -> drive one strip width -> turn 90 (gyro)
//      The turn direction alternates every pass, so the robot
//      snakes across the arena (net effect: 180 turn + shift
//      into the next strip).
//   3. If the boundary is hit while shifting sideways, the arena
//      edge has been reached -> stop, run complete.
//
// Safety: 5-minute limit, e-stop pin (LOW = stop), any turn that
// fails (IMU unrecoverable / timeout / no rotation) stops the robot.
//
// Files needed in this sketch folder (copies of Arduino_Due/MainController):
//   Config.h  MotorHandler.h/.cpp  IMUHandler.h/.cpp
//   LineSensorHandler.h/.cpp  TurnController.h/.cpp
// ============================================================

#include "Config.h"
#include "MotorHandler.h"
#include "LineSensorHandler.h"
#include "TurnController.h"
#include "IMUHandler.h"


// false = use the thresholds in Config.h (measured values).
// true  = re-run the interactive calibration at boot.
const bool CALIBRATE_LINES_ON_BOOT = false;

// Direction of the FIRST U-turn (it alternates after that).
// Start with the arena wall on the robot's LEFT  -> false (turn right first).
// Start with the arena wall on the robot's RIGHT -> true  (turn left first).
const bool FIRST_TURN_IS_LEFT = false;

unsigned long runStartMs = 0;
unsigned int  passCount  = 0;
unsigned int  stuckBackoffs = 0;       // consecutive back-offs that never cleared
bool          sweepReversed = false;   // flips when the arena edge is reached


// plain delay (kept as waitMs so the rest of the code is unchanged)
static void waitMs(unsigned long ms) {
  delay(ms);
}


// ==================================================
// HELPERS
// ==================================================

static bool emergencyStopPressed() {
  return digitalRead(ESTOP_PIN) == LOW;
}

static void haltRobot(const char *reason) {

  stopAllMotors();

  Serial.println();
  Serial.print("*** STOPPED: ");
  Serial.println(reason);

  // Stay stopped
  while (true) {
    delay(50);
  }
}

static void countdown(const char *message, int seconds) {

  Serial.println();
  Serial.println(message);

  for (int s = seconds; s > 0; s--) {

    Serial.print("  ");
    Serial.print(s);
    Serial.println("...");

    waitMs(1000);
  }
}

// Average both sensors over ms milliseconds
static void sampleLines(unsigned long ms, int &left, int &right) {

  long sumL = 0;
  long sumR = 0;
  int  n    = 0;

  unsigned long start = millis();

  while (millis() - start < ms) {


    sumL += lineSensorRawLeft();
    sumR += lineSensorRawRight();
    n++;

    delay(5);
  }

  left  = (n > 0) ? (int)(sumL / n) : 0;
  right = (n > 0) ? (int)(sumR / n) : 0;
}


// ==================================================
// LINE SENSOR CALIBRATION (interactive, ~25 s)
// ==================================================

static void calibrateLines() {

  int floorL, floorR, tapeL, tapeR, dummy;

  Serial.println();
  Serial.println("================================");
  Serial.println("LINE SENSOR CALIBRATION");
  Serial.println("================================");

  countdown("Put BOTH sensors over the plain FLOOR (hold still)", 5);
  sampleLines(1500, floorL, floorR);

  Serial.print("Floor: left ");
  Serial.print(floorL);
  Serial.print(" | right ");
  Serial.println(floorR);

  countdown("Put the LEFT sensor over the BLACK TAPE (hold still)", 5);
  sampleLines(1500, tapeL, dummy);

  Serial.print("Tape (left): ");
  Serial.println(tapeL);

  countdown("Put the RIGHT sensor over the BLACK TAPE (hold still)", 5);
  sampleLines(1500, dummy, tapeR);

  Serial.print("Tape (right): ");
  Serial.println(tapeR);

  bool okL = lineSensorSetCalibration(LINE_SIDE_LEFT,  floorL, tapeL);
  bool okR = lineSensorSetCalibration(LINE_SIDE_RIGHT, floorR, tapeR);

  Serial.println();

  Serial.print("LEFT  threshold ");
  Serial.print(lineSensorThresholdLeft());
  Serial.print(lineSensorBoundaryIsHighLeft() ? " (tape reads HIGHER)" : " (tape reads LOWER)");
  Serial.println(okL ? "" : "  <-- FAILED, readings too close");

  Serial.print("RIGHT threshold ");
  Serial.print(lineSensorThresholdRight());
  Serial.print(lineSensorBoundaryIsHighRight() ? " (tape reads HIGHER)" : " (tape reads LOWER)");
  Serial.println(okR ? "" : "  <-- FAILED, readings too close");

  if (!okL || !okR) {
    haltRobot("line calibration failed - check wiring/mounting height");
  }

  Serial.println("Put these in Config.h to skip calibration next time.");
}


// ==================================================
// BOUNDARY RECOVERY
// ==================================================

// Drive sideways into the next strip. Returns false if the boundary
// was hit on the way (arena edge).
static bool shiftIntoNextStrip() {

  unsigned long start = millis();

  driveForward(SHIFT_PWM);

  while (millis() - start < STRIP_SHIFT_MS) {


    if (emergencyStopPressed()) {
      haltRobot("emergency stop");
    }

    LineSide side = LINE_SIDE_NONE;

    LineReadStatus status = lineSensorRead(&side);

    if (status == LINE_READ_ERROR) {
      haltRobot("line sensor read error");
    }

    // Ignore the line sensors at the very start (tape we just turned away from)
    if (status == LINE_BOUNDARY && millis() - start >= SHIFT_BLIND_MS) {

      stopAllMotors();

      Serial.print("[SHIFT] boundary after ");
      Serial.print(millis() - start);
      Serial.print(" ms | triggered: ");
      Serial.print(lineSideText(side));
      Serial.print(" | raw L ");
      Serial.print(lineSensorLastRawLeft());
      Serial.print(" R ");
      Serial.println(lineSensorLastRawRight());

      return false;
    }

    delay(5);
  }

  stopAllMotors();

  return true;
}

// Turn `deg` degrees with the gyro. If the gyro turn fails (IMU lost for good
// or timeout) the remaining angle is finished with a timed turn so the robot
// keeps going. E-stop and "no rotation at all" still stop the robot.
static void doTurn(bool left, float deg) {

  TurnReport rep = {0, 0, 0, 0};

  TurnResult result = left ? turnLeft(deg, &rep) : turnRight(deg, &rep);

  if (result == TURN_OK) {
    return;
  }

  bool canFallBack = TURN_FALLBACK_ENABLED &&
                     (result == TURN_IMU_FAIL || result == TURN_TIMEOUT);

  if (!canFallBack) {
    haltRobot(turnResultText(result));
  }

  stopAllMotors();

  float remaining = deg - rep.measuredDeg;

  if (remaining > deg) remaining = deg;

  Serial.print("[TURN FALLBACK] gyro turn ended with ");
  Serial.print(turnResultText(result));
  Serial.print(" at ");
  Serial.print(rep.measuredDeg, 1);
  Serial.print(" deg; finishing ");
  Serial.print(remaining, 1);
  Serial.println(" deg with a TIMED turn (accuracy not guaranteed)");

  if (remaining > 2.0f) {

    // Use this turn's own speed if it ran long enough to be meaningful
    float dps = TURN_FALLBACK_DPS;

    if (rep.measuredDeg >= 30.0f && rep.motorOnMs > 0) {

      float measured = rep.measuredDeg * 1000.0f / (float)rep.motorOnMs;

      if (measured >= 15.0f && measured <= 60.0f) {
        dps = measured;
      }
    }

    unsigned long ms = (unsigned long)(remaining / dps * 1000.0f);

    unsigned long start = millis();

    if (left) pivotLeft(TURN_PWM); else pivotRight(TURN_PWM);

    while (millis() - start < ms) {


      if (emergencyStopPressed()) {
        haltRobot("emergency stop");
      }

      delay(5);
    }

    stopAllMotors();
  }

  waitMs(200);

  // Try to bring the IMU back for the next turn
  imuRecover();
}

// Reverse until both sensors are off the tape (the tape is wider than a short
// reverse), then a bit more for margin.
// (The full robot only reverses if the rear Nano sensor reports clear.)
static void backOffFromTape() {

  unsigned long revStart = millis();

  bool cleared = false;

  driveReverse(BOUNDARY_REVERSE_PWM);

  while (millis() - revStart < BOUNDARY_CLEAR_MAX_MS) {


    if (emergencyStopPressed()) {
      haltRobot("emergency stop");
    }

    if (millis() - revStart >= BOUNDARY_REVERSE_MS) {

      LineReadStatus st = lineSensorRead();

      if (st == LINE_READ_ERROR) {
        haltRobot("line sensor read error");
      }

      if (st == LINE_CLEAR) {
        cleared = true;
        break;
      }
    }

    delay(5);
  }

  if (cleared) {
    waitMs(BOUNDARY_CLEAR_EXTRA_MS);   // still reversing
  }

  stopAllMotors();

  Serial.print("[BOUNDARY] reversed ");
  Serial.print(millis() - revStart);
  Serial.print(" ms | raw L ");
  Serial.print(lineSensorLastRawLeft());
  Serial.print(" R ");
  Serial.println(lineSensorLastRawRight());

  // Still reading "boundary" after the maximum reverse: the threshold is probably
  // too low for this part of the floor. Keep going (the turn is still the right
  // move), but stop if it happens several times in a row.
  if (!cleared) {

    stuckBackoffs++;

    Serial.println("[WARN] line sensors still read boundary after the maximum reverse - check thresholds");

    if (stuckBackoffs >= 3) {
      haltRobot("line sensors keep reading the boundary - check thresholds / sensor height");
    }

  } else {

    stuckBackoffs = 0;
  }
}

static void performBoundaryRecovery(LineSide side) {

  stopAllMotors();

  waitMs(100);

  Serial.println();
  Serial.print("[BOUNDARY] pass ");
  Serial.print(passCount + 1);
  Serial.print(" | triggered: ");
  Serial.print(lineSideText(side));
  Serial.print(" | raw L ");
  Serial.print(lineSensorLastRawLeft());
  Serial.print(" R ");
  Serial.println(lineSensorLastRawRight());

  backOffFromTape();

  waitMs(100);

  // Alternate direction every pass so the robot snakes across the arena
  bool toLeft = (((passCount % 2 == 0) == FIRST_TURN_IS_LEFT) != sweepReversed);

  // ---- first 90 ----
  doTurn(toLeft, 90.0f);

  waitMs(100);

  // ---- one strip across ----
  if (!shiftIntoNextStrip()) {

    // Boundary ahead while shifting = arena edge on this side.
    Serial.println("[BOUNDARY] hit the boundary while shifting: arena edge reached");

    if (STOP_AT_ARENA_EDGE) {
      haltRobot("RUN COMPLETE (arena edge)");
    }

    // Keep going: back off the tape, finish the U-turn, and sweep the other way
    backOffFromTape();

    sweepReversed = !sweepReversed;
  }

  waitMs(100);

  // ---- second 90 (same direction) ----
  doTurn(toLeft, 90.0f);

  waitMs(100);

  passCount++;
}


// ==================================================
// SETUP / LOOP
// ==================================================

void setup() {

  // Motors off as early as possible (after any reset the pins float)
  motorBegin();

  Serial.begin(115200);

  waitMs(1500);

  Serial.println();
  Serial.println("================================");
  Serial.println("LINE BOUNDARY + TURN TEST");
  Serial.println("================================");

  analogReadResolution(12);

  pinMode(ESTOP_PIN, INPUT_PULLUP);

  lineSensorBegin();

  Serial.println("Keep the robot completely still: calibrating gyro...");

  waitMs(2000);

  if (!turnControllerBegin()) {
    haltRobot("IMU init / calibration failed");
  }

  // e-stop also interrupts a turn in progress
  turnControllerSetAbortCheck(emergencyStopPressed);

  if (CALIBRATE_LINES_ON_BOOT) {
    calibrateLines();
  }

  countdown("Place the robot at the start of a strip. Starting in", 5);

  runStartMs = millis();
}

void loop() {


  if (emergencyStopPressed()) {
    haltRobot("emergency stop");
  }

  if (millis() - runStartMs >= MAX_RUN_MS) {
    haltRobot("5-minute run limit reached");
  }

  LineSide side = LINE_SIDE_NONE;

  LineReadStatus status = lineSensorRead(&side);

  if (status == LINE_READ_ERROR) {
    haltRobot("line sensor read error");
  }

  if (status == LINE_BOUNDARY) {

    performBoundaryRecovery(side);

    return;
  }

  driveForward(DRIVE_PWM);

  delay(5);
}