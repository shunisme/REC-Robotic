#include <ESP32Servo.h>
#include <Bluepad32.h>
#include <math.h>

// pin
const int IN1 = 16;  //M1
const int IN2 = 17;  //M1
const int IN3 = 19;  //M2
const int IN4 = 21;  //M2

const int ENA = 18; //M1 PWM
const int ENB = 22; //M2 PWM

const int SERVO1_PIN = 33;   // claw
const int SERVO2_PIN = 32;   // claw

const int SHOULDER_PIN = 25; // CHANGE to your shoulder servo pin
const int ELBOW_PIN    = 26; // CHANGE to your elbow servo pin

// ================= ARM GEOMETRY (measure these) =================
// Coordinates: origin = shoulder pivot, x = forward, y = up (mm)
const float L1 = 80.0;   // shoulder pivot -> elbow pivot (mm)
const float L2 = 80.0;   // elbow pivot -> claw tip (mm)

// Servo calibration
const float SHOULDER_OFFSET = 90.0; // servo value when upper arm points straight forward (horizontal)
const float ELBOW_OFFSET    = 90.0; // servo value when forearm is in line with upper arm (straight)
const int   SHOULDER_DIR    = 1;    // +1 if a higher servo value lifts the upper arm, else -1
const int   ELBOW_DIR       = 1;    // +1 if a higher servo value bends the forearm upward, else -1
const bool  ELBOW_UP        = true; // which of the two IK solutions to use
const bool  ELBOW_ABSOLUTE  = false; // true if the elbow servo sits at the base and drives the forearm via a linkage

// Workspace limits
const float Y_MIN = -40.0;          // lowest the tip may go (keep it above the table/chassis)
const float R_MARGIN = 5.0;         // stay a little inside full reach

// Home position of the claw tip
const float HOME_X = 90.0;
const float HOME_Y = 40.0;

// Arm speed at full stick deflection, in mm per loop (loop ≈ 20 ms)
const float ARM_SPEED = 2.0;
const int   ARM_DEADZONE = 50;

float targetX = HOME_X;
float targetY = HOME_Y;

// Battery Monitoring
const int BATTERY_PIN = 34; // Must be an ADC1 pin
unsigned long lastBatteryCheck = 0;

const float R1 = 27000.0; // Connected to positive
const float R2 = 10000.0; // Connected to gnd
const float VOLTAGE_MULTIPLIER = (R1 + R2) / R2;

const float VALUE_MAX = 8400.0;
const float VALUE_MIN = 6400.0;

// manual claw config
const int CLAW_MIN_LIMIT = 10;
const int CLAW_MAX_LIMIT = 110;
const int STEP_SIZE = 2;
const unsigned long CLAW_STEP_MS = 50;
int currentClawAngle = 10;
unsigned long lastClawStep = 0;

unsigned long lastDebug = 0;

Servo gripper1;
Servo gripper2;
Servo shoulder;
Servo elbow;
GamepadPtr myGamepad = nullptr;

// Forward declarations
bool solveIK(float x, float y, float &shoulderDeg, float &elbowDeg);
void updateGripper(int targetAngle);
void stopRobot();

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  gripper1.attach(SERVO1_PIN);
  gripper2.attach(SERVO2_PIN);
  shoulder.attach(SHOULDER_PIN);
  elbow.attach(ELBOW_PIN);

  ledcAttach(ENA, 5000, 8);   // 5 kHz, duty 0–255
  ledcAttach(ENB, 5000, 8);

  updateGripper(currentClawAngle);

  // Move the arm to home
  float s, e;
  if (solveIK(targetX, targetY, s, e)) {
    shoulder.write((int)s);
    elbow.write((int)e);
  } else {
    Serial.println("HOME position unreachable - check L1, L2, HOME_X, HOME_Y");
  }

  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  BP32.enableVirtualDevice(false);
}

void loop() {
  BP32.update();
  if (myGamepad && myGamepad->isConnected()) {
    processGamepadInput();
  } else {
    stopRobot();
    setSpeed(0);
  }

  checkBattery();
  delay(20);
}

void onConnectedGamepad(GamepadPtr gp) {
  if (!myGamepad) {
    myGamepad = gp;
    Serial.println("Controller connected!");
  }
}

void onDisconnectedGamepad(GamepadPtr gp) {
  if (myGamepad == gp) {
    myGamepad = nullptr;
    Serial.println("Controller disconnected!");
    stopRobot();
  }
}

int toDuty(int v) { return map(constrain(abs(v), 50, 512), 50, 512, 90, 255); }
void setSpeed(int d) { ledcWrite(ENA, d); ledcWrite(ENB, d); }

// ================= INVERSE KINEMATICS =================
// Converts a tip position (x, y) into servo values.
// Returns false (and leaves outputs untouched) if the point can't be reached.
bool solveIK(float x, float y, float &shoulderDeg, float &elbowDeg) {
  if (y < Y_MIN) return false;

  float r2 = x * x + y * y;
  float r = sqrt(r2);
  if (r > L1 + L2 - R_MARGIN) return false;       // too far
  if (r < fabs(L1 - L2) + R_MARGIN) return false; // too close to the shoulder

  float c2 = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
  c2 = constrain(c2, -1.0, 1.0);

  float t2 = acos(c2);              // elbow bend, 0 = straight
  if (ELBOW_UP) t2 = -t2;

  float t1 = atan2(y, x) - atan2(L2 * sin(t2), L1 + L2 * cos(t2)); // upper arm angle from horizontal

  float elbowAngle = ELBOW_ABSOLUTE ? (t1 + t2) : t2;

  float s = SHOULDER_OFFSET + SHOULDER_DIR * degrees(t1);
  float e = ELBOW_OFFSET    + ELBOW_DIR    * degrees(elbowAngle);

  if (s < 0 || s > 180 || e < 0 || e > 180) return false; // beyond servo travel

  shoulderDeg = s;
  elbowDeg = e;
  return true;
}

void processArm() {
  int rx = myGamepad->axisRX();
  int ry = myGamepad->axisRY();
  if (abs(rx) < ARM_DEADZONE) rx = 0;
  if (abs(ry) < ARM_DEADZONE) ry = 0;
  if (rx == 0 && ry == 0) return;

  // Stick right = reach forward, stick up (negative Y) = move up
  float newX = targetX + (rx / 512.0) * ARM_SPEED;
  float newY = targetY - (ry / 512.0) * ARM_SPEED;

  float s, e;
  if (solveIK(newX, newY, s, e)) {
    targetX = newX;
    targetY = newY;
    shoulder.write((int)s);
    elbow.write((int)e);
  }
  // If unreachable, the arm simply stops at the edge of its workspace.

  if (millis() - lastDebug > 200) {
    lastDebug = millis();
    Serial.printf("tip x=%.1f y=%.1f  shoulder=%.0f elbow=%.0f\n", targetX, targetY, s, e);
  }
}

void processGamepadInput() {
  int axisY = myGamepad->axisY();
  int axisX = myGamepad->axisX();

  if      (axisY < -50) { moveForward();  setSpeed(toDuty(axisY)); }
  else if (axisY >  50) { moveBackward(); setSpeed(toDuty(axisY)); }
  else if (axisX >  50) { turnRight();    setSpeed(toDuty(axisX)); }
  else if (axisX < -50) { turnLeft();     setSpeed(toDuty(axisX)); }
  else                  { stopRobot();    setSpeed(0); }

  processArm();

  // Claw: non-blocking so the arm and wheels keep responding while A/B is held
  if (millis() - lastClawStep >= CLAW_STEP_MS) {
    if (myGamepad->a()) {
      updateGripper(currentClawAngle + STEP_SIZE);
      lastClawStep = millis();
    } else if (myGamepad->b()) {
      updateGripper(currentClawAngle - STEP_SIZE);
      lastClawStep = millis();
    }
  }

  // Press Y to send the arm back home
  if (myGamepad->y()) {
    float s, e;
    if (solveIK(HOME_X, HOME_Y, s, e)) {
      targetX = HOME_X;
      targetY = HOME_Y;
      shoulder.write((int)s);
      elbow.write((int)e);
    }
  }
}

void checkBattery() {
  if (millis() - lastBatteryCheck > 5000) {
    lastBatteryCheck = millis();

    uint32_t pinMv = analogReadMilliVolts(BATTERY_PIN);
    float actualMv = pinMv * VOLTAGE_MULTIPLIER;

    float batteryPct = ((actualMv - VALUE_MIN) / (VALUE_MAX - VALUE_MIN)) * 100.0;
    batteryPct = constrain(batteryPct, 0.0, 100.0);

    Serial.print("Battery Level: ");
    Serial.print(batteryPct, 0);
    Serial.println("%");

    if (batteryPct < 50.0) {
      Serial.println("WARNING: Battery below 50% - Swap Soon!");
    }
  }
}

void moveForward() { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void moveBackward() { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
void turnLeft() { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void turnRight() { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
void stopRobot() { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }

void updateGripper(int targetAngle) {
  int safeAngle = constrain(targetAngle, CLAW_MIN_LIMIT, CLAW_MAX_LIMIT);
  currentClawAngle = safeAngle;
  gripper1.write(safeAngle);
  gripper2.write(safeAngle);
}