#include <ESP32Servo.h>
#include <Bluepad32.h>

// --- PIN DEFINITIONS (Matching your schematic) ---
const int IN1 = 16;  
const int IN2 = 17;  
const int IN3 = 19;  
const int IN4 = 21;  

const int ENA = 18;
const int ENB = 22;

const int SERVO1_PIN = 33; 
const int SERVO2_PIN = 32; 

const int ANGLE_OPEN = 10;
const int ANGLE_CLOSE = 90;

Servo gripper1;
Servo gripper2;

// Controller instance pointer
GamepadPtr myGamepad = nullptr;

void setup() {
  Serial.begin(115200);

  // Initialize Motor Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  // Initialize Servos
  gripper1.attach(SERVO1_PIN);
  gripper2.attach(SERVO2_PIN);
  openGripper();

  // Initialize Bluepad32 for ESP32 Bluetooth controllers
  BP32.setup();
  BP32.enableVirtualDevice(false);
}

void loop() {
  // Update the controller states
  bool dataUpdated = BP32.update();
  if (dataUpdated && myGamepad && myGamepad->isConnected()) {
    processGamepadInput();
  } else {
    // If controller disconnects, stop the robot for safety
    stopRobot();
  }
  delay(20); // Small loop delay
}

// Callback when controller connects
void onConnectedGamepad(GamepadPtr gp) {
  if (!myGamepad) {
    myGamepad = gp;
    Serial.println("Controller connected!");
  }
}

// Callback when controller disconnects
void onDisconnectedGamepad(GamepadPtr gp) {
  if (myGamepad == gp) {
    myGamepad = nullptr;
    Serial.println("Controller disconnected!");
    stopRobot();
  }
}

void processGamepadInput() {
  // Read joystick values (axis typically ranges from -511 to 512 or -128 to 127 depending on library)
  int axisY = myGamepad->axisY(); // Left stick vertical (forward/backward)
  int axisX = myGamepad->axisX(); // Left stick horizontal (turning)

  // Simple threshold / deadzone to prevent drifting
  if (axisY < -50) {
    moveForward();
  } else if (axisY > 50) {
    moveBackward();
  } else if (axisX > 50) {
    turnRight();
  } else if (axisX < -50) {
    turnLeft();
  } else {
    stopRobot();
  }

  // Button controls for Gripper (e.g., Cross/A button to close, Circle/B button to open)
  if (myGamepad->a() || myGamepad->b()) { 
    closeGripper();
  } else {
    openGripper();
  }
}

// ==========================================
// REUSABLE LOCOMOTION FUNCTIONS
// ==========================================
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  *Verification:* Both motors spin forward when joystick is pushed up.
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  *Verification:* Both motors spin in reverse when joystick is pulled down.
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  *Verification:* Robot pivots left when left stick is pushed left.
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  *Verification:* Robot pivots right when left stick is pushed right.
}

void stopRobot() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  *Verification:* Motors halt when joysticks are centered.
}

// ==========================================
// REUSABLE GRIPPING FUNCTIONS
// ==========================================
void openGripper() {
  gripper1.write(ANGLE_OPEN);
  gripper2.write(ANGLE_OPEN);
  *Verification:* Servos move to open position.
}

void closeGripper() {
  gripper1.write(ANGLE_CLOSE);
  gripper2.write(ANGLE_CLOSE);
  *Verification:* Servos clamp down.
}