#include <ESP32Servo.h>
#include <Bluepad32.h>

// pin
const int IN1 = 16;  //M1
const int IN2 = 17;  //M1  
const int IN3 = 19;  //M2
const int IN4 = 21;  //M2

const int ENA = 18; //M1 PWM
const int ENB = 22; //M2 PWM

const int SERVO1_PIN = 33; 
const int SERVO2_PIN = 32; 

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
int currentClawAngle = 10; 

Servo gripper1;
Servo gripper2;
GamepadPtr myGamepad = nullptr;

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  gripper1.attach(SERVO1_PIN);
  gripper2.attach(SERVO2_PIN);
  updateGripper(currentClawAngle);

  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  BP32.enableVirtualDevice(false);
}

void loop() {
  BP32.update();
  if (myGamepad && myGamepad->isConnected()) {
    processGamepadInput();
  } else {
    stopRobot();
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

void processGamepadInput() {
  int axisY = myGamepad->axisY(); 
  int axisX = myGamepad->axisX(); 

  if (axisY < -50) moveForward();
  else if (axisY > 50) moveBackward();
  else if (axisX > 50) turnRight();
  else if (axisX < -50) turnLeft();
  else stopRobot();

  if (myGamepad->a()) {
    currentClawAngle += STEP_SIZE; 
    updateGripper(currentClawAngle);
    delay(50); 
  } else if (myGamepad->b()) {
    currentClawAngle -= STEP_SIZE; 
    updateGripper(currentClawAngle);
    delay(50);
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


