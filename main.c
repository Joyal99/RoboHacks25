#include <Arduino.h>
#include <util/delay.h>

//-------------------- Motor Control Pins --------------------
#define LEFTM_FOWARD_PIN 4   // PD4 (Swapped with RIGHTM)
#define LEFTM_BACK_PIN   5   // PD5 (Swapped with RIGHTM)
#define RIGHTM_FOWARD_PIN 6  // PD6 (Swapped with LEFTM)
#define RIGHTM_BACK_PIN  7   // PD7 (Swapped with LEFTM)
// Enable (PWM) pins for L298N
#define RIGHTM_EN_PIN      3   // PWM for right motor (Swapped with LEFTM)
#define LEFTM_EN_PIN       11  // PWM for left motor (Swapped with RIGHTM)

#define DEFAULT_SPEED 75   // PWM speed (0-255)

//-------------------- IR Sensor Pins --------------------
#define RIGHT_SENSOR 12      // Right sensor pin (Swapped with LEFT_SENSOR)
#define LEFT_SENSOR 8        // Left sensor pin (Swapped with RIGHT_SENSOR)

//--------------------------------------------
// Motor Functions
//--------------------------------------------

void motorPinSetup() {
  pinMode(LEFTM_FOWARD_PIN, OUTPUT);
  pinMode(LEFTM_BACK_PIN,   OUTPUT);
  pinMode(RIGHTM_FOWARD_PIN, OUTPUT);
  pinMode(RIGHTM_BACK_PIN,  OUTPUT);
  pinMode(RIGHTM_EN_PIN,      OUTPUT);
  pinMode(LEFTM_EN_PIN,     OUTPUT);
}

void moveForward(uint8_t speed) {
  // Set both motors for forward motion (Reversed)
  digitalWrite(LEFTM_FOWARD_PIN, HIGH);
  digitalWrite(LEFTM_BACK_PIN,   LOW);
  digitalWrite(RIGHTM_FOWARD_PIN, HIGH);
  digitalWrite(RIGHTM_BACK_PIN,  LOW);
  analogWrite(LEFTM_EN_PIN, speed);
  analogWrite(RIGHTM_EN_PIN, speed);
}

void turnLeft(uint8_t speed) {
  // Turn left by reversing right motor
  digitalWrite(LEFTM_FOWARD_PIN, HIGH);
  digitalWrite(LEFTM_BACK_PIN,   LOW);
  analogWrite(LEFTM_EN_PIN, speed);
  
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,  HIGH);
  analogWrite(RIGHTM_EN_PIN, speed);
}

void turnRight(uint8_t speed) {
  // Turn right by reversing left motor
  digitalWrite(RIGHTM_FOWARD_PIN, HIGH);
  digitalWrite(RIGHTM_BACK_PIN,   LOW);
  analogWrite(RIGHTM_EN_PIN, speed);
  
  digitalWrite(LEFTM_FOWARD_PIN, LOW);
  digitalWrite(LEFTM_BACK_PIN,  HIGH);
  analogWrite(LEFTM_EN_PIN, speed);
}

void stopMotors() {
  analogWrite(LEFTM_EN_PIN,  0);
  analogWrite(RIGHTM_EN_PIN, 0);
  digitalWrite(LEFTM_FOWARD_PIN, LOW);
  digitalWrite(LEFTM_BACK_PIN,   LOW);
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,  LOW);
}

//--------------------------------------------
// IR Sensor Functions
//--------------------------------------------
void setupSensors() {
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(LEFT_SENSOR, INPUT);
}

int readLeftSensor() {
  int leftValue = digitalRead(LEFT_SENSOR);
  Serial.print("Left Sensor: ");
  Serial.println(leftValue);
  return leftValue;
}

int readRightSensor() {
  int rightValue = digitalRead(RIGHT_SENSOR);
  Serial.print("Right Sensor: ");
  Serial.println(rightValue);
  return rightValue;
}

//--------------------------------------------
// Main Function for Line Following
//--------------------------------------------
int main(){
  init();  // Initialize Arduino core functions
  Serial.begin(9600);
  
  motorPinSetup();
  setupSensors();
  
  while(1) {
    int leftVal  = readLeftSensor();
    int rightVal = readRightSensor();
    
    Serial.print("Left: ");
    Serial.print(leftVal);
    Serial.print(" | Right: ");
    Serial.println(rightVal);
    
    if (leftVal == HIGH && rightVal == LOW) {
      Serial.println("Action: Turn Right");
      turnRight(DEFAULT_SPEED);
    }
    else if (leftVal == LOW && rightVal == HIGH) {
      Serial.println("Action: Turn Left");
      turnLeft(DEFAULT_SPEED);
    }
    else {
      Serial.println("Action: Move Backward");
      moveForward(DEFAULT_SPEED);
    }
  }
  return 0;
}
