#include <Arduino.h>
#include <util/delay.h>

//-------------------- Motor Control Pins --------------------
#define RIGHTM_FOWARD_PIN 4   // PD4
#define RIGHTM_BACK_PIN   5   // PD5
#define LEFTM_FOWARD_PIN  6   // PD6
#define LEFTM_BACK_PIN    7   // PD7
// Enable (PWM) pins for L298N
#define LEFTM_EN_PIN      3   // PWM for left motor
#define RIGHTM_EN_PIN     11  // PWM for right motor

#define DEFAULT_SPEED 75   // PWM speed (0-255)

//-------------------- IR Sensor Pins --------------------
#define LEFT_SENSOR 12      // Left sensor pin
#define RIGHT_SENSOR 8      // Right sensor pin

//--------------------------------------------
// Motor Functions
//--------------------------------------------

void motorPinSetup() {
  pinMode(RIGHTM_FOWARD_PIN, OUTPUT);
  pinMode(RIGHTM_BACK_PIN,   OUTPUT);
  pinMode(LEFTM_FOWARD_PIN,  OUTPUT);
  pinMode(LEFTM_BACK_PIN,    OUTPUT);
  pinMode(LEFTM_EN_PIN,      OUTPUT);
  pinMode(RIGHTM_EN_PIN,     OUTPUT);
}

/*
  Our "moveForward" command has been set up so that the robot moves
  forward when both motors run in the proper forward direction.
  (If the robot still moves backward, try swapping the HIGH/LOW
  settings in this function.)
*/
void moveForward(uint8_t speed) {
  // Set both motors for forward motion
  digitalWrite(LEFTM_FOWARD_PIN, LOW);
  digitalWrite(LEFTM_BACK_PIN,   HIGH);
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,  HIGH);
  analogWrite(LEFTM_EN_PIN, speed);
  analogWrite(RIGHTM_EN_PIN, speed);
}

/*
  For turning left, we disable the left wheel (stop it) and run
  the right wheel forward.
*/
void turnLeft(uint8_t speed) {
  // Stop left motor
  analogWrite(LEFTM_EN_PIN, 0);
  digitalWrite(LEFTM_FOWARD_PIN, LOW);
  digitalWrite(LEFTM_BACK_PIN,   LOW);

  // Run right motor forward
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,  HIGH);
  analogWrite(RIGHTM_EN_PIN, speed);
}

/*
  For turning right, we disable the right wheel (stop it) and run
  the left wheel forward.
*/
void turnRight(uint8_t speed) {
  // Stop right motor
  analogWrite(RIGHTM_EN_PIN, 0);
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,  LOW);

  // Run left motor forward
  digitalWrite(LEFTM_FOWARD_PIN, LOW);
  digitalWrite(LEFTM_BACK_PIN,   HIGH);
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
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
}

int readLeftSensor() {
  int leftValue = digitalRead(LEFT_SENSOR);
  Serial.print("Left Sensor: ");
  Serial.println(leftValue);
  delay(10);
  return leftValue;
}

int readRightSensor() {
  int rightValue = digitalRead(RIGHT_SENSOR);
  Serial.print("Right Sensor: ");
  Serial.println(rightValue);
  delay(10);
  return rightValue;
}

//--------------------------------------------
// Main Function for Line Following
//--------------------------------------------
int main(){
  init();  // Initialize Arduino core functions
  Serial.begin(9600);
  delay(1000);
  
  motorPinSetup();
  setupSensors();
  
  while(1) {
    int leftVal  = readLeftSensor();
    int rightVal = readRightSensor();
    
    Serial.print("Left: ");
    Serial.print(leftVal);
    Serial.print(" | Right: ");
    Serial.println(rightVal);
    
    // Line-following logic:
    // If left sensor sees black (HIGH) and right sensor sees white (LOW):
    // → disable left wheel and turn left (i.e. run right wheel only)
    if (leftVal == HIGH && rightVal == LOW) {
      Serial.println("Action: Turn Left");
      turnLeft(DEFAULT_SPEED);
    }
    // If left sensor sees white (LOW) and right sensor sees black (HIGH):
    // → disable right wheel and turn right (i.e. run left wheel only)
    else if (leftVal == LOW && rightVal == HIGH) {
      Serial.println("Action: Turn Right");
      turnRight(DEFAULT_SPEED);
    }
    // If both sensors see the same (both black or both white), go forward.
    else {
      Serial.println("Action: Move Forward");
      moveForward(DEFAULT_SPEED);
    }
    
    delay(50);  // Short delay to allow sensor readings to stabilize
  }
  
  return 0;
}
