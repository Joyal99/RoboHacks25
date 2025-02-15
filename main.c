#include <Arduino.h>
#include <util/delay.h>

//-------------------- Motor Control Pins --------------------
// (These definitions are swapped relative to the original wiring.)
#define LEFTM_FOWARD_PIN 4    // PD4 (Swapped with RIGHTM)
#define LEFTM_BACK_PIN   5    // PD5 (Swapped with RIGHTM)
#define RIGHTM_FOWARD_PIN 6   // PD6 (Swapped with LEFTM)
#define RIGHTM_BACK_PIN  7    // PD7 (Swapped with LEFTM)
// Enable (PWM) pins for L298N
#define RIGHTM_EN_PIN    3    // PWM for right motor (Swapped with LEFTM)
#define LEFTM_EN_PIN     11   // PWM for left motor (Swapped with RIGHTM)

#define DEFAULT_SPEED 75      // PWM speed (0-255)

//-------------------- IR Sensor Pins --------------------
// (Sensors are also swapped relative to original.)
#define RIGHT_SENSOR 12       // Right sensor pin (Swapped with LEFT_SENSOR)
#define LEFT_SENSOR 8         // Left sensor pin (Swapped with RIGHT_SENSOR)

//--------------------------------------------
// Motor Functions
//--------------------------------------------
void motorPinSetup() {
  pinMode(LEFTM_FOWARD_PIN, OUTPUT);
  pinMode(LEFTM_BACK_PIN,   OUTPUT);
  pinMode(RIGHTM_FOWARD_PIN, OUTPUT);
  pinMode(RIGHTM_BACK_PIN,  OUTPUT);
  pinMode(RIGHTM_EN_PIN,    OUTPUT);
  pinMode(LEFTM_EN_PIN,     OUTPUT);
}

void moveForward(uint8_t speed) {
  // Set both motors for forward motion.
  digitalWrite(LEFTM_FOWARD_PIN, HIGH);
  digitalWrite(LEFTM_BACK_PIN,   LOW);
  digitalWrite(RIGHTM_FOWARD_PIN, HIGH);
  digitalWrite(RIGHTM_BACK_PIN,  LOW);
  analogWrite(LEFTM_EN_PIN, speed);
  analogWrite(RIGHTM_EN_PIN, speed);
}

// For turning, we disable one wheel and run the other forward.
void turnLeft(uint8_t speed) {
  // Disable right motor.
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,   LOW);
  analogWrite(RIGHTM_EN_PIN, 0);
  
  // Run left motor forward.
  digitalWrite(LEFTM_FOWARD_PIN, HIGH);
  digitalWrite(LEFTM_BACK_PIN,   LOW);
  analogWrite(LEFTM_EN_PIN, speed);
}

void turnRight(uint8_t speed) {
  // Disable left motor.
  digitalWrite(LEFTM_FOWARD_PIN, LOW);
  digitalWrite(LEFTM_BACK_PIN,   LOW);
  analogWrite(LEFTM_EN_PIN, 0);
  
  // Run right motor forward.
  digitalWrite(RIGHTM_FOWARD_PIN, HIGH);
  digitalWrite(RIGHTM_BACK_PIN,  LOW);
  analogWrite(RIGHTM_EN_PIN, speed);
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
  delay(1000);
  
  motorPinSetup();
  setupSensors();
  
  while(1) {
    // Read sensor values.
    int leftVal  = readLeftSensor();
    int rightVal = readRightSensor();
    
    Serial.print("Left: ");
    Serial.print(leftVal);
    Serial.print(" | Right: ");
    Serial.println(rightVal);
    
    // Upgraded line tracking logic:
    // Assuming sensor HIGH indicates a dark surface (line) and LOW indicates a light surface.
    if (leftVal == HIGH && rightVal == LOW) {
      // Left sensor sees the dark line, right sensor sees light → pivot left.
      Serial.println("Action: Turn Left");
      turnLeft(DEFAULT_SPEED);
    } 
    else if (leftVal == LOW && rightVal == HIGH) {
      // Right sensor sees the dark line, left sensor sees light → pivot right.
      Serial.println("Action: Turn Right");
      turnRight(DEFAULT_SPEED);
    } 
    else if (leftVal == LOW && rightVal == LOW) {
      // Both sensors see dark → line is centered; move forward.
      Serial.println("Action: Move Forward");
      moveForward(DEFAULT_SPEED);
    } 
    else if (leftVal == HIGH && rightVal == HIGH) {
      // Both sensors see light → line lost; perform a search maneuver.
      Serial.println("Action: Searching for line");
      // For example, slowly pivot right to try to reacquire the line.
      turnRight(DEFAULT_SPEED/2);
      delay(100);
    }
    
    delay(50);  // Short delay for sensor updates.
  }
  
  return 0;
}
