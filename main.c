#include <Arduino.h>
#include <util/delay.h>

//-------------------- Motor Control Pins --------------------
#define LEFTM_FOWARD_PIN 4    
#define LEFTM_BACK_PIN   5    
#define RIGHTM_FOWARD_PIN 6   
#define RIGHTM_BACK_PIN  7    
#define RIGHTM_EN_PIN    3    
#define LEFTM_EN_PIN     11   

#define DEFAULT_SPEED 75      

//-------------------- IR Sensor Pins --------------------
#define RIGHT_SENSOR 12       
#define LEFT_SENSOR 8         

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
  digitalWrite(LEFTM_FOWARD_PIN, HIGH);
  digitalWrite(LEFTM_BACK_PIN,   LOW);
  digitalWrite(RIGHTM_FOWARD_PIN, HIGH);
  digitalWrite(RIGHTM_BACK_PIN,  LOW);
  analogWrite(LEFTM_EN_PIN, speed);
  analogWrite(RIGHTM_EN_PIN, speed + 10);
}

// Corrected turning logic: one wheel moves forward, the other moves backward.
void turnLeft(uint8_t speed) {
  digitalWrite(RIGHTM_FOWARD_PIN, HIGH);
  digitalWrite(RIGHTM_BACK_PIN,   LOW);
  digitalWrite(LEFTM_FOWARD_PIN, LOW);
  digitalWrite(LEFTM_BACK_PIN,   HIGH);
  analogWrite(LEFTM_EN_PIN, speed);
  analogWrite(RIGHTM_EN_PIN, speed + 10);
}

void turnRight(uint8_t speed) {
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,   HIGH);
  digitalWrite(LEFTM_FOWARD_PIN, HIGH);
  digitalWrite(LEFTM_BACK_PIN,   LOW);
  analogWrite(LEFTM_EN_PIN, speed);
  analogWrite(RIGHTM_EN_PIN, speed + 5);
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

// Function to read the left sensor value
int readLeftSensor() {
    int leftValue = digitalRead(LEFT_SENSOR);
    return leftValue;
}

// Function to read the right sensor value
int readRightSensor() {
    int rightValue = digitalRead(RIGHT_SENSOR);
    return rightValue;
}

//--------------------------------------------
// Main Function for Line Following
//--------------------------------------------
int main(){
  init();  
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
    
    if (leftVal == LOW && rightVal == HIGH) {
      Serial.println("Action: Turn Right");
      turnRight(DEFAULT_SPEED);
    } 
    else if (leftVal == HIGH && rightVal == LOW) {
      Serial.println("Action: Turn Left");
      turnLeft(DEFAULT_SPEED);
    } 
    else if (leftVal == LOW && rightVal == LOW) {
      Serial.println("Action: Move Forward");
      moveForward(DEFAULT_SPEED);
    } 
    else if (leftVal == HIGH && rightVal == HIGH) {
      Serial.println("Action: Searching for line");
      turnLeft(DEFAULT_SPEED / 2);
    }
    
    delay(5);
  }
  
  return 0;
}
