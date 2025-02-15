#include <Arduino.h>
#include <util/delay.h>

//-------------------- Motor Control Pins --------------------
// Direction pins
#define RIGHTM_FOWARD_PIN 4   // PD4
#define RIGHTM_BACK_PIN   5   // PD5
#define LEFTM_FOWARD_PIN  6   // PD6
#define LEFTM_BACK_PIN    7   // PD7
// Enable (PWM) pins for L298N
#define LEFTM_EN_PIN   3    // Left motor enable (PWM)
#define RIGHTM_EN_PIN  11   // Right motor enable (PWM)

#define DEFAULT_SPEED 150   // PWM speed (0-255)

//-------------------- Sensor Pins --------------------
#define LEFT_SENSOR 12      // Left sensor pin
#define RIGHT_SENSOR 8      // Right sensor pin

//-------------------- Motor Functions --------------------
void motorPinSetup(){
  pinMode(RIGHTM_FOWARD_PIN, OUTPUT);
  pinMode(RIGHTM_BACK_PIN,   OUTPUT);
  pinMode(LEFTM_FOWARD_PIN,  OUTPUT);
  pinMode(LEFTM_BACK_PIN,    OUTPUT);
  pinMode(LEFTM_EN_PIN, OUTPUT);
  pinMode(RIGHTM_EN_PIN, OUTPUT);
}

/* 
   Given that your motors were turning in the opposite direction, we have 
   inverted the logic here:
   - "Forward" now sets the forward pins LOW and the backward pins HIGH.
   - "Backward" (if used) would be the opposite.
*/
void moveFoward(uint8_t speed) {
  // Both motors move "forward" (i.e. the robot moves ahead)
  digitalWrite(LEFTM_FOWARD_PIN, LOW);
  digitalWrite(LEFTM_BACK_PIN,   HIGH);
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,  HIGH);
  analogWrite(LEFTM_EN_PIN, speed);
  analogWrite(RIGHTM_EN_PIN, speed);
}

/*
   To pivot left, we want the left motor to run in reverse (forward pins HIGH)
   and the right motor to keep moving forward.
*/
void turnLeft(uint8_t speed) {
  // Left motor runs in reverse; right motor runs forward.
  digitalWrite(LEFTM_FOWARD_PIN, HIGH);
  digitalWrite(LEFTM_BACK_PIN,   LOW);
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,  HIGH);
  analogWrite(LEFTM_EN_PIN, speed);
  analogWrite(RIGHTM_EN_PIN, speed);
}

/*
   To pivot right, we want the right motor to run in reverse and the left motor 
   to keep moving forward.
*/
void turnRight(uint8_t speed) {
  // Right motor runs in reverse; left motor runs forward.
  digitalWrite(LEFTM_FOWARD_PIN, LOW);
  digitalWrite(LEFTM_BACK_PIN,   HIGH);
  digitalWrite(RIGHTM_FOWARD_PIN, HIGH);
  digitalWrite(RIGHTM_BACK_PIN,  LOW);
  analogWrite(LEFTM_EN_PIN, speed);
  analogWrite(RIGHTM_EN_PIN, speed);
}

void stopMotors() {
  analogWrite(LEFTM_EN_PIN, 0);
  analogWrite(RIGHTM_EN_PIN, 0);
  digitalWrite(LEFTM_FOWARD_PIN, LOW);
  digitalWrite(LEFTM_BACK_PIN,   LOW);
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,  LOW);
}

//-------------------- Main Function --------------------
int main(){
  init();  // Initialize the Arduino core
  
  Serial.begin(9600);
  delay(1000);
  
  motorPinSetup();
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  
  while(1){
    // Read sensor values (assume HIGH = black, LOW = white)
    int leftVal = digitalRead(LEFT_SENSOR);
    int rightVal = digitalRead(RIGHT_SENSOR);
    
    Serial.print("Left Sensor: ");
    Serial.print(leftVal);
    Serial.print(" | Right Sensor: ");
    Serial.println(rightVal);
    
    // Logic for line following:
    // When right sensor sees white (0) and left sees black (1): turn left.
    // When left sensor sees white (0) and right sees black (1): turn right.
    // Otherwise, go straight.
    if(leftVal == HIGH && rightVal == LOW) {
      Serial.println("Turning Left");
      turnLeft(DEFAULT_SPEED);
    }
    else if(leftVal == LOW && rightVal == HIGH) {
      Serial.println("Turning Right");
      turnRight(DEFAULT_SPEED);
    }
    else {
      Serial.println("Moving Forward");
      moveFoward(DEFAULT_SPEED);
    }
    
    delay(50); // Short delay for sensor update
  }
  
  return 0;
}
