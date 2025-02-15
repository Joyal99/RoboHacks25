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

// In our current configuration, the correct "forward" movement 
// (based on your wiring) is achieved by setting the forward pins LOW 
// and the backward pins HIGH.
void moveFoward(uint8_t speed) {
  digitalWrite(LEFTM_FOWARD_PIN, LOW);
  digitalWrite(LEFTM_BACK_PIN,   HIGH);
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,  HIGH);
  analogWrite(LEFTM_EN_PIN, speed);
  analogWrite(RIGHTM_EN_PIN, speed);
}

// To turn left, we make the left motor go in reverse (forward pin HIGH) 
// and keep the right motor going forward.
void turnLeft(uint8_t speed) {
  digitalWrite(LEFTM_FOWARD_PIN, HIGH);
  digitalWrite(LEFTM_BACK_PIN,   LOW);
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,  HIGH);
  analogWrite(LEFTM_EN_PIN, speed);
  analogWrite(RIGHTM_EN_PIN, speed);
}

// To turn right, we make the right motor go in reverse (forward pin HIGH) 
// and keep the left motor going forward.
void turnRight(uint8_t speed) {
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
  init();  // Initialize Arduino core functions
  Serial.begin(9600);
  delay(1000);
  
  motorPinSetup();
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  
  while(1){
    // Read sensor values (assuming HIGH = black, LOW = white)
    int leftVal = digitalRead(LEFT_SENSOR);
    int rightVal = digitalRead(RIGHT_SENSOR);
    
    // Debug print of sensor states.
    Serial.print("Left Sensor: ");
    Serial.print(leftVal);
    Serial.print(" | Right Sensor: ");
    Serial.println(rightVal);
    
    // Explicit handling of all sensor combinations:
    if(leftVal == HIGH && rightVal == LOW) {
      // Left sees black, right sees white: robot is off to the right,
      // so turn left.
      Serial.println("Action: Turning Left");
      turnLeft(DEFAULT_SPEED);
    }
    else if(leftVal == LOW && rightVal == HIGH) {
      // Left sees white, right sees black: robot is off to the left,
      // so turn right.
      Serial.println("Action: Turning Right");
      turnRight(DEFAULT_SPEED);
    }
    else if(leftVal == HIGH && rightVal == HIGH) {
      // Both sensors see black: on track, move forward.
      Serial.println("Action: Both black, moving forward");
      moveFoward(DEFAULT_SPEED);
    }
    else if(leftVal == LOW && rightVal == LOW) {
      // Both sensors see white: line lost. You can choose to stop or
      // keep moving forward. Here we choose to move forward.
      Serial.println("Action: Both white, moving forward");
      moveFoward(DEFAULT_SPEED);
    }
    
    delay(50); // Short delay for sensor updates.
  }
  
  return 0;
}
