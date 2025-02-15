#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <Servo.h>

//-------------------- Motor Control Pins --------------------
// Direction pins (unchanged)
#define RIGHTM_FOWARD_PIN 4   // PD4
#define RIGHTM_BACK_PIN   5   // PD5
#define LEFTM_FOWARD_PIN  6   // PD6
#define LEFTM_BACK_PIN    7   // PD7
// Enable (PWM) pins for L298N:
// Right motor remains on digital pin 11 (Timer2)
// Left motor enable is moved from pin 10 (Timer1) to digital pin 3 (Timer2)
#define LEFTM_EN_PIN   3    // Now on Timer2 PWM channel
#define RIGHTM_EN_PIN  11   // Timer2 PWM channel

//-------------------- Color Sensor (CS) Pins --------------------
// CS_OUT_PIN remains unchanged (digital 2)
#define CS_OUT_PIN 2         // PD2, unchanged
// Other CS sensor pins are moved to analog inputs
#define CS_S0_PIN A0         // formerly PB0
#define CS_S1_PIN A1         // formerly PB1
#define CS_S2_PIN A2         // formerly PB2
#define CS_S3_PIN A3         // formerly PB3
#define CS_LED_PIN A4        // formerly PB5

//-------------------- IR Sensor Pins --------------------
// Left sensor remains the same; reassign right sensor from digital 3 to digital 8.
#define LEFT_SENSOR 12       // digital 12
#define RIGHT_SENSOR 8       // digital 8 (was 3, to free up Timer2 channel on pin 3)

//-------------------- Servo & Other Definitions --------------------
#define LEFT_MAX 2000    // 2ms pulse (full left)
#define RIGHT_MAX 1000   // 1ms pulse (full right)
#define DEFAULT_SPEED 150  // PWM speed (0-255)

Servo myServo;           // Create servo object
const int servoPin = 9;  // Servo connected to digital 9 (uses Timer1)

volatile uint16_t pulse_count = 0;  

//================================================================
// MOTOR CONTROL FUNCTIONS (using Arduino functions & PWM enable)
//================================================================

// Initialize motor direction and enable pins.
void motorPinSetup(){
  pinMode(RIGHTM_FOWARD_PIN, OUTPUT);
  pinMode(RIGHTM_BACK_PIN,   OUTPUT);
  pinMode(LEFTM_FOWARD_PIN,  OUTPUT);
  pinMode(LEFTM_BACK_PIN,    OUTPUT);
  pinMode(LEFTM_EN_PIN, OUTPUT);
  pinMode(RIGHTM_EN_PIN, OUTPUT);
}

// Moves both motors forward at the specified speed.
void moveFoward(uint8_t speed) {
  digitalWrite(LEFTM_FOWARD_PIN, HIGH);
  digitalWrite(LEFTM_BACK_PIN,   LOW);
  digitalWrite(RIGHTM_FOWARD_PIN, HIGH);
  digitalWrite(RIGHTM_BACK_PIN,  LOW);
  analogWrite(LEFTM_EN_PIN,  speed);
  analogWrite(RIGHTM_EN_PIN, speed);
}

// Moves both motors backward at the specified speed.
void moveBackward(uint8_t speed) {
  digitalWrite(LEFTM_FOWARD_PIN, LOW);
  digitalWrite(LEFTM_BACK_PIN,   HIGH);
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,  HIGH);
  analogWrite(LEFTM_EN_PIN,  speed);
  analogWrite(RIGHTM_EN_PIN, speed);
}

// Pivots left: right motor moves forward, left motor moves backward.
void turnLeft(uint8_t speed) {
  digitalWrite(RIGHTM_FOWARD_PIN, HIGH);
  digitalWrite(RIGHTM_BACK_PIN,   LOW);
  analogWrite(RIGHTM_EN_PIN, speed);
  digitalWrite(LEFTM_FOWARD_PIN, LOW);
  digitalWrite(LEFTM_BACK_PIN,   HIGH);
  analogWrite(LEFTM_EN_PIN, speed);
}

// Pivots right: left motor moves forward, right motor moves backward.
void turnRight(uint8_t speed) {
  digitalWrite(LEFTM_FOWARD_PIN, HIGH);
  digitalWrite(LEFTM_BACK_PIN,   LOW);
  analogWrite(LEFTM_EN_PIN, speed);
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,   HIGH);
  analogWrite(RIGHTM_EN_PIN, speed);
}

// Stops both motors.
void stop() {
  analogWrite(LEFTM_EN_PIN,  0);
  analogWrite(RIGHTM_EN_PIN, 0);
  digitalWrite(LEFTM_FOWARD_PIN, LOW);
  digitalWrite(LEFTM_BACK_PIN,   LOW);
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,  LOW);
  _delay_ms(8000);
}

//================================================================
// COLOR SENSOR FUNCTIONS (using Arduino functions)
//================================================================

void pinSetupCS() {
  pinMode(CS_S0_PIN, OUTPUT);
  pinMode(CS_S1_PIN, OUTPUT);
  pinMode(CS_S2_PIN, OUTPUT);
  pinMode(CS_S3_PIN, OUTPUT);
  pinMode(CS_LED_PIN, OUTPUT);
  pinMode(CS_OUT_PIN, INPUT_PULLUP);  // Keep CS_OUT_PIN unchanged.
  // Set frequency scaling to 20%
  digitalWrite(CS_S0_PIN, HIGH);    
  digitalWrite(CS_S1_PIN, LOW);       
}

void interruptSetupCS() {
  EIMSK |= (1 << INT0);     // Enable INT0
  EICRA |= (1 << ISC00);    // Trigger on any edge
  sei();                    // Enable global interrupts
}

void colorSel(char color) {
  switch(color) {
    case 'R':
      digitalWrite(CS_S2_PIN, LOW);
      digitalWrite(CS_S3_PIN, LOW);
      break;
    case 'G':
      digitalWrite(CS_S2_PIN, HIGH);
      digitalWrite(CS_S3_PIN, HIGH);
      break;
    case 'B':
      digitalWrite(CS_S2_PIN, LOW);
      digitalWrite(CS_S3_PIN, HIGH);
      break;
  }
}

void enableSensorLED() {
  digitalWrite(CS_LED_PIN, HIGH);
}

void disableSensorLED() {
  digitalWrite(CS_LED_PIN, LOW);
}

uint16_t measureColorFreq(char color) {
  pulse_count = 0;
  colorSel(color);
  _delay_ms(50);  // Allow filter to settle
  pulse_count = 0;  // Reset after settling
  _delay_ms(100);  // Measure for 100ms
  return pulse_count;
}

uint16_t calibrateColor(uint16_t rawValue, char color) {
  const uint16_t minValues[] = {30, 30, 25};
  const float scalingFactors[] = {1.0, 1.1, 0.9};
  uint8_t colorIndex;
  switch(color) {
    case 'R': colorIndex = 0; break;
    case 'G': colorIndex = 1; break;
    case 'B': colorIndex = 2; break;
    default: return rawValue;
  }
  if (rawValue <= minValues[colorIndex]) return 0;
  return (uint16_t)((rawValue - minValues[colorIndex]) * scalingFactors[colorIndex]);
}

char getColor() {
  enableSensorLED();
  _delay_ms(50);  // Stabilization
  uint16_t red   = calibrateColor(measureColorFreq('R'), 'R');
  uint16_t green = calibrateColor(measureColorFreq('G'), 'G');
  uint16_t blue  = calibrateColor(measureColorFreq('B'), 'B');
  char col;
  if (red < 150 && green < 150 && blue < 150) {
    col = 'B';
  } else if (green > red && green > blue) {
    col = 'G';
  } else if (red > green && red > blue) {
    col = 'R';
  } else if (blue > red && blue > green) {
    col = 'B';
  } else {
    col = 'U';
  }
  disableSensorLED();
  _delay_ms(2);
  return col;
}

ISR(INT0_vect) {
  pulse_count++;
}

//================================================================
// SEED DROP FUNCTIONS
//================================================================
void dropSeed(){
  for(int i = 0; i < 2; i++){
    myServo.writeMicroseconds(LEFT_MAX);
    _delay_ms(3);
    myServo.writeMicroseconds(RIGHT_MAX);
    _delay_ms(10);
    myServo.writeMicroseconds(LEFT_MAX);
    Serial.println("Servo centered.");
    _delay_ms(3);
  }
}

//================================================================
// IR SENSOR FUNCTIONS
//================================================================
void setupSensors() {
  Serial.begin(9600);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
}

int readLeftSensor() {
  int leftValue = digitalRead(LEFT_SENSOR);
  Serial.println(leftValue);
  delay(10);
  return leftValue;
}

int readRightSensor() {
  int rightValue = digitalRead(RIGHT_SENSOR);
  Serial.println(rightValue);
  delay(10);
  return rightValue;
}

//================================================================
// MAIN FUNCTION
//================================================================
int main(){
  init();  // Initialize Arduino core functions
  
  uart_init();
  pinSetupCS();
  interruptSetupCS();
  motorPinSetup();
  setupSensors();
  myServo.attach(servoPin); 

  while(1){
    // Drive forward at default speed.
    moveFoward(DEFAULT_SPEED);

    int leftSensor = readLeftSensor();
    int rightSensor = readRightSensor();

    // For debugging, print a message after reading right sensor.
    Serial.println("hello");

    // Adjust course based on sensor input.
    while(leftSensor == 0){
      turnRight(DEFAULT_SPEED);
      leftSensor = readLeftSensor();
    }
    while(rightSensor == 0){
      turnLeft(DEFAULT_SPEED);
      rightSensor = readRightSensor();
    }

    // Check color sensor and act.
    char colorSeen = getColor();
    if(colorSeen == 'G') {
      dropSeed();
    }
    else if(colorSeen == 'B'){
      stop();
    }
    _delay_us(10);
  }
}
