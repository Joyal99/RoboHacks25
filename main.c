#include <Arduino.h>
#include <util/delay.h>
#include <avr/io.h>

//-------------------- Motor Control Pins --------------------
#define LEFTM_FOWARD_PIN 4    
#define LEFTM_BACK_PIN   5    
#define RIGHTM_FOWARD_PIN 6   
#define RIGHTM_BACK_PIN  7    
#define RIGHTM_EN_PIN    3    
#define LEFTM_EN_PIN     11   

#define DEFAULT_SPEED 100      

//-------------------- IR Sensor Pins --------------------
#define RIGHT_SENSOR 12       
#define LEFT_SENSOR 8   

//--------------------- CColor Sensor Pins ---------------
volatile uint16_t pulse_count = 0;
#define CS_S0_PIN PC0  // A0
#define CS_S1_PIN PC1  // A1
#define CS_S2_PIN PC2  // A2
#define CS_S3_PIN PC3  // A3
#define CS_OUT_PIN PD2  // Use PD2 for INT0
#define CS_LED_PIN PB5  // LED control pin
volatile char prevChar;
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
//-------------------------------------------
// Color Sensor
//-------------------------------------------
// ISR with correct edge triggering
ISR(INT0_vect) {
    pulse_count++;
}

// Optimized measureColorFreq
uint16_t measureColorFreq(char color) {
    colorSel(color);
    _delay_ms(5);   // Settling time
    pulse_count = 0;
    _delay_ms(20);  // Measurement time
    return pulse_count;
}

// Revised getColor() with thresholds
char getColor() {
    enableSensorLED();
    _delay_ms(2);  // Reduced stabilization

    uint16_t red = measureColorFreq('R');
    uint16_t green = measureColorFreq('G');
    uint16_t blue = measureColorFreq('B');

    red = calibrateColor(red, 'R');
    green = calibrateColor(green, 'G');
    blue = calibrateColor(blue, 'B');

    char col = 'U';
    
    if (red < 75 && green < 75 && blue < 75) {
        col = 'B';
    } else if (green > red * 1.2 && green > blue * 1.2) {
        col = 'G';
    } else if (red > green * 1.2 && red > blue * 1.2) {
        col = 'R';
    } else if (blue > red * 1.2 && blue > green * 1.2) {
        col = 'B';
    }
    
    disableSensorLED();
    _delay_ms(1);
    return col;
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
    char col = getColor();
    
    if(col == 'G' && prevChar != 'G'){
      stopMotors();
      _delay_ms(1000);
      prevChar = 'G';
    }
    else if(col == 'B' && prevChar != 'B'){
      stopMotors();
      _delay_ms(8000);
      prevChar = 'B';
    }
    else{

    }


    if (leftVal == LOW && rightVal == HIGH) {
      turnRight(DEFAULT_SPEED);
    } 
    else if (leftVal == HIGH && rightVal == LOW) {
      turnLeft(DEFAULT_SPEED);
    } 
    else if (leftVal == LOW && rightVal == LOW) {
      moveForward(DEFAULT_SPEED);
    } 
    else if (leftVal == HIGH && rightVal == HIGH) {
      turnLeft(DEFAULT_SPEED / 2);
    }
    delay(5);
  }
  
  return 0;
}
