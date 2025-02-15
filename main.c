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
char getColor() {
    
        enableSensorLED();
        _delay_ms(5);  // Increased stabilization time
        
        uint16_t red = measureColorFreq('R');
        uint16_t green = measureColorFreq('G');
        uint16_t blue = measureColorFreq('B');
        
        // Apply calibration
        red = calibrateColor(red, 'R');
        green = calibrateColor(green, 'G');
        blue = calibrateColor(blue, 'B');
        
        char col;
        
        // Improved color detection thresholds and logic
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
        _delay_ms(2);  // Reduced delay between readings
        return col;
}

ISR(INT0_vect) {
    pulse_count++;
}

void pinSetupCS() {
    // Set S0-S3 and LED pin as outputs
    DDRC |= (1 << CS_S0_PIN) | (1 << CS_S1_PIN) | (1 << CS_S2_PIN) | (1 << CS_S3_PIN);
    // Set OUT pin as input with pull-up
    DDRD &= ~(1<<CS_OUT_PIN);
    PORTD |= (1<<CS_OUT_PIN);
}

void interruptSetupCS() {
    EIMSK |= (1 << INT0);     // Enable INT0
    EICRA |= (1 << ISC00);    // Trigger on any edge
    sei();                     // Enable global interrupts
}

void colorSel(char color) {
    switch(color) {
        case 'R':
            PORTB &= ~((1 << CS_S2_PIN) | (1 << CS_S3_PIN));  // S2=0, S3=0
            break;
        case 'G':
            PORTB |= (1 << CS_S2_PIN);     // S2=1
            PORTB |= (1 << CS_S3_PIN);     // S3=1
            break;
        case 'B':
            PORTB &= ~(1 << CS_S2_PIN);    // S2=0
            PORTB |= (1 << CS_S3_PIN);     // S3=1
            break;
    }
}

void enableSensorLED() {
    PORTB |= (1 << CS_LED_PIN);
}

void disableSensorLED() {
    PORTB &= ~(1 << CS_LED_PIN);
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
    // Adjusted calibration values based on typical sensor response
    const uint16_t minValues[] = {30, 30, 25};  // Minimum values for R, G, B
    const float scalingFactors[] = {1.0, 1.1, 0.9};  // Scaling factors for R, G, B
    
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
    
    /**if(col == 'G' && prevChar != 'G'){
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

    }**/


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
