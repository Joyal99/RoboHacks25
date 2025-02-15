#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <Servo.h>

// Motor Direction Pins (kept the same)
#define RIGHTM_FOWARD_PIN 4   // originally PD4
#define RIGHTM_BACK_PIN   5   // originally PD5
#define LEFTM_FOWARD_PIN  6   // originally PD6
#define LEFTM_BACK_PIN    7   // originally PD7

// Motor Enable Pins (PWM) for L298N
#define LEFTM_EN_PIN   10   // PWM-capable pin for left motor
#define RIGHTM_EN_PIN  11   // PWM-capable pin for right motor

// Other pin definitions remain the same
#define CS_S0_PIN PB0
#define CS_S1_PIN PB1
#define CS_S2_PIN PB2
#define CS_S3_PIN PB3
#define CS_OUT_PIN PD2  
#define CS_LED_PIN PB5
#define LEFT_SENSOR 12
#define RIGHT_SENSOR 3  

#define LEFT_MAX 2000    // 2ms pulse (full left)
#define RIGHT_MAX 1000   // 1ms pulse (full right)

#define DEFAULT_SPEED 150   // Default PWM speed (0-255)

Servo myServo;           // Create servo object
const int servoPin = 9;  // PWM pin connected to servo

volatile uint16_t pulse_count = 0;  

//------------------ Motor Control Functions ----------------------

// Initializes motor direction and enable pins
void motorPinSetup(){
    // Set motor direction pins as outputs
    pinMode(RIGHTM_FOWARD_PIN, OUTPUT);
    pinMode(RIGHTM_BACK_PIN,   OUTPUT);
    pinMode(LEFTM_FOWARD_PIN,  OUTPUT);
    pinMode(LEFTM_BACK_PIN,    OUTPUT);
    // Set motor enable pins as outputs
    pinMode(LEFTM_EN_PIN, OUTPUT);
    pinMode(RIGHTM_EN_PIN, OUTPUT);
}

// Moves both motors forward at the specified speed
void moveFoward(uint8_t speed) {
    // Set directions: both motors forward
    digitalWrite(LEFTM_FOWARD_PIN, HIGH);
    digitalWrite(LEFTM_BACK_PIN,   LOW);
    digitalWrite(RIGHTM_FOWARD_PIN, HIGH);
    digitalWrite(RIGHTM_BACK_PIN,  LOW);
    // Set motor speeds using PWM on the enable pins
    analogWrite(LEFTM_EN_PIN, speed);
    analogWrite(RIGHTM_EN_PIN, speed);
}

// Moves both motors backward at the specified speed
void moveBackward(uint8_t speed) {
    // Set directions: both motors backward
    digitalWrite(LEFTM_FOWARD_PIN, LOW);
    digitalWrite(LEFTM_BACK_PIN,   HIGH);
    digitalWrite(RIGHTM_FOWARD_PIN, LOW);
    digitalWrite(RIGHTM_BACK_PIN,  HIGH);
    analogWrite(LEFTM_EN_PIN, speed);
    analogWrite(RIGHTM_EN_PIN, speed);
}

// Pivots left: right motor moves forward, left motor moves backward
void turnLeft(uint8_t speed) {
    digitalWrite(RIGHTM_FOWARD_PIN, HIGH);
    digitalWrite(RIGHTM_BACK_PIN,   LOW);
    analogWrite(RIGHTM_EN_PIN, speed);
    digitalWrite(LEFTM_FOWARD_PIN, LOW);
    digitalWrite(LEFTM_BACK_PIN,   HIGH);
    analogWrite(LEFTM_EN_PIN, speed);
}

// Pivots right: left motor moves forward, right motor moves backward
void turnRight(uint8_t speed) {
    digitalWrite(LEFTM_FOWARD_PIN, HIGH);
    digitalWrite(LEFTM_BACK_PIN,   LOW);
    analogWrite(LEFTM_EN_PIN, speed);
    digitalWrite(RIGHTM_FOWARD_PIN, LOW);
    digitalWrite(RIGHTM_BACK_PIN,   HIGH);
    analogWrite(RIGHTM_EN_PIN, speed);
}

// Stops both motors immediately
void stop() {
    // Cut motor enable signals to zero speed
    analogWrite(LEFTM_EN_PIN,  0);
    analogWrite(RIGHTM_EN_PIN, 0);
    // Optionally, clear the direction pins
    digitalWrite(LEFTM_FOWARD_PIN, LOW);
    digitalWrite(LEFTM_BACK_PIN,   LOW);
    digitalWrite(RIGHTM_FOWARD_PIN,LOW);
    digitalWrite(RIGHTM_BACK_PIN,  LOW);
    _delay_ms(8000);
}
//-----------------------------------------------------------------

//------------------ COLOR SENSOR FUNCTIONS -----------------------
char getColor() {  
    enableSensorLED();
    _delay_ms(50);  // Increased stabilization time
        
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
    DDRB |= (1<<CS_S0_PIN) | (1<<CS_S1_PIN) | (1<<CS_S2_PIN) | (1<<CS_S3_PIN) | (1<<CS_LED_PIN);
    // Set OUT pin as input with pull-up
    DDRD &= ~(1<<CS_OUT_PIN);
    PORTD |= (1<<CS_OUT_PIN);
    // Set frequency scaling to 20%
    PORTB |= (1 << CS_S0_PIN);    // S0 HIGH
    PORTB &= ~(1 << CS_S1_PIN);   // S1 LOW
}

void interruptSetupCS() {
    EIMSK |= (1 << INT0);     // Enable INT0
    EICRA |= (1 << ISC00);    // Trigger on any edge
    sei();                    // Enable global interrupts
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
//-----------------------------------------------------------------

//------------------ SEED DROP FUNCTIONS --------------------------
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
//-----------------------------------------------------------------

//------------------ IR SENSOR FUNCTIONS --------------------------
void setupSensors() {
    Serial.begin(9600);
    pinMode(LEFT_SENSOR, INPUT);
    pinMode(RIGHT_SENSOR, INPUT);
}

// Function to read the left sensor value
int readLeftSensor() {
    int leftValue = digitalRead(LEFT_SENSOR);
    Serial.println(leftValue);
    delay(10);
    return leftValue;
}

// Function to read the right sensor value
int readRightSensor() {
    int rightValue = digitalRead(RIGHT_SENSOR);
    Serial.println(rightValue);
    delay(10);
    return rightValue;
}
//-----------------------------------------------------------------

//------------------ MAIN FUNCTION -------------------------------
int main(){
    uart_init();
    pinSetupCS();
    interruptSetupCS();
    motorPinSetup();
    setupSensors();
    myServo.attach(servoPin); 

    while(1){
        // Move forward at the default speed
        moveFoward(DEFAULT_SPEED);

        int leftSensor = readLeftSensor();
        int rightSensor = readRightSensor();

        // Adjust course based on sensor readings
        while(leftSensor == 0){
            turnRight(DEFAULT_SPEED);
            leftSensor = readLeftSensor();
        }
        while(rightSensor == 0){
            turnLeft(DEFAULT_SPEED);
            rightSensor = readRightSensor();
        }
        // Check color and act accordingly
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
