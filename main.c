#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#define RIGHTM_FOWARD_PIN PD4
#define RIGHTM_BACK_PIN PD5
#define LEFTM_FOWARD_PIN PD6
#define LEFTM_BACK_PIN PD7

#define CS_S0_PIN PB0
#define CS_S1_PIN PB1
#define CS_S2_PIN PB2
#define CS_S3_PIN PB3
#define CS_OUT_PIN PD2  
#define CS_LED_PIN PB5  

volatile uint16_t pulse_count = 0;  


void motorPinSetup() {
    DDRD |= (1 << RIGHTM_FOWARD_PIN) | (1 << LEFTM_FOWARD_PIN) | (1 << RIGHTM_BACK_PIN) | (1 << LEFTM_BACK_PIN);
}

void moveForward() {
    PORTD &= ~(1 << LEFTM_BACK_PIN);
    PORTD &= ~(1 << RIGHTM_BACK_PIN);
    PORTD |= (1 << LEFTM_FOWARD_PIN);
    PORTD |= (1 << RIGHTM_FOWARD_PIN);
}

void turnLeft() {
    PORTD |= (1 << RIGHTM_FOWARD_PIN);
    PORTD &= ~(1 << RIGHTM_BACK_PIN);
    PORTD &= ~(1 << LEFTM_FOWARD_PIN);
    PORTD |= (1 << LEFTM_BACK_PIN);
    _delay_ms(500); 
}

void turnRight() {
    PORTD |= (1 << LEFTM_FOWARD_PIN);
    PORTD &= ~(1 << LEFTM_BACK_PIN);
    PORTD &= ~(1 << RIGHTM_FOWARD_PIN);
    PORTD |= (1 << RIGHTM_BACK_PIN);
    _delay_ms(500); 
}

void stopMotors() {
    PORTD &= ~(1 << LEFTM_BACK_PIN);
    PORTD &= ~(1 << RIGHTM_BACK_PIN);
    PORTD &= ~(1 << LEFTM_FOWARD_PIN);
    PORTD &= ~(1 << RIGHTM_FOWARD_PIN);
}

void pinSetupCS() {
    DDRB |= (1 << CS_S0_PIN) | (1 << CS_S1_PIN) | (1 << CS_S2_PIN) | (1 << CS_S3_PIN) | (1 << CS_LED_PIN);
    DDRD &= ~(1 << CS_OUT_PIN);  
    PORTD |= (1 << CS_OUT_PIN);
}

void interruptSetupCS() {
    EIMSK |= (1 << INT0);     
    EICRA |= (1 << ISC00);    
    sei();                    
}

void enableSensorLED() {
    PORTB |= (1 << CS_LED_PIN);
}

void disableSensorLED() {
    PORTB &= ~(1 << CS_LED_PIN);
}

void colorSel(char color) {
    switch (color) {
        case 'R': PORTB &= ~((1 << CS_S2_PIN) | (1 << CS_S3_PIN)); break;  // Red
        case 'G': PORTB |= (1 << CS_S2_PIN); PORTB |= (1 << CS_S3_PIN); break;  // Green
        case 'B': PORTB &= ~(1 << CS_S2_PIN); PORTB |= (1 << CS_S3_PIN); break;  // Blue
    }
}

uint16_t measureColorFreq(char color) {
    pulse_count = 0;
    colorSel(color);
    _delay_ms(50);  
    pulse_count = 0;
    _delay_ms(100);  
    return pulse_count;
}

uint16_t calibrateColor(uint16_t rawValue, char color) {
    const uint16_t minValues[] = {30, 30, 25};  
    const float scalingFactors[] = {1.0, 1.1, 0.9};  
    uint8_t colorIndex;

    switch (color) {
        case 'R': colorIndex = 0; break;
        case 'G': colorIndex = 1; break;
        case 'B': colorIndex = 2; break;
        default: return rawValue;
    }

    if (rawValue <= minValues[colorIndex]) return 0;
    return (uint16_t)((rawValue - minValues[colorIndex]) * scalingFactors[colorIndex]);
}


void setup() {
    Serial.begin(9600);
    
    motorPinSetup();
    pinSetupCS();
    interruptSetupCS();

    Serial.println("System Initialized!");
    
    PORTB |= (1 << CS_S0_PIN);  // S0 HIGH
    PORTB &= ~(1 << CS_S1_PIN); // S1 LOW
}

void loop() {
    enableSensorLED();
    _delay_ms(100); // Stabilization time

    // Read color sensor values
    uint16_t red = measureColorFreq('R');
    uint16_t green = measureColorFreq('G');
    uint16_t blue = measureColorFreq('B');

    // Apply calibration
    red = calibrateColor(red, 'R');
    green = calibrateColor(green, 'G');
    blue = calibrateColor(blue, 'B');

    Serial.print("R: "); Serial.print(red);
    Serial.print(" G: "); Serial.print(green);
    Serial.print(" B: "); Serial.println(blue);

    if (red < 150 && green < 150 && blue < 150) {
        Serial.println("Detected: Black → Continue Moving Forward");
        moveForward();
    } 
    else if (green > red && green > blue) {
        Serial.println("Detected: Green → Turn Left");
        turnLeft();
    } 
    else if (red > green && red > blue) {
        Serial.println("Detected: Red → Stop");
        stop
