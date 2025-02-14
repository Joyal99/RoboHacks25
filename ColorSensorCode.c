#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

// Pin Definitions with Arduino Nano mappings
#define CS_S0_PIN PB0    // D8  on Nano
#define CS_S1_PIN PB1    // D9  on Nano
#define CS_S2_PIN PB2    // D10 on Nano
#define CS_S3_PIN PB3    // D11 on Nano
#define CS_OUT_PIN PD2   // D2  on Nano (INT0)
#define CS_LED_PIN PB5   // D13 on Nano (Built-in LED)

// Maximum expected raw value for normalization
#define MAX_RAW_VALUE 1500

volatile uint16_t pulse_count = 0;

// Function Prototypes
void pinSetupCS();
void interruptSetupCS();
void colorSel(char color);
void enableSensorLED();
void disableSensorLED();
uint16_t measureColorFreq(char color);
uint8_t normalizeColor(uint16_t rawValue, char color);

int main() {
    pinSetupCS();
    interruptSetupCS();
    Serial.begin(9600);
    
    // Set frequency scaling to 20%
    PORTB |= (1 << CS_S0_PIN);
    PORTB &= ~(1 << CS_S1_PIN);
    
    while(1) {
        enableSensorLED();
        _delay_ms(100);
        
        // Measure raw frequencies
        uint16_t rawRed = measureColorFreq('R');
        uint16_t rawGreen = measureColorFreq('G');
        uint16_t rawBlue = measureColorFreq('B');
        
        // Normalize to 0-255 range
        uint8_t red = normalizeColor(rawRed, 'R');
        uint8_t green = normalizeColor(rawGreen, 'G');
        uint8_t blue = normalizeColor(rawBlue, 'B');
        
        Serial.print("R: "); Serial.println(red);
        Serial.print("G: "); Serial.println(green);
        Serial.print("B: "); Serial.println(blue);
        
        // Color detection using normalized values
        if (red < 30 && green < 30 && blue < 30) {
            Serial.println("Detected: Black");
        } else if (red > 200 && green > 200 && blue > 200) {
            Serial.println("Detected: White");
        } else if (green > red * 1.2 && green > blue * 1.2) {
            Serial.println("Detected: Green");
        } else if (red > green * 1.2 && red > blue * 1.2) {
            Serial.println("Detected: Red");
        } else if (blue > red * 1.2 && blue > green * 1.2) {
            Serial.println("Detected: Blue");
        } else {
            Serial.println("Detected: Unknown");
        }
        
        Serial.println();
        disableSensorLED();
        _delay_ms(200);
    }
}

ISR(INT0_vect) {
    pulse_count++;
}

void pinSetupCS() {
    // Set S0-S3 and LED pin as outputs (D8-D11, D13)
    DDRB |= (1<<CS_S0_PIN) | (1<<CS_S1_PIN) | (1<<CS_S2_PIN) | (1<<CS_S3_PIN) | (1<<CS_LED_PIN);
    // Set OUT pin as input with pull-up (D2)
    DDRD &= ~(1<<CS_OUT_PIN);
    PORTD |= (1<<CS_OUT_PIN);
}

void interruptSetupCS() {
    EIMSK |= (1 << INT0);
    EICRA |= (1 << ISC00);
    sei();
}

void colorSel(char color) {
    switch(color) {
        case 'R':
            PORTB &= ~((1 << CS_S2_PIN) | (1 << CS_S3_PIN));
            break;
        case 'G':
            PORTB |= (1 << CS_S2_PIN);
            PORTB |= (1 << CS_S3_PIN);
            break;
        case 'B':
            PORTB &= ~(1 << CS_S2_PIN);
            PORTB |= (1 << CS_S3_PIN);
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
    _delay_ms(50);
    pulse_count = 0;
    _delay_ms(100);
    return pulse_count;
}

uint8_t normalizeColor(uint16_t rawValue, char color) {
    // Color-specific scaling factors (adjust these based on your sensor's response)
    float scalingFactors[] = {1.0, 1.1, 0.9};  // R, G, B
    uint16_t minValues[] = {30, 30, 25};       // Minimum values to consider
    
    uint8_t colorIndex;
    switch(color) {
        case 'R': colorIndex = 0; break;
        case 'G': colorIndex = 1; break;
        case 'B': colorIndex = 2; break;
        default: colorIndex = 0;
    }
    
    // Apply minimum threshold
    if (rawValue <= minValues[colorIndex]) return 0;
    rawValue -= minValues[colorIndex];
    
    // Apply scaling factor
    float scaledValue = rawValue * scalingFactors[colorIndex];
    
    // Normalize to 0-255 range
    uint8_t normalizedValue = (uint8_t)((scaledValue * 255.0) / MAX_RAW_VALUE);
    
    // Ensure we don't exceed 255
    return (normalizedValue > 255) ? 255 : normalizedValue;
}
