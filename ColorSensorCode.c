#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

// Pin Definitions
#define CS_S0_PIN PB0
#define CS_S1_PIN PB1
#define CS_S2_PIN PB2
#define CS_S3_PIN PB3
#define CS_OUT_PIN PD2  // Use PD2 for INT0
#define CS_LED_PIN PB5  // LED control pin

volatile uint16_t pulse_count = 0;

// Function Prototypes
void pinSetupCS();
void interruptSetupCS();
void colorSel(char color);
void enableSensorLED();
void disableSensorLED();
uint16_t measureColorFreq(char color);
uint16_t calibrateColor(uint16_t rawValue, char color);

int main() {
    pinSetupCS();
    interruptSetupCS();
    Serial.begin(9600);
    
    // Set frequency scaling to 20%
    PORTB |= (1 << CS_S0_PIN);    // S0 HIGH
    PORTB &= ~(1 << CS_S1_PIN);   // S1 LOW
    
    while(1) {
        enableSensorLED();
        _delay_ms(100);  // Increased stabilization time
        
        uint16_t red = measureColorFreq('R');
        uint16_t green = measureColorFreq('G');
        uint16_t blue = measureColorFreq('B');
        
        // Apply calibration
        red = calibrateColor(red, 'R');
        green = calibrateColor(green, 'G');
        blue = calibrateColor(blue, 'B');
        
        Serial.print("R: "); Serial.println(red);
        Serial.print("G: "); Serial.println(green);
        Serial.print("B: "); Serial.println(blue);
        
        // Improved color detection thresholds and logic
        if (red < 150 && green < 150 && blue < 150) {
            Serial.println("Detected: Black");
        } else if (green > red && green > blue) {
            Serial.println("Detected: Green");
        } else if (red > green && red > blue) {
            Serial.println("Detected: Red");
        } else if (blue > red && blue > green) {
            Serial.println("Detected: Blue");
        } else {
            Serial.println("Detected: Unknown");
        }
        
        Serial.println();
        disableSensorLED();
        _delay_ms(2);  // Reduced delay between readings
    }
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
