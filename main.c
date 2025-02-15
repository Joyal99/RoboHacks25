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

// UART functions
void uart_init() {
    UBRR0H = 0;  // Set baud rate to 9600 (16MHz clock)
    UBRR0L = 103;
    UCSR0B = (1 << TXEN0);  // Enable transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8-bit data format
}

void uart_print(const char* str) {
    while (*str) {
        while (!(UCSR0A & (1 << UDRE0)));  // Wait for empty buffer
        UDR0 = *str++;
    }
}

void motorPinSetup() {
    DDRD |= (1 << RIGHTM_FOWARD_PIN) | (1 << LEFTM_FOWARD_PIN) 
            | (1 << RIGHTM_BACK_PIN) | (1 << LEFTM_BACK_PIN);
}

void moveForward() {
    PORTD &= ~((1 << LEFTM_BACK_PIN) | (1 << RIGHTM_BACK_PIN));
    PORTD |= (1 << LEFTM_FOWARD_PIN) | (1 << RIGHTM_FOWARD_PIN);
}

void turnLeft() {
    PORTD |= (1 << RIGHTM_FOWARD_PIN) | (1 << LEFTM_BACK_PIN);
    PORTD &= ~((1 << RIGHTM_BACK_PIN) | (1 << LEFTM_FOWARD_PIN));
    _delay_ms(500); 
}

void turnRight() {
    PORTD |= (1 << LEFTM_FOWARD_PIN) | (1 << RIGHTM_BACK_PIN);
    PORTD &= ~((1 << LEFTM_BACK_PIN) | (1 << RIGHTM_FOWARD_PIN));
    _delay_ms(500); 
}

void stopMotors() {
    PORTD &= ~((1 << LEFTM_BACK_PIN) | (1 << RIGHTM_BACK_PIN) 
            | (1 << LEFTM_FOWARD_PIN) | (1 << RIGHTM_FOWARD_PIN));
}

void pinSetupCS() {
    DDRB |= (1 << CS_S0_PIN) | (1 << CS_S1_PIN) 
            | (1 << CS_S2_PIN) | (1 << CS_S3_PIN) | (1 << CS_LED_PIN);
    DDRD &= ~(1 << CS_OUT_PIN);  
    PORTD |= (1 << CS_OUT_PIN);
}

ISR(INT0_vect) {
    pulse_count++;
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
        case 'R': // Red
            PORTB &= ~((1 << CS_S2_PIN) | (1 << CS_S3_PIN));
            break;
        case 'G': // Green
            PORTB |= (1 << CS_S2_PIN) | (1 << CS_S3_PIN);
            break;
        case 'B': // Blue
            PORTB &= ~(1 << CS_S2_PIN);
            PORTB |= (1 << CS_S3_PIN);
            break;
    }
}

uint16_t measureColorFreq(char color) {
    pulse_count = 0;
    colorSel(color);
    
    // Measure for 100ms using Timer1
    TCCR1A = 0;
    TCCR1B = (1 << CS12);  // 256 prescaler
    TCNT1 = 0;
    
    while (TCNT1 < 62500); // ~100ms @ 16MHz
    TCCR1B = 0;  // Stop timer
    
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

    return (rawValue > minValues[colorIndex]) 
            ? (uint16_t)((rawValue - minValues[colorIndex]) * scalingFactors[colorIndex])
            : 0;
}

void setup() {
    uart_init();
    motorPinSetup();
    pinSetupCS();
    interruptSetupCS();

    // Set color sensor frequency scaling to 20%
    PORTB |= (1 << CS_S0_PIN);
    PORTB &= ~(1 << CS_S1_PIN);

    uart_print("System Initialized!\r\n");
}

void loop() {
    enableSensorLED();
    _delay_ms(100);  // LED stabilization

    uint16_t red = calibrateColor(measureColorFreq('R'), 'R');
    uint16_t green = calibrateColor(measureColorFreq('G'), 'G');
    uint16_t blue = calibrateColor(measureColorFreq('B'), 'B');
    
    disableSensorLED();

    // Debug output
    char buffer[60];
    snprintf(buffer, sizeof(buffer), "R: %4u  G: %4u  B: %4u\r\n", red, green, blue);
    uart_print(buffer);

    if (red < 150 && green < 150 && blue < 150) {
        uart_print("Black → Forward\r\n");
        moveForward();
    } 
    else if (green > red && green > blue) {
        uart_print("Green → Left\r\n");
        turnLeft();
    } 
    else if (red > green && red > blue) {
        uart_print("Red → Stop\r\n");
        stopMotors();
    }
    else {
        uart_print("No clear color → Forward\r\n");
        moveForward();
    }

    _delay_ms(100);
}

int main(void) {
    setup();
    while(1) loop();
    return 0;
}
