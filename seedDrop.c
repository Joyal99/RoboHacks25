#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <Servo.h>

//-------------------- Configuration --------------------
#define DEFAULT_SPEED     90
#define LEFT_MAX        2000
#define RIGHT_MAX       1000
#define COLOR_CHECK_MS   500

//-------------------- Motor Control --------------------
// Pin mapping for Arduino Nano
#define LEFTM_FWD  4  // PD4
#define LEFTM_BCK  5  // PD5
#define RIGHTM_FWD 6  // PD6
#define RIGHTM_BCK 7  // PD7
#define RIGHTM_EN  3  // PD3 (OCR2B)
#define LEFTM_EN   11 // PB3 (OCR2A)

//-------------------- IR Sensors --------------------
#define LEFT_IR    8   // PB0
#define RIGHT_IR   12  // PD6

//-------------------- Color Sensor --------------------
volatile uint16_t pulse_count = 0;
#define S0  A0  // PC0
#define S1  A1  // PC1
#define S2  A2  // PC2
#define S3  A3  // PC3
#define OUT 2   // PD2 (INT0)
#define LED 13  // PB5

Servo seedServo;

//-------------------- State Management --------------------
volatile uint8_t last_color = 0;
uint32_t last_check = 0;

//-------------------- Port Manipulation Helpers --------------------
#define MOTOR_DIR(a,b,c,d) PORTD = (PORTD & 0x0F) | (a<<4 | b<<5 | c<<6 | d<<7)
#define SET_PWM(left,right) do{OCR2A = left; OCR2B = right;} while(0)

void setupPWM() {
    // Configure Timer2 for Fast PWM (8-bit)
    TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS22);  // Prescaler 64 (~490Hz)
}

void motorSetup() {
    DDRD  |= _BV(DDD3) | _BV(DDD4) | _BV(DDD5) | _BV(DDD6) | _BV(DDD7);
    DDRB  |= _BV(DDB3);
    setupPWM();
    SET_PWM(0, 0);
    MOTOR_DIR(0,0,0,0);
}

void irSetup() {
    DDRB  &= ~_BV(DDB0);
    PORTB |= _BV(PORTB0);
    DDRD  &= ~_BV(DDD6);
    PORTD |= _BV(PORTD6);
}

uint8_t readIRs() {
    return ((PIND >> 6) & 1) | ((PINB & 1) << 1);
}

//-------------------- Color Sensor Functions --------------------
void colorSetup() {
    ADCSRA &= ~_BV(ADEN);  // Disable ADC
    DDRC  |= _BV(DDC0) | _BV(DDC1) | _BV(DDC2) | _BV(DDC3);
    PORTC |= _BV(PORTC0);  // S0=1, S1=0 (20% scaling)
    
    EICRA  = _BV(ISC00);   // Any edge triggers INT0
    EIMSK  = _BV(INT0);
    DDRB  |= _BV(DDB5);    // LED pin
}

ISR(INT0_vect) { pulse_count++; }

uint16_t measureChannel(uint8_t filter) {
    PORTC = (PORTC & 0xF3) | ((filter & 3) << 2);
    _delay_us(500);
    pulse_count = 0;
    _delay_ms(3);
    return pulse_count * 15;
}

uint8_t detectColor() {
    PORTB |= _BV(PORTB5);
    
    const uint16_t r = measureChannel(0);
    const uint16_t g = measureChannel(3);
    const uint16_t b = measureChannel(2);
    
    PORTB &= ~_BV(PORTB5);

    if (r + g + b < 300) return 0;  // Black
    if (abs(r-g)<50 && abs(r-b)<50) return 4;  // White
    
    if (g > r && g > b) return 1;  // Green
    if (r > g && r > b) return 2;  // Red
    if (b > g && b > r) return 3;  // Blue
    
    return 0xFF;  // Unknown
}

//-------------------- Servo Control --------------------
void dropSeed() {
    seedServo.writeMicroseconds(LEFT_MAX);
    _delay_ms(300);
    seedServo.writeMicroseconds(RIGHT_MAX);
    _delay_ms(300);
}

//-------------------- Main Program --------------------
void setup() {
    seedServo.attach(9);
    motorSetup();
    irSetup();
    colorSetup();
    sei();
}

void loop() {
    // Line following
    switch(readIRs()) {
        case 0:  // Both on line
            MOTOR_DIR(1,0,1,0);
            SET_PWM(DEFAULT_SPEED, DEFAULT_SPEED);
            break;
        case 1:  // Right off
            MOTOR_DIR(1,0,0,1);
            SET_PWM(DEFAULT_SPEED, DEFAULT_SPEED);
            break;
        case 2:  // Left off
            MOTOR_DIR(0,1,1,0);
            SET_PWM(DEFAULT_SPEED, DEFAULT_SPEED);
            break;
        case 3:  // Both off
            MOTOR_DIR(0,1,0,1);
            SET_PWM(DEFAULT_SPEED/2, DEFAULT_SPEED/2);
    }

    // Color detection
    if (millis() - last_check > COLOR_CHECK_MS) {
        last_check = millis();
        const uint8_t color = detectColor();
        
        if (color != last_color) {
            last_color = color;
            
            if (color == 1) {  // Green
                SET_PWM(0, 0);
                dropSeed();
                _delay_ms(1000);
            }
            else if (color == 3) {  // Blue
                SET_PWM(0, 0);
                _delay_ms(8000);
            }
        }
    }
}
