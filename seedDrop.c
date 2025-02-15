#include <Arduino.h>
#include <util/delay.h>
#include <avr/io.h>
#include <Servo.h>

//-------------------- Motor Control Pins --------------------
#define LEFTM_FORWARD_PIN  PD4
#define LEFTM_BACK_PIN     PD5
#define RIGHTM_FORWARD_PIN PD6
#define RIGHTM_BACK_PIN    PD7
#define RIGHTM_EN_PIN      PD3
#define LEFTM_EN_PIN       PB3

#define DEFAULT_SPEED      90

//-------------------- IR Sensor Pins --------------------
#define RIGHT_SENSOR_PIN PD6  // Pin 12 (PORTD6)
#define LEFT_SENSOR_PIN  PB0  // Pin 8  (PORTB0)

//-------------------- Color Sensor Pins --------------------
volatile uint16_t pulse_count = 0;
#define CS_S0_PIN   PC0
#define CS_S1_PIN   PC1
#define CS_S2_PIN   PC2
#define CS_S3_PIN   PC3
#define CS_OUT_PIN  PD2
#define CS_LED_PIN  PB5

Servo myServo;
const int servoPin = 9;

//-------------------- State Management --------------------
volatile char prevColor = 'U';
unsigned long lastColorCheck = 0;
const unsigned long colorCheckInterval = 500;

//-------------------- Port Manipulation Macros --------------------
#define SET_MOTOR_DIR(fwd, back, port) (port = (port & 0x0F) | (fwd << 4) | (back << 5))
#define READ_SENSOR(pin, port) ((port & (1 << pin)) ? HIGH : LOW

//-------------------- Motor Control Functions --------------------
void motorPinSetup() {
    DDRD  |= 0xF8;  // PD3-PD7 as outputs (pins 3-7)
    DDRB  |= (1 << LEFT_EN_PIN);  // PB3 as output (pin 11)
    PORTD &= 0x07;  // Initialize motor pins low
}

void setMotors(uint8_t leftSpeed, uint8_t rightSpeed) {
    OCR2A = leftSpeed;   // PB3 (pin 11)
    OCR2B = rightSpeed;  // PD3 (pin 3)
}

void moveForward() {
    SET_MOTOR_DIR(1, 0, PORTD);
    setMotors(DEFAULT_SPEED, DEFAULT_SPEED);
}

void turnLeft() {
    SET_MOTOR_DIR(0, 1, PORTD);
    setMotors(DEFAULT_SPEED, DEFAULT_SPEED);
}

void turnRight() {
    SET_MOTOR_DIR(1, 0, PORTD);
    setMotors(DEFAULT_SPEED, DEFAULT_SPEED);
}

void stopMotors() {
    setMotors(0, 0);
    PORTD &= 0x0F;  // Clear all motor direction pins
}

//-------------------- IR Sensor Functions --------------------
void setupSensors() {
    DDRB  &= ~(1 << LEFT_SENSOR_PIN);  // PB0 as input
    PORTB |=  (1 << LEFT_SENSOR_PIN);  // Enable pull-up
    DDRD  &= ~(1 << RIGHT_SENSOR_PIN); // PD6 as input
    PORTD |=  (1 << RIGHT_SENSOR_PIN); // Enable pull-up
}

uint8_t readSensors() {
    return ((PIND >> RIGHT_SENSOR_PIN) & 1) | ((PINB << 1) & 2);
}

//-------------------- Color Sensor Functions --------------------
void pinSetupCS() {
    ADCSRA &= ~(1 << ADEN);
    DDRC  |= 0x0F;
    PORTC |= (1 << CS_S0_PIN);
    DDRB  |= (1 << CS_LED_PIN);
}

ISR(INT0_vect) { pulse_count++; }

uint16_t measureColorFreq(char color) {
    pulse_count = 0;
    PORTC = (PORTC & 0xF3) | ((color == 'G') ? 0x0C : (color == 'B') ? 0x08 : 0x00);
    _delay_us(500);
    return pulse_count * 25;  // Compensate for shorter measurement
}

char getColor() {
    PORTB |= (1 << CS_LED_PIN);
    uint16_t r = measureColorFreq('R');
    uint16_t g = measureColorFreq('G');
    uint16_t b = measureColorFreq('B');
    PORTB &= ~(1 << CS_LED_PIN);

    if (r + g + b < 300) return 'L';
    if (abs(r - g) < 50 && abs(r - b) < 50 && abs(g - b) < 50) return 'W';
    if (g > r && g > b) return 'G';
    if (r > g && r > b) return 'R';
    if (b > r && b > g) return 'B';
    return 'U';
}

//-------------------- Main Program --------------------
void setup() {
    TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS22);
    motorPinSetup();
    setupSensors();
    pinSetupCS();
    myServo.attach(servoPin);
    EIMSK = (1 << INT0);
    sei();
}

void loop() {
    switch(readSensors()) {
        case 0: moveForward(); break;
        case 1: turnRight();  break;
        case 2: turnLeft();   break;
        case 3: turnLeft();  break;
    }

    if (millis() - lastColorCheck >= colorCheckInterval) {
        lastColorCheck = millis();
        char color = getColor();
        
        if (color != prevColor) {
            prevColor = color;
            if (color == 'G') {
                stopMotors();
                myServo.writeMicroseconds(LEFT_MAX);
                _delay_ms(300);
                myServo.writeMicroseconds(RIGHT_MAX);
                _delay_ms(300);
            }
            else if (color == 'B') {
                stopMotors();
                _delay_ms(8000);
            }
        }
    }
}
