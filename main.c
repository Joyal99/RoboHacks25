#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <Servo.h>

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

#define LEFT_MAX 2000    // 2ms pulse (full left)
#define RIGHT_MAX 1000   // 1ms pulse (full right)

Servo myServo;           // Create servo object
const int servoPin = 9;  // PWM pin connected to servo

volatile uint16_t pulse_count = 0;  

// UART STUFF****************************************
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
//*************************************************** */

//MOTOR STUFF ******************************************
void motorPinSetup(){
    // Set motor pins as outputs
    DDRD |= (1<<RIGHTM_FOWARD_PIN) | (1<<LEFTM_FOWARD_PIN) | (1<<RIGHTM_BACK_PIN) | (1<<LEFTM_BACK_PIN);
}
  
void turnLeft(){
    // Pivot left: Right motor forward, Left motor backward
    PORTD |= (1<<RIGHTM_FOWARD_PIN);
    PORTD &= ~(1<<RIGHTM_BACK_PIN);
    _delay_us(10);
    PORTD &= ~(1<<LEFTM_FOWARD_PIN);
    PORTD |= (1<<LEFTM_BACK_PIN); 
}
  
void turnRight(){
    // Pivot right: Left motor forward, Right motor backward
    PORTD |= (1<<LEFTM_FOWARD_PIN);
    PORTD &= ~(1<<LEFTM_BACK_PIN);
    _delay_us(10);
    PORTD &= ~(1<<RIGHTM_FOWARD_PIN);
    PORTD |= (1<<RIGHTM_BACK_PIN);
}
  
void moveFoward(){
    // Both motors forward
    PORTD &= ~(1<<LEFTM_BACK_PIN);
    PORTD &= ~(1<<RIGHTM_BACK_PIN);
    PORTD |= (1<<LEFTM_FOWARD_PIN);
    PORTD |= (1<<RIGHTM_FOWARD_PIN);
}
  
void moveBackward(){
    // Both motors backward
    PORTD &= ~(1<<LEFTM_FOWARD_PIN);
    PORTD &= ~(1<<RIGHTM_FOWARD_PIN);
    PORTD |= (1<<LEFTM_BACK_PIN);
    PORTD |= (1<<RIGHTM_BACK_PIN);
}
  
void stop(){
    // Stop both motors immediately
    PORTD &= ~(1<<LEFTM_BACK_PIN);
    PORTD &= ~(1<<RIGHTM_BACK_PIN);
    PORTD &= ~(1<<LEFTM_FOWARD_PIN);
    PORTD &= ~(1<<RIGHTM_FOWARD_PIN);
    _delay_ms(8000);
}
//************************************************** */

//COLOR SENSOR STUFF**********************************
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
/******************************************************************** */

//SEED DROP STUFF***************************************************
void dropSeed(){
    for(int i = 0; i<2; i++){
      myServo.writeMicroseconds(LEFT_MAX);
      _delay_ms(3);
      myServo.writeMicroseconds(RIGHT_MAX);
      _delay_ms(10);
      myServo.writeMicroseconds(LEFT_MAX);
      Serial.println("Servo centered.");
      _delay_ms(3);
    }
  }
//******************************************************************** */

//MAIN STUFF YESSSS********************************************
int main(){
    uart_init();
    pinSetupCS();
    interruptSetupCS();
    motorPinSetup();
    myServo.attach(servoPin); 

    while(1){
        moveFoward();

        int leftSensor = getRead();
        int rightSensor = getRead();

        while(leftSensor == 0){
            turnRight();
            leftSensor = getRead();
        }
        while(rightSensor == 0){
            turnLeft();
            rightSensor = getRead();
        }
        char colorSeen = getColor();
        if(colorSeen == 'G') {
            dropSeed();
        }
        else if(colorSeen == 'B'){
            stop();
        }
        else{}
        
    }
}
