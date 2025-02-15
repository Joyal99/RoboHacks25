#include <Servo.h>
#include <util/delay.h>

// Define servo pulse width constants (microseconds)
#define LEFT_MAX 2000    // 2ms pulse (full left)
#define RIGHT_MAX 1000   // 1ms pulse (full right)

Servo myServo;           // Create servo object
const int servoPin = 9;  // PWM pin connected to servo

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

  
