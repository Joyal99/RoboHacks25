#include <Arduino.h>
#include <util/delay.h>
#include <avr/io.h>

//-------------------- Motor Control Pins --------------------
#define LEFTM_FOWARD_PIN  4
#define LEFTM_BACK_PIN    5
#define RIGHTM_FOWARD_PIN 6
#define RIGHTM_BACK_PIN   7
#define RIGHTM_EN_PIN     3
#define LEFTM_EN_PIN      11

#define DEFAULT_SPEED     100

//-------------------- IR Sensor Pins --------------------
#define RIGHT_SENSOR 12
#define LEFT_SENSOR  8

//-------------------- Color Sensor Pins --------------------
volatile uint16_t pulse_count = 0;    // Incremented by ISR
#define CS_S0_PIN   PC0  // A0
#define CS_S1_PIN   PC1  // A1
#define CS_S2_PIN   PC2  // A2
#define CS_S3_PIN   PC3  // A3
#define CS_OUT_PIN  PD2  // Use PD2 for INT0
#define CS_LED_PIN  PB5  // LED control pin

// Tracks previously detected color so we don't re-trigger the same event repeatedly
volatile char prevChar = 'U';

//-------------------- Timing Variables --------------------
// We'll check color once every 300 ms (adjust as needed)
unsigned long lastColorCheck = 0;
const unsigned long colorCheckInterval = 300;  // ms

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
  digitalWrite(LEFTM_FOWARD_PIN,  HIGH);
  digitalWrite(LEFTM_BACK_PIN,    LOW);
  digitalWrite(RIGHTM_FOWARD_PIN, HIGH);
  digitalWrite(RIGHTM_BACK_PIN,   LOW);
  analogWrite(LEFTM_EN_PIN, speed);
  analogWrite(RIGHTM_EN_PIN, speed);
}

void turnLeft(uint8_t speed) {
  digitalWrite(RIGHTM_FOWARD_PIN, HIGH);
  digitalWrite(RIGHTM_BACK_PIN,   LOW);
  digitalWrite(LEFTM_FOWARD_PIN,  LOW);
  digitalWrite(LEFTM_BACK_PIN,    HIGH);
  analogWrite(LEFTM_EN_PIN, speed);
  analogWrite(RIGHTM_EN_PIN, speed);
}

void turnRight(uint8_t speed) {
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,   HIGH);
  digitalWrite(LEFTM_FOWARD_PIN,  HIGH);
  digitalWrite(LEFTM_BACK_PIN,    LOW);
  analogWrite(LEFTM_EN_PIN, speed);
  analogWrite(RIGHTM_EN_PIN, speed);
}

void stopMotors() {
  analogWrite(LEFTM_EN_PIN,  0);
  analogWrite(RIGHTM_EN_PIN, 0);
  digitalWrite(LEFTM_FOWARD_PIN,  LOW);
  digitalWrite(LEFTM_BACK_PIN,    LOW);
  digitalWrite(RIGHTM_FOWARD_PIN, LOW);
  digitalWrite(RIGHTM_BACK_PIN,   LOW);
}

//--------------------------------------------
// Color Sensor
//--------------------------------------------
ISR(INT0_vect) {
  pulse_count++;
}

void pinSetupCS() {
  // Disable ADC to use A0-A3 as digital
  ADCSRA &= ~(1 << ADEN);

  // Set S0-S3 and LED pin as outputs
  DDRC |= (1 << CS_S0_PIN) | (1 << CS_S1_PIN) | (1 << CS_S2_PIN) | (1 << CS_S3_PIN);
  // Frequency scaling to 20% (S0=HIGH, S1=LOW)
  PORTC |= (1 << CS_S0_PIN);
  PORTC &= ~(1 << CS_S1_PIN);

  // Set OUT pin as input with pull-up
  DDRD &= ~(1 << CS_OUT_PIN);
  PORTD |= (1 << CS_OUT_PIN);

  // Set up LED pin
  DDRB |= (1 << CS_LED_PIN);
}

void interruptSetupCS() {
  EIMSK |= (1 << INT0);    // Enable INT0
  EICRA |= (1 << ISC00);   // Trigger on any edge
  sei();                   // Enable global interrupts
}

void colorSel(char color) {
  switch (color) {
    case 'R': // Red filter => S2=0, S3=0
      PORTC &= ~((1 << CS_S2_PIN) | (1 << CS_S3_PIN));
      break;
    case 'G': // Green filter => S2=1, S3=1
      PORTC |= (1 << CS_S2_PIN);
      PORTC |= (1 << CS_S3_PIN);
      break;
    case 'B': // Blue filter => S2=0, S3=1
      PORTC &= ~(1 << CS_S2_PIN);
      PORTC |=  (1 << CS_S3_PIN);
      break;
  }
}

void enableSensorLED() {
  PORTB |= (1 << CS_LED_PIN);
}

void disableSensorLED() {
  PORTB &= ~(1 << CS_LED_PIN);
}

/**
 * measureColorFreq():
 *  1) Select the filter (R/G/B)
 *  2) Wait briefly for settling
 *  3) Count pulses for a short time
 *
 * Shorter times => less blocking but also less accurate.
 */
uint16_t measureColorFreq(char color) {
  pulse_count = 0;
  colorSel(color);

  // Shorter settle time
  _delay_ms(10);

  // Count pulses for a short measurement window
  pulse_count = 0;
  _delay_ms(20);

  return pulse_count;
}

/**
 * Simple calibration function:
 *  - Subtracts a minimum offset
 *  - Applies a scaling factor
 */
uint16_t calibrateColor(uint16_t rawValue, char color) {
  const uint16_t minValues[] = {30, 30, 25};     // R, G, B offsets
  const float scalingFactors[] = {1.0, 1.1, 0.9}; // R, G, B scale

  uint8_t colorIndex;
  switch (color) {
    case 'R': colorIndex = 0; break;
    case 'G': colorIndex = 1; break;
    case 'B': colorIndex = 2; break;
    default:  return rawValue;
  }

  if (rawValue <= minValues[colorIndex]) return 0;
  return (uint16_t)((rawValue - minValues[colorIndex]) * scalingFactors[colorIndex]);
}

/**
 * getColor():
 *  - Turns on sensor LED
 *  - Measures freq for R, G, B
 *  - Applies thresholds to decide color
 */
char getColor() {
  enableSensorLED();
  _delay_ms(5); // small LED stabilization

  uint16_t red   = measureColorFreq('R');
  uint16_t green = measureColorFreq('G');
  uint16_t blue  = measureColorFreq('B');

  // Calibrate each
  red   = calibrateColor(red,   'R');
  green = calibrateColor(green, 'G');
  blue  = calibrateColor(blue,  'B');

  char col;

  // Basic color detection logic
  if (red < 150 && green < 150 && blue < 150) {
    col = 'L'; // Possibly black/low light
  }
  else if (green > 2000 && red > 2000 && blue > 2000) {
    col = 'W'; // White
  }
  else if (green > red && green > blue) {
    col = 'G'; // Green
  }
  else if (red > green && red > blue) {
    col = 'R'; // Red
  }
  else if (blue > red && blue > green) {
    col = 'B'; // Blue
  }
  else {
    col = 'U'; // Unknown
  }

  disableSensorLED();
  return col;
}

//--------------------------------------------
// IR Sensor Functions
//--------------------------------------------
void setupSensors() {
  pinMode(LEFT_SENSOR,  INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
}

int readLeftSensor() {
  return digitalRead(LEFT_SENSOR);
}

int readRightSensor() {
  return digitalRead(RIGHT_SENSOR);
}

/**
 * followLine():
 *  - Reads IR sensors
 *  - Adjusts motor signals accordingly
 */
void followLine() {
  int leftVal  = readLeftSensor();
  int rightVal = readRightSensor();

  // Basic line-follow logic
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
    // Both off the line => mild turn
    turnLeft(DEFAULT_SPEED / 2);
  }
}

//--------------------------------------------
// Arduino Setup & Loop
//--------------------------------------------
void setup() {
  Serial.begin(9600);

  pinSetupCS();
  motorPinSetup();
  setupSensors();
  interruptSetupCS();
}

/**
 * loop():
 *  - Continuously follow line (fast)
 *  - Check color every 'colorCheckInterval' ms
 *  - If new color is G/B, stop for 1s or 8s
 */
void loop() {
  // 1) Continuously follow line
  followLine();

  // 2) Periodically check color
  if (millis() - lastColorCheck >= colorCheckInterval) {
    lastColorCheck = millis();

    char col = getColor();

    // If we detect green (G) and it's a new detection
    if (col == 'G' && prevChar != 'G') {
      stopMotors();
      delay(1000);     // 1 second stop
      prevChar = 'G';
    } 
    // If we detect blue (B) and it's a new detection
    else if (col == 'B' && prevChar != 'B') {
      stopMotors();
      delay(8000);     // 8 second stop
      prevChar = 'B';
    }
    // White, black, or red? Just continue line following
  }

  // The loop repeats quickly, so the car remains responsive.
}
