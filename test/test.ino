// Define Motor Driver Pins
#define IN1 4  // Left Motor Forward
#define IN2 5  // Left Motor Backward
#define IN3 6  // Right Motor Forward
#define IN4 7  // Right Motor Backward

void setup() {
    // Set Motor Driver pins as OUTPUT
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void loop() {
    // Move Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(2000); // Move for 2 seconds

    // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    delay(1000); // Stop for 1 second

    // Move Backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(2000); // Move back for 2 seconds

    // Stop again
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    delay(1000); // Stop for 1 second
}
