#define IR_PIN A0                  

#define BLACK_THRESHOLD 500      

void setup() {
    Serial.begin(9600);

    pinMode(IR_PIN, INPUT);
}

void loop() {
    int sensorValue = analogRead(IR_PIN);

    if (sensorValue < BLACK_THRESHOLD) {
        Serial.println("Black line detected!");
    } else {
        Serial.println("White surface detected.");
    }

    delay(100); // Delay between readings
}
