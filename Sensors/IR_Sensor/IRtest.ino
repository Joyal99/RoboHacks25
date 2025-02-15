#define MS 12      // Middle Sensor

void setup() {
  Serial.begin(9600);
  pinMode(MS, INPUT);  // Set the Middle Sensor as input
}

void loop() {
  // Read the Middle Sensor value
  int sensorValue = digitalRead(MS);

  // Check if the sensor reads HIGH or LOW
  if (sensorValue == HIGH) {
    Serial.println("Sensor is HIGH");
  } else {
    Serial.println("Sensor is LOW");
  }

  delay(100);  // Add a small delay for stability
}
