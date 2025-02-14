
#define IR_PIN 0                  

#define BLACK_THRESHOLD 500       

void setup() {
    Serial.begin(9600);
    initADC();
}

void loop() {
    uint16_t sensorValue = readADC();

    if (sensorValue < BLACK_THRESHOLD) {
        Serial.println("Black line detected!");
    } else {
        Serial.println("White surface detected.");
    }

    delay(100);
}

void initADC() {
    ADMUX = (1 << REFS0);     // AVCC with external capacitor at AREF pin (5V reference)
    ADCSRA = (1 << ADEN) |    // Enable ADC
             (1 << ADPS2) |    // Set prescaler to 128
             (1 << ADPS1) |    // for 125KHz ADC clock
             (1 << ADPS0);
    
    // First conversion takes longer, so do a dummy read
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
}

uint16_t readADC() {
    const int numReadings = 5;
    uint32_t sum = 0;
    
    for(int i = 0; i < numReadings; i++) {
        // Set reference voltage to AVCC and select IR_PIN
        ADMUX = (1 << REFS0) | (IR_PIN & 0x0F); 
        
        // Start conversion
        ADCSRA |= (1 << ADSC);  
        
        // Wait for conversion to complete
        while (ADCSRA & (1 << ADSC));  
        
        sum += ADC; // Read the ADC value
        delay(10); // Small delay between readings
    }
    
    // Average the readings
    return (uint16_t)(sum / numReadings);
}
