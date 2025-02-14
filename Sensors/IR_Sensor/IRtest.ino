#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

// Pin Definitions
#define IR_PIN 0                  // IR sensor pin (ADC0)

// Function Prototypes
float measureIRDistance(void);
void initIRSensor(void);
void initUART(void);
void UART_sendChar(char data);
void UART_sendString(const char* str);

int main(void) {
    // Initialize the IR sensor and UART for printing
    initIRSensor();
    initUART();

    while (1) {
        // Measure the distance
        float distance = measureIRDistance();

        // Print the distance
        char buffer[50];
        sprintf(buffer, "Distance: %.2f cm\n", distance);
        UART_sendString(buffer);

        _delay_ms(500); // Delay between measurements
    }
}

float measureIRDistance(void) {
    // Take multiple readings for stability
    const int numReadings = 5;
    long sum = 0;
    
    for(int i = 0; i < numReadings; i++) {
        // Set reference voltage to AVCC and select IR_PIN
        ADMUX = (1 << REFS0) | (IR_PIN & 0x0F); 
        
        // Start conversion
        ADCSRA |= (1 << ADSC);  
        
        // Wait for conversion to complete
        while (ADCSRA & (1 << ADSC));  
        
        sum += ADC;
        _delay_ms(10); // Small delay between readings
    }
    
    // Average the readings
    int reading = sum / numReadings;

    // Convert ADC reading to distance (in cm)
    // This formula is an example and should be calibrated for your specific sensor
    float voltage = (reading / 1023.0) * 5.0; // Convert ADC to voltage (assuming 5V reference)
    float distance = 27.86 * pow(voltage, -1.15); // Example formula for distance calculation

    return distance;
}

void initIRSensor(void) {
    // Set up ADC
    ADMUX = (1 << REFS0);     // AVCC with external capacitor at AREF pin
    ADCSRA = (1 << ADEN) |    // Enable ADC
             (1 << ADPS2) |    // Set prescaler to 128
             (1 << ADPS1) |    // for 125KHz ADC clock
             (1 << ADPS0);
    
    // First conversion takes longer, so do a dummy read
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
}

void initUART(void) {
    // Set baud rate to 9600
    uint16_t ubrr_value = 103; // For 16MHz clock and 9600 baud
    UBRR0H = (uint8_t)(ubrr_value >> 8);
    UBRR0L = (uint8_t)ubrr_value;

    // Enable transmitter
    UCSR0B = (1 << TXEN0);

    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_sendChar(char data) {
    // Wait for the transmit buffer to be empty
    while (!(UCSR0A & (1 << UDRE0)));
    // Send the data
    UDR0 = data;
}

void UART_sendString(const char* str) {
    // Send each character in the string
    while (*str) {
        UART_sendChar(*str++);
    }
}
