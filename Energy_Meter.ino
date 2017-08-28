#include <Arduino.h>

//todo Project description

// System specific constants


// General constants


// Calculated constants (at compile time)


// Pin names and functions

#define V1PIN 0
#define I1PIN 1
#define V2PIN 2
#define I2PIN 3
#define V3PIN 4
#define I3PIN 5


// Global Variables





void setup() {
    // setup pins

}

void loop() {
    // Calculate averages and transmit data to base station

}

// Timer interrupt
ISR(TIMER1_COMPA_vect){

    ADMUX = _BV(REFS0) | V1PIN; // Set ADC conversion to start on V1Pin
    ADCSRA |= _BV(ADSC); // Start ADC conversion

}

// ADC interrupt
ISR(ADC_vect){
    // Variables that persist between conversions
    static int newV1, newI1, newV2, newI2, newV3, newI3;
    static int lastV1, lastV2, lastV3;

    // Other variables
    int result;

    // Collect the result from the registers and combine
    result = ADCL;
    result |= ADCH<<8;

    // Determine which conversion completed and go to next conversion
    switch(ADMUX & 0x0f){

        case V1PIN: // V1 Just completed
            ADMUX = _BV(REFS0) | I1PIN; // Set ADC conversion to start on I1Pin
            ADCSRA |= _BV(ADSC); // Start ADC conversion

            // Perform calculations for V1

            // Apply low pass filter



    }

}