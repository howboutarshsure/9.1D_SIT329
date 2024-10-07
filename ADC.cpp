#include "ADC.h"

// Constructor to initialize ADC pin
ADCModule::ADCModule(uint8_t pin) : pin(pin) {
}

// Initialize ADC
void ADCModule::begin() {
    pinMode(pin, INPUT); // Set pin mode to INPUT
}

// Read value from ADC
int ADCModule::readValue() {
    return analogRead(pin); // Return the ADC reading
}
