#ifndef ADC_H
#define ADC_H

#include <Arduino.h>

class ADCModule {
public:
    ADCModule(uint8_t pin); // Constructor
    void begin(); // Initialize ADC
    int readValue(); // Read value from ADC

private:
    uint8_t pin; // ADC pin number
};

#endif // ADC_H
