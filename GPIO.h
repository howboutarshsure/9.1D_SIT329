#ifndef GPIO_H
#define GPIO_H

#include <Arduino.h>

class GPIO_DEAKIN {
public:
    GPIO_DEAKIN(uint8_t pin); // Constructor
    void setHigh(); // Set pin high
    void setLow(); // Set pin low
    void toggle(); // Toggle pin state
    int read(); // Read pin state

private:
    uint8_t pin; // GPIO pin number
};

#endif // GPIO_H
