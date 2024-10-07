#include "GPIO.h"

// Constructor to initialize GPIO pin
GPIO_DEAKIN::GPIO_DEAKIN(uint8_t pin) : pin(pin) {
    pinMode(pin, OUTPUT); // Set pin mode to OUTPUT
}

// Set the GPIO pin high
void GPIO_DEAKIN::setHigh() {
    digitalWrite(pin, HIGH);
}

// Set the GPIO pin low
void GPIO_DEAKIN::setLow() {
    digitalWrite(pin, LOW);
}

// Toggle the GPIO pin state
void GPIO_DEAKIN::toggle() {
    digitalWrite(pin, !digitalRead(pin));
}

// Read the GPIO pin state
int GPIO_DEAKIN::read() {
    return digitalRead(pin);
}
