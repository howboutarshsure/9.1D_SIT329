#include "Timer.h"

// Constructor
Timer::Timer() : previousMillis(0), interval(0) {}

// Start the timer
void Timer::start(unsigned long interval) {
    this->interval = interval;
    previousMillis = millis(); // Record the current time
}

// Check if the timer has elapsed
bool Timer::hasElapsed() {
    if (millis() - previousMillis >= interval) {
        previousMillis = millis(); // Reset the previous time
        return true;               // Timer has elapsed
    }
    return false;                // Timer has not elapsed
}

// Reset the timer
void Timer::reset() {
    previousMillis = millis(); // Reset the recorded time
}
