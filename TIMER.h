#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>

class Timer {
public:
    Timer();
    void start(unsigned long interval); // Start the timer
    bool hasElapsed(); // Check if timer has elapsed
    void reset(); // Reset the timer

private:
    unsigned long previousMillis; // Time the timer was last reset
    unsigned long interval;       // Interval for the timer
};

#endif // TIMER_H
