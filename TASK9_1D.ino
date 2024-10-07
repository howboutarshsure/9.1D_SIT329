#include <FreeRTOS_SAMD21.h>
#include <DHT.h>
#include "GPIO.h"
#include "ADC.h"
#include "Timer.h"

// Pin Definitions
#define PRESSURE_SENSOR_PIN A0
#define DHT_PIN 2
#define DHT_TYPE DHT22
#define LED_PIN 13
#define BUTTON_PIN 3  // Pin connected to the R16503 button
#define BUZZER_PIN 4

// Variables for Alarm Conditions
int tapCount = 0;
unsigned long lastTapTime = 0;
unsigned long lastButtonPressTime = 0;  // Time of last button press
unsigned long lastDebounceTime = 0;     // Time of last debounce check
unsigned long debounceDelay = 200;      // 200ms debounce period
unsigned long tapTimeout = 2000;        // 6 seconds to register 5 taps
unsigned long nurseCallDelay = 5000;    // 5 seconds delay after nurse is called (halved delay)
bool alarmTriggered = false;
bool buttonPressed = false;
bool buttonState = HIGH;  // Default to HIGH due to internal pull-up
bool lastButtonState = HIGH;  // Previous state of the button
bool nurseCalled = false;     // Indicates if the nurse was called
unsigned long nurseCallTime = 0;  // Time when nurse was called
bool resetAllowed = false;    // Tracks if resetting the alarm is allowed after delay

// Create Objects
DHT dht(DHT_PIN, DHT_TYPE);
GPIO_DEAKIN led(LED_PIN);
GPIO_DEAKIN buzzer(BUZZER_PIN);
ADCModule pressureSensor(PRESSURE_SENSOR_PIN);
Timer temperatureTimer;
Timer pressureTimer;

// Task Handlers
TaskHandle_t temperatureTaskHandle;
TaskHandle_t pressureTaskHandle;

// Function to measure body temperature and handle the LED and alarm system
void measureTemperature(void *pvParameters) {
    (void) pvParameters;
    temperatureTimer.start(300000); // 5 minutes interval
    while (1) {
        if (temperatureTimer.hasElapsed()) {
            float temperature = dht.readTemperature();  // Read temperature from DHT sensor
            if (!isnan(temperature)) {
                Serial.print("Temperature: ");
                Serial.print(temperature);
                Serial.println(" °C");

                if (temperature >= 38.0 && temperature < 39.0) {
                    Serial.println("Warning: Mild fever detected. Blinking LED at 1-second intervals.");
                    for (int i = 0; i < 10; i++) { // Blink for 10 seconds as an example
                        led.toggle();
                        vTaskDelay(1000 / portTICK_PERIOD_MS);  // 1-second delay between toggles
                    }
                    led.setLow(); // Make sure LED is off after blinking
                } else if (temperature >= 39.0 || temperature < 35.0) {
                    Serial.println("ALARM: Temperature out of range! Triggering buzzer.");
                    buzzer.setHigh();
                    vTaskDelay(1000 / portTICK_PERIOD_MS); // Sound for 1 second
                    buzzer.setLow();
                } else {
                    Serial.println("Temperature is within the normal range.");
                }
            } else {
                Serial.println("Failed to read temperature.");
            }
            temperatureTimer.reset();
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);  // Yield control to FreeRTOS
    }
}

// Function to measure bed occupancy and handle tap alarm for HELP signal
void measurePressure(void *pvParameters) {
    (void) pvParameters;
    pressureTimer.start(60000);  // 1 minute interval
    while (1) {
        if (pressureTimer.hasElapsed()) {
            int pressureValue = pressureSensor.readValue();  // Read value from ADC (pressure sensor)
            Serial.print("Pressure Sensor Reading: ");
            Serial.println(pressureValue);

            if (pressureValue < 100) {
                Serial.println("Bed Unoccupied.");
            } else {
                Serial.println("Bed Occupied.");
            }
            pressureTimer.reset();
        }

        // Nurse call delay logic: Ignore button presses during nurseCallDelay
        if (nurseCalled && (millis() - nurseCallTime) < nurseCallDelay) {
            vTaskDelay(100 / portTICK_PERIOD_MS);  // Yield control to FreeRTOS and continue
            continue;
        } else if (nurseCalled && (millis() - nurseCallTime) >= nurseCallDelay && !resetAllowed) {
            resetAllowed = true;  // Allow resetting the alarm after the delay period
            Serial.println("Reset allowed after nurse call delay.");
        }

        // Button debouncing and handling
        int reading = digitalRead(BUTTON_PIN);

        // If the button state changed
        if (reading != lastButtonState) {
            lastDebounceTime = millis();  // Reset debounce timer
        }

        // Only consider the press if it has been stable for debounceDelay
        if ((millis() - lastDebounceTime) > debounceDelay) {
            // If button is pressed (LOW), and it wasn't pressed before
            if (reading == LOW && !buttonPressed) {
                if (millis() - lastButtonPressTime > debounceDelay) {
                    lastButtonPressTime = millis();  // Update last press time
                    // If this is the first tap, start the 6-second timeout
                    if (tapCount == 0) {
                        lastTapTime = millis();  // Record time of first tap
                    }
                    tapCount++;
                    Serial.print("Tap detected. Count: ");
                    Serial.println(tapCount);

                    if (tapCount == 5) {
                        Serial.println("ALARM: Nurse HELP Needed!");
                        alarmTriggered = true;
                        buzzer.setHigh();  // Sound the buzzer
                        nurseCalled = true;  // Start nurse call delay
                        nurseCallTime = millis();  // Record the time when the nurse was called
                        resetAllowed = false;  // Prevent immediate reset
                    }
                    buttonPressed = true;  // Register the press
                }
            } else if (reading == HIGH) {
                // Button released, allow next press
                buttonPressed = false;
            }
        }

        // Save the reading for the next iteration
        lastButtonState = reading;

        // Reset tap count if 6 seconds have passed without reaching 5 taps
        if (tapCount > 0 && (millis() - lastTapTime) > tapTimeout && tapCount < 5) {
            Serial.println("Tap timeout. Resetting count.");
            tapCount = 0;  // Reset tap count
        }

        // If alarm is triggered and reset is allowed, reset when button is pressed again
        if (alarmTriggered && resetAllowed && digitalRead(BUTTON_PIN) == LOW) {
            Serial.println("Nurse Attended. ALARM Reset.");
            buzzer.setLow();
            alarmTriggered = false;
            resetAllowed = false;  // Reset the state for future calls
            nurseCalled = false;   // Reset nurse call state
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);  // Yield control to FreeRTOS
    }
}

void setup() {
    Serial.begin(9600);

    // Initialize DHT sensor and ADC
    dht.begin();
    pressureSensor.begin();
    led.setLow();
    buzzer.setLow();

    // Initialize button pin with internal pull-up resistor
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // Enable internal pull-up resistor

    Serial.println("System Initialized.");

    // Create FreeRTOS tasks
    xTaskCreate(measureTemperature, "Measure Temperature", 1000, NULL, 1, &temperatureTaskHandle);
    xTaskCreate(measurePressure, "Measure Pressure", 1000, NULL, 1, &pressureTaskHandle);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
}

void loop() {
    // Empty loop as FreeRTOS handles all task scheduling
}
