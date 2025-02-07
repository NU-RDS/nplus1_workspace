#include <Arduino.h>

// Built-in LED is on pin 13
const int LED_PIN = 13;

void setup() {
    // Initialize LED pin as an output
    pinMode(LED_PIN, OUTPUT);
    
    // Initialize serial communication
    Serial1.begin(9600);
    while (!Serial1) {
        ; // Wait for serial connection
    }
    
    Serial1.println("Teensy 4.1 LED Blink started!");
}

void loop() {
    // Turn LED on
    digitalWriteFast(LED_PIN, HIGH);
    Serial1.println("LED ON");
    delay(1000);  // Wait for 1 second
    
    // Turn LED off
    digitalWriteFast(LED_PIN, LOW);
    Serial1.println("LED OFF");
    delay(1000);  // Wait for 1 second
}