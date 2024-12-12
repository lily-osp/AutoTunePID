/*
 * Auto-Tune Greenhouse Humidity Control
 *
 * - A humidity sensor measures the current relative humidity.
 * - The AutoTunePID library dynamically tunes the PID values.
 * - A low-pass filter smooths noisy sensor readings.
 * - The PID output controls a humidifier using PWM.
 * - Ideal for greenhouse environments or automated climate control.
 */

#include <AutoTunePID.h>

// Define pins
#define sensorPin A0
#define humidifierPin 3

// Create PID instance
AutoTunePID pid(255, 0, 15000); // PWM range for humidifier control

void setup() {
    pinMode(humidifierPin, OUTPUT);
    Serial.begin(9600);

    // Enable auto-tuning
    pid.enableAutoTune(true);
    pid.setFilter(0.1); // Low-pass filter factor to smooth sensor readings
}

float getHumidity() {
    // Simulate humidity sensor reading (e.g., DHT22)
    return analogRead(sensorPin) * (100.0 / 1023.0); // Convert to percentage
}

void loop() {
    // Read and filter the current humidity level
    float rawHumidity = getHumidity();
    float currentHumidity = pid.filterInput(rawHumidity);

    // Compute PID output to control humidifier
    float targetHumidity = 60.0; // Desired humidity in percentage
    float output = pid.compute(currentHumidity, targetHumidity);

    // Adjust humidifier output
    analogWrite(humidifierPin, constrain(output, 0, 255));

    // Debugging info
    Serial.print("Current Humidity: ");
    Serial.print(currentHumidity);
    Serial.print("% | Target Humidity: ");
    Serial.print(targetHumidity);
    Serial.print("% | PID Output: ");
    Serial.println(output);

    delay(1000); // Sampling delay
}
