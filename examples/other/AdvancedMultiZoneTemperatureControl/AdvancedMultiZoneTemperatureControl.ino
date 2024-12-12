#include "AutoTunePID.h"
#include <EEPROM.h>
#include "RTClib.h"
#include <SD.h>

// Number of temperature zones to control
const int NUM_ZONES = 3;

// Pin definitions for each zone
struct ZoneConfig {
    uint8_t sensorPin;    // Temperature sensor pin
    uint8_t heaterPin;    // Heater control pin
    uint8_t fanPin;       // Cooling fan pin
    float setpoint;       // Target temperature
    float minTemp;        // Minimum allowable temperature
    float maxTemp;        // Maximum allowable temperature
};

ZoneConfig zones[NUM_ZONES] = {
    {A0, 3, 6, 25.0, 20.0, 30.0},  // Zone 1: Precision control (±0.5°C)
    {A1, 5, 9, 40.0, 35.0, 45.0},  // Zone 2: High temperature (±1.0°C)
    {A2, 6, 10, 15.0, 10.0, 20.0}  // Zone 3: Low temperature (±0.5°C)
};

// PID controllers for each zone
struct ZoneController {
    AutoTunePID* pid;
    float lastTemp;
    float lastOutput;
    unsigned long lastCrossing;  // For adaptive tuning
    int oscillationCount;
    bool tuningEnabled;
    float bestKp, bestKi, bestKd;
    float bestPerformance;
};

ZoneController controllers[NUM_ZONES];

// Data logging configuration
const int CHIP_SELECT_PIN = 4;
RTC_DS3231 rtc;
File logFile;

// EEPROM addresses for storing PID parameters
const int EEPROM_START_ADDRESS = 0;
const int ZONE_DATA_SIZE = 12;  // 3 floats (Kp, Ki, Kd) * 4 bytes

// Performance monitoring
struct PerformanceMetrics {
    float averageError;
    float maxError;
    float responseTime;
    int oscillationCount;
    unsigned long settlingTime;
};

PerformanceMetrics zoneMetrics[NUM_ZONES];

void setup() {
    Serial.begin(115200);

    // Initialize RTC
    if (!rtc.begin()) {
        Serial.println("RTC failed!");
        while (1);
    }

    // Initialize SD card
    if (!SD.begin(CHIP_SELECT_PIN)) {
        Serial.println("SD card initialization failed!");
        while (1);
    }

    // Initialize controllers and load saved parameters
    for (int i = 0; i < NUM_ZONES; i++) {
        // Initialize pins
        pinMode(zones[i].heaterPin, OUTPUT);
        pinMode(zones[i].fanPin, OUTPUT);

        // Create PID controller
        controllers[i].pid = new AutoTunePID(0, 255, ZieglerNichols);
        controllers[i].pid->setSetpoint(zones[i].setpoint);
        controllers[i].pid->enableInputFilter(0.1);  // Smooth temperature readings
        controllers[i].pid->enableOutputFilter(0.15);  // Smooth control outputs

        // Load saved PID parameters if available
        loadPIDParameters(i);

        // Initialize performance metrics
        resetPerformanceMetrics(i);
    }

    // Create new log file with timestamp
    DateTime now = rtc.now();
    String filename = String(now.year()) + String(now.month()) + String(now.day()) + ".csv";
    logFile = SD.open(filename, FILE_WRITE);
    if (logFile) {
        logFile.println("Timestamp,Zone,Temperature,Setpoint,Output,Kp,Ki,Kd,Error");
        logFile.close();
    }
}

void loop() {
    static unsigned long lastLog = 0;
    static unsigned long lastPerformanceCheck = 0;
    DateTime now = rtc.now();

    // Update each zone
    for (int i = 0; i < NUM_ZONES; i++) {
        // Read temperature
        float temperature = readTemperature(i);

        // Check for temperature limits
        if (temperature < zones[i].minTemp || temperature > zones[i].maxTemp) {
            handleTemperatureAlarm(i, temperature);
        }

        // Update PID controller
        controllers[i].pid->update(temperature);
        float output = controllers[i].pid->getOutput();

        // Apply control output
        applyOutput(i, output);

        // Check for adaptive tuning
        if (controllers[i].tuningEnabled) {
            checkAdaptiveTuning(i, temperature);
        }

        // Update performance metrics
        updatePerformanceMetrics(i, temperature);

        // Store values for next iteration
        controllers[i].lastTemp = temperature;
        controllers[i].lastOutput = output;
    }

    // Log data every 5 seconds
    if (millis() - lastLog >= 5000) {
        logZoneData();
        lastLog = millis();
    }

    // Check performance and adjust tuning every minute
    if (millis() - lastPerformanceCheck >= 60000) {
        for (int i = 0; i < NUM_ZONES; i++) {
            checkPerformanceAndAdjust(i);
        }
        lastPerformanceCheck = millis();
    }

    // Process serial commands
    if (Serial.available()) {
        processSerialCommand();
    }
}

float readTemperature(int zone) {
    // Read analog value and convert to temperature
    int rawValue = analogRead(zones[zone].sensorPin);
    // Example conversion for LM35 sensor (10mV/°C)
    return (rawValue * 5.0 * 100.0) / 1024.0;
}

void applyOutput(int zone, float output) {
    // Determine if heating or cooling is needed
    if (controllers[zone].lastTemp < zones[zone].setpoint) {
        // Heating mode
        analogWrite(zones[zone].heaterPin, output);
        analogWrite(zones[zone].fanPin, 0);
    } else {
        // Cooling mode
        analogWrite(zones[zone].heaterPin, 0);
        analogWrite(zones[zone].fanPin, output);
    }
}

void checkAdaptiveTuning(int zone, float temperature) {
    float setpoint = zones[zone].setpoint;
    unsigned long now = millis();

    // Detect zero crossing
    if ((controllers[zone].lastTemp < setpoint && temperature >= setpoint) ||
        (controllers[zone].lastTemp > setpoint && temperature <= setpoint)) {

        if (controllers[zone].lastCrossing != 0) {
            unsigned long period = now - controllers[zone].lastCrossing;
            controllers[zone].oscillationCount++;

            // After 5 oscillations, calculate new parameters
            if (controllers[zone].oscillationCount >= 5) {
                calculateNewParameters(zone, period);
                controllers[zone].tuningEnabled = false;
            }
        }
        controllers[zone].lastCrossing = now;
    }
}

void calculateNewParameters(int zone, unsigned long period) {
    // Calculate new PID parameters based on observed oscillations
    float Ku = 4.0 * controllers[zone].lastOutput / (3.14159 * temperature);
    float Tu = period / 1000.0;  // Convert to seconds

    float newKp = 0.6 * Ku;
    float newKi = 1.2 * Ku / Tu;
    float newKd = 0.075 * Ku * Tu;

    // Apply new parameters if performance improved
    controllers[zone].pid->setManualGains(newKp, newKi, newKd);
}

void updatePerformanceMetrics(int zone, float temperature) {
    float error = abs(zones[zone].setpoint - temperature);
    zoneMetrics[zone].averageError = (zoneMetrics[zone].averageError * 0.95) + (error * 0.05);
    zoneMetrics[zone].maxError = max(zoneMetrics[zone].maxError, error);

    // Check settling time
    if (error < 0.5 && zoneMetrics[zone].settlingTime == 0) {
        zoneMetrics[zone].settlingTime = millis();
    }
}

void logZoneData() {
    DateTime now = rtc.now();
    logFile = SD.open("temp.csv", FILE_WRITE);

    if (logFile) {
        for (int i = 0; i < NUM_ZONES; i++) {
            logFile.print(now.timestamp());
            logFile.print(",");
            logFile.print(i);
            logFile.print(",");
            logFile.print(controllers[i].lastTemp);
            logFile.print(",");
            logFile.print(zones[i].setpoint);
            logFile.print(",");
            logFile.print(controllers[i].lastOutput);
            logFile.print(",");
            logFile.print(controllers[i].pid->getKp());
            logFile.print(",");
            logFile.print(controllers[i].pid->getKi());
            logFile.print(",");
            logFile.print(controllers[i].pid->getKd());
            logFile.println();
        }
        logFile.close();
    }
}

void processSerialCommand() {
    String command = Serial.readStringUntil('\n');

    if (command.startsWith("setpoint")) {
        // Format: setpoint zone value
        int zone = command.substring(9, 10).toInt();
        float value = command.substring(11).toFloat();
        if (zone >= 0 && zone < NUM_ZONES) {
            zones[zone].setpoint = value;
            controllers[zone].pid->setSetpoint(value);
            Serial.println("Setpoint updated");
        }
    } else if (command.startsWith("tune")) {
        // Format: tune zone
        int zone = command.substring(5).toInt();
        if (zone >= 0 && zone < NUM_ZONES) {
            controllers[zone].tuningEnabled = true;
            controllers[zone].oscillationCount = 0;
            controllers[zone].lastCrossing = 0;
            Serial.println("Started tuning");
        }
    }
}

void savePIDParameters(int zone) {
    int address = EEPROM_START_ADDRESS + (zone * ZONE_DATA_SIZE);
    EEPROM.put(address, controllers[zone].pid->getKp());
    EEPROM.put(address + 4, controllers[zone].pid->getKi());
    EEPROM.put(address + 8, controllers[zone].pid->getKd());
}

void loadPIDParameters(int zone) {
    int address = EEPROM_START_ADDRESS + (zone * ZONE_DATA_SIZE);
    float kp, ki, kd;
    EEPROM.get(address, kp);
    EEPROM.get(address + 4, ki);
    EEPROM.get(address + 8, kd);

    // Check if values are valid
    if (isnan(kp) || isnan(ki) || isnan(kd)) {
        // Use default values
        controllers[zone].pid->setManualGains(2.0, 0.5, 0.1);
    } else {
        controllers[zone].pid->setManualGains(kp, ki, kd);
    }
}

void resetPerformanceMetrics(int zone) {
    zoneMetrics[zone].averageError = 0;
    zoneMetrics[zone].maxError = 0;
    zoneMetrics[zone].responseTime = 0;
    zoneMetrics[zone].oscillationCount = 0;
    zoneMetrics[zone].settlingTime = 0;
}

void handleTemperatureAlarm(int zone, float temperature) {
    // Emergency shutdown if temperature is out of bounds
    digitalWrite(zones[zone].heaterPin, LOW);
    digitalWrite(zones[zone].fanPin, HIGH);  // Run fan at full speed

    // Log alarm
    logFile = SD.open("alarms.txt", FILE_WRITE);
    if (logFile) {
        DateTime now = rtc.now();
        logFile.print(now.timestamp());
        logFile.print(" Zone ");
        logFile.print(zone);
        logFile.print(" Temperature alarm: ");
        logFile.println(temperature);
        logFile.close();
    }

    // Notify via Serial
    Serial.print("ALARM: Zone ");
    Serial.print(zone);
    Serial.print(" temperature out of bounds: ");
    Serial.println(temperature);
}
