# Integration Guide

This guide shows how to integrate AutoTunePID with other Arduino libraries, sensors, actuators, and external systems.

## Sensor Integration

### Analog Sensors

#### Basic Analog Read
```cpp
#include "AutoTunePID.h"

AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols);
const int sensorPin = A0;

void setup() {
    pid.setSetpoint(512); // Half of 1023
}

void loop() {
    int rawValue = analogRead(sensorPin);
    pid.update(rawValue);
    analogWrite(9, pid.getOutput());
}
```

#### Scaled Temperature Sensor (LM35/TMP117)
```cpp
#include "AutoTunePID.h"

AutoTunePID tempPID(0, 255, TuningMethod::CohenCoon);
const int tempPin = A0;

float readTemperature() {
    // LM35: 10mV/°C, 0-100°C range
    return analogRead(tempPin) * (100.0 / 1023.0);
}

void setup() {
    tempPID.setSetpoint(25.0); // Target 25°C
    tempPID.enableInputFilter(0.1); // Smooth temperature readings
}

void loop() {
    float currentTemp = readTemperature();
    tempPID.update(currentTemp);
    analogWrite(heaterPin, tempPID.getOutput());
}
```

#### Pressure Sensor (MPX5700AP)
```cpp
#include "AutoTunePID.h"

AutoTunePID pressurePID(0, 255, TuningMethod::IMC);
const int pressurePin = A0;

float readPressure() {
    // MPX5700AP: 0-5V = 0-700kPa
    float voltage = analogRead(pressurePin) * (5.0 / 1023.0);
    return voltage * (700.0 / 5.0); // Convert to kPa
}

void setup() {
    pressurePID.setSetpoint(350.0); // Target 350kPa
    pressurePID.enableAntiWindup(true);
}
```

### Digital Sensors

#### Rotary Encoder (Speed Control)
```cpp
#include "AutoTunePID.h"
#include <Encoder.h>

AutoTunePID speedPID(0, 255, TuningMethod::ZieglerNichols);
Encoder motorEncoder(2, 3);

volatile long encoderCount = 0;
unsigned long lastTime = 0;

float readRPM() {
    unsigned long currentTime = millis();
    long count = encoderCount;
    encoderCount = 0;

    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Encoder: 1000 pulses per revolution
    return (count / 1000.0) * (60.0 / dt);
}

void setup() {
    speedPID.setSetpoint(1800); // Target 1800 RPM
}

void loop() {
    float currentRPM = readRPM();
    speedPID.update(currentRPM);
    analogWrite(motorPin, speedPID.getOutput());
}
```

#### Ultrasonic Distance Sensor (HC-SR04)
```cpp
#include "AutoTunePID.h"

AutoTunePID distancePID(0, 255, TuningMethod::Manual);
const int trigPin = 9;
const int echoPin = 10;

float readDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    return duration * 0.034 / 2; // Convert to cm
}

void setup() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    distancePID.setSetpoint(50.0); // Target 50cm
    distancePID.setManualGains(2.0, 0.1, 0.5);
}
```

## Actuator Integration

### PWM Actuators

#### DC Motor with H-Bridge (L298N)
```cpp
#include "AutoTunePID.h"

AutoTunePID motorPID(0, 255, TuningMethod::ZieglerNichols);
const int pwmPin = 9;
const int dirPin1 = 7;
const int dirPin2 = 8;

void setMotorSpeed(float speed) {
    // Constrain and map to 0-255
    int pwmValue = constrain(abs(speed), 0, 255);

    // Direction control
    if (speed >= 0) {
        digitalWrite(dirPin1, HIGH);
        digitalWrite(dirPin2, LOW);
    } else {
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, HIGH);
    }

    analogWrite(pwmPin, pwmValue);
}

void setup() {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin1, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    motorPID.setSetpoint(1500); // Target RPM
}
```

#### LED Brightness Control
```cpp
#include "AutoTunePID.h"

AutoTunePID ledPID(0, 255, TuningMethod::Manual);
const int ledPin = 9;

// Simulate light sensor input (0-100 lux)
float readLightLevel() {
    int sensorValue = analogRead(A0);
    return map(sensorValue, 0, 1023, 0, 100);
}

void setup() {
    ledPID.setSetpoint(50); // Target 50 lux
    ledPID.setManualGains(1.0, 0.05, 0.0);
}

void loop() {
    float currentLight = readLightLevel();
    ledPID.update(currentLight);
    analogWrite(ledPin, pid.getOutput());
}
```

### Servo Integration

#### Standard Servo
```cpp
#include "AutoTunePID.h"
#include <Servo.h>

AutoTunePID servoPID(0, 180, TuningMethod::Manual);
Servo myServo;

float readPosition() {
    // Read potentiometer or encoder
    return analogRead(A0) * (180.0 / 1023.0);
}

void setup() {
    myServo.attach(9);
    servoPID.setSetpoint(90); // Target 90 degrees
    servoPID.setManualGains(1.2, 0.1, 0.3);
}

void loop() {
    float currentPos = readPosition();
    servoPID.update(currentPos);
    myServo.write(pid.getOutput());
}
```

#### Continuous Rotation Servo
```cpp
#include "AutoTunePID.h"
#include <Servo.h>

AutoTunePID speedPID(90, 180, TuningMethod::ZieglerNichols); // 90 = stop
Servo continuousServo;

float readSpeed() {
    // Read encoder or tachometer
    return calculateRPM();
}

void setup() {
    continuousServo.attach(9);
    speedPID.setSetpoint(135); // Moderate speed (90+45)
}

void loop() {
    float currentSpeed = readSpeed();
    speedPID.update(currentSpeed);

    // Map PID output to servo range
    int servoCommand = map(pid.getOutput(), 0, 180, 0, 180);
    continuousServo.write(servoCommand);
}
```

## Library Integration

### With Timer Libraries

#### TimerOne (Precise Timing)
```cpp
#include "AutoTunePID.h"
#include <TimerOne.h>

AutoTunePID pid(0, 255, TuningMethod::Manual);

void controlInterrupt() {
    static float input = 0;

    // Read fast input (no filtering needed in ISR)
    input = analogRead(A0) * (100.0 / 1023.0);

    pid.update(input);
    analogWrite(9, pid.getOutput());
}

void setup() {
    pid.setSetpoint(50);
    pid.setManualGains(2.0, 0.1, 0.0);

    // 100Hz control loop
    Timer1.initialize(10000); // 10ms = 100Hz
    Timer1.attachInterrupt(controlInterrupt);
}
```

### With Communication Libraries

#### Serial Command Interface
```cpp
#include "AutoTunePID.h"

AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols);
String commandBuffer = "";

void processCommand(String cmd) {
    if (cmd.startsWith("SETPOINT ")) {
        float sp = cmd.substring(9).toFloat();
        pid.setSetpoint(sp);
        Serial.print("Setpoint set to: ");
        Serial.println(sp);
    }
    else if (cmd.startsWith("GAINS ")) {
        // Parse Kp,Ki,Kd
        int firstComma = cmd.indexOf(',');
        int secondComma = cmd.indexOf(',', firstComma + 1);

        float kp = cmd.substring(6, firstComma).toFloat();
        float ki = cmd.substring(firstComma + 1, secondComma).toFloat();
        float kd = cmd.substring(secondComma + 1).toFloat();

        pid.setManualGains(kp, ki, kd);
        Serial.println("Gains updated");
    }
    else if (cmd == "STATUS") {
        Serial.print("Setpoint: "); Serial.println(pid.getSetpoint());
        Serial.print("Output: "); Serial.println(pid.getOutput());
        Serial.print("Kp: "); Serial.println(pid.getKp());
    }
}

void setup() {
    Serial.begin(9600);
    pid.setSetpoint(100);
}

void loop() {
    // PID control
    float input = analogRead(A0);
    pid.update(input);
    analogWrite(9, pid.getOutput());

    // Serial command processing
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            processCommand(commandBuffer);
            commandBuffer = "";
        } else {
            commandBuffer += c;
        }
    }
}
```

### With Data Logging

#### SD Card Logging
```cpp
#include "AutoTunePID.h"
#include <SD.h>
#include <SPI.h>

AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols);
File logFile;

void logData(float setpoint, float input, float output) {
    if (logFile) {
        unsigned long timestamp = millis();
        logFile.print(timestamp);
        logFile.print(",");
        logFile.print(setpoint);
        logFile.print(",");
        logFile.print(input);
        logFile.print(",");
        logFile.println(output);
        logFile.flush(); // Ensure data is written
    }
}

void setup() {
    Serial.begin(9600);

    if (!SD.begin(10)) {
        Serial.println("SD card initialization failed!");
        return;
    }

    logFile = SD.open("pid_log.csv", FILE_WRITE);
    if (logFile) {
        logFile.println("timestamp,setpoint,input,output");
    }

    pid.setSetpoint(128);
}

void loop() {
    float input = analogRead(A0);
    pid.update(input);
    float output = pid.getOutput();

    analogWrite(9, output);
    logData(pid.getSetpoint(), input, output);

    delay(100);
}
```

## External System Integration

### Modbus RTU (Industrial Protocol)
```cpp
#include "AutoTunePID.h"
#include <ModbusRTU.h>

AutoTunePID pid(0, 1000, TuningMethod::Manual);
ModbusRTU modbus;

bool coilRead(uint16_t address) {
    // Handle Modbus coil reads
}

bool coilWrite(uint16_t address, bool value) {
    // Handle Modbus coil writes
}

uint16_t holdingRead(uint16_t address) {
    switch(address) {
        case 0: return (uint16_t)pid.getSetpoint();
        case 1: return (uint16_t)pid.getOutput();
        case 2: return (uint16_t)(pid.getKp() * 100);
        case 3: return (uint16_t)(pid.getKi() * 100);
        case 4: return (uint16_t)(pid.getKd() * 100);
        default: return 0;
    }
}

bool holdingWrite(uint16_t address, uint16_t value) {
    switch(address) {
        case 0: pid.setSetpoint(value); return true;
        case 2: pid.setManualGains(value / 100.0, pid.getKi(), pid.getKd()); return true;
        case 3: pid.setManualGains(pid.getKp(), value / 100.0, pid.getKd()); return true;
        case 4: pid.setManualGains(pid.getKp(), pid.getKi(), value / 100.0); return true;
        default: return false;
    }
}

void setup() {
    Serial.begin(9600, SERIAL_8N2); // Modbus RTU
    modbus.begin(&Serial);
    modbus.slave(1); // Slave ID 1

    modbus.addCoilReadCallback(coilRead);
    modbus.addCoilWriteCallback(coilWrite);
    modbus.addHoldingReadCallback(holdingRead);
    modbus.addHoldingWriteCallback(holdingWrite);

    pid.setSetpoint(500);
    pid.setManualGains(2.0, 0.5, 0.1);
}

void loop() {
    float processValue = readProcessVariable();
    pid.update(processValue);
    writeToActuator(pid.getOutput());

    modbus.task();
    delay(50); // Modbus timing
}
```

### MQTT Integration (IoT)
```cpp
#include "AutoTunePID.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols);
WiFiClient espClient;
PubSubClient client(espClient);

void callback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    if (strcmp(topic, "pid/setpoint") == 0) {
        float setpoint = message.toFloat();
        pid.setSetpoint(setpoint);
    }
    else if (strcmp(topic, "pid/mode") == 0) {
        if (message == "manual") {
            pid.setOperationalMode(OperationalMode::Manual);
        } else if (message == "auto") {
            pid.setOperationalMode(OperationalMode::Normal);
        }
    }
}

void reconnect() {
    while (!client.connected()) {
        if (client.connect("ArduinoPID")) {
            client.subscribe("pid/setpoint");
            client.subscribe("pid/mode");
        }
        delay(5000);
    }
}

void setup() {
    WiFi.begin("SSID", "PASSWORD");
    client.setServer("mqtt_server", 1883);
    client.setCallback(callback);

    pid.setSetpoint(100);
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // PID Control
    float input = analogRead(A0);
    pid.update(input);
    analogWrite(9, pid.getOutput());

    // Publish telemetry
    static unsigned long lastPublish = 0;
    if (millis() - lastPublish > 5000) {
        client.publish("pid/input", String(input).c_str());
        client.publish("pid/output", String(pid.getOutput()).c_str());
        client.publish("pid/setpoint", String(pid.getSetpoint()).c_str());
        lastPublish = millis();
    }
}
```

## Multi-Controller Systems

### Cascade Control
```cpp
#include "AutoTunePID.h"

// Primary controller (position/level)
AutoTunePID primaryPID(-100, 100, TuningMethod::ZieglerNichols);

// Secondary controller (flow/velocity)
AutoTunePID secondaryPID(0, 255, TuningMethod::CohenCoon);

void setup() {
    // Primary: Level control
    primaryPID.setSetpoint(50.0); // Target level
    primaryPID.setManualGains(1.0, 0.1, 0.0);

    // Secondary: Flow control
    secondaryPID.setSetpoint(0); // Will be set by primary
    secondaryPID.setManualGains(2.0, 0.2, 0.1);
}

void loop() {
    float currentLevel = readLevelSensor();

    // Primary control calculates desired flow
    float desiredFlow = primaryPID.update(currentLevel);

    // Secondary control achieves the desired flow
    float currentFlow = readFlowSensor();
    float valveOutput = secondaryPID.update(currentFlow - desiredFlow);

    setValvePosition(valveOutput);
}
```

### Ratio Control
```cpp
#include "AutoTunePID.h"

AutoTunePID ratioPID(0, 255, TuningMethod::Manual);

float masterFlow = 0;
float slaveFlow = 0;
const float TARGET_RATIO = 2.5; // Slave/Master ratio

void setup() {
    ratioPID.setSetpoint(0); // Error should be zero
    ratioPID.setManualGains(5.0, 1.0, 0.2);
}

void loop() {
    masterFlow = readMasterFlow();
    slaveFlow = readSlaveFlow();

    // Calculate ratio error
    float currentRatio = slaveFlow / masterFlow;
    float ratioError = TARGET_RATIO - currentRatio;

    // PID controls slave flow to maintain ratio
    ratioPID.update(ratioError);
    setSlaveValve(pid.getOutput());
}
```

## Performance Optimization

### Interrupt-Driven Control
```cpp
#include "AutoTunePID.h"

AutoTunePID pid(0, 255, TuningMethod::Manual);
volatile float latestInput = 0;
volatile bool newDataAvailable = false;

ISR(TIMER1_COMPA_vect) {
    // Read sensor in ISR (fast)
    latestInput = analogRead(A0) * SCALE;
    newDataAvailable = true;
}

void setup() {
    // Configure Timer1 for 100Hz interrupts
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    OCR1A = 2499; // 100Hz at 16MHz
    TIMSK1 = (1 << OCIE1A);

    pid.setManualGains(2.0, 0.1, 0.05);
}

void loop() {
    if (newDataAvailable) {
        pid.update(latestInput);
        analogWrite(9, pid.getOutput());
        newDataAvailable = false;
    }

    // Other non-time-critical tasks
    handleSerialCommands();
    updateDisplays();
}
```

### Memory-Efficient Configurations
```cpp
#include "AutoTunePID.h"

// For AVR with limited RAM
AutoTunePID pid(0, 255, TuningMethod::Manual);

// Configure minimal features
void setup() {
    pid.setSetpoint(128);
    pid.setManualGains(1.5, 0.05, 0.0);

    // Disable memory-intensive features
    // pid.enableInputFilter(0.1);  // Skip if RAM limited
    // pid.enableOutputFilter(0.1); // Skip if RAM limited
    // pid.enableAntiWindup(true);  // Skip if RAM limited
}
```

## Safety Integration

### Watchdog Timer
```cpp
#include "AutoTunePID.h"
#include <avr/wdt.h>

AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols);
const unsigned long WATCHDOG_TIMEOUT = 2000; // 2 seconds
static unsigned long lastControlUpdate = 0;

void setup() {
    wdt_enable(WDTO_2S); // Enable watchdog
    pid.setSetpoint(100);
}

void loop() {
    unsigned long now = millis();

    // Control logic
    float input = readSensor();
    pid.update(input);
    writeActuator(pid.getOutput());

    lastControlUpdate = now;
    wdt_reset(); // Pet the watchdog

    // Safety check
    if (now - lastControlUpdate > WATCHDOG_TIMEOUT) {
        // Emergency shutdown
        pid.setOperationalMode(OperationalMode::Override);
        pid.setOverrideOutput(0);
        digitalWrite(emergencyPin, HIGH);
    }
}
```

## Testing Integration

### Automated Test Suite
```cpp
#include "AutoTunePID.h"

AutoTunePID pid(0, 100, TuningMethod::Manual);

void runPIDTests() {
    Serial.println("Running PID integration tests...");

    // Test 1: Basic functionality
    pid.setSetpoint(50);
    pid.setManualGains(1.0, 0.0, 0.0);
    pid.update(25);
    assert(pid.getOutput() > 0);

    // Test 2: Operational modes
    pid.setOperationalMode(OperationalMode::Manual);
    pid.setManualOutput(75);
    pid.update(50);
    assert(abs(pid.getOutput() - 75) < 1);

    // Test 3: Override mode
    pid.setOperationalMode(OperationalMode::Override);
    pid.setOverrideOutput(100);
    pid.update(0);
    assert(pid.getOutput() == 100);

    Serial.println("All integration tests passed!");
}

void setup() {
    Serial.begin(9600);
    runPIDTests();
}
```

Remember: When integrating AutoTunePID with other systems, always consider timing constraints, resource limitations, and safety requirements. Test thoroughly and implement appropriate error handling! 🔧⚙️
