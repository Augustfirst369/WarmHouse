#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Configuration
const char* ssid = "Telia-EE6B65-Greitas";
const char* password = "xxx";
const char* mqtt_server = "192.168.1.11"; // Update this

// Pins
#define ENCODER_PIN D2
#define RELAY_CLOSE D3 
#define RELAY_OPEN  D4
#define RELAY_HEAT  D5
#define ONE_WIRE_BUS D6

// Constants
const unsigned long ENCODER_TIMEOUT = 2000; // 2 seconds
const int TEMP_SAMPLES = 10;
const float TEMP_LOW = 21.0;
const float TEMP_HIGH = 24.0;
const float HEATER_ON = 3.0;
const float HEATER_OFF = 6.0;

// State Variables
enum Mode { AUTO, MANUAL_OPEN, MANUAL_CLOSE };
Mode currentMode = AUTO;
float tempHistory[TEMP_SAMPLES];
int tempIndex = 0;
float avgTemp = 0;
volatile unsigned long lastEncoderPulse = 0;
unsigned long lastTempUpdate = 0;
unsigned long lastMqttRetry = 0;
unsigned long lastMqttReport = 0;
unsigned long actuatorStartTime = 0;
bool isMoving = false;
bool isWaiting = false;
unsigned long waitStartTime = 0;
bool sensorError = false;
bool errorActionTaken = false;
float tempSum = 0;
bool firstTempReading = true;
bool forceAutoStatusPublish = false;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void IRAM_ATTR encoderISR() {
    lastEncoderPulse = millis();
}

void stopActuator() {
    isMoving = false;
    digitalWrite(RELAY_OPEN, LOW);
    digitalWrite(RELAY_CLOSE, LOW);
}

void moveActuator(bool open) {
    stopActuator();
    delay(50); // Small dead-time to protect relay contacts
    if (open) {
        digitalWrite(RELAY_OPEN, HIGH);
        mqttClient.publish("esp/snamis/status", "opening");
    } else {
        digitalWrite(RELAY_CLOSE, HIGH);
        mqttClient.publish("esp/snamis/status", "closing");
    }
    lastEncoderPulse = millis();
    actuatorStartTime = millis();
    isMoving = true;
}

void callback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (int i = 0; i < length; i++) message += (char)payload[i];

    if (message == "open") {
        currentMode = MANUAL_OPEN;
        moveActuator(true);
    } else if (message == "close") {
        currentMode = MANUAL_CLOSE;
        moveActuator(false);
    } else if (message == "auto") {
        currentMode = AUTO;
        stopActuator();
        isWaiting = false;
        mqttClient.publish("esp/snamis/status", "auto");
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(RELAY_OPEN, OUTPUT);
    pinMode(RELAY_CLOSE, OUTPUT);
    pinMode(RELAY_HEAT, OUTPUT);
    pinMode(ENCODER_PIN, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
    
    sensors.begin();
    sensors.setWaitForConversion(false); // Enable async mode
    sensors.requestTemperatures();       // Start first conversion
    setup_wifi();
    mqttClient.setServer(mqtt_server, 1883);
    mqttClient.setCallback(callback);
}

void setup_wifi() {
    WiFi.begin(ssid, password);
    // Non-blocking: the reconnect() function in loop() will handle connection maintenance
}

void reconnect() {
    bool wifiConnected = (WiFi.status() == WL_CONNECTED);
    bool mqttConnected = mqttClient.connected();

    if (!wifiConnected || !mqttConnected) {
        // Fallback to AUTO mode if communication is lost
        if (currentMode != AUTO) {
            currentMode = AUTO;
            stopActuator(); // Safely stop any manual movement override
            isWaiting = false; // Reset the duty cycle timer to check temp immediately
            forceAutoStatusPublish = true; 
        }

        if (millis() - lastMqttRetry > 5000) {
            lastMqttRetry = millis();
            if (!wifiConnected) {
                WiFi.begin(ssid, password);
            } else if (!mqttConnected) {
                if (mqttClient.connect("WarmHouseClient")) {
                    mqttClient.subscribe("esp/snamis/door");
                    if (forceAutoStatusPublish) {
                        mqttClient.publish("esp/snamis/status", "auto");
                        forceAutoStatusPublish = false;
                    }
                }
            }
        }
    }
}

void updateTemperature() {
    if (millis() - lastTempUpdate >= 1000) {
        float currentTemp = sensors.getTempCByIndex(0);
        
        if (currentTemp != DEVICE_DISCONNECTED_C) {
            if (firstTempReading) {
                for (int i = 0; i < TEMP_SAMPLES; i++) tempHistory[i] = currentTemp;
                tempSum = currentTemp * TEMP_SAMPLES;
                firstTempReading = false;
            } else {
                tempSum -= tempHistory[tempIndex];
                tempHistory[tempIndex] = currentTemp;
                tempSum += tempHistory[tempIndex];
            }
            tempIndex = (tempIndex + 1) % TEMP_SAMPLES;
            avgTemp = tempSum / TEMP_SAMPLES;
            sensorError = false;
            errorActionTaken = false; 
        } else {
            // Sensor failed - fail safe: close the door
            sensorError = true;
            avgTemp = -99; // Indicator for UI
        }
        
        sensors.requestTemperatures(); // Async request for next loop
        lastTempUpdate = millis();
    }
}

void handleHeater() {
    if (avgTemp <= HEATER_ON) {
        if (digitalRead(RELAY_HEAT) == LOW) {
            digitalWrite(RELAY_HEAT, HIGH);
            mqttClient.publish("esp/snamis/status", "heating on");
        }
    } else if (avgTemp >= HEATER_OFF) {
        if (digitalRead(RELAY_HEAT) == HIGH) {
            digitalWrite(RELAY_HEAT, LOW);
            mqttClient.publish("esp/snamis/status", "heating off");
        }
    }
}

void handleActuatorLogic() {
    // Fail-safe: If sensor fails, override to manual close
    if (sensorError && !isMoving && currentMode == AUTO && !errorActionTaken) {
        moveActuator(false); 
        errorActionTaken = true;
        // We don't change mode to MANUAL so it can recover if sensor returns,
        // but we trigger a close cycle.
    }

    if (isMoving) {
        // Stop if stalled (encoder timeout)
        if (millis() - lastEncoderPulse > ENCODER_TIMEOUT) {
            bool wasOpening = (digitalRead(RELAY_OPEN) == HIGH);
            stopActuator();
            if (currentMode == MANUAL_OPEN) mqttClient.publish("esp/snamis/status", "open-manual");
            else if (currentMode == MANUAL_CLOSE) mqttClient.publish("esp/snamis/status", "closed-manual");
            else {
                // In Auto mode, stalled means it reached limits
                mqttClient.publish("esp/snamis/status", wasOpening ? "open-auto" : "closed-auto");
            }
            return;
        }

        // In Auto mode, only move in 6-second bursts
        if (currentMode == AUTO && !sensorError && (millis() - actuatorStartTime >= 6000)) {
            stopActuator();
            isWaiting = true;
            waitStartTime = millis();
        }
    }

    if (currentMode == AUTO) {
        if (isWaiting) {
            if (millis() - waitStartTime >= 60000) {
                isWaiting = false;
            }
        } else if (!isMoving && !sensorError) {
            if (avgTemp > TEMP_HIGH) {
                moveActuator(true); // Cooling
            } else if (avgTemp < TEMP_LOW) {
                moveActuator(false); // Warming
            }
        }
    }
}

void loop() {
    reconnect();
    mqttClient.loop();

    updateTemperature();
    handleHeater();
    handleActuatorLogic();

    // Periodic reporting
    if (millis() - lastMqttReport >= 60000) {
        char tempStr[8];
        dtostrf(avgTemp, 1, 2, tempStr);
        mqttClient.publish("esp/snamis/T", tempStr);
        lastMqttReport = millis();
    }
}