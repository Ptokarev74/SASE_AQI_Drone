#include <Arduino.h>
#include <ArduinoJson.h>
#include "states.h"
#include "motors.h"
#include "pid.h"
#include "main.h"
#include "ultrasonic.h"
#include "bluetooth.h"
#include "imu.h"

// Current state
DroneState State = IDLE;

// State-specific variables
unsigned long takeoffStartTime = 0;
const unsigned long takeoffDuration = 4000; // 4 seconds to reach hover
unsigned long landingStartTime = 0;
const unsigned long landingDuration = 4000; // 4 seconds to land
int previousHeightCM = 0;
int takeoffLastHeight = 0;
unsigned long lastDistanceSampleTime = 0;

void initStates() {
    State = IDLE;
    // Initialize to idle state
    thro_des = PWM_OFF;
    roll_des = 0;
    pitch_des = 0;
    yaw_des = 0;
}

void updateStates() {
    // Bluetooth Fail-Safe: Automatically land if connection to PC drops for 3 seconds
    if (!bluetoothConnected && State != IDLE && State != LANDING) {
        setState(LANDING);
    }

    // Sample ultrasonic distance - more frequently during takeoff/landing
    unsigned long sampleInterval = (State == TAKEOFF || State == LANDING) ? 100 : 1000;
    if (millis() - lastDistanceSampleTime >= sampleInterval) {
        previousHeightCM = distance_cm; 
        startUltrasonicMeasurement(); // Trigger a new measurement
        lastDistanceSampleTime = millis();
    }
    
    String rxMessage;
    if (bluetoothReadMessage(rxMessage)) {
        // DEBUG: Echo back exactly what the Drone buffer processed
        Serial1.print("RAW RX: ");
        Serial1.print(rxMessage);

        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, rxMessage);
        if (!error) {
            // Process the command in doc and update state variables as needed
            const char* action = doc["action"].as<const char*>();
            if (!action) action = "idle";
            float value = doc["value"].as<float>();
            
            if (State != IDLE || strcmp(action, "takeoff") == 0) {
                
                // Absolutely force state transition bindings
                if (strcmp(action, "takeoff") == 0) {
                    digitalWrite(LED_BUILTIN, HIGH); // Force the LED ON and leave it ON!
                    setState(TAKEOFF);
                }
                else if (strcmp(action, "cancel") == 0) {
                    setState(USERCNTRL);
                }
                else if (strcmp(action, "program_control") == 0) {
                    setState(PRGMCNTRL);
                }
                else if (strcmp(action, "land") == 0) {
                    setState(LANDING);
                }

                // Handle Movement Commands (WASD & Throttle Slider)
                if (strcmp(action, "throttle") == 0) {
                    thro_des = (int)value; // Adjust throttle based on UI slider
                }
                else if (strcmp(action, "forward") == 0) {
                    pitch_des = -20; // Set desired forward pitch
                }
                else if (strcmp(action, "backward") == 0) {
                    pitch_des = 20; 
                }
                else if (strcmp(action, "left") == 0) {
                    roll_des = -20;  // Set desired left roll
                }
                else if (strcmp(action, "right") == 0) {
                    roll_des = 20; 
                }
                else if (strcmp(action, "stop_pitch") == 0) {
                    pitch_des = 0; // Return to neutral pitch
                }
                else if (strcmp(action, "stop_roll") == 0) {
                    roll_des = 0; // Return to neutral roll
                }
            }
        }
        else {
            // DEBUG: Send exactly why ArduinoJson rejected it!
            Serial1.print("JSON Error: ");
            Serial1.println(error.c_str());
        }
    }

    switch (State) {
        case IDLE:
            // Motors off, wait for takeoff command
            thro_des = PWM_OFF;
            roll_des = 0;
            pitch_des = 0;
            yaw_des = 0;
            // TODO: Check for takeoff command from controller
            break;

        case TAKEOFF:
            break;
        case USERCNTRL:
            break;
        case PRGMCNTRL:
            break;
        case LANDING:
            break;
    }

    // Continuous Telemetry Broadcast (Runs 2 times a second)
    static unsigned long lastTelemetry = 0;
    if (millis() - lastTelemetry > 500) {
        lastTelemetry = millis();
        const char* stateStr = "idle";
        switch(State) {
            case IDLE: stateStr = "idle"; break;
            case TAKEOFF: stateStr = "takeoff"; break;
            case USERCNTRL: stateStr = "user_control"; break;
            case PRGMCNTRL: stateStr = "program_control"; break;
            case LANDING: stateStr = "landing"; break;
        }
        JsonDocument statusDoc;
        statusDoc["status"] = stateStr;
        statusDoc["pitch"] = pitch_IMU;
        statusDoc["target_pitch"] = pitch_des;   // What the website is demanding
        statusDoc["target_roll"] = roll_des;     // What the website is demanding
        statusDoc["battery"] = 100; // Simulated
        bluetoothSendJson(statusDoc);
    }
}

void setState(DroneState newState) {
    // Reset state-specific variables when changing states
    if (newState != State) {
        takeoffStartTime = 0;
        landingStartTime = 0;
        takeoffLastHeight = 0;
    }
    State = newState;
}