#include <ArduinoJson.h>

int simulatedBattery = 100;

void setup() {
  Serial.begin(115200);   // Main serial port for debugging
  Serial1.begin(9600);    // Bluetooth Pins 0 & 1
}

void loop() {
  if (Serial1.available()) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, Serial1);

    if (!error) {
      int mission_id = doc["mission_id"].as<int>();
      Serial.print("New Mission Received: ");
      Serial.println(mission_id);

      // Loop through the sequence of commands
      JsonArray commands = doc["commands"];
      for (JsonObject cmd : commands) {
        const char* action = cmd["action"];
        float duration = cmd["value"]; // Used as 'power/value' for throttle

        // 1. Log to the regular PC Serial Monitor
        Serial.print("Executing: ");
        Serial.print(action);
        Serial.println("...");

        // 2. Send live JSON Telemetry BACK over Bluetooth (Serial1)
        JsonDocument statusDoc;
        statusDoc["status"] = "executing";
        statusDoc["action"] = action;
        statusDoc["battery"] = simulatedBattery;
        serializeJson(statusDoc, Serial1);
        Serial1.println(); // Send newline so Python readline() knows it's done

        // 3. Simulate the drone movement
        if (strcmp(action, "throttle") == 0) {
            // For throttle, 'duration' is actually the power value (0-1000)
            delay(500); 
        } else {
            delay(duration * 1000); // Standard directional delays
        }
        
        // Decrease dummy battery
        simulatedBattery -= 1;
        if (simulatedBattery < 0) simulatedBattery = 0;
      }
      
      // 4. Send final 'Idle' completion status back over Bluetooth
      JsonDocument completeDoc;
      completeDoc["status"] = "idle";
      completeDoc["last_message"] = "Mission Complete";
      completeDoc["battery"] = simulatedBattery;
      serializeJson(completeDoc, Serial1);
      Serial1.println();

      Serial.println("Mission Complete.");
    }
  }
}