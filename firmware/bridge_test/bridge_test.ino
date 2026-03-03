#include <ArduinoBLE.h>

// ---------------------------------------------------------------------------
// UUID Definitions (Must match bridge config.py exactly)
// ---------------------------------------------------------------------------
#define BLE_SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define TELEMETRY_CHAR_UUID     "19b10011-e8f2-537e-4f6c-d104768a1214"
#define COMMAND_CHAR_UUID       "19b10012-e8f2-537e-4f6c-d104768a1214"

// ---------------------------------------------------------------------------
// Data Structures
// ---------------------------------------------------------------------------
// Must match Python's struct.pack("<ffffBI") EXACTLY (21 bytes)
struct __attribute__((packed)) ControlCommand {
  float vx;
  float vy;
  float vz;
  float yaw;
  uint8_t arm;
  uint32_t crc32;
};

// ---------------------------------------------------------------------------
// BLE Service & Characteristics
// ---------------------------------------------------------------------------
BLEService droneService(BLE_SERVICE_UUID);

// Telemetry is what the Arduino sends TO the bridge (NOTIFY)
// Sized up to 244 to allow for larger MTUs to take effect
BLEStringCharacteristic telemetryChar(TELEMETRY_CHAR_UUID, BLERead | BLENotify, 244);

// Commands are what the Arduino receives FROM the bridge (WRITE)
BLECharacteristic commandChar(COMMAND_CHAR_UUID, BLEWrite | BLEWriteWithoutResponse, sizeof(ControlCommand));


unsigned long lastTelemetryTime = 0;
int simulatedAqi = 50;
float simulatedBatt = 12.6;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait up to 3 seconds for Serial

  Serial.println("--- AQI Drone Bridge Test ---");

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Set Advertised Name (Must match BLE_DEVICE_NAME in config.py)
  BLE.setLocalName("AQI_Drone");
  BLE.setAdvertisedService(droneService);

  // Add characteristics to the service
  droneService.addCharacteristic(telemetryChar);
  droneService.addCharacteristic(commandChar);

  // Add the service
  BLE.addService(droneService);

  // Set an event handler for when a command is written
  commandChar.setEventHandler(BLEWritten, commandWrittenCallback);

  // Start advertising
  BLE.advertise();
  Serial.println("BLE Advertising as 'AQI_Drone'...");
}

void loop() {
  // Listen for BLE peripherals to connect
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      // Send mock telemetry periodically (e.g., 10 Hz)
      if (millis() - lastTelemetryTime >= 100) {
        lastTelemetryTime = millis();
        sendTelemetry();
      }
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void sendTelemetry() {
  // Simulate some dynamic data
  simulatedAqi = 50 + (millis() / 1000) % 20; 
  simulatedBatt = 12.6 - ((millis() % 100000) / 100000.0);

  // Format MUST be NDJSON (Newline Delimited JSON) -> MUST end in '\n'
  String telemetryJson = "{\"status\":\"testing\",\"aqi\":" + String(simulatedAqi) + 
                         ",\"batt_v\":" + String(simulatedBatt, 2) + "}\n";

  telemetryChar.writeValue(telemetryJson);
}

void commandWrittenCallback(BLEDevice central, BLECharacteristic characteristic) {
  // 1. Verify we received exactly 21 bytes
  if (characteristic.valueLength() == sizeof(ControlCommand)) {
    ControlCommand cmd;
    
    // Copy the raw bytes into our struct
    memcpy(&cmd, characteristic.value(), sizeof(ControlCommand));
    
    // Debug print out what the bridge sent us
    Serial.println("--- Command Received ---");
    Serial.print("vx:  "); Serial.println(cmd.vx);
    Serial.print("vy:  "); Serial.println(cmd.vy);
    Serial.print("vz:  "); Serial.println(cmd.vz);
    Serial.print("yaw: "); Serial.println(cmd.yaw);
    Serial.print("arm: "); Serial.println(cmd.arm ? "TRUE" : "FALSE");
    Serial.print("crc: "); Serial.println(cmd.crc32, HEX);
    
    // Note: In real firmware, you would calculate the CRC32 of [vx, vy, vz, yaw, arm]
    // and compare it against cmd.crc32 to detect corruption over the air.
  } else {
    Serial.print("Warning: Received malformed command packet. Length: ");
    Serial.println(characteristic.valueLength());
  }
}
