#include <Arduino.h>
#include <ArduinoJson.h>

#include "bluetooth.h"
#include "hardware_config.h"
#include "states.h"

bool bluetoothConnected = false;

static String bluetoothRxBuffer;
static uint32_t lastBluetoothActivityMs = 0;

void initBluetooth(unsigned long baud) {
    Serial1.begin(baud, SERIAL_8N1, BT_RX_PIN, BT_TX_PIN);
    bluetoothRxBuffer = "";
    lastBluetoothActivityMs = millis();
    bluetoothConnected = true;
}

bool bluetoothSendJson(const JsonDocument &doc) {
    String payload;
    serializeJson(doc, payload);
    payload += '\n';
    Serial1.print(payload);
    bluetoothConnected = true;
    lastBluetoothActivityMs = millis();
    return true;
}

static void handleBluetoothMessage(const String &message) {
    JsonDocument doc;
    const DeserializationError error = deserializeJson(doc, message);
    if (error) {
        Serial.print("Bluetooth JSON error: ");
        Serial.println(error.c_str());
        return;
    }

    const char *action = doc["action"] | "idle";
    const float value = doc["value"] | 0.0f;
    bluetoothConnected = true;
    lastBluetoothActivityMs = millis();
    applyCommand(action, value);
}

void bluetoothPoll() {
    while (Serial1.available() > 0) {
        const char c = (char)Serial1.read();
        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            if (bluetoothRxBuffer.length() > 0) {
                handleBluetoothMessage(bluetoothRxBuffer);
                bluetoothRxBuffer = "";
            }
            continue;
        }

        bluetoothRxBuffer += c;
        if (bluetoothRxBuffer.length() > 256) {
            bluetoothRxBuffer = "";
        }
    }

    if (millis() - lastBluetoothActivityMs > BLUETOOTH_TIMEOUT_MS) {
        bluetoothConnected = false;
    }
}
