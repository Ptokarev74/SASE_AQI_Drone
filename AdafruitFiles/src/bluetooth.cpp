#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>

#include "bluetooth.h"
#include "hardware_config.h"
#include "states.h"

bool bluetoothConnected = false;

static String bluetoothRxBuffer;
static uint32_t lastBluetoothActivityMs = 0;
static constexpr int MAX_BLUETOOTH_BYTES_PER_POLL = 64;

static void markBluetoothControlActivity() {
    bluetoothConnected = true;
    lastBluetoothActivityMs = millis();
}

void initBluetooth(unsigned long baud) {
    // Commands and telemetry use newline-delimited JSON over Serial1.
    Serial1.begin(baud, SERIAL_8N1, BT_RX_PIN, BT_TX_PIN);
    bluetoothRxBuffer = "";
    lastBluetoothActivityMs = 0;
    bluetoothConnected = false;
}

bool bluetoothSendJson(const JsonDocument &doc) {
    String payload;
    serializeJson(doc, payload);
    payload += '\n';
    return Serial1.print(payload) == payload.length();
}

static void handleBluetoothMessage(const String &message) {
    JsonDocument doc;
    const DeserializationError error = deserializeJson(doc, message);
    if (error) {
        Serial.print("Bluetooth JSON error: ");
        Serial.println(error.c_str());
        return;
    }

    const char *action = doc["action"] | "";
    const float value = doc["value"] | 0.0f;
    if (action[0] == '\0') {
        return;
    }

    markBluetoothControlActivity();
    if (strcmp(action, "heartbeat") == 0 || strcmp(action, "ping") == 0) {
        return;
    }

    applyCommand(action, value);
}

void bluetoothPoll() {
    int bytesProcessed = 0;
    while (Serial1.available() > 0 && bytesProcessed < MAX_BLUETOOTH_BYTES_PER_POLL) {
        bytesProcessed++;
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
        // Drop malformed or unterminated packets before they can grow forever.
        if (bluetoothRxBuffer.length() > 256) {
            bluetoothRxBuffer = "";
        }
    }

    if (lastBluetoothActivityMs != 0 && millis() - lastBluetoothActivityMs > BLUETOOTH_TIMEOUT_MS) {
        bluetoothConnected = false;
    }
}
