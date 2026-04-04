#include "bluetooth.h"
#include <ArduinoJson.h>

static bool bluetoothRxReady = false;
static String bluetoothRxMessage;

static void bluetoothResetBuffer() {
    bluetoothRxReady = false;
    bluetoothRxMessage = String();
}

void initBluetooth(unsigned long baud) {
    Serial1.begin(baud);
    bluetoothResetBuffer();
}

void bluetoothPoll() {
    while (Serial1.available()) {
        char c = (char)Serial1.read();

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, bluetoothRxMessage);
            if (!error) {
                bluetoothRxReady = true;
            } else {
                // Parsing failed, reset buffer and ignore this message
                bluetoothResetBuffer();
            }
        }
        else {
            bluetoothRxMessage += c;
        }
    }
}

bool bluetoothMessageAvailable() {
    return bluetoothRxReady;
}

bool bluetoothReadMessage(String &message) {
    if (!bluetoothRxReady) {
        return false;
    }

    message = bluetoothRxMessage;
    bluetoothResetBuffer();
    return true;
}

bool bluetoothParseMessage(const String &message, JsonDocument &doc, DeserializationError &error) {
    error = deserializeJson(doc, message);
    return (error == DeserializationError::Ok);
}

bool bluetoothPackJson(const JsonDocument &doc, String &out) {
    out = String();
    serializeJson(doc, out);
    out += '\n';
    return true;
}

bool bluetoothSendJson(const JsonDocument &doc) {
    String payload;
    if (!bluetoothPackJson(doc, payload)) {
        return false;
    }
    Serial1.print(payload);
    return true;
}

bool bluetoothSendJsonString(const String &json) {
    if (json.length() == 0) {
        return false;
    }
    Serial1.print(json);
    if (!json.endsWith("\n")) {
        Serial1.print('\n');
    }
    return true;
}
