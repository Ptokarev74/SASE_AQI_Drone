#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <ArduinoJson.h>

void initBluetooth(unsigned long baud = 9600);

void bluetoothPoll();

bool bluetoothMessageAvailable();

bool bluetoothReadMessage(String &message);

bool bluetoothParseMessage(const String &message, JsonDocument &doc, DeserializationError &error);

bool bluetoothPackJson(const JsonDocument &doc, String &out);

bool bluetoothSendJson(const JsonDocument &doc);

bool bluetoothSendJsonString(const String &json);

#endif // BLUETOOTH_H
