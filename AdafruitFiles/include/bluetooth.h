#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <ArduinoJson.h>

extern bool bluetoothConnected;

void initBluetooth(unsigned long baud);
void bluetoothPoll();
bool bluetoothSendJson(const JsonDocument &doc);

#endif
