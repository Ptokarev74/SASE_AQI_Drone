#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

extern int throttleDesired;

void initMotors();
void stopMotors();
void mixMotors();

#endif
