#ifndef ULTRASONIC_H
#define ULTRASONIC_H

void initUltrasonic();
void startUltrasonicMeasurement();
void updateUltrasonic();
bool isUltrasonicOk();
float getLatestUltrasonicCm();

#endif
