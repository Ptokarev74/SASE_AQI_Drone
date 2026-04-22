#ifndef BAROMETER_H
#define BAROMETER_H

bool initBarometer();
bool updateBarometer();
bool isBarometerReady();
bool isBarometerOk();
float getLatestPressureHpa();
float getLatestAltitudeM();

#endif
