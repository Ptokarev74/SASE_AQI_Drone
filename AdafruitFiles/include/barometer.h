#ifndef BAROMETER_H
#define BAROMETER_H

bool initBarometer();
bool updateBarometer();
bool isBarometerReady();
float getLatestPressureHpa();
float getLatestAltitudeM();

#endif
