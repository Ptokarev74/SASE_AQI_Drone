#ifndef TOF_H
#define TOF_H

bool initTofSensor();
void updateTofSensor();
bool isTofReady();
bool isTofOk();
float getLatestTofCm();

#endif
