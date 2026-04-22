#ifndef STATES_H
#define STATES_H

enum DroneState {
    IDLE,
    TAKEOFF,
    USERCNTRL,
    PRGMCNTRL,
    LANDING
};

extern DroneState state;
extern bool droneArmed;

const char *stateName();
void setState(DroneState newState);
void applyCommand(const char *action, float value);
void updateState();

#endif
