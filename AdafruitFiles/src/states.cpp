#include <Arduino.h>
#include <string.h>

#include "bluetooth.h"
#include "hardware_config.h"
#include "motors.h"
#include "pid.h"
#include "sensors.h"
#include "states.h"

DroneState state = IDLE;

static uint32_t takeoffStartMs = 0;
static uint32_t landingStartMs = 0;
static int landingInitialThrottle = 0;
static constexpr uint32_t TAKEOFF_DURATION_MS = 4000;
static constexpr uint32_t LANDING_DURATION_MS = 4000;
static constexpr int HOVER_THROTTLE_OFFSET_US = 450;
static constexpr float LANDING_THRESHOLD_CM = 20.0f;

const char *stateName() {
    switch (state) {
        case IDLE:
            return "idle";
        case TAKEOFF:
            return "takeoff";
        case USERCNTRL:
            return "user_control";
        case PRGMCNTRL:
            return "program_control";
        case LANDING:
            return "landing";
        default:
            return "unknown";
    }
}

void setState(DroneState newState) {
    if (newState == state) {
        return;
    }

    takeoffStartMs = 0;
    landingStartMs = 0;

    if (newState == LANDING) {
        landingInitialThrottle = throttleDesired;
        rollDesired = 0.0f;
        pitchDesired = 0.0f;
        yawDesired = 0.0f;
    }

    if (newState == IDLE) {
        throttleDesired = 0;
        rollDesired = 0.0f;
        pitchDesired = 0.0f;
        yawDesired = 0.0f;
        integralRoll = 0.0f;
        integralPitch = 0.0f;
        integralYaw = 0.0f;
    }

    state = newState;
}

void applyCommand(const char *action, float value) {
    if (strcmp(action, "takeoff") == 0) {
        setState(TAKEOFF);
        return;
    }

    if (state == IDLE) {
        return;
    }

    if (strcmp(action, "cancel") == 0) {
        setState(USERCNTRL);
    } else if (strcmp(action, "program_control") == 0) {
        setState(PRGMCNTRL);
    } else if (strcmp(action, "land") == 0) {
        setState(LANDING);
    } else if (strcmp(action, "throttle") == 0) {
        throttleDesired = constrain((int)value, 0, THROTTLE_MAX_OFFSET_US);
    } else if (strcmp(action, "up") == 0) {
        const int step = (value > 0.0f) ? (int)value : 50;
        throttleDesired = constrain(throttleDesired + step, 0, THROTTLE_MAX_OFFSET_US);
    } else if (strcmp(action, "down") == 0) {
        const int step = (value > 0.0f) ? (int)value : 50;
        throttleDesired = constrain(throttleDesired - step, 0, THROTTLE_MAX_OFFSET_US);
    } else if (strcmp(action, "forward") == 0) {
        pitchDesired = -20.0f;
    } else if (strcmp(action, "backward") == 0) {
        pitchDesired = 20.0f;
    } else if (strcmp(action, "left") == 0) {
        rollDesired = -20.0f;
    } else if (strcmp(action, "right") == 0) {
        rollDesired = 20.0f;
    } else if (strcmp(action, "stop_pitch") == 0) {
        pitchDesired = 0.0f;
    } else if (strcmp(action, "stop_roll") == 0) {
        rollDesired = 0.0f;
    }
}

void updateState() {
    if (!bluetoothConnected && state != IDLE && state != LANDING) {
        setState(LANDING);
    }

    const uint32_t nowMs = millis();
    switch (state) {
        case IDLE:
            throttleDesired = 0;
            rollDesired = 0.0f;
            pitchDesired = 0.0f;
            yawDesired = 0.0f;
            break;

        case TAKEOFF:
            if (takeoffStartMs == 0) {
                takeoffStartMs = nowMs;
            }
            if (nowMs - takeoffStartMs < TAKEOFF_DURATION_MS) {
                throttleDesired = map(nowMs - takeoffStartMs, 0, TAKEOFF_DURATION_MS, 0, HOVER_THROTTLE_OFFSET_US);
            } else {
                throttleDesired = HOVER_THROTTLE_OFFSET_US;
                setState(USERCNTRL);
            }
            break;

        case USERCNTRL:
        case PRGMCNTRL:
            break;

        case LANDING: {
            const SensorSnapshot sensors = getSensorSnapshot();
            if (landingStartMs == 0) {
                landingStartMs = nowMs;
                landingInitialThrottle = throttleDesired;
            }

            const uint32_t elapsedMs = nowMs - landingStartMs;
            if ((sensors.ultrasonicOk && sensors.ultrasonicCm <= LANDING_THRESHOLD_CM) ||
                elapsedMs >= LANDING_DURATION_MS) {
                setState(IDLE);
                break;
            }

            const float progress = (float)elapsedMs / (float)LANDING_DURATION_MS;
            throttleDesired = constrain((int)((1.0f - progress) * landingInitialThrottle), 0, THROTTLE_MAX_OFFSET_US);
            rollDesired = 0.0f;
            pitchDesired = 0.0f;
            yawDesired = 0.0f;
            break;
        }
    }
}
