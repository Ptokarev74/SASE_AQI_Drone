#include <Arduino.h>
#include <string.h>

#include "bluetooth.h"
#include "hardware_config.h"
#include "main.h"
#include "motors.h"
#include "pid.h"
#include "sensors.h"
#include "states.h"

DroneState state = IDLE;
bool droneArmed = false;

static uint32_t takeoffStartMs = 0;
static uint32_t landingStartMs = 0;
static int landingInitialThrottle = 0;
static constexpr uint32_t TAKEOFF_DURATION_MS = 4000;
static constexpr uint32_t LANDING_DURATION_MS = 4000;
static constexpr uint32_t LANDING_STOP_TIMEOUT_MS = LANDING_DURATION_MS + 1000;
static constexpr int HOVER_THROTTLE_OFFSET_US = 450;
static constexpr int LANDING_HOLD_THROTTLE_OFFSET_US = 80;
static constexpr float LANDING_THRESHOLD_CM = 20.0f;

static bool anySensorOk(const SensorSnapshot &sensors) {
    return sensors.barometerOk || sensors.ultrasonicOk || sensors.tofOk;
}

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
    if (newState == IDLE) {
        droneArmed = false;
        throttleDesired = 0;
        rollDesired = 0.0f;
        pitchDesired = 0.0f;
        yawDesired = 0.0f;
        integralRoll = 0.0f;
        integralPitch = 0.0f;
        integralYaw = 0.0f;
    }

    if (newState == state) {
        return;
    }

    takeoffStartMs = 0;
    landingStartMs = 0;

    // Landing always starts from the current throttle and neutral attitude.
    if (newState == LANDING) {
        landingInitialThrottle = throttleDesired;
        rollDesired = 0.0f;
        pitchDesired = 0.0f;
        yawDesired = 0.0f;
    }

    state = newState;
}

void applyCommand(const char *action, float value) {
    if (strcmp(action, "disarm") == 0) {
        setState(IDLE);
        return;
    }

    if (strcmp(action, "arm") == 0) {
        const SensorSnapshot sensors = getSensorSnapshot();
        if (state == IDLE && flightSystemReady && bluetoothConnected && anySensorOk(sensors)) {
            droneArmed = true;
        } else {
            Serial.println("Arm rejected: not idle, not ready, no control link, or no healthy sensors");
        }
        return;
    }

    // Accept takeoff before the IDLE guard so the drone can leave IDLE.
    if (strcmp(action, "takeoff") == 0) {
        const SensorSnapshot sensors = getSensorSnapshot();
        if (state == IDLE && droneArmed && flightSystemReady && bluetoothConnected && anySensorOk(sensors)) {
            setState(TAKEOFF);
        } else {
            Serial.println("Takeoff rejected: not idle, not armed, not ready, no control link, or no healthy sensors");
        }
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
    // Lost Bluetooth control triggers an autonomous landing instead of holding
    // the last user command.
    if (!bluetoothConnected && state != IDLE && state != LANDING) {
        setState(LANDING);
    }

    const uint32_t nowMs = millis();
    switch (state) {
        case IDLE:
            if (droneArmed && !areSensorsReady()) {
                droneArmed = false;
                Serial.println("Disarmed: no healthy sensors");
            }
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
                // Ramp up slowly so the ESCs and frame are not stepped abruptly.
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

            const bool groundDetected =
                (sensors.ultrasonicOk && sensors.ultrasonicCm <= LANDING_THRESHOLD_CM) ||
                (sensors.tofOk && sensors.tofCm <= LANDING_THRESHOLD_CM);
            if (groundDetected) {
                setState(IDLE);
                break;
            }

            const uint32_t elapsedMs = nowMs - landingStartMs;
            if (elapsedMs >= LANDING_STOP_TIMEOUT_MS) {
                Serial.println("Landing timeout elapsed; stopping motors");
                setState(IDLE);
                break;
            }

            const float progress = min((float)elapsedMs / (float)LANDING_DURATION_MS, 1.0f);
            const int holdThrottle = min(landingInitialThrottle, LANDING_HOLD_THROTTLE_OFFSET_US);
            throttleDesired = constrain(
                (int)((1.0f - progress) * landingInitialThrottle + progress * holdThrottle),
                0,
                THROTTLE_MAX_OFFSET_US);
            rollDesired = 0.0f;
            pitchDesired = 0.0f;
            yawDesired = 0.0f;
            break;
        }
    }
}
