#include <Arduino.h>

#include "hardware_config.h"
#include "motors.h"
#include "pid.h"
#include "states.h"

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

class EscOutput {
public:
    void attach(int pin, int channel) {
        pin_ = pin;
        channel_ = channel;
        pinMode(pin_, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcAttach(pin_, ESC_PWM_HZ, ESC_PWM_BITS);
#else
        ledcSetup(channel_, ESC_PWM_HZ, ESC_PWM_BITS);
        ledcAttachPin(pin_, channel_);
#endif
        writeMicroseconds(PWM_OFF_US);
    }

    void writeMicroseconds(int microseconds) const {
        microseconds = constrain(microseconds, PWM_OFF_US, PWM_MAX_US);
        const uint32_t maxDuty = (1UL << ESC_PWM_BITS) - 1UL;
        const uint32_t duty = (uint32_t)((uint64_t)microseconds * ESC_PWM_HZ * maxDuty / 1000000ULL);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcWrite(pin_, duty);
#else
        ledcWrite(channel_, duty);
#endif
    }

private:
    int pin_ = -1;
    int channel_ = -1;
};

static constexpr int MOTOR_PINS[4] = {
    MOTOR_FRONT_RIGHT_PIN,
    MOTOR_BACK_RIGHT_PIN,
    MOTOR_BACK_LEFT_PIN,
    MOTOR_FRONT_LEFT_PIN,
};

static EscOutput escs[4];

int throttleDesired = 0;

void initMotors() {
    for (int i = 0; i < 4; i++) {
        escs[i].attach(MOTOR_PINS[i], i);
    }
    delay(3000);
}

void stopMotors() {
    for (const EscOutput &esc : escs) {
        esc.writeMicroseconds(PWM_OFF_US);
    }
}

void mixMotors() {
    if (state == IDLE) {
        stopMotors();
        return;
    }

    const int base = PWM_OFF_US + throttleDesired;
    int motorOutputs[4];
    motorOutputs[0] = (int)(base + rollPid - pitchPid + yawPid);
    motorOutputs[1] = (int)(base + rollPid + pitchPid - yawPid);
    motorOutputs[2] = (int)(base - rollPid + pitchPid + yawPid);
    motorOutputs[3] = (int)(base - rollPid - pitchPid - yawPid);

    for (int i = 0; i < 4; i++) {
        escs[i].writeMicroseconds(constrain(motorOutputs[i], PWM_OFF_US, PWM_MAX_US));
    }
}
