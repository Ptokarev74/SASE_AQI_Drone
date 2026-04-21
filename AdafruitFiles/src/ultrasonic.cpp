#include <Arduino.h>
#include <driver/gpio.h>

#include "hardware_config.h"
#include "ultrasonic.h"

static portMUX_TYPE ultrasonicMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t echoStartUs = 0;
static volatile uint32_t echoEndUs = 0;
static volatile uint32_t triggerTimeUs = 0;
static volatile bool echoReceived = false;
static volatile bool measurementInProgress = false;

static float latestUltrasonicCm = 9999.0f;
static bool latestUltrasonicOk = false;

static void IRAM_ATTR ultrasonicEchoISR() {
    const uint32_t nowUs = micros();
    const int level = gpio_get_level((gpio_num_t)ULTRASONIC_ECHO_PIN);

    portENTER_CRITICAL_ISR(&ultrasonicMux);
    if (level == 1) {
        echoStartUs = nowUs;
        measurementInProgress = true;
    } else {
        echoEndUs = nowUs;
        echoReceived = true;
        measurementInProgress = false;
    }
    portEXIT_CRITICAL_ISR(&ultrasonicMux);
}

void initUltrasonic() {
    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    attachInterrupt(digitalPinToInterrupt(ULTRASONIC_ECHO_PIN), ultrasonicEchoISR, CHANGE);
}

void startUltrasonicMeasurement() {
    bool busy = false;
    portENTER_CRITICAL(&ultrasonicMux);
    busy = measurementInProgress;
    if (!busy) {
        echoReceived = false;
        echoStartUs = 0;
        echoEndUs = 0;
        triggerTimeUs = micros();
        measurementInProgress = true;
    }
    portEXIT_CRITICAL(&ultrasonicMux);

    if (busy) {
        return;
    }

    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
}

void updateUltrasonic() {
    static constexpr uint32_t ECHO_TIMEOUT_US = 30000;
    bool ready = false;
    bool timedOut = false;
    uint32_t startUs = 0;
    uint32_t endUs = 0;
    uint32_t ageUs = 0;

    portENTER_CRITICAL(&ultrasonicMux);
    ready = echoReceived;
    startUs = echoStartUs;
    endUs = echoEndUs;
    ageUs = micros() - triggerTimeUs;
    timedOut = measurementInProgress && ageUs > ECHO_TIMEOUT_US;
    if (ready || timedOut) {
        echoReceived = false;
        measurementInProgress = false;
    }
    portEXIT_CRITICAL(&ultrasonicMux);

    if (timedOut) {
        latestUltrasonicCm = 9999.0f;
        latestUltrasonicOk = false;
        return;
    }

    if (!ready || endUs <= startUs) {
        return;
    }

    latestUltrasonicCm = (float)(endUs - startUs) * 0.01715f;
    latestUltrasonicOk = latestUltrasonicCm > 0.0f && latestUltrasonicCm < 500.0f;
}

bool isUltrasonicOk() {
    return latestUltrasonicOk;
}

float getLatestUltrasonicCm() {
    return latestUltrasonicCm;
}
