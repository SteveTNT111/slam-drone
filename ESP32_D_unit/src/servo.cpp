#include "servo.h"

#include <ESP32Servo.h>

namespace LocalServo {
namespace {

Servo servo;
State currentState = State::Closed;
bool initialized = false;

bool writeAngle(const int angle, const State newState) {
    if (!initialized) {
        return false;
    }

    servo.write(angle);
    currentState = newState;
    return true;
}

}  // namespace

bool begin() {
    servo.setPeriodHertz(50);
    servo.attach(GPIO_PIN, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);
    initialized = servo.attached();

    if (!initialized) {
        return false;
    }

    return writeAngle(CLOSED_ANGLE, State::Closed);
}

bool open() {
    return writeAngle(OPEN_ANGLE, State::Open);
}

bool close() {
    return writeAngle(CLOSED_ANGLE, State::Closed);
}

bool toggle() {
    return currentState == State::Closed ? open() : close();
}

bool ready() {
    return initialized;
}

State state() {
    return currentState;
}

const char *stateName() {
    return currentState == State::Open ? "OPEN" : "CLOSED";
}

}  // namespace LocalServo
