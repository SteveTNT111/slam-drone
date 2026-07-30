#pragma once

#include <stdint.h>

namespace LocalServo {

constexpr uint8_t GPIO_PIN = 18;
constexpr int CLOSED_ANGLE = 0;
constexpr int OPEN_ANGLE = 90;
constexpr int MIN_PULSE_WIDTH_US = 500;
constexpr int MAX_PULSE_WIDTH_US = 2400;

enum class State : uint8_t {
    Closed,
    Open,
};

bool begin();
bool open();
bool close();
bool toggle();
bool ready();
State state();
const char *stateName();

}  // namespace LocalServo
