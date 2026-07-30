#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

namespace Uart {

constexpr uint32_t BAUD_RATE = 115200;
constexpr size_t MAX_LINE_LENGTH = 200;

void begin();
void poll();
bool readLine(char *output, size_t outputSize);
void writeLine(const char *text);
void printfLine(const char *format, ...);

}  // namespace Uart
