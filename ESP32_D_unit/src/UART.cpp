#include "UART.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace Uart {
namespace {

constexpr size_t LINE_QUEUE_DEPTH = 8;
constexpr size_t PRINT_BUFFER_LENGTH = 320;

char currentLine[MAX_LINE_LENGTH + 1] = {};
size_t currentLength = 0;
bool discardingLongLine = false;

char lineQueue[LINE_QUEUE_DEPTH][MAX_LINE_LENGTH + 1] = {};
size_t queueHead = 0;
size_t queueTail = 0;
size_t queueCount = 0;

void enqueueCurrentLine() {
    if (currentLength == 0) {
        return;
    }

    currentLine[currentLength] = '\0';

    if (queueCount >= LINE_QUEUE_DEPTH) {
        printfLine("ERROR UART_QUEUE_FULL depth=%u",
                   static_cast<unsigned>(LINE_QUEUE_DEPTH));
        return;
    }

    memcpy(lineQueue[queueTail], currentLine, currentLength + 1);
    queueTail = (queueTail + 1) % LINE_QUEUE_DEPTH;
    ++queueCount;
}

}  // namespace

void begin() {
    Serial.begin(BAUD_RATE, SERIAL_8N1);
}

void poll() {
    while (Serial.available() > 0) {
        const char incoming = static_cast<char>(Serial.read());

        if (incoming == '\r') {
            continue;
        }

        if (incoming == '\n') {
            if (discardingLongLine) {
                printfLine("ERROR UART_LINE_TOO_LONG max=%u",
                           static_cast<unsigned>(MAX_LINE_LENGTH));
            } else {
                enqueueCurrentLine();
            }

            currentLength = 0;
            currentLine[0] = '\0';
            discardingLongLine = false;
            continue;
        }

        if (discardingLongLine) {
            continue;
        }

        if (currentLength >= MAX_LINE_LENGTH) {
            currentLength = 0;
            currentLine[0] = '\0';
            discardingLongLine = true;
            continue;
        }

        currentLine[currentLength++] = incoming;
    }
}

bool readLine(char *output, const size_t outputSize) {
    if (output == nullptr || outputSize == 0 || queueCount == 0) {
        return false;
    }

    strncpy(output, lineQueue[queueHead], outputSize - 1);
    output[outputSize - 1] = '\0';

    queueHead = (queueHead + 1) % LINE_QUEUE_DEPTH;
    --queueCount;
    return true;
}

void writeLine(const char *text) {
    Serial.println(text == nullptr ? "" : text);
}

void printfLine(const char *format, ...) {
    if (format == nullptr) {
        return;
    }

    char output[PRINT_BUFFER_LENGTH] = {};
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(output, sizeof(output), format, arguments);
    va_end(arguments);
    writeLine(output);
}

}  // namespace Uart
