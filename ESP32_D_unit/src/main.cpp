#include <Arduino.h>
#include <WiFi.h>
#include <esp_err.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <cstring>

#include "UART.h"

namespace {

constexpr uint8_t ESPNOW_CHANNEL = 6;
constexpr size_t ESPNOW_MAX_TEXT_LENGTH = Uart::MAX_LINE_LENGTH;
constexpr size_t ESPNOW_RX_QUEUE_DEPTH = 8;
constexpr size_t ESPNOW_TX_QUEUE_DEPTH = 16;
constexpr size_t ESPNOW_RESULT_QUEUE_DEPTH = 4;
constexpr uint8_t STATUS_LED_PIN = 2;
constexpr uint32_t LINK_ACTIVE_TIMEOUT_MS = 5000;
constexpr uint32_t LED_WAITING_TOGGLE_MS = 500;
constexpr uint32_t LED_ERROR_TOGGLE_MS = 150;

constexpr uint8_t BROADCAST_ADDRESS[] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};

struct ReceivedMessage {
    uint8_t sourceMac[6];
    uint16_t originalLength;
    bool tooLong;
    char data[ESPNOW_MAX_TEXT_LENGTH + 1];
};

struct TransmitMessage {
    char data[ESPNOW_MAX_TEXT_LENGTH + 1];
};

QueueHandle_t receiveQueue = nullptr;
QueueHandle_t transmitQueue = nullptr;
QueueHandle_t sendResultQueue = nullptr;

bool espNowReady = false;
bool transmitInFlight = false;
TransmitMessage activeTransmission = {};
bool hasReceivedWirelessData = false;
uint32_t lastWirelessReceiveMs = 0;

portMUX_TYPE statisticsMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t droppedReceiveMessages = 0;
uint32_t droppedSendResults = 0;

void incrementCounter(uint32_t &counter) {
    portENTER_CRITICAL(&statisticsMux);
    ++counter;
    portEXIT_CRITICAL(&statisticsMux);
}

uint32_t takeCounter(uint32_t &counter) {
    portENTER_CRITICAL(&statisticsMux);
    const uint32_t value = counter;
    counter = 0;
    portEXIT_CRITICAL(&statisticsMux);
    return value;
}

bool wirelessLinkActive() {
    return hasReceivedWirelessData &&
           millis() - lastWirelessReceiveMs <= LINK_ACTIVE_TIMEOUT_MS;
}

void updateStatusLed() {
    const uint32_t now = millis();
    bool ledOn = false;

    if (!espNowReady) {
        ledOn = ((now / LED_ERROR_TOGGLE_MS) % 2U) == 0U;
    } else if (wirelessLinkActive()) {
        ledOn = true;
    } else {
        ledOn = ((now / LED_WAITING_TOGGLE_MS) % 2U) == 0U;
    }

    digitalWrite(STATUS_LED_PIN, ledOn ? HIGH : LOW);
}

void onEspNowReceive(const uint8_t *sourceMac, const uint8_t *data,
                     const int dataLength) {
    if (receiveQueue == nullptr || sourceMac == nullptr || data == nullptr ||
        dataLength <= 0) {
        return;
    }

    ReceivedMessage message = {};
    memcpy(message.sourceMac, sourceMac, sizeof(message.sourceMac));
    message.originalLength = static_cast<uint16_t>(dataLength);
    message.tooLong =
        static_cast<size_t>(dataLength) > ESPNOW_MAX_TEXT_LENGTH;

    if (!message.tooLong) {
        memcpy(message.data, data, static_cast<size_t>(dataLength));
        message.data[dataLength] = '\0';
    }

    if (xQueueSend(receiveQueue, &message, 0) != pdTRUE) {
        incrementCounter(droppedReceiveMessages);
    }
}

void onEspNowSend(const uint8_t *destinationMac,
                  const esp_now_send_status_t status) {
    (void)destinationMac;

    if (sendResultQueue == nullptr ||
        xQueueSend(sendResultQueue, &status, 0) != pdTRUE) {
        incrementCounter(droppedSendResults);
    }
}

bool createQueues() {
    receiveQueue =
        xQueueCreate(ESPNOW_RX_QUEUE_DEPTH, sizeof(ReceivedMessage));
    transmitQueue =
        xQueueCreate(ESPNOW_TX_QUEUE_DEPTH, sizeof(TransmitMessage));
    sendResultQueue = xQueueCreate(ESPNOW_RESULT_QUEUE_DEPTH,
                                   sizeof(esp_now_send_status_t));

    if (receiveQueue == nullptr || transmitQueue == nullptr ||
        sendResultQueue == nullptr) {
        Uart::writeLine("ERROR QUEUE_CREATE_FAILED");
        return false;
    }

    return true;
}

bool initializeEspNow() {
    if (!WiFi.mode(WIFI_STA)) {
        Uart::writeLine("ERROR WIFI_STA_MODE_FAILED");
        return false;
    }

    WiFi.disconnect(false, false);

    esp_err_t result =
        esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (result != ESP_OK) {
        Uart::printfLine("ERROR WIFI_CHANNEL_SET_FAILED code=%d name=%s",
                         static_cast<int>(result), esp_err_to_name(result));
        return false;
    }

    uint8_t actualChannel = 0;
    wifi_second_chan_t secondaryChannel = WIFI_SECOND_CHAN_NONE;
    result = esp_wifi_get_channel(&actualChannel, &secondaryChannel);
    if (result != ESP_OK || actualChannel != ESPNOW_CHANNEL) {
        Uart::printfLine(
            "ERROR WIFI_CHANNEL_VERIFY_FAILED expected=%u actual=%u code=%d",
            static_cast<unsigned>(ESPNOW_CHANNEL),
            static_cast<unsigned>(actualChannel), static_cast<int>(result));
        return false;
    }

    result = esp_now_init();
    if (result != ESP_OK) {
        Uart::printfLine("ERROR ESPNOW_INIT_FAILED code=%d name=%s",
                         static_cast<int>(result), esp_err_to_name(result));
        return false;
    }

    result = esp_now_register_recv_cb(onEspNowReceive);
    if (result != ESP_OK) {
        Uart::printfLine("ERROR ESPNOW_RECV_CB_FAILED code=%d name=%s",
                         static_cast<int>(result), esp_err_to_name(result));
        return false;
    }

    result = esp_now_register_send_cb(onEspNowSend);
    if (result != ESP_OK) {
        Uart::printfLine("ERROR ESPNOW_SEND_CB_FAILED code=%d name=%s",
                         static_cast<int>(result), esp_err_to_name(result));
        return false;
    }

    esp_now_peer_info_t broadcastPeer = {};
    memcpy(broadcastPeer.peer_addr, BROADCAST_ADDRESS,
           sizeof(BROADCAST_ADDRESS));
    broadcastPeer.channel = ESPNOW_CHANNEL;
    broadcastPeer.ifidx = WIFI_IF_STA;
    broadcastPeer.encrypt = false;

    result = esp_now_add_peer(&broadcastPeer);
    if (result != ESP_OK) {
        Uart::printfLine("ERROR ESPNOW_ADD_BROADCAST_PEER_FAILED code=%d name=%s",
                         static_cast<int>(result), esp_err_to_name(result));
        return false;
    }

    return true;
}

void queueBroadcast(const char *text) {
    if (!espNowReady) {
        Uart::writeLine("ERROR ESPNOW_NOT_READY");
        return;
    }

    if (text == nullptr || text[0] == '\0') {
        Uart::writeLine("ERROR ESPNOW_EMPTY_DATA");
        return;
    }

    const size_t length = strlen(text);
    if (length > ESPNOW_MAX_TEXT_LENGTH) {
        Uart::printfLine("ERROR ESPNOW_TX_TOO_LONG max=%u",
                         static_cast<unsigned>(ESPNOW_MAX_TEXT_LENGTH));
        return;
    }

    TransmitMessage message = {};
    memcpy(message.data, text, length + 1);

    if (xQueueSend(transmitQueue, &message, 0) != pdTRUE) {
        Uart::printfLine("ERROR ESPNOW_TX_QUEUE_FULL depth=%u",
                         static_cast<unsigned>(ESPNOW_TX_QUEUE_DEPTH));
    }
}

void processSendResultsAndTransmitQueue() {
    esp_now_send_status_t status = ESP_NOW_SEND_FAIL;
    if (transmitInFlight &&
        xQueueReceive(sendResultQueue, &status, 0) == pdTRUE) {
        Uart::printfLine(status == ESP_NOW_SEND_SUCCESS
                             ? "ESPNOW_TX_OK data=%s"
                             : "ESPNOW_TX_FAIL data=%s",
                         activeTransmission.data);
        transmitInFlight = false;
        activeTransmission.data[0] = '\0';
    }

    if (transmitInFlight || !espNowReady) {
        return;
    }

    if (xQueueReceive(transmitQueue, &activeTransmission, 0) != pdTRUE) {
        return;
    }

    transmitInFlight = true;
    const esp_err_t result =
        esp_now_send(BROADCAST_ADDRESS,
                     reinterpret_cast<const uint8_t *>(activeTransmission.data),
                     strlen(activeTransmission.data));

    if (result != ESP_OK) {
        Uart::printfLine("ESPNOW_TX_FAIL data=%s error=%s code=%d",
                         activeTransmission.data, esp_err_to_name(result),
                         static_cast<int>(result));
        transmitInFlight = false;
        activeTransmission.data[0] = '\0';
    }
}

void processReceivedMessages() {
    ReceivedMessage message = {};
    while (xQueueReceive(receiveQueue, &message, 0) == pdTRUE) {
        hasReceivedWirelessData = true;
        lastWirelessReceiveMs = millis();

        char macText[18] = {};
        snprintf(macText, sizeof(macText), "%02X:%02X:%02X:%02X:%02X:%02X",
                 message.sourceMac[0], message.sourceMac[1],
                 message.sourceMac[2], message.sourceMac[3],
                 message.sourceMac[4], message.sourceMac[5]);

        if (message.tooLong) {
            Uart::printfLine("ERROR ESPNOW_RX_TOO_LONG mac=%s len=%u max=%u",
                             macText,
                             static_cast<unsigned>(message.originalLength),
                             static_cast<unsigned>(ESPNOW_MAX_TEXT_LENGTH));
            continue;
        }

        Uart::printfLine("ESPNOW_RX mac=%s data=%s", macText, message.data);
    }

    const uint32_t receiveDrops = takeCounter(droppedReceiveMessages);
    if (receiveDrops > 0) {
        Uart::printfLine("ERROR ESPNOW_RX_QUEUE_FULL dropped=%lu",
                         static_cast<unsigned long>(receiveDrops));
    }

    const uint32_t resultDrops = takeCounter(droppedSendResults);
    if (resultDrops > 0) {
        Uart::printfLine("ERROR ESPNOW_TX_RESULT_QUEUE_FULL dropped=%lu",
                         static_cast<unsigned long>(resultDrops));
        transmitInFlight = false;
        activeTransmission.data[0] = '\0';
    }
}

void printStatus() {
    Uart::printfLine(
        "STATUS espnow=%s uart=READY channel=%u baud=%lu mode=BROADCAST "
        "link=%s led_pin=%u",
        espNowReady ? "READY" : "NOT_READY",
        static_cast<unsigned>(ESPNOW_CHANNEL),
        static_cast<unsigned long>(Uart::BAUD_RATE),
        wirelessLinkActive() ? "ACTIVE" : "WAITING",
        static_cast<unsigned>(STATUS_LED_PIN));
}

void printHelp() {
    Uart::writeLine("HELP SEND <text> | <text> | STATUS | HELP");
}

void processUartCommand(const char *command) {
    if (command == nullptr || command[0] == '\0') {
        return;
    }

    Uart::printfLine("UART_RX %s", command);

    if (strcmp(command, "STATUS") == 0) {
        printStatus();
        return;
    }

    if (strcmp(command, "HELP") == 0) {
        printHelp();
        return;
    }

    constexpr char SEND_PREFIX[] = "SEND ";
    if (strncmp(command, SEND_PREFIX, sizeof(SEND_PREFIX) - 1) == 0) {
        const char *payload = command + sizeof(SEND_PREFIX) - 1;
        if (payload[0] == '\0') {
            Uart::writeLine("ERROR SEND_EMPTY_DATA");
            return;
        }

        queueBroadcast(payload);
        return;
    }

    queueBroadcast(command);
}

}  // namespace

void setup() {
    Uart::begin();
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    const bool queuesReady = createQueues();
    espNowReady = queuesReady && initializeEspNow();

    Uart::printfLine(
        "BOOT channel=%u baud=%lu mode=BROADCAST espnow=%s led_pin=%u",
                     static_cast<unsigned>(ESPNOW_CHANNEL),
                     static_cast<unsigned long>(Uart::BAUD_RATE),
                     espNowReady ? "READY" : "NOT_READY",
                     static_cast<unsigned>(STATUS_LED_PIN));

}

void loop() {
    Uart::poll();

    char command[Uart::MAX_LINE_LENGTH + 1] = {};
    while (Uart::readLine(command, sizeof(command))) {
        processUartCommand(command);
    }

    processReceivedMessages();
    processSendResultsAndTransmitQueue();
    updateStatusLed();
    delay(1);
}
