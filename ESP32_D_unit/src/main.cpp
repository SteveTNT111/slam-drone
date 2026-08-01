#include <Arduino.h>
#include <WiFi.h>
#include <esp_err.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <cmath>
#include <cstring>

#include "UART.h"

namespace {

constexpr uint8_t ESPNOW_CHANNEL = 6;
constexpr size_t ESPNOW_MAX_PAYLOAD_LENGTH = 250;
constexpr size_t ESPNOW_RX_QUEUE_DEPTH = 12;
constexpr size_t ESPNOW_TX_QUEUE_DEPTH = 16;
constexpr size_t ESPNOW_RESULT_QUEUE_DEPTH = 4;
constexpr uint8_t STATUS_LED_PIN = 2;
constexpr uint32_t LINK_ACTIVE_TIMEOUT_MS = 5000;
constexpr uint32_t LED_WAITING_TOGGLE_MS = 500;
constexpr uint32_t LED_ERROR_TOGGLE_MS = 150;

constexpr uint8_t BROADCAST_ADDRESS[6] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};

// Must match the car firmware components/17-ESPNOW/ESPNOW.c.
constexpr uint8_t DRONE_ADDRESS[6] = {
    0x30, 0xC9, 0x22, 0xEF, 0x21, 0xA0,
};

struct EspNowMessage {
    uint8_t mac[6];
    uint8_t pad[2];
    char kind[8];
    char carDrone[24];
    int32_t x;
    int32_t y;
    int32_t speed;
    int32_t z;
    int32_t yaw;
    uint8_t battery;
    uint8_t status;
    uint8_t pad2[2];
    uint32_t seq;
};

static_assert(sizeof(EspNowMessage) == 68,
              "EspNowMessage must remain wire-compatible (68 bytes)");

struct ReceivedMessage {
    uint8_t sourceMac[6];
    uint16_t length;
    uint8_t data[ESPNOW_MAX_PAYLOAD_LENGTH];
};

struct TransmitMessage {
    uint8_t destinationMac[6];
    uint16_t length;
    char label[40];
    uint8_t data[ESPNOW_MAX_PAYLOAD_LENGTH];
};

QueueHandle_t receiveQueue = nullptr;
QueueHandle_t transmitQueue = nullptr;
QueueHandle_t sendResultQueue = nullptr;

bool espNowReady = false;
bool transmitInFlight = false;
TransmitMessage activeTransmission = {};
bool hasReceivedWirelessData = false;
uint32_t lastWirelessReceiveMs = 0;
uint8_t localMac[6] = {};
uint32_t transmitSequence = 0;

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

void macToText(const uint8_t *mac, char *output, const size_t outputSize) {
    snprintf(output, outputSize, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool parseMac(const char *text, uint8_t output[6]) {
    if (text == nullptr || output == nullptr) {
        return false;
    }
    unsigned int values[6] = {};
    if (sscanf(text, "%2x:%2x:%2x:%2x:%2x:%2x", &values[0], &values[1],
               &values[2], &values[3], &values[4], &values[5]) != 6) {
        return false;
    }
    for (size_t i = 0; i < 6; ++i) {
        output[i] = static_cast<uint8_t>(values[i]);
    }
    return true;
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
        dataLength <= 0 || dataLength > static_cast<int>(ESPNOW_MAX_PAYLOAD_LENGTH)) {
        if (dataLength > static_cast<int>(ESPNOW_MAX_PAYLOAD_LENGTH)) {
            incrementCounter(droppedReceiveMessages);
        }
        return;
    }

    ReceivedMessage message = {};
    memcpy(message.sourceMac, sourceMac, sizeof(message.sourceMac));
    message.length = static_cast<uint16_t>(dataLength);
    memcpy(message.data, data, static_cast<size_t>(dataLength));
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
    receiveQueue = xQueueCreate(ESPNOW_RX_QUEUE_DEPTH, sizeof(ReceivedMessage));
    transmitQueue = xQueueCreate(ESPNOW_TX_QUEUE_DEPTH, sizeof(TransmitMessage));
    sendResultQueue = xQueueCreate(ESPNOW_RESULT_QUEUE_DEPTH,
                                   sizeof(esp_now_send_status_t));
    if (receiveQueue == nullptr || transmitQueue == nullptr ||
        sendResultQueue == nullptr) {
        Uart::writeLine("ERROR QUEUE_CREATE_FAILED");
        return false;
    }
    return true;
}

bool ensurePeer(const uint8_t mac[6]) {
    if (esp_now_is_peer_exist(mac)) {
        return true;
    }
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    const esp_err_t result = esp_now_add_peer(&peer);
    if (result != ESP_OK && result != ESP_ERR_ESPNOW_EXIST) {
        char macText[18] = {};
        macToText(mac, macText, sizeof(macText));
        Uart::printfLine("ERROR ESPNOW_ADD_PEER mac=%s code=%d name=%s", macText,
                         static_cast<int>(result), esp_err_to_name(result));
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
    esp_wifi_set_ps(WIFI_PS_NONE);

    esp_err_t result =
        esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (result != ESP_OK) {
        Uart::printfLine("ERROR WIFI_CHANNEL_SET_FAILED code=%d name=%s",
                         static_cast<int>(result), esp_err_to_name(result));
        return false;
    }
    result = esp_wifi_get_mac(WIFI_IF_STA, localMac);
    if (result != ESP_OK) {
        Uart::printfLine("ERROR WIFI_MAC_READ_FAILED code=%d name=%s",
                         static_cast<int>(result), esp_err_to_name(result));
        return false;
    }
    result = esp_now_init();
    if (result != ESP_OK) {
        Uart::printfLine("ERROR ESPNOW_INIT_FAILED code=%d name=%s",
                         static_cast<int>(result), esp_err_to_name(result));
        return false;
    }
    if (esp_now_register_recv_cb(onEspNowReceive) != ESP_OK ||
        esp_now_register_send_cb(onEspNowSend) != ESP_OK) {
        Uart::writeLine("ERROR ESPNOW_CALLBACK_REGISTER_FAILED");
        return false;
    }
    return ensurePeer(BROADCAST_ADDRESS) && ensurePeer(DRONE_ADDRESS);
}

bool queueRaw(const uint8_t *data, const size_t length,
              const uint8_t destinationMac[6], const char *label) {
    if (!espNowReady) {
        Uart::writeLine("ERROR ESPNOW_NOT_READY");
        return false;
    }
    if (data == nullptr || length == 0 || length > ESPNOW_MAX_PAYLOAD_LENGTH) {
        Uart::writeLine("ERROR ESPNOW_INVALID_TX_LENGTH");
        return false;
    }
    TransmitMessage message = {};
    memcpy(message.destinationMac, destinationMac, 6);
    message.length = static_cast<uint16_t>(length);
    memcpy(message.data, data, length);
    snprintf(message.label, sizeof(message.label), "%s", label == nullptr ? "raw" : label);
    if (xQueueSend(transmitQueue, &message, 0) != pdTRUE) {
        Uart::printfLine("ERROR ESPNOW_TX_QUEUE_FULL depth=%u",
                         static_cast<unsigned>(ESPNOW_TX_QUEUE_DEPTH));
        return false;
    }
    return true;
}

bool queueStructured(EspNowMessage message, const uint8_t destinationMac[6],
                     const char *label) {
    memcpy(message.mac, localMac, sizeof(message.mac));
    message.seq = transmitSequence++;
    return queueRaw(reinterpret_cast<const uint8_t *>(&message), sizeof(message),
                    destinationMac, label);
}

void queueLegacyBroadcast(const char *text) {
    if (text == nullptr || text[0] == '\0') {
        Uart::writeLine("ERROR ESPNOW_EMPTY_DATA");
        return;
    }
    const size_t length = strlen(text);
    if (length > Uart::MAX_LINE_LENGTH) {
        Uart::printfLine("ERROR ESPNOW_TX_TOO_LONG max=%u",
                         static_cast<unsigned>(Uart::MAX_LINE_LENGTH));
        return;
    }
    queueRaw(reinterpret_cast<const uint8_t *>(text), length, BROADCAST_ADDRESS,
             "legacy_text");
}

void sendTaskCommand(const uint8_t task) {
    EspNowMessage message = {};
    snprintf(message.kind, sizeof(message.kind), "Car");
    snprintf(message.carDrone, sizeof(message.carDrone),
             task == 1 ? "Drone_Task1Off" : "Drone_Task2Off");
    queueStructured(message, DRONE_ADDRESS, task == 1 ? "TASK1" : "TASK2");
}

void sendDroneAck(const uint8_t destinationMac[6]) {
    EspNowMessage message = {};
    snprintf(message.kind, sizeof(message.kind), "Drone");
    snprintf(message.carDrone, sizeof(message.carDrone), "Drone_Rec_Cmd_Ok");
    queueStructured(message, destinationMac, "DRONE_ACK");
}

void sendDroneTelemetry(const int32_t x, const int32_t y, const int32_t speed,
                        const int32_t z, const int32_t yaw,
                        const int battery, const int status) {
    EspNowMessage message = {};
    snprintf(message.kind, sizeof(message.kind), "Drone");
    message.x = x;
    message.y = y;
    message.speed = speed;
    message.z = z;
    message.yaw = yaw;
    message.battery = static_cast<uint8_t>(constrain(battery, 0, 100));
    message.status = static_cast<uint8_t>(constrain(status, 0, 255));
    queueStructured(message, BROADCAST_ADDRESS, "DRONE_TELEMETRY");
}

void processSendResultsAndTransmitQueue() {
    esp_now_send_status_t status = ESP_NOW_SEND_FAIL;
    if (transmitInFlight && xQueueReceive(sendResultQueue, &status, 0) == pdTRUE) {
        Uart::printfLine(status == ESP_NOW_SEND_SUCCESS
                             ? "ESPNOW_TX_OK type=%s len=%u"
                             : "ESPNOW_TX_FAIL type=%s len=%u",
                         activeTransmission.label,
                         static_cast<unsigned>(activeTransmission.length));
        transmitInFlight = false;
        activeTransmission = {};
    }
    if (transmitInFlight || !espNowReady) {
        return;
    }
    if (xQueueReceive(transmitQueue, &activeTransmission, 0) != pdTRUE) {
        return;
    }
    if (!ensurePeer(activeTransmission.destinationMac)) {
        activeTransmission = {};
        return;
    }
    transmitInFlight = true;
    const esp_err_t result = esp_now_send(activeTransmission.destinationMac,
                                          activeTransmission.data,
                                          activeTransmission.length);
    if (result != ESP_OK) {
        Uart::printfLine("ESPNOW_TX_FAIL type=%s error=%s code=%d",
                         activeTransmission.label, esp_err_to_name(result),
                         static_cast<int>(result));
        transmitInFlight = false;
        activeTransmission = {};
    }
}

void printCarJson(const EspNowMessage &message) {
    Uart::printfLine(
        "{\"kind\":\"car\",\"x_cm\":%ld,\"y_cm\":%ld,\"speed_cm_s\":%.2f,"
        "\"phase\":2,\"seq\":%lu,\"source\":1}",
        static_cast<long>(message.x), static_cast<long>(message.y),
        message.speed / 100.0f, static_cast<unsigned long>(message.seq));
}

void printDroneJson(const EspNowMessage &message) {
    Uart::printfLine(
        "{\"kind\":\"drone\",\"x_cm\":%ld,\"y_cm\":%ld,\"height_cm\":%ld,"
        "\"yaw_deg\":%.2f,\"horizontal_speed_cm_s\":%.2f,"
        "\"vertical_speed_cm_s\":0,\"target_error_cm\":0,"
        "\"battery_pct\":%u,\"phase\":%u,\"seq\":%lu,\"source\":2}",
        static_cast<long>(message.x), static_cast<long>(message.y),
        static_cast<long>(message.z), message.yaw / 100.0f,
        message.speed / 100.0f, static_cast<unsigned>(message.battery),
        static_cast<unsigned>(message.status),
        static_cast<unsigned long>(message.seq));
}

void processStructuredMessage(const ReceivedMessage &received) {
    EspNowMessage message = {};
    memcpy(&message, received.data, sizeof(message));
    message.kind[sizeof(message.kind) - 1] = '\0';
    message.carDrone[sizeof(message.carDrone) - 1] = '\0';

    char sourceMacText[18] = {};
    macToText(received.sourceMac, sourceMacText, sizeof(sourceMacText));

    if (strcmp(message.kind, "Car") == 0) {
        if (message.carDrone[0] == '\0') {
            printCarJson(message);
            return;
        }
        int task = 0;
        if (strcmp(message.carDrone, "Drone_Task1Off") == 0) {
            task = 1;
        } else if (strcmp(message.carDrone, "Drone_Task2Off") == 0) {
            task = 2;
        }
        Uart::printfLine(
            "{\"kind\":\"command\",\"source\":\"car\",\"src_mac\":\"%s\","
            "\"command\":\"%s\",\"task\":%d,\"seq\":%lu}",
            sourceMacText, message.carDrone, task,
            static_cast<unsigned long>(message.seq));
        return;
    }

    if (strcmp(message.kind, "Drone") == 0) {
        if (message.carDrone[0] == '\0') {
            printDroneJson(message);
        } else {
            Uart::printfLine(
                "{\"kind\":\"status\",\"text\":\"%s\",\"source\":2,"
                "\"src_mac\":\"%s\",\"seq\":%lu}",
                message.carDrone, sourceMacText,
                static_cast<unsigned long>(message.seq));
        }
        return;
    }

    Uart::printfLine("ERROR ESPNOW_UNKNOWN_STRUCT kind=%s len=%u", message.kind,
                     static_cast<unsigned>(received.length));
}

void processReceivedMessages() {
    ReceivedMessage message = {};
    while (xQueueReceive(receiveQueue, &message, 0) == pdTRUE) {
        hasReceivedWirelessData = true;
        lastWirelessReceiveMs = millis();

        if (message.length == sizeof(EspNowMessage)) {
            processStructuredMessage(message);
            continue;
        }

        char macText[18] = {};
        macToText(message.sourceMac, macText, sizeof(macText));
        char text[ESPNOW_MAX_PAYLOAD_LENGTH + 1] = {};
        memcpy(text, message.data, message.length);
        text[message.length] = '\0';
        Uart::printfLine("ESPNOW_RX mac=%s data=%s", macText, text);
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
        activeTransmission = {};
    }
}

void printStatus() {
    char localMacText[18] = {};
    macToText(localMac, localMacText, sizeof(localMacText));
    Uart::printfLine(
        "STATUS espnow=%s uart=READY channel=%u baud=%lu link=%s mac=%s "
        "protocol=NUEDC68",
        espNowReady ? "READY" : "NOT_READY",
        static_cast<unsigned>(ESPNOW_CHANNEL),
        static_cast<unsigned long>(Uart::BAUD_RATE),
        wirelessLinkActive() ? "ACTIVE" : "WAITING", localMacText);
}

void printHelp() {
    Uart::writeLine(
        "HELP TASK1|START1 | TASK2|START2 | DRONE_ACK <MAC> | "
        "DRONE_TELEMETRY <x_cm> <y_cm> <speed_x100> <z_cm> "
        "<yaw_x100> <battery_pct> <phase> | SEND <text> | STATUS");
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
    if (strcmp(command, "TASK1") == 0 || strcmp(command, "START1") == 0) {
        sendTaskCommand(1);
        return;
    }
    if (strcmp(command, "TASK2") == 0 || strcmp(command, "START2") == 0) {
        sendTaskCommand(2);
        return;
    }

    constexpr char ACK_PREFIX[] = "DRONE_ACK ";
    if (strncmp(command, ACK_PREFIX, sizeof(ACK_PREFIX) - 1) == 0) {
        uint8_t destination[6] = {};
        if (!parseMac(command + sizeof(ACK_PREFIX) - 1, destination)) {
            Uart::writeLine("ERROR DRONE_ACK_INVALID_MAC");
            return;
        }
        sendDroneAck(destination);
        return;
    }

    long x = 0;
    long y = 0;
    long speed = 0;
    long z = 0;
    long yaw = 0;
    int battery = 0;
    int status = 0;
    if (sscanf(command, "DRONE_TELEMETRY %ld %ld %ld %ld %ld %d %d",
               &x, &y, &speed, &z, &yaw, &battery, &status) == 7) {
        sendDroneTelemetry(static_cast<int32_t>(x), static_cast<int32_t>(y),
                           static_cast<int32_t>(speed), static_cast<int32_t>(z),
                           static_cast<int32_t>(yaw), battery, status);
        return;
    }
    if (strncmp(command, "DRONE_TELEMETRY", 15) == 0) {
        Uart::writeLine("ERROR DRONE_TELEMETRY_FORMAT");
        return;
    }

    constexpr char SEND_PREFIX[] = "SEND ";
    if (strncmp(command, SEND_PREFIX, sizeof(SEND_PREFIX) - 1) == 0) {
        queueLegacyBroadcast(command + sizeof(SEND_PREFIX) - 1);
        return;
    }
    queueLegacyBroadcast(command);
}

}  // namespace

void setup() {
    Uart::begin();
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    const bool queuesReady = createQueues();
    espNowReady = queuesReady && initializeEspNow();

    char localMacText[18] = {};
    macToText(localMac, localMacText, sizeof(localMacText));
    Uart::printfLine(
        "BOOT channel=%u baud=%lu espnow=%s mac=%s protocol=NUEDC68",
        static_cast<unsigned>(ESPNOW_CHANNEL),
        static_cast<unsigned long>(Uart::BAUD_RATE),
        espNowReady ? "READY" : "NOT_READY", localMacText);
    Uart::writeLine("{\"kind\":\"status\",\"text\":\"ground_online\",\"source\":3}");
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
