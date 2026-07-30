#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <FastLED.h>

// WS2812和蜂鸣器引脚定义
#define LED_PIN 5 // 将LED_PIN改为WS2812_PIN并使用GPIO16
#define LASER_PIN 15
#define NUM_LEDS 1 // WS2812 LED的数量
#define COLOR_ORDER GRB // WS2812的颜色顺序
#define CHIPSET WS2812B // LED类型
#define LASER_PIN 17 // 激光笔

// 定义LED数组
CRGB leds[NUM_LEDS];

// 任务句柄
TaskHandle_t serialRecvTaskHandle = NULL;
TaskHandle_t alertTaskHandle = NULL;

// 硬件串口对象
HardwareSerial MySerial(1);

// 定义不同的提示类型
#define ALERT_TYPE_A 1  
#define ALERT_TYPE_B 2  

void alertTask(void *pvParameters) {
  // 初始化FastLED
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(100); // 设置亮度为100%
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW); // 确保初始状态为关闭
  while(1) {
    // 等待来自serialRecvTask的通知
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
       if(ulNotificationValue == ALERT_TYPE_B) {
        digitalWrite(LASER_PIN, HIGH); // 打开激光
        delay(500); // 延时0.5秒
        digitalWrite(LASER_PIN, LOW); // 关闭激光
        delay(500); // 延时0.5秒
      }
   }
}

// 串口接收任务函数
void serialRecvTask(void *parameter) {
  char receivedChar;
  
  while(1) {
    // 检查串口是否有数据
    if(Serial.available() > 0) {
      // 读取一个字符
      receivedChar = Serial.read();
      
   if(receivedChar == 0x01) {
        // 通知声光提示任务，发送类型为ALERT_TYPE_B
        xTaskNotify(alertTaskHandle, ALERT_TYPE_B, eSetValueWithOverwrite);
        // 通过串口发送确认消息
        Serial.println("收到'1'，启动激光");
      }
    }
    
    // 短暂延时，避免占用过多CPU
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

#define LED_PIN 5  // 重新定义LED_PIN为GPIO5

void setup() {
  Serial.begin(115200);

  // 设置 GPIO5 为输出，并置为高电平
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // 创建声光提示任务
  xTaskCreate(
    alertTask,
    "AlertTask",
    2048,
    NULL,
    1,
    &alertTaskHandle
  );

  // 创建串口接收任务
  xTaskCreate(
    serialRecvTask,
    "SerialRecvTask",
    2048,
    NULL,
    1,
    &serialRecvTaskHandle
  );

  Serial.println("系统已启动，等待串口输入...");
}

void loop() {
  // 主循环为空，所有工作都在FreeRTOS任务中完成
}