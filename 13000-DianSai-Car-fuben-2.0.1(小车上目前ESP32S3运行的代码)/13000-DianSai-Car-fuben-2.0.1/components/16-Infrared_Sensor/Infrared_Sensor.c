#include "Infrared_Sensor.h"
#include "Steering_Engine.h"
#include "Pid.h"
#include "Motor.h"
#include "Pid.h"
#include "OLED.h"
#include "driver/gpio.h"
#include <stdio.h>

/*============================================================
 *  引脚定义 (左到右)
 *============================================================*/
static const gpio_num_t ir_pins[5] = 
{
    GPIO_NUM_38,  /* L2 最左 */
    GPIO_NUM_39,  /* L1 */
    GPIO_NUM_40,  /* C  中心 */
    GPIO_NUM_41,  /* R1 */
    GPIO_NUM_42,  /* R2 最右 */
};

/*============================================================
 *  舵机角度参数 (需根据实际安装方向调整)
 *  角度越小越偏左, 越大越偏右
 *============================================================*/
#define SERVO_HARD_LEFT     110     /* 大左转 */
#define SERVO_SLIGHT_LEFT   130     /* 微左转 */
#define SERVO_CENTER        140     /* 直行   */
#define SERVO_SLIGHT_RIGHT  165     /* 微右转 */
#define SERVO_HARD_RIGHT    180     /* 大右转 */

/* 寻线行驶速度 */
#define LINE_FOLLOW_SPEED   18

/* 全白去抖阈值: 连续这么多次全白才判定丢线停车
 * 主循环~1ms/tick, 100≈100ms+; 黑线过传感器间隙的瞬时全白远小于此不停车 */
#define WHITE_LOST_LIMIT    700

/*============================================================
 *  初始化
 *============================================================*/
void Infrared_Init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << GPIO_NUM_38) | (1ULL << GPIO_NUM_39) |
                        (1ULL << GPIO_NUM_40) | (1ULL << GPIO_NUM_41) |
                        (1ULL << GPIO_NUM_42),
        .mode           = GPIO_MODE_INPUT,
        .pull_up_en     = GPIO_PULLUP_ENABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    printf("Infrared_Init OK (GPIO38-42)\r\n");
}

/*============================================================
 *  读取单个传感器
 *  返回 0 = 检测到黑线, 1 = 白线
 *============================================================*/
uint8_t Infrared_Read(uint8_t index)
{
    if (index > 4)
        return 1;
    /* 硬件更换后电平反转: 取反使 0=黑线, 1=白线 保持不变 */
    return (uint8_t)(!gpio_get_level(ir_pins[index]));
}

/*============================================================
 *  读取全部5个传感器
 *  返回5位, bit0=GPIO38 ... bit4=GPIO42
 *  某位为0 = 该路检测到黑线
 *============================================================*/
uint8_t Infrared_ReadAll(void)
{
    uint8_t val = 0;
    for (int i = 0; i < 5; i++) {
        val |= (Infrared_Read(i) << i);
    }
    return val;
}

/*============================================================
 *  获取黑线位置偏移
 *  -2(最左) ~ +2(最右), 0=居中, 0xFF=丢线
 *============================================================*/
int8_t Infrared_GetPosition(void)
{
    int sum = 0, count = 0;

    for (int i = 0; i < 5; i++) {
        if (Infrared_Read(i) == 0) {
            sum   += (i - 2);
            count++;
        }
    }

    if (count == 0)
        return 0xFF;

    return (int8_t)(sum / count);
}

/*============================================================
 *  寻线控制: 读传感器 -> 加权平均算黑线中心 -> 线性插值调舵机
 *
 *  加权平均算黑线中心位置(连续值), 天然支持
 *  "黑线卡在两个传感器之间"(两个相邻传感器同时检测到黑线)的情况,
 *  角度按线性插值平滑过渡, 无跳变。
 *  全黑 -> 立即停车(终点);
 *  全白 -> 去抖: 瞬时(传感器间隙)不停车保持滑行, 持续(丢线)才停车;
 *  找到线 -> 行驶 + 舵机跟踪
 *============================================================*/
void Infrared_LineFollow(void)
{
    static uint16_t white_lost_cnt = 0;  /* 连续全白计数, 用于丢线去抖 */
    uint8_t s[5];
    int sum = 0, black_count = 0;

    for (int i = 0; i < 5; i++)
    {
        s[i] = Infrared_Read(i);     /* 0=黑线, 1=白线 */
        if (s[i] == 0) { sum += i; black_count++; }
    }

    /* 全黑 -> 终点, 立即停车 */
    if (black_count == 5)
    {
        // Motor_Stop();
        PID_SetSpeed(0,0);
        Set_Angle_180(SERVO_CENTER);
        white_lost_cnt = 0;
        return;
    }

    /* 全白 -> 可能是传感器间隙(瞬时)或真正丢线(持续)
     * 去抖: 计数未满则保持上次电机+舵机继续滑行, 不停车;
     *       避免黑线过两个传感器间隙时误触发停车导致"卡住" */
    if (black_count == 0)
    {
        if (white_lost_cnt < WHITE_LOST_LIMIT)
        {
            white_lost_cnt++;
            return;   /* 不停车, 保持上次状态等重新抓线 */
        }
        // Motor_Stop();                 /* 连续超限, 确认丢线停车 */
        PID_SetSpeed(0,0);
        Set_Angle_180(SERVO_CENTER);
        return;
    }
    white_lost_cnt = 0;               /* 检测到黑线, 复位计数 */

    /* 加权平均算黑线中心 pos10: 0=最左  20=中心  40=最右
     * 两个相邻传感器同时为黑线时 pos10 取中间值
     */
    int pos10 = sum * 10 / black_count;

    /* 行驶 + 线性插值映射舵机角度
     * angle = HARD_LEFT + pos10*(HARD_RIGHT-HARD_LEFT)/40
     * pos=0->110  pos=1->128  pos=2->145  pos=3->163  pos=4->180
     * pos=1.5->136(L1/C间)  pos=2.5->154(C/R1间)  连续平滑无跳变
     */
    // Motor_Forward(LINE_FOLLOW_SPEED);
    PID_SetSpeed(LINE_FOLLOW_SPEED+3,LINE_FOLLOW_SPEED);
    int angle = SERVO_HARD_LEFT + pos10 * (SERVO_HARD_RIGHT - SERVO_HARD_LEFT) / 40;
    Set_Angle_180(angle);
}

/*============================================================
 *  OLED 显示5路传感器返回值
 *  显示在第4行:  IR: 0 0 0 0 0
 *  从左到右对应 GPIO38~42, 0=黑线 1=白线
 *============================================================*/
void Infrared_ShowOLED(void)
{
    OLED_ShowString(4, 1, "IR:", BLACK, WHITE);
    OLED_ShowNum(4,  5, Infrared_Read(0), 1, RED, WHITE);
    OLED_ShowNum(4,  7, Infrared_Read(1), 1, RED, WHITE);
    OLED_ShowNum(4,  9, Infrared_Read(2), 1, RED, WHITE);
    OLED_ShowNum(4, 11, Infrared_Read(3), 1, RED, WHITE);
    OLED_ShowNum(4, 13, Infrared_Read(4), 1, RED, WHITE);
}
