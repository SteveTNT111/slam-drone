#include "Encoder.h"
#include "driver/pulse_cnt.h"
#include <stdio.h>

/*======================================
  编码器驱动: 使用 PCNT 硬件脉冲计数器做正交解码(4倍频)
  4倍频: A/B 两相的每个上升沿和下降沿都计数, 分辨率x4
  方向: 通过另一相的电平自动判断正反转

  正交解码真值表:
    通道A(边沿=A, 电平=B):
      A上升沿, B低 -> 加    A上升沿, B高 -> 减
      A下降沿, B低 -> 减    A下降沿, B高 -> 加
    通道B(边沿=B, 电平=A):
      B上升沿, A高 -> 加    B上升沿, A低 -> 减
      B下降沿, A高 -> 减    B下降沿, A低 -> 加
======================================*/

/* 引脚定义 */
#define ENC_LEFT_A_GPIO     1
#define ENC_LEFT_B_GPIO     2
#define ENC_RIGHT_A_GPIO    21
#define ENC_RIGHT_B_GPIO    47

/* PCNT 配置 */
#define PCNT_HIGH_LIMIT     32767
#define PCNT_LOW_LIMIT      (-32768)
#define PCNT_GLITCH_NS      1000    // 毛刺滤波 1us

static pcnt_unit_handle_t left_unit = NULL;
static pcnt_unit_handle_t right_unit = NULL;

/* 初始化一个编码器的 PCNT 单元 (4倍频正交解码) */
static pcnt_unit_handle_t encoder_init_unit(int a_gpio, int b_gpio)
{
    pcnt_unit_handle_t unit = NULL;

    /* 1. 创建 PCNT 单元, 开启自动累加防止溢出 */
    pcnt_unit_config_t unit_config = {
        .clk_src = PCNT_CLK_SRC_DEFAULT,
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
        .flags = { .accum_count = 1 },  // 自动累加, 32位不会溢出
    };
    if (pcnt_new_unit(&unit_config, &unit) != ESP_OK) {
        printf("Encoder pcnt_new_unit error! (A%d B%d)\r\n", a_gpio, b_gpio);
        return NULL;
    }

    /* 2. 设置毛刺滤波 */
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = PCNT_GLITCH_NS,
    };
    pcnt_unit_set_glitch_filter(unit, &filter_config);

    /* 3. 创建通道A: 边沿=A, 电平=B */
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = a_gpio,
        .level_gpio_num = b_gpio,
    };
    pcnt_channel_handle_t chan_a = NULL;
    if (pcnt_new_channel(unit, &chan_a_config, &chan_a) != ESP_OK) {
        printf("Encoder pcnt_new_channel A error!\r\n");
        return NULL;
    }
    pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,   // A上升沿
        PCNT_CHANNEL_EDGE_ACTION_DECREASE);  // A下降沿
    pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE,   // B高时反转方向
        PCNT_CHANNEL_LEVEL_ACTION_KEEP);     // B低时保持方向

    /* 4. 创建通道B: 边沿=B, 电平=A */
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = b_gpio,
        .level_gpio_num = a_gpio,
    };
    pcnt_channel_handle_t chan_b = NULL;
    if (pcnt_new_channel(unit, &chan_b_config, &chan_b) != ESP_OK) {
        printf("Encoder pcnt_new_channel B error!\r\n");
        return NULL;
    }
    pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,   // B上升沿
        PCNT_CHANNEL_EDGE_ACTION_DECREASE);  // B下降沿
    pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,      // A高时保持方向
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);  // A低时反转方向

    /* 5. 使能 -> 清零 -> 启动 */
    pcnt_unit_enable(unit);
    pcnt_unit_clear_count(unit);
    pcnt_unit_start(unit);

    return unit;
}

void Encoder_Init(void)
{
    left_unit = encoder_init_unit(ENC_LEFT_A_GPIO, ENC_LEFT_B_GPIO);
    right_unit = encoder_init_unit(ENC_RIGHT_A_GPIO, ENC_RIGHT_B_GPIO);
    printf("Encoder_Init OK (L:A%d/B%d  R:A%d/B%d)\r\n",
           ENC_LEFT_A_GPIO, ENC_LEFT_B_GPIO,
           ENC_RIGHT_A_GPIO, ENC_RIGHT_B_GPIO);
}

int Encoder_GetLeft(void)
{
    int count = 0;
    if (left_unit) {
        pcnt_unit_get_count(left_unit, &count);
    }
    return count;
}

int Encoder_GetRight(void)
{
    int count = 0;
    if (right_unit) {
        pcnt_unit_get_count(right_unit, &count);
    }
    return count;
}

void Encoder_ClearLeft(void)
{
    if (left_unit) {
        pcnt_unit_clear_count(left_unit);
    }
}

void Encoder_ClearRight(void)
{
    if (right_unit) {
        pcnt_unit_clear_count(right_unit);
    }
}

void Encoder_ClearAll(void)
{
    Encoder_ClearLeft();
    Encoder_ClearRight();
}
