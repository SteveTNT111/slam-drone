#include "Odometry.h"
#include "Encoder.h"
#include "Steering_Engine.h"
#include "esp_timer.h"
#include <math.h>

/*============================================================
 *  里程计实现 (阿克曼转向模型)
 *
 *  小车结构: 舵机转向(前轮) + 后驱两个编码电机
 *
 *  推算公式 (阿克曼模型):
 *    dL, dR      = 左右编码器脉冲增量
 *    dc          = (dL + dR) / 2 * MM_PER_TICK * 标定系数  中心位移(mm)
 *    steer_deg   = (舵机角度 - 145) * 转向比              前轮转向角(度)
 *                 (145 = 红外巡线公式中心: 110+20*70/40)
 *    steer_rad   = steer_deg * PI / 180
 *    dtheta      = -dc * tan(steer_rad) / 轴距              转角增量(rad)
 *    theta      += dtheta
 *    x          += dc * cos(theta)
 *    y          += dc * sin(theta)
 *
 *  标定方法 (3个参数, 互不干扰):
 *    ODOM_DIST_SCALE: 跑直线A->B(150cm), 看网页终点Y
 *       系数 = 150 / (终点Y - 200)
 *       例: 终点Y=340 -> 系数=150/140=1.071
 *    STEER_RATIO: 跑半圆B->C, 终点偏左就减小, 偏右就增大
 *    AXLE_BASE_MM: 量前后轴间距, 调大=转弯更快
 *
 *  初始朝向: theta = PI/2 (+Y方向, A->B)
 *    cos(PI/2)=0 -> x不变, sin(PI/2)=1 -> y增  (A->B直线正确)
 *============================================================*/

#define PI              3.14159265358979f
#define WHEEL_CIRC_MM   (PI * WHEEL_DIA_MM)                  /* 轮周长 = 267.0mm */
#define MM_PER_TICK     (WHEEL_CIRC_MM / TICKS_PER_REV)      /* 每脉冲 0.01822mm */

#define TRACK_LEFT_X_MM      1500.0f
#define TRACK_RIGHT_X_MM     3000.0f
#define TRACK_CENTER_X_MM    2250.0f
#define TRACK_BOTTOM_Y_MM    2000.0f
#define TRACK_TOP_Y_MM       3500.0f
#define TRACK_RADIUS_MM       750.0f
#define TRACK_STRAIGHT_MM    1500.0f
#define TRACK_ARC_MM         (PI * TRACK_RADIUS_MM)
#define TRACK_BC_START_MM    TRACK_STRAIGHT_MM
#define TRACK_CD_START_MM    (TRACK_BC_START_MM + TRACK_ARC_MM)
#define TRACK_DA_START_MM    (TRACK_CD_START_MM + TRACK_STRAIGHT_MM)
#define TRACK_TOTAL_MM       (TRACK_DA_START_MM + TRACK_ARC_MM)

static float s_x = ODOM_INIT_X_MM;       /* x 坐标 mm, A点起始 1500 */
static float s_y = ODOM_INIT_Y_MM;       /* y 坐标 mm, A点起始 2000 */
static float s_theta = ODOM_INIT_THETA;  /* 朝向 rad, 初始 PI/2 (朝+Y, A->B) */
static float s_distance = 0.0f;          /* 累计行驶距离 mm */
static float s_speed_cm_100 = 0.0f;      /* 瞬时速度 cm/s * 100 */
static int   s_last_left = 0;            /* 上次左编码器值 */
static int   s_last_right = 0;           /* 上次右编码器值 */
static int64_t s_last_speed_us = 0;      /* 上次测速时刻 */

typedef struct {
    float x;
    float y;
    float yaw_deg;
    float progress;
} track_pose_t;

static float wrap_progress(float s)
{
    while (s >= TRACK_TOTAL_MM) {
        s -= TRACK_TOTAL_MM;
    }
    while (s < 0.0f) {
        s += TRACK_TOTAL_MM;
    }
    return s;
}

static track_pose_t track_pose_from_progress(float progress_mm)
{
    float s = wrap_progress(progress_mm);
    track_pose_t pose = {
        .x = TRACK_LEFT_X_MM,
        .y = TRACK_BOTTOM_Y_MM,
        .yaw_deg = 0.0f,
        .progress = s,
    };

    if (s < TRACK_BC_START_MM) {
        pose.y = TRACK_BOTTOM_Y_MM + s;
        return pose;
    }

    if (s < TRACK_CD_START_MM) {
        float u = (s - TRACK_BC_START_MM) / TRACK_RADIUS_MM;
        float theta = PI - u;
        pose.x = TRACK_CENTER_X_MM + TRACK_RADIUS_MM * cosf(theta);
        pose.y = TRACK_TOP_Y_MM + TRACK_RADIUS_MM * sinf(theta);
        pose.yaw_deg = u * 180.0f / PI;
        return pose;
    }

    if (s < TRACK_DA_START_MM) {
        float d = s - TRACK_CD_START_MM;
        pose.x = TRACK_RIGHT_X_MM;
        pose.y = TRACK_TOP_Y_MM - d;
        pose.yaw_deg = 180.0f;
        return pose;
    }

    float u = (s - TRACK_DA_START_MM) / TRACK_RADIUS_MM;
    float theta = -u;
    pose.x = TRACK_CENTER_X_MM + TRACK_RADIUS_MM * cosf(theta);
    pose.y = TRACK_BOTTOM_Y_MM + TRACK_RADIUS_MM * sinf(theta);
    pose.yaw_deg = 180.0f + u * 180.0f / PI;
    if (pose.yaw_deg >= 360.0f) {
        pose.yaw_deg -= 360.0f;
    }
    return pose;
}

void Odometry_Init(void)
{
    Odometry_Reset();
}

void Odometry_Reset(void)
{
    s_x = ODOM_INIT_X_MM;       /* A点起始坐标 1500mm */
    s_y = ODOM_INIT_Y_MM;       /* A点起始坐标 2000mm */
    s_theta = ODOM_INIT_THETA;  /* 朝向 +Y (A->B) */
    s_distance = 0.0f;          /* 行驶距离清零 */
    s_last_speed_us = 0;        /* 重置测速时刻 */
    s_speed_cm_100 = 0.0f;      /* 速度清零 */
    Encoder_ClearAll();
    s_last_left  = Encoder_GetLeft();
    s_last_right = Encoder_GetRight();
}

void Odometry_Update(void)
{
    int left  = Encoder_GetLeft();
    int right = Encoder_GetRight();
    int dL = left  - s_last_left;
    int dR = right - s_last_right;
    s_last_left  = left;
    s_last_right = right;

    /* 中心位移(mm): 取左右轮平均值, 并施加标定系数 */
    float dc = (dL + dR) * 0.5f * MM_PER_TICK * ODOM_DIST_SCALE;

    /* 瞬时速度: 中心位移 / 时间间隔, 结果 ×100 (cm/s*100) */
    int64_t now_us = esp_timer_get_time();
    if (s_last_speed_us != 0) {
        float dt_s = (now_us - s_last_speed_us) / 1000000.0f;
        if (dt_s > 0.001f) {
            s_speed_cm_100 = (dc / 10.0f) / dt_s;   /* mm→cm 除以dt得到cm/s, 保留原始值 */
        }
    }
    s_last_speed_us = now_us;

    /* 阿克曼转向: 用舵机角度算转角, 不再用轮速差 */
    float servo_angle = (float)Steering_GetAngle();
    float steer_deg = (servo_angle - SERVO_CENTER_ANGLE) * STEER_RATIO;
    /* 死区滤波: 小角度视为直行, 消除直线段舵机微调引起的X抖动 */
    if (steer_deg > -STEER_DEADBAND_DEG && steer_deg < STEER_DEADBAND_DEG) {
        steer_deg = 0.0f;
    }
    float steer_rad = steer_deg * PI / 180.0f;
    float dtheta = -dc * tanf(steer_rad) / AXLE_BASE_MM;

    s_theta += dtheta;
    s_x += dc * cosf(s_theta);
    s_y += dc * sinf(s_theta);
    s_distance += fabsf(dc);    /* 累计行驶距离(mm), 取绝对值 */
}

int32_t Odometry_GetX(void)     { return (int32_t)s_x; }
int32_t Odometry_GetY(void)     { return (int32_t)s_y; }
int32_t Odometry_GetDistance(void) { return (int32_t)s_distance; }
float   Odometry_GetTheta(void) { return s_theta; }

int32_t Odometry_GetTrackX(void)
{
    float s = s_distance;
    if (s >= TRACK_TOTAL_MM) s = TRACK_TOTAL_MM - 1; /* 钳制, 防止回绕 */
    return (int32_t)track_pose_from_progress(s).x;
}

int32_t Odometry_GetTrackY(void)
{
    float s = s_distance;
    if (s >= TRACK_TOTAL_MM) s = TRACK_TOTAL_MM - 1; /* 钳制, 防止回绕 */
    return (int32_t)track_pose_from_progress(s).y;
}

int32_t Odometry_GetTrackYawDeg(void)
{
    return (int32_t)track_pose_from_progress(s_distance).yaw_deg;
}

int32_t Odometry_GetTrackProgress(void)
{
    return (int32_t)wrap_progress(s_distance);
}

int32_t Odometry_GetTrackTotal(void)
{
    return (int32_t)TRACK_TOTAL_MM;
}

int Odometry_IsLapComplete(void)
{
    /* 加 20% 安全余量: 防止标定误差/打滑导致 s_distance 偏大时提前触发。
     * 用户可从串口 >> LAP COMPLETE dist=xxx total=7712 日志中读出实际 dist,
     * 若 dist 明显大于 7712, 说明 SCALE 偏大, 需调小 ODOM_DIST_SCALE。
     * 标定准确后可把 1.2f 改回 1.0f。 */
    return s_distance >= TRACK_TOTAL_MM * 1.2f;
}

int32_t Odometry_GetSpeedCmS(void)
{
    return (int32_t)s_speed_cm_100;
}
