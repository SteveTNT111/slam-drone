#ifndef __ODOMETRY_H_
#define __ODOMETRY_H_

#include <stdint.h>

/*============================================================
 *  差速小车里程计 (基于左右编码器推算平面坐标)
 *
 *  电机: JGB37-520 霍尔编码器
 *  减速比: 1:333
 *  车轮直径: 8.5cm
 *
 *  4倍频正交解码: 输出轴每转脉冲数 = PPR * 4 * 减速比
 *============================================================*/

/* ---- 里程计参数 (WHEEL_BASE_MM 需实测标定) ---- */
#define ENCODER_PPR       11        /* JGB37-520 霍尔编码器每转脉冲数 */
#define GEAR_RATIO        29        /* 实际减速比 1:29 (原333有误, 由标定反推: 333/11.54≈28.9) */
#define QUAD              4         /* 4倍频正交解码 */
#define TICKS_PER_REV     (ENCODER_PPR * QUAD * GEAR_RATIO)  /* 输出轴每转脉冲数 = 1276 */
#define WHEEL_DIA_MM      85        /* 车轮直径 8.5cm */
#define WHEEL_BASE_MM     170       /* 轮距(mm), 左右后轮间距 */

/* ---- 阿克曼转向参数 (舵机转向模型) ---- */
#define AXLE_BASE_MM      145       /* 轴距(mm), 前后轴距离, 用户实测 14.5cm */
#define SERVO_CENTER_ANGLE 145      /* 舵机直行角度 (与红外巡线公式一致:
                                     * pos10=20 -> 110+20*70/40 = 145) */
#define STEER_RATIO       1.0f      /* 舵机角度->前轮转向角比值, 待标定
                                     * tan(steer) = AXLE / R = 145/750 => steer≈11°
                                     * 跑半圆偏左就减小, 偏右就增大 */
#define STEER_DEADBAND_DEG 4.0f     /* 转向死区(度): |steer|<死区 → 视为直行
                                     * 消除直线段舵机微调引起的X坐标抖动
                                     * 红外巡线±2°正常振荡, 3°足够覆盖 */
#define ODOM_DIST_SCALE   1.18f        /* 距离标定系数 (1.0=理论值, 需实测微调)
                                     * 标定方法: 跑A->B直道(150cm), 看网页终点Y
                                     *   系数 = 150 / (终点Y_cm - 200)
                                     * 例: 终点Y=340 -> 150/140=1.071
                                     *     终点Y=350 -> 150/150=1.0 (完美) */

/* 初始位置: A点 (150cm, 200cm), 单位 mm */
#define ODOM_INIT_X_MM    1500    /* A点 x = 150cm */
#define ODOM_INIT_Y_MM    2000    /* A点 y = 200cm */
/* 初始朝向: +Y方向 (A->B), theta=PI/2 表示朝上 */
#define ODOM_INIT_THETA   1.5707963267948966f  /* PI/2 */

/* 初始化里程计(清零) */
void Odometry_Init(void);

/* 清零: 设为A点起始坐标(1500,2000)mm, 清编码器并记录当前值作为基准 */
void Odometry_Reset(void);

/* 更新里程计: 读编码器增量, 推算 x/y/theta
 * 需在主循环中周期调用(建议每次巡线后调用一次) */
void Odometry_Update(void);

/* 获取坐标 (单位 mm) */
int32_t Odometry_GetX(void);
int32_t Odometry_GetY(void);

/* Get display pose projected onto the known NUEDC D track (unit: mm). */
int32_t Odometry_GetTrackX(void);
int32_t Odometry_GetTrackY(void);
int32_t Odometry_GetTrackYawDeg(void);
int32_t Odometry_GetTrackProgress(void);
int32_t Odometry_GetTrackTotal(void);
int     Odometry_IsLapComplete(void);

/* 获取累计行驶距离 (单位 mm), 从启动/复位开始计算 */
int32_t Odometry_GetDistance(void);

/* 获取当前瞬时速度 (单位 cm/s * 100, 例: 150.50cm/s → 15050) */
int32_t Odometry_GetSpeedCmS(void);

/* 获取朝向 (单位 弧度) */
float Odometry_GetTheta(void);

#endif
