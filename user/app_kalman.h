#ifndef APP_KALMAN_H
#define APP_KALMAN_H
#include "stm32f10x.h"                  // Device header

/*
 * 卡尔曼滤波姿态融合（独立模块，不修改 app_MPU6050.c）
 * ------------------------------------------------------------
 * 上层用法：在 main 的 while(1) 里 App_MPU6050_Proc() 之后调用
 *           App_Kalman_Proc()，然后读 App_Kalman_GetPitch() 等。
 *
 * 数据来源：全部通过 App_MPU6050_GetAx/Ay/Az/Gx/Gy/Gz() 获取，
 *           不直接访问驱动的任何内部变量，也不改动互补滤波。
 *
 * 滤波器状态取 [角度, 陀螺零偏] 两个量：
 *   angle(k) = angle(k-1) + dt * ( rate(k) - bias(k-1) )
 *   bias(k)  = bias(k-1)
 *   z(k)     = angle(k) + v        （v 为加速度计解算角度的噪声）
 *
 * 相比互补滤波（固定权重 0.95238），卡尔曼的权重由协方差矩阵 P
 * 实时算出，并且附带把陀螺零偏估出来，抑制积分漂移。
 * ------------------------------------------------------------
 */

/* 过程噪声：角度。越大越"相信陀螺"，跟随快但噪声大 */
#ifndef KALMAN_Q_ANGLE
#define KALMAN_Q_ANGLE      0.001f
#endif

/* 过程噪声：零偏。越大零偏收敛越快，但易把真实运动误判成零偏 */
#ifndef KALMAN_Q_BIAS
#define KALMAN_Q_BIAS       0.003f
#endif

/* 量测噪声：加速度计解算角度的方差，单位 (°)²。
 * 建议实测：车静止时记录加速度计解算角的波动，算出标准差 σ，令 R = σ²。
 * 真实 MPU6050 通常落在 0.5 ~ 4 之间。 */
#ifndef KALMAN_R_MEASURE
#define KALMAN_R_MEASURE    1.000f
#endif

/* 加速度模值门限：车体急加速/受撞击时 |a| 明显偏离 1g，此时
 * atan2 解算的角度不可信，只做预测不做校正。1=启用 0=关闭 */
#ifndef KALMAN_ACC_GATE
#define KALMAN_ACC_GATE     0
#endif


typedef struct
{
	float angle;        // 融合后的角度估计，单位 °
	float bias;         // 陀螺零偏估计，单位 °/s
	float rate;         // 去掉零偏后的角速度，单位 °/s

	float P[2][2];      // 状态估计误差协方差矩阵

	float Q_angle;      // 角度过程噪声
	float Q_bias;       // 零偏过程噪声
	float R_measure;    // 量测噪声
} Kalman_t;


/* ---------- 通用卡尔曼算法（可用于任意"角度+零偏"场景） ---------- */
void  App_Kalman_Init(Kalman_t *kf, float initAngle);
void  App_Kalman_Predict(Kalman_t *kf, float rateMeasure, float dt);
float App_Kalman_Correct(Kalman_t *kf, float angleMeasure);
float App_Kalman_Update(Kalman_t *kf, float angleMeasure, float rateMeasure, float dt, uint8_t useMeasure);

/* ---------- 姿态融合层（与 MPU6050 驱动对接） ---------- */
void  App_Kalman_Proc(void);
void  App_Kalman_GyroCalibrate(uint16_t samples);

float App_Kalman_GetPitch(void);
float App_Kalman_GetRoll(void);
float App_Kalman_GetYaw(void);

float App_Kalman_GetPitchBias(void);
float App_Kalman_GetRollBias(void);

/* 已去掉零偏的角速度，可直接给控制环做 D 项和转向反馈。
 * GetPitchRate 减的是"标定零偏 + 滤波器估计的 bias"，比原始 gx 干净；
 * GetYawRate 减的是标定时测到的 gz 零偏（未标定时等于原始 gz）。 */
float App_Kalman_GetPitchRate(void);
float App_Kalman_GetYawRate(void);

#endif
