#include "stm32f10x.h"                  // Device header
#include "PID.h"
#include "task.h"
#include "app_MPU6050.h"
#include "qmath.h"
#include "delay.h"
#include "app_motor.h"
#include "app_encoder.h"
#include "app_kalman.h"

//姿态反馈源选择：1 = 卡尔曼滤波，0 = 原来的互补滤波
//（main 里两种滤波是并行跑的，改这一个宏就能做 A/B 对比）
#define CONTROL_USE_KALMAN   1

static PID_TypeDef pid_velocity;//速度环的PID控制器
static PID_TypeDef pid_theta;//θ环的PID控制器
static PID_TypeDef pid_theta_dot;//θ点环的PID控制器
static PID_TypeDef pid_turn;//转向环的PID控制器

static const float g = 9.8;//重力加速度,单位m/s^2
static const float lp = 0.062;//平衡车中心到电池中心的距离， 单位m
static const float rw = 0.032;//轮胎的半径，单位m
static float omega_ref = 0.0f;
static uint64_t lastTime = 0;
//
//@简介：负责初始化平衡车控制系统
//
void App_Control_Init(void)
{
	PID_Init(&pid_velocity, 10.0f,1.0f,0.0f);//初始化速度环的PID
	PID_LimitConfig(&pid_velocity, +0.5 * g, -0.5 * g);//将速度环的PID控制器输出限制在+-0.5g之间
	
	PID_Init(&pid_theta, 4.0f,0.0f,0.0f);//初始化θ环的PID
	PID_LimitConfig(&pid_theta, +12.57f, -12.57);//将θ环的PID控制器输出限制在+-4Π rad/s之间
	
	PID_Init(&pid_theta_dot, 10.0f,10.0f,0.0f);//初始化θ点环的PID
	PID_LimitConfig(&pid_theta_dot, +125.7f, -125.7f);//将θ点环的PID控制器输出限制在+-40Π rad/s^2之间
	
	PID_Init(&pid_turn, 1.0f,0.0f,0.0f);//初始化转向环的PID
	PID_LimitConfig(&pid_turn, +15.0f, -15.0f);//将转向环的PID控制器输出限制在+-15.0之间
}
	
//
//@简介：平衡车控制系统的进程函数，在while循环中调用
//
void App_Control_Proc(void)
{
	PERIODIC(5)//控制程序，每5ms执行一次
	
	uint64_t now = GetUs();
	float deltaT = (now - lastTime) * 1.0e-6f;
	
	//#1. 改变速度环的设定值
	//PID_ChangeSP(&pid_velocity, 0.0f);
	
	//#2. 读取传感器的值
	float omega = 0.5 * (App_Encoder_GetSpeed_L() + App_Encoder_GetSpeed_R());
#if CONTROL_USE_KALMAN
	//卡尔曼滤波：角度由"陀螺积分 + 加速度计校正"融合，
	//角速度已扣掉标定零偏和滤波器估计的 bias
	float theta     = App_Kalman_GetPitch() * 0.0174533f;//单位rad
	float theta_dot = App_Kalman_GetPitchRate() * 0.0174533f;//单位rad/s
#else
	float theta     = App_MPU6050_GetPitch() * 0.0174533f;//单位rad
	float theta_dot = App_MPU6050_GetGx() * 0.0174533f;//单位rad/s
#endif
	
	//#3. 计算速度环的反馈值x_dot
	float omega2 = -theta_dot * (lp + rw) / rw;
	float omega1 = omega - omega2;
	float x_dot = omega1 * rw;
	
	//#4. 计算速度环PID的输出
	float theta_ref = qatan(PID_Compute(&pid_velocity, x_dot) / g);
	
	//#5. 设定倒立摆外环的设定值SP
	PID_ChangeSP(&pid_theta, theta_ref);
	
	//#6. 计算倒立摆外环PID的输出
	float theta_dot_ref = PID_Compute(&pid_theta, theta);
	
	//#7. 改变倒立摆内环的设定值SP
	PID_ChangeSP(&pid_theta_dot, theta_dot_ref);
	
	//#8. 计算倒立摆内环PID的输出
	float theta_dot_dot_ref = PID_Compute(&pid_theta_dot, theta_dot);
	
	//#9. 倒立摆的逆解算
	float x_dot_dot_ref = (g * qsin(theta) - theta_dot_dot_ref * lp) / qcos(theta);
	
	//#10. 计算轮胎转速
	if(lastTime != 0 )
	{
		omega_ref += 1.0f / rw * x_dot_dot_ref * deltaT;
	}
	
	//转向环
#if CONTROL_USE_KALMAN
	float gz = App_Kalman_GetYawRate() * 0.0174533f;//已扣掉 z 轴零偏，静止时不抖不缓慢自转
#else
	float gz = App_MPU6050_GetGz() * 0.0174533f;
#endif
	float omega_diff = PID_Compute(&pid_turn, gz);
	
	
	//#11. 设置轮胎转速
	App_Motor_SetOmega_L(omega_ref + omega_diff);
	App_Motor_SetOmega_R(omega_ref - omega_diff);
	
	lastTime = now;
}
	
//
//@简介：对控制系统进行复位
//
void App_Control_Reset(void)
{
	//#1. 复位暂存的值
	lastTime = 0;
	omega_ref = 0;
	
	//#2. 复位PID控制器
	PID_Reset(&pid_velocity);
	PID_Reset(&pid_theta);
	PID_Reset(&pid_theta_dot);
}
	
//
// @简介：改变平衡车移动的速度
// @参数：MoveSpeed - 速度，单位m/s，最大行进速度0.7m/s
//
void App_Control_SetMoveSpeed(float MoveSpeed)
{
	PID_ChangeSP(&pid_velocity, MoveSpeed);
}

//
// @简介：改变平衡车转向的速度
// @参数：TurnSpeed - 转向速度，单位是rad/s，最大转速15rad/s
//
void App_Control_SetTurnSpeed(float TurnSpeed)
{
	PID_ChangeSP(&pid_turn, TurnSpeed);
}
