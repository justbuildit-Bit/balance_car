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

//角速度环增益实验开关：验证 1.8Hz 极限环是否由内环增益过高引起
//  0 = 原参数 Kp=10, Ki=10 —— 开环穿越 9.77 rad/s (1.55 Hz)，相位裕度 70.4°
//  1 = 实验参数 Kp=6, Ki=6 —— 开环穿越 6.01 rad/s (0.96 Hz)，相位裕度 72.0°
//  2 = 更低一档 Kp=4, Ki=4 —— 开环穿越 4.10 rad/s (0.65 Hz)，相位裕度 70.4°
//
//注意：Kp 和 Ki 必须按同一比例缩。只降 Kp 会把零点从 1 rad/s 推到 1.67 rad/s，
//      损失相位超前，相位裕度反而从 70.4° 降到 66.1°，实验会得出错误结论。
//
//【实验结论 2026-11-06，桌面 A/B】档位 1 已实测，结论为「否」：
//   穿越频率降了 38%（1.55 → 0.96 Hz），摆动周期却在 29.6 采样点(1.69 Hz)原地不动，
//   与基线 27.6~29.3 采样点完全重叠；而稳态峰峰从 7° 涨到 13°、推力峰峰从 25° 涨到 58°。
//   → 极限环的频率不由角速度环增益决定，降增益只是把系统变软。
//   故默认值回到 0。下一轮用 VOFA 探针通道(ch5 θ_ref / ch6 ẋ)定位真正的激励源。
#define EXP_OMEGA_GAIN_LOW   0

//速度环开环实验开关：θ_ref 强制为 0，把外环摘掉，只留角度环 + 角速度环闭环
//  0 = 正常（速度环工作）
//  1 = 速度环开环 —— 用来判断 1.7Hz 摆动是不是速度环引起的
//      ⚠ 开环后车不再维持位置，会持续朝一个方向漂走，测几秒就扶住
#define EXP_SPEED_LOOP_OPEN  0

static PID_TypeDef pid_velocity;//速度环的PID控制器
static PID_TypeDef pid_theta;//θ环的PID控制器
static PID_TypeDef pid_theta_dot;//θ点环的PID控制器
static PID_TypeDef pid_turn;//转向环的PID控制器

static const float g = 9.8;//重力加速度,单位m/s^2
static const float lp = 0.062;//平衡车中心到电池中心的距离， 单位m
static const float rw = 0.032;//轮胎的半径，单位m
static float omega_ref = 0.0f;
static uint64_t lastTime = 0;

//调试探针：只给 VOFA+ 读，不参与控制运算（写在 Proc 里、读在同一个 while 循环里，无需加锁）
static float probe_theta_ref = 0.0f;//[rad]  速度环输出 = 倒立摆外环的设定值
static float probe_x_dot     = 0.0f;//[m/s]  速度环反馈值
static float probe_theta_dot = 0.0f;//[rad/s]角速度环反馈值
//
//@简介：负责初始化平衡车控制系统
//
void App_Control_Init(void)
{
	PID_Init(&pid_velocity, 10.0f,1.0f,0.0f);//初始化速度环的PID
	PID_LimitConfig(&pid_velocity, +0.5 * g, -0.5 * g);//将速度环的PID控制器输出限制在+-0.5g之间
	
	PID_Init(&pid_theta, 4.0f,0.0f,0.0f);//初始化θ环的PID
	PID_LimitConfig(&pid_theta, +12.57f, -12.57);//将θ环的PID控制器输出限制在+-4Π rad/s之间
	
#if   EXP_OMEGA_GAIN_LOW == 1
	PID_Init(&pid_theta_dot, 6.0f,6.0f,0.0f);//实验：内环整体降 6dB，零点仍在 1 rad/s
#elif EXP_OMEGA_GAIN_LOW == 2
	PID_Init(&pid_theta_dot, 4.0f,4.0f,0.0f);//实验：内环整体降 8dB，零点仍在 1 rad/s
#else
	PID_Init(&pid_theta_dot, 10.0f,10.0f,0.0f);//初始化θ点环的PID
#endif
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
#if EXP_SPEED_LOOP_OPEN
	float theta_ref = 0.0f;//实验：速度环开环，外环设定值恒为 0（车会缓慢漂走，属正常）
#else
	float theta_ref = qatan(PID_Compute(&pid_velocity, x_dot) / g);
#endif

	//探针：把两个环路之间的中间量存下来给波形用
	probe_x_dot     = x_dot;
	probe_theta_ref = theta_ref;
	probe_theta_dot = theta_dot;
	
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

//
// @简介：读取速度环输出（倒立摆外环设定值），单位 度
//
float App_Control_GetThetaRef(void)
{
	return probe_theta_ref * 57.29578f;
}

//
// @简介：读取速度环反馈值，单位 cm/s（乘 100 是为了和角度量级接近，方便同图显示）
//
float App_Control_GetXdot(void)
{
	return probe_x_dot * 100.0f;
}

//
// @简介：读取角速度环反馈值，单位 度每秒
//
float App_Control_GetThetaDot(void)
{
	return probe_theta_dot * 57.29578f;
}
