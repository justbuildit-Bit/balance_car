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

//==================================================================
// 速度环增益实验：验证 1.6 Hz 稳态摆动是否由速度环决定
//------------------------------------------------------------------
// 已被推翻的三个假设（判据全都一样：摆动频率跟不跟着改动量走）
//   ① 桌面打滑         → 换地面复测，周期不变         → 排除
//   ② 角速度环增益过高 → 穿越频率降 38%，周期不变     → 排除
//   ③ 线束侧向约束     → 拔线后仍摆 1.56 Hz，不变     → 排除
// 三个都不是，剩下唯一没动过的环就是速度环。
//
// 本次实验的全部依据（推导，不是猜）：
//   速度环输出的是加速度，经 θ_ref = atan(u/g) ≈ u/g 换算成角度指令；
//   车身近似满足 ẍ = g·θ，代入得闭环：
//       ẍ = Kp·(v_sp − v)
//   即速度环闭环是一阶惯性环节，时间常数 τ = 1/Kp，
//   截止频率 f_c = Kp/(2π)：
//       Kp = 10 → τ = 0.100 s → f_c = 1.59 Hz
//       Kp =  5 → τ = 0.200 s → f_c = 0.80 Hz
//       Kp = 2.5 → τ = 0.400 s → f_c = 0.40 Hz
//   而实测摆动是 1.69~1.81 Hz（有线 VOFA）、1.56 Hz（拔线视频），
//   与 f_c(10) 只差 2%~15%——是目前唯一还没被证伪的对应关系。
//
// 判据（只看频率，不看幅度）：
//   · 摆动频率随 Kp 成比例下降 → 确认速度环，可开始定向设计相位补偿
//   · 摆动频率不动             → 控制回路整体排除，只剩机械/传感器侧
//
// 注意 1：Kp 与 Ki 必须同比例降。积分零点在 Ki/Kp，10/1 时是 0.1 rad/s；
//         若只降 Kp（10/1 → 5/1），零点被推到 0.2 rad/s，低频积分作用
//         弱一半，稳态会跑偏，而且引入额外相位滞后，实验结论就不可信了。
// 注意 2：降 Kp 的直接代价是抗扰变弱——推一把后回位更慢、更容易漂走。
//         这不是故障，是一阶系统时间常数从 0.1 s 变 0.2 s 的必然结果，
//         所以观察时要看"频率"这个量，不要因为"看起来更晃"就否定。
// 注意 3：本宏只改速度环。角度环 4/0、角速度环 10/10、电机环 0.5/7 全部
//         保持不动，确保与基线（main 787a03b）只差这一个变量。
// 注意 4（降 Kp 为什么应该让频率下降）：一阶环节相位最多到 -90°，速度环
//         自己不会自激；真正决定频率的是整个回路相位凑到 -180° 的那个频点。
//         降 Kp 等于降回路增益，穿越频率整体左移，振荡频点随之左移。
//         反过来说，若降 Kp 后频率完全不动，说明频率被非线性钉死
//         （齿轮间隙、静摩擦、编码器量化、PWM 死区都可能造成），
//         那就与控制增益无关，得往机械侧查。
//==================================================================
#define EXP_SPEED_GAIN_LOW   1      // 0=10/1  1=5/0.5  2=2.5/0.25

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
#if   EXP_SPEED_GAIN_LOW == 0
	PID_Init(&pid_velocity, 10.0f, 1.0f, 0.0f);//基线：τ=0.100s, f_c=1.59Hz
#elif EXP_SPEED_GAIN_LOW == 1
	PID_Init(&pid_velocity,  5.0f, 0.5f, 0.0f);//半增益：τ=0.200s, f_c=0.80Hz
#else
	PID_Init(&pid_velocity,  2.5f, 0.25f, 0.0f);//四分之一：τ=0.400s, f_c=0.40Hz
#endif
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
