#include "app_kalman.h"
#include "app_MPU6050.h"
#include "task.h"
#include "qmath.h"

/*
 * 卡尔曼滤波实现
 * ------------------------------------------------------------
 * 状态空间（对应《自动控制原理》现代控制部分）：
 *
 *   x(k) = A * x(k-1) + B * u(k)
 *   A = [ 1  -dt ]      B = [ dt ]      u = rate（陀螺仪测量值）
 *       [ 0   1  ]          [ 0  ]
 *
 *   z(k) = H * x(k) + v      H = [ 1  0 ]   （只观测角度）
 *
 * 预测：x = A*x + B*u        P = A*P*A' + Q
 * 校正：K = P*H'/(H*P*H'+R)  x = x + K*(z - H*x)   P = (I-K*H)*P
 *
 * A、H 结构固定，下面把矩阵乘法全部展开成标量运算，
 * 避免通用矩阵库的循环开销（STM32F103 没有硬件浮点）。
 * ------------------------------------------------------------
 */

//#1. 两个滤波器实例：俯仰角一个、翻滚角一个
static Kalman_t kf_pitch;
static Kalman_t kf_roll;

//#2. 融合输出
static float pitch, roll, yaw;
static float yaw_rate = 0.0f;//去零偏后的 z 轴角速度，单位°/s

//#3. 首次进入时用加速度计角度做初值，避免从 0° 慢慢往上爬
static uint8_t kf_ready = 0;

//#4. 陀螺仪静态零偏（App_Kalman_GyroCalibrate 标定后才有值）
static float gx_offset = 0.0f;
static float gy_offset = 0.0f;
static float gz_offset = 0.0f;

//#5. 采样周期 5ms
#define KALMAN_DT    0.005f


//
//@简介：初始化卡尔曼滤波器
//@参数：kf - 滤波器对象
//@参数：initAngle - 初始角度，单位°
//
void App_Kalman_Init(Kalman_t *kf, float initAngle)
{
	kf->angle = initAngle;
	kf->bias  = 0.0f;
	kf->rate  = 0.0f;

	//初始协方差：一开始并不完全相信陀螺积分，所以给 1
	kf->P[0][0] = 1.0f;
	kf->P[0][1] = 0.0f;
	kf->P[1][0] = 0.0f;
	kf->P[1][1] = 1.0f;

	kf->Q_angle   = KALMAN_Q_ANGLE;
	kf->Q_bias    = KALMAN_Q_BIAS;
	kf->R_measure = KALMAN_R_MEASURE;
}

//
//@简介：预测步（时间更新）
//@参数：kf - 滤波器对象
//@参数：rateMeasure - 陀螺仪角速度，单位°/s
//@参数：dt - 采样周期，单位s
//
void App_Kalman_Predict(Kalman_t *kf, float rateMeasure, float dt)
{
	//#1. 状态预测
	kf->rate   = rateMeasure - kf->bias;
	kf->angle += dt * kf->rate;

	//#2. 协方差预测 P = A*P*A' + Q，按 A 的结构展开
	kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
	kf->P[0][1] -= dt * kf->P[1][1];
	kf->P[1][0] -= dt * kf->P[1][1];
	kf->P[1][1] += kf->Q_bias * dt;
}

//
//@简介：校正步（量测更新）
//@参数：kf - 滤波器对象
//@参数：angleMeasure - 加速度计解算出的角度，单位°
//@返回值：融合后的角度，单位°
//
float App_Kalman_Correct(Kalman_t *kf, float angleMeasure)
{
	float y, S, K0, K1, P00, P01;

	//#1. 新息，并把差值折算到 ±180° 内，
	//防止角度在 ±180° 附近跳变时被当成突变 360° 而疯狂修正
	y = angleMeasure - kf->angle;
	while (y >  180.0f) y -= 360.0f;
	while (y < -180.0f) y += 360.0f;

	//#2. 卡尔曼增益，H = [1 0]
	S  = kf->P[0][0] + kf->R_measure;
	K0 = kf->P[0][0] / S;
	K1 = kf->P[1][0] / S;

	//#3. 状态更新
	kf->angle += K0 * y;
	kf->bias  += K1 * y;

	//#4. 协方差更新，必须先用旧值算出新值
	P00 = kf->P[0][0];
	P01 = kf->P[0][1];

	kf->P[0][0] -= K0 * P00;
	kf->P[0][1] -= K0 * P01;
	kf->P[1][0] -= K1 * P00;
	kf->P[1][1] -= K1 * P01;

	return kf->angle;
}

//
//@简介：一步完成"预测 + 校正"
//@参数：kf - 滤波器对象
//@参数：angleMeasure - 加速度计解算角度，单位°
//@参数：rateMeasure - 陀螺仪角速度，单位°/s
//@参数：dt - 采样周期，单位s
//@参数：useMeasure - 0 = 本次量测不可信，只做预测
//@返回值：融合后的角度，单位°
//
float App_Kalman_Update(Kalman_t *kf, float angleMeasure, float rateMeasure, float dt, uint8_t useMeasure)
{
	App_Kalman_Predict(kf, rateMeasure, dt);

	if (useMeasure)
	{
		return App_Kalman_Correct(kf, angleMeasure);
	}

	return kf->angle;
}


//
//@简介：姿态融合进程函数，每 5ms 执行一次
//@说明：必须在 App_MPU6050_Proc() 之后调用，
//       否则读到的是上一周期的传感器数据
//
void App_Kalman_Proc(void)
{
	float ax, ay, az, gx, gy, gz;
	float pitch_a, roll_a;
	uint8_t use_acc = 1;

	PERIODIC(5)//进程函数每5ms执行一次

	//#1. 通过驱动的 Get 接口取值，不碰驱动内部变量
	ax = App_MPU6050_GetAx();
	ay = App_MPU6050_GetAy();
	az = App_MPU6050_GetAz();

	gx = App_MPU6050_GetGx() - gx_offset;
	gy = App_MPU6050_GetGy() - gy_offset;
	gz = App_MPU6050_GetGz() - gz_offset;

	//#2. 加速度计解算角度（轴向约定与互补滤波版本一致）
	pitch_a = qatan2(ay, az) / 3.1415927 * 180.0f;
	roll_a  = qatan2(ax, az) / 3.1415927 * 180.0f;

	//#3. 首次进入用加速度计角度做初值
	if (kf_ready == 0)
	{
		App_Kalman_Init(&kf_pitch, pitch_a);
		App_Kalman_Init(&kf_roll,  roll_a);
		kf_ready = 1;
	}

#if KALMAN_ACC_GATE
	//静止时 |a|² 约等于 1，明显偏离说明有额外线加速度，
	//此时 atan2 算出来的角度不可信，只做预测不做校正
	{
		float acc_norm2 = ax * ax + ay * ay + az * az;
		if (acc_norm2 < 0.49f || acc_norm2 > 1.69f)//0.7g ~ 1.3g
		{
			use_acc = 0;
		}
	}
#endif

	//#4. 融合
	//俯仰角：陀螺仪绕 x 轴，正方向与 gx 一致
	pitch = App_Kalman_Update(&kf_pitch, pitch_a, gx,  KALMAN_DT, use_acc);
	//翻滚角：陀螺仪绕 y 轴，正方向与 gy 相反（沿用原代码约定）
	roll  = App_Kalman_Update(&kf_roll,  roll_a, -gy, KALMAN_DT, use_acc);

	//#5. 偏航角：没有磁力计就没有绝对参考，
	//做校正反而会被噪声拉偏，所以只用陀螺积分
	yaw_rate = gz;
	yaw = yaw + gz * KALMAN_DT;
}


//
//@简介：陀螺仪零偏标定（可选，上电静止时调用一次）
//@参数：samples - 采样点数，建议 200~500
//@说明：会调用 App_MPU6050_Update() 直接读传感器，耗时约 samples 毫秒。
//       标定后偏航角的积分漂移会明显减小；
//       俯仰/翻滚即使不标定，卡尔曼的 bias 状态也会自己收敛。
//
void App_Kalman_GyroCalibrate(uint16_t samples)
{
	float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
	uint16_t i;

	if (samples == 0)
	{
		return;
	}

	for (i = 0; i < samples; i++)
	{
		App_MPU6050_Update();//读一次传感器

		sumX += App_MPU6050_GetGx();
		sumY += App_MPU6050_GetGy();
		sumZ += App_MPU6050_GetGz();

		Delay(1);
	}

	gx_offset = sumX / (float)samples;
	gy_offset = sumY / (float)samples;
	gz_offset = sumZ / (float)samples;
}


//
//@简介：获取俯仰角，单位°
//
float App_Kalman_GetPitch(void)
{
	return pitch;
}

//
//@简介：获取翻滚角，单位°
//
float App_Kalman_GetRoll(void)
{
	return roll;
}

//
//@简介：获取偏航角，单位°
//
float App_Kalman_GetYaw(void)
{
	return yaw;
}

//
//@简介：获取俯仰通道的陀螺零偏估计，单位°/s（调试用）
//
float App_Kalman_GetPitchBias(void)
{
	return kf_pitch.bias;
}

//
//@简介：获取翻滚通道的陀螺零偏估计，单位°/s（调试用）
//
float App_Kalman_GetRollBias(void)
{
	return kf_roll.bias;
}

//
//@简介：获取去零偏后的俯仰角速度，单位°/s（控制环 D 项直接可用）
//
float App_Kalman_GetPitchRate(void)
{
	return kf_pitch.rate;
}

//
//@简介：获取去零偏后的偏航角速度，单位°/s（转向环反馈直接可用）
//
float App_Kalman_GetYawRate(void)
{
	return yaw_rate;
}
