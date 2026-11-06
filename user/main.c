#include "stm32f10x.h"
#include "bat_test.h"
#include "app_bat.h"
#include "app_button.h"
#include "app_pwm.h"
#include "pwm_test.h"
#include "encoder_test.h"
#include "MPU6050_test.h"
#include "qmath_test.h"
#include "app_encoder.h"
#include "app_motor.h"
#include "delay.h"
#include "app_usart2.h"
#include "app_vofa.h"
#include "task.h"
#include "app_MPU6050.h"
#include "app_kalman.h"
#include "app_control.h"
#include "app_rc.h"

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
//	QMath_Test();
//	MPU6050_EularAngleTest();
//	MPU6050_Test();
//	Encoder_T_Method_Test();
//	Encoder_M_Method_Test();
//	Encoder_Test();
//	Bat_Test();
//	PWM_Test();
	//串口波形输出（VOFA+），占用 USART2：PA2=TX、PA3=RX，波特率见 app_vofa.h
	//它会重新配置 USART2，所以不再单独调用 App_USART2_Init()
	App_Vofa_Init();
	App_Button_Init();
	App_Encoder_Init();
	App_PWM_Init();
	App_Bat_Init();
	App_Motor_Init();
	App_MPU6050_Init();
	
	//上电后保持车体静止约 0.3s，标定陀螺仪的静态零偏
	//只影响偏航角漂移，俯仰/翻滚不标定卡尔曼也会自己收敛
	App_Kalman_GyroCalibrate(300);
	
	App_Control_Init();
	App_RC_Init();
	
	while(1)
	{
		App_Bat_Proc();
		App_Button_Proc();
		App_Motor_Proc();
		App_MPU6050_Proc();//互补滤波（保留，用于对照）
		App_Kalman_Proc();//卡尔曼滤波：必须在 MPU6050_Proc 之后，读的才是本周期的数据
		App_Vofa_Proc();//串口发波形：放在两个滤波之后，发出去的才是本周期结果
		App_Control_Proc();
		App_RC_Proc();
	}
}
