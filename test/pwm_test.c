#include "stm32f10x.h"                  // Device header
#include "app_pwm.h"
#include "delay.h"
//测试PWM模块
//先使能电机
//电机正转30% 2秒
//电机正转60% 2秒
//电机正转90% 2秒
//关闭电机
void PWM_Test(void)
{
	App_PWM_Init();
	App_PWM_Cmd(1);//唤醒电机
	App_PWM_Set_L(30);//正转30%
	App_PWM_Set_R(30);//正转30%
	Delay(2000);//延迟2s
	
	App_PWM_Set_L(60);//正转60%
	App_PWM_Set_R(60);//正转60%
	Delay(2000);//延迟2s
	
	App_PWM_Set_L(90);//正转90%
	App_PWM_Set_R(90);//正转90%
	Delay(2000);//延迟2s
	
	App_PWM_Cmd(0);//关闭电机
	
	while(1)
	{
		
	}
}
	