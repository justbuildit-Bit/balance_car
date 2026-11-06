#include "app_button.h"
#include "button.h"
#include "app_motor.h"
#include "app_control.h"

static Button_TypeDef userKey;//用户按钮
static void OnUserKey_Clicked(uint8_t clicks);

//按钮初始化
void App_Button_Init(void)
{
	Button_InitTypeDef Button_InitStructure = {0};
	Button_InitStructure.GPIOx = GPIOA;
	Button_InitStructure.GPIO_Pin = GPIO_Pin_11;
	
	My_Button_Init(&userKey, &Button_InitStructure);
	
	My_Button_SetClickCb(&userKey, OnUserKey_Clicked);
}

void App_Button_Proc(void)
{
	My_Button_Proc(&userKey);
}

static uint8_t pwm_on = 0;//0-电机休眠，1-电机活动

//按钮点击的回调函数
void OnUserKey_Clicked(uint8_t clicks)
{
	if(clicks == 1)
	{
		App_Control_Reset();
		
		//翻转电机状态
		if(pwm_on == 0)
		{
			pwm_on = 1;
		}
		else
		{
			pwm_on = 0;
		}
		
		App_Motor_Cmd(pwm_on);
	}
}
	