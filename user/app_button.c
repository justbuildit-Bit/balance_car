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

//按钮点击的回调函数
void OnUserKey_Clicked(uint8_t clicks)
{
	if(clicks == 1)
	{
		//以电机的实际状态取反，不再用本地变量记状态：
		//倾角保护会把电机拉进待机，本地变量并不知道，会出现"按一次没反应、按两次才启动"
		uint8_t on = (App_Motor_GetState() == 0) ? 1 : 0;
		
		App_Control_Reset();//清 PID 积分、lastTime 与倾角保护故障位
		App_Motor_Cmd(on);
	}
}
