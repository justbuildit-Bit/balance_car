#include "stm32f10x.h"                  // Device header
#include "app_usart2.h"
#include "delay.h"
#include "app_encoder.h"

//对编码器的位置信号进行测试
//他会通过调试用的串口把编码器的当前位置显示在Vofa上
void Encoder_Test(void)
{
	App_USART2_Init();
	App_Encoder_Init();
	
	while(1)
	{
		float pos_l = App_Encoder_GetPos_L();
		float pos_r = App_Encoder_GetPos_R();
		
		My_USART_Printf(USART2, "%f,%f\n", pos_l, pos_r);
		
		
		Delay(50);
		
	}
}
	

static float last_pos_l = 0.0f;
static float last_pos_r = 0.0f;

//对M法测速进行测试
void Encoder_M_Method_Test(void)
{
	App_Encoder_Init();//对编码器初始化
	App_USART2_Init();//串口2进行初始化
	
	while(1)
	{
		Delay(1);
		
		float pos_l = App_Encoder_GetPos_L();
		float pos_r = App_Encoder_GetPos_R();
		
		float M_l = pos_l - last_pos_l;
		float M_r = pos_r - last_pos_r;
		
		float omega_l = M_l / 0.001f;
		float omega_r = M_r / 0.001f;
		
		My_USART_Printf(USART2, "%f,%f,%f,%f\n", pos_l, pos_r, omega_l, omega_r);
		
		last_pos_l = pos_l;
		last_pos_r = pos_r;
	}
}
	
//T法测速的测试代码
//通过串口把T法测试的omega值发送到Vofa显示
void Encoder_T_Method_Test(void)
{
	App_USART2_Init();
	App_Encoder_Init();
	
	while(1)
	{
		Delay(1);
		
		float omega_l = App_Encoder_GetSpeed_L();
		float omega_r = App_Encoder_GetSpeed_R();
		
		My_USART_Printf(USART2, "%f,%f\n", omega_l, omega_r);
	}
}
	