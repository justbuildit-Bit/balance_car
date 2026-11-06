#include "stm32f10x.h"                  // Device header
#include <math.h>

static void STBY_Pin_Init(void);//初始化STBY引脚
static void Motor_L_Init(void);//左电机的初始化
static void Motor_R_Init(void);//右电机的初始化


//对TB6612初始化
void App_PWM_Init(void)
{
	STBY_Pin_Init();//STBY引脚初始化
	Motor_L_Init();//对左电机初始化
	Motor_R_Init();//对右电机初始化
}



//控制TB6612进入休眠或活动状态
//参数：on 0 - 休眠状态，向STBY写L
//         非零 - 活动状态，向STBY写H
void App_PWM_Cmd(uint8_t on)
{
	if(on == 0)
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);//休眠
	}
	else
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);//活动
	}
}
	

//设置左电机的占空比
//参数：duty- 占空比具体值，范围 -100.0f ~ +100.0f
void App_PWM_Set_L(float Duty)
{
	float sign;//符号，正数- +1，负数- -1
	
	if(Duty >= 0) sign = 1;
	else sign = -1;
	
	Duty = fabsf(Duty);
	
	if(sign < 0)//正转
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_9, Bit_SET);//AIN1-高电压
		GPIO_WriteBit(GPIOA, GPIO_Pin_10, Bit_RESET);//AIN2-低电压
	}
	else 
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_9, Bit_RESET);//AIN1-低电压
		GPIO_WriteBit(GPIOA, GPIO_Pin_10, Bit_SET);//AIN2-高电压
	}
		
	uint16_t CCR = Duty / 100.0f * 999;
	
	TIM_SetCompare1(TIM1, CCR);
}
	
//设置右电机的占空比
//参数：duty- 占空比具体值，范围 -100.0f ~ +100.0f
void App_PWM_Set_R(float Duty)
{
	float sign;//符号，正数- +1，负数- -1
	
	if(Duty >= 0) sign = 1;
	else sign = -1;
	
	Duty = fabsf(Duty);
	
	if(sign >= 0)//正转
	{
		GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);//BIN1-高电压
		GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET);//BIN2-低电压
	}
	else 
	{
		GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);//BIN1-低电压
		GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);//BIN2-高电压
	}
		
	uint16_t CCR = Duty / 100.0f * 999;
	
	TIM_SetCompare1(TIM4, CCR);
}
	
	
//初始化STBY引脚
//PA1-Out_PP
static void STBY_Pin_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}
	
//左电机的初始化
//AIN1-PA9-Out_PP
//AIN2-PA10-Out_PP
static void Motor_L_Init(void)
{
	//初始化AIN1,AIN2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//初始化PWM
	//PA8-AF_PP
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//对定时器1进行初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//开启TIM1的时钟
	
	//设置时基单元的参数
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
	
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 999;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
	
	//配置输出比较
	TIM_OCInitTypeDef TIM_OCInitTypeStructure = {0};
	
	TIM_OCInitTypeStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitTypeStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitTypeStructure.TIM_OutputState = ENABLE;
	TIM_OCInitTypeStructure.TIM_Pulse = 0;
	
	TIM_OC1Init(TIM1, &TIM_OCInitTypeStructure);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);//配置MOE开关
	TIM_Cmd(TIM1,ENABLE);//闭合定时器总开关
}
	
//右电机的初始化
//BIN1-PB5-Out_PP
//BIN2-PB7-Out_PP
static void Motor_R_Init(void)
{
	//初始化BIN1,BIN2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	//初始化PWM
	//PB6-AF_PP
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	//对定时器4进行初始化
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//开启TIM4的时钟
	
	//设置时基单元的参数
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
	
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 999;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
	
	//配置输出比较
	TIM_OCInitTypeDef TIM_OCInitTypeStructure = {0};
	
	TIM_OCInitTypeStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitTypeStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitTypeStructure.TIM_OutputState = ENABLE;
	TIM_OCInitTypeStructure.TIM_Pulse = 0;
	
	TIM_OC1Init(TIM4, &TIM_OCInitTypeStructure);
	TIM_CtrlPWMOutputs(TIM4, ENABLE);//配置MOE开关
	TIM_Cmd(TIM4,ENABLE);//闭合定时器总开关
}
