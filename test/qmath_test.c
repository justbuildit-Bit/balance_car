#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "app_usart2.h"
#include <math.h>
#include "qmath.h"

static void I32Plus_Test(void); // 32位整数加法
static void I32Minus_Test(void); // 32位整数减法
static void I32Multiply_Test(void); // 32位整数乘法
static void I32Divide_Test(void); // 32位整数除法

static void FloatPlus_Test(void); // 单浮点数加法
static void FloatMinus_Test(void); // 单浮点数减法
static void FloatMultiply_Test(void); // 单浮点数乘法
static void FloatDivide_Test(void); // 单浮点数除法

static void Sin_Test(void); // 正弦
static void Cos_Test(void); // 余弦
static void Tan_Test(void); // 正切
static void Atan_Test(void); // 反正切
static void Atan2_Test(void); // 反正切2

static void QSin_Test(void); // 改进后的正弦
static void QCos_Test(void); // 改进后的余弦
static void QTan_Test(void); // 改进后的正切
static void QAtan_Test(void); // 改进后的反正切
static void QAtan2_Test(void); // 改进后的反正切2

void QMath_Test(void)
{
	App_USART2_Init();
	
//	// 整数+-*/
//	I32Plus_Test();
//	I32Minus_Test();
//	I32Multiply_Test();
//	I32Divide_Test();
//	
//	// 单浮点数+-*/
//	FloatPlus_Test();
//	FloatMinus_Test();
//	FloatMultiply_Test();
//	FloatDivide_Test();
//	
//	// 三角函数
//	Sin_Test();
//	Cos_Test();
//	Tan_Test();
//	Atan_Test();
//	Atan2_Test();
	
	// 快速三角函数
	QSin_Test();
	QCos_Test();
	QTan_Test();
	QAtan_Test();
	QAtan2_Test();
	
	while(1){}
}

// 
// @简介：测试I32加法的运算速度
//
static void I32Plus_Test(void)
{
	uint32_t t1,t2; // 计时
	
	// +法
	My_USART_Printf(USART2, "\n测试I32+法运算速度：");
	
	int a=1,b=2,c;
	
	t1 = GetTick();
	
	for(uint32_t i = 0; i < 1000000; i++)
	{
		c = a + b;
	}
	
	t2 = GetTick();
	
	(void)c;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", (t2 - t1) / 1000.0f);
}

// 
// @简介：测试I32减法的运算速度
//
static void I32Minus_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// -法
	My_USART_Printf(USART2, "\n测试I32-法运算速度：");
	
	int a=1,b=2,c;
	
	stopWatch = GetTick();
	
	for(uint32_t i = 0; i < 1000000; i++)
	{
		c = a - b;
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)c;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 1000.0f);
}

// 
// @简介：测试I32乘法的运算速度
//
static void I32Multiply_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// *法
	My_USART_Printf(USART2, "\n测试I32*法运算速度：");
	
	stopWatch = GetTick();
	
	int a = 3,b = 2, c;
	
	for(uint32_t i = 0; i < 1000000; i++)
	{
		c = a * b;
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)c;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 1000.0f);
}

// 
// @简介：测试I32除法的运算速度
//
static void I32Divide_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// ÷法
	My_USART_Printf(USART2, "\n测试I32÷法运算速度：");
	
	stopWatch = GetTick();
	
	int a=10,b=3,c;
	
	for(uint32_t i = 0; i < 1000000; i++)
	{
		c = a / b;
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)c;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 1000.0f);
}

static void FloatPlus_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// -法
	My_USART_Printf(USART2, "\n测试float+法运算速度：");
	
	float a=1,b=2,c;
	
	stopWatch = GetTick();
	
	for(uint32_t i = 0; i < 1000000; i++)
	{
		c = a + b;
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)c;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 1000.0f);
}

static void FloatMinus_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// -法
	My_USART_Printf(USART2, "\n测试float-法运算速度：");
	
	float a=1,b=2,c;
	
	stopWatch = GetTick();
	
	for(uint32_t i = 0; i < 1000000; i++)
	{
		c = a - b;
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)c;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 1000.0f);
}

static void FloatMultiply_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// *法
	My_USART_Printf(USART2, "\n测试float*法运算速度：");
	
	float a=1.1,b=3,c;
	
	stopWatch = GetTick();
	
	for(uint32_t i = 0; i < 1000000; i++)
	{
		c = a * b;
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)c;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 1000.0f);
}

static void FloatDivide_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// ÷法
	My_USART_Printf(USART2, "\n测试float÷法运算速度：");
	
	float a=1,b=3,c;
	
	stopWatch = GetTick();
	
	for(uint32_t i = 0; i < 1000000; i++)
	{
		c = a / b;
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)c;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 1000.0f);
}

static void Sin_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// sin
	My_USART_Printf(USART2, "\n测试sin运算速度：");
	
	float a=1,b;
	
	stopWatch = GetTick();
	
	for(uint32_t i = 0; i < 100000; i++)
	{
		b = sin(a);
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)b;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 100.0f);
}

static void Cos_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// cos
	My_USART_Printf(USART2, "\n测试cos运算速度：");
	
	float a=1,b;
	
	stopWatch = GetTick();
	
	for(uint32_t i = 0; i < 100000; i++)
	{
		b = cos(a);
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)b;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 100.0f);
}

static void Tan_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// tan
	My_USART_Printf(USART2, "\n测试tan运算速度：");
	
	float a=1,b;
	
	stopWatch = GetTick();
	
	for(uint32_t i = 0; i < 100000; i++)
	{
		b = tan(a);
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)b;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 100.0f);
}

static void Atan_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// atan
	My_USART_Printf(USART2, "\n测试atan运算速度：");
	
	float a=1,b;
	
	stopWatch = GetTick();
	
	for(uint32_t i = 0; i < 100000; i++)
	{
		b = atan(a);
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)b;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 100.0f);
}

static void Atan2_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// atan
	My_USART_Printf(USART2, "\n测试atan2运算速度：");
	
	float a=1,b=1,c=1;
	
	stopWatch = GetTick();
	
	for(uint32_t i = 0; i < 1000000; i++)
	{
		c = atan2(a,b);
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)c;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 1000.0f);
}

static void QSin_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// sin
	My_USART_Printf(USART2, "\n测试Qsin运算速度：");
	
	float a=1,b;
	
	stopWatch = GetTick();
	
	for(uint32_t i = 0; i < 1000000; i++)
	{
		b = qsin(a);
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)b;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 1000.0f);
}

static void QCos_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// cos
	My_USART_Printf(USART2, "\n测试Qcos运算速度：");
	
	float a=1,b;
	
	stopWatch = GetTick();
	
	for(uint32_t i = 0; i < 1000000; i++)
	{
		b = qcos(a);
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)b;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 1000.0f);
}

static void QTan_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// tan
	My_USART_Printf(USART2, "\n测试Qtan运算速度：");
	
	float a=1,b;
	
	stopWatch = GetTick();
	
	for(uint32_t i = 0; i < 1000000; i++)
	{
		b = qtan(a);
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)b;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 1000.0f);
}

static void QAtan_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// atan
	My_USART_Printf(USART2, "\n测试Qatan运算速度：");
	
	float a=1,b;
	
	stopWatch = GetTick();
	
	for(uint32_t i = 0; i < 1000000; i++)
	{
		b = qatan(a);
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)b;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 1000.0f);
}

static void QAtan2_Test(void)
{
	uint32_t stopWatch; // 停表
	uint32_t timeElapsed; // 消耗的时间
	
	// atan
	My_USART_Printf(USART2, "\n测试Qatan2运算速度：");
	
	float a=1,b=1,c=1;
	
	stopWatch = GetTick();
	
	for(uint32_t i = 0; i < 1000000; i++)
	{
		c = qatan2(a,b);
	}
	
	timeElapsed = GetTick() - stopWatch;
	
	(void)c;
	
	My_USART_Printf(USART2, "每次运算耗时%.3fus\n", timeElapsed / 1000.0f);
}

	