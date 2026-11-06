#include "stm32f10x.h"                  // Device header
#include "app_usart2.h"
#include "app_MPU6050.h"
#include "delay.h"
#include "task.h"

void MPU6050_Test(void)
{
	App_USART2_Init();
	App_MPU6050_Init();
	
	while(1)
	{
		App_MPU6050_Update();
		float ax = App_MPU6050_GetAx();
		float ay = App_MPU6050_GetAy();
		float az = App_MPU6050_GetAz();
		
		float temperature = App_MPU6050_GetTemperature();
		
		float gx = App_MPU6050_GetGx();
		float gy = App_MPU6050_GetGy();
		float gz = App_MPU6050_GetGz();
		
		My_USART_Printf(USART2, "%f,%f,%f,%f,%f,%f,%f\n", ax, ay, az, temperature, gx, gy, gz);
		
		Delay(10);
	}
}

static void USART2_Proc(void);
	
//
//@简介：用来测试欧拉角（Vofa）
//
void MPU6050_EularAngleTest(void)
{
	App_USART2_Init();
	App_MPU6050_Init();
	
	while(1)
	{
		App_MPU6050_Proc();
		USART2_Proc();
	}
}

//
//@简介：每间隔10ms把欧拉角的计算结果发送给电脑
//
static void USART2_Proc(void)
{
	PERIODIC(10)
	float ax = App_MPU6050_GetAx();
	float ay = App_MPU6050_GetAy();
	float az = App_MPU6050_GetAz();
	
	float temperature = App_MPU6050_GetTemperature();
		
	float gx = App_MPU6050_GetGx();
	float gy = App_MPU6050_GetGy();
	float gz = App_MPU6050_GetGz();
	
	float yaw = App_MPU6050_GetYaw();
	float pitch = App_MPU6050_GetPitch();
	float roll = App_MPU6050_GetRoll();
	
	My_USART_Printf(USART2, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", ax, ay, az, temperature, gx, gy, gz, yaw, pitch, roll);
}
	