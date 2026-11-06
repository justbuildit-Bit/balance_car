#include "bat_test.h"
#include "app_usart2.h"
#include "app_bat.h"
#include "delay.h"
void Bat_Test(void)
{
	App_USART2_Init();
	App_Bat_Init();
	
	while(1)
	{
		float volt = App_Bat_Get();
		My_USART_Printf(USART2, "%.3f\n", volt);
		Delay(10);
	}
}
