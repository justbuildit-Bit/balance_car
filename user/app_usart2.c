#include "app_usart2.h"        

void App_USART2_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStucture = {0};
	GPIO_InitStucture.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStucture.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStucture.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStucture);
	
	GPIO_InitStucture.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStucture.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStucture);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	USART_InitTypeDef USART_InitStructure = {0};
	USART_InitStructure.USART_BaudRate = 576000;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &USART_InitStructure);
	
	USART_Cmd(USART2, ENABLE);
	
}
	