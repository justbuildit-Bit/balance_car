#ifndef __APP_MOTOR_H
#define __APP_MOTOR_H
#include "stm32f10x.h"                  // Device header

void App_Motor_Init(void);
void App_Motor_Proc(void);
void App_Motor_SetOmega_L(float Omega);
void App_Motor_SetOmega_R(float Omega);
void App_Motor_Cmd(uint8_t On);
uint8_t App_Motor_GetState(void);
#endif 
