#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H
#include "stm32f10x.h"                  // Device header

void App_Control_Init(void);
void App_Control_Proc(void);
void App_Control_Reset(void);
void App_Control_SetMoveSpeed(float MoveSpeed);
void App_Control_SetTurnSpeed(float TurnSpeed);

#endif
