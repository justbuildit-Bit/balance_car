#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H
#include "stm32f10x.h"                  // Device header

void App_Control_Init(void);
void App_Control_Proc(void);
void App_Control_Reset(void);
void App_Control_SetMoveSpeed(float MoveSpeed);
void App_Control_SetTurnSpeed(float TurnSpeed);

/* 调试探针：把控制环之间的中间量暴露出来，供 VOFA+ 输出
 *   GetThetaRef   速度环输出 = 倒立摆外环设定值，单位 度[°]
 *   GetXdot       速度环反馈（含轮速补偿），单位 厘米每秒[cm/s]
 *   GetThetaDot   角速度环反馈，单位 度每秒[°/s]
 * 只读，不影响控制。数值在 App_Control_Proc() 里更新。 */
float App_Control_GetThetaRef(void);
float App_Control_GetXdot(void);
float App_Control_GetThetaDot(void);

#endif
