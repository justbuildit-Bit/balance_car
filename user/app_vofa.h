#ifndef APP_VOFA_H
#define APP_VOFA_H
#include "stm32f10x.h"                  // Device header

/*
 * 串口波形输出（VOFA+ / FireWater 协议）
 * ------------------------------------------------------------
 * 作用：把姿态解算的中间量按固定周期从串口发出去，用 VOFA+ 画成曲线。
 *       默认输出三个通道，叠在一张波形图里就能直观看出滤波效果：
 *
 *   ch0  加速度计直接解算的角度  —— 噪声大，抖动明显
 *   ch1  互补滤波后的角度        —— app_MPU6050 里原有的算法
 *   ch2  卡尔曼滤波后的角度      —— 本例的融合结果
 *
 * 数据帧格式（FireWater）：
 *   angle:<ch0>,<ch1>,<ch2>\r\n
 *   冒号是数据起点，逗号分隔通道，换行结束一帧。
 *
 * 用法：main 里初始化一次 App_Vofa_Init()，
 *       然后在 while(1) 中调用 App_Vofa_Proc()（内部自己控周期）。
 * ------------------------------------------------------------
 */

/* 1 = 启用波形输出；0 = 完全关闭（发布固件时建议关掉） */
#ifndef APP_VOFA_ENABLE
#define APP_VOFA_ENABLE      1
#endif

/* 用哪个串口输出：1 = USART1（PA9=TX / PA10=RX），2 = USART2（PA2=TX / PA3=RX）
 * 不要选 3 —— USART3 已经被遥控（app_rc）占用 */
#ifndef APP_VOFA_PORT
#define APP_VOFA_PORT        2
#endif

/* 波特率，必须和 VOFA+ 里设的一致 */
#ifndef APP_VOFA_BAUD
#define APP_VOFA_BAUD        115200
#endif

/* 输出周期，单位 ms。20ms = 50Hz，波形流畅度和串口带宽比较平衡；
 * 想要更细的细节可以改到 10，但 FireWater 是字符串协议，频率太高会吃带宽 */
#ifndef APP_VOFA_PERIOD_MS
#define APP_VOFA_PERIOD_MS   20
#endif

void App_Vofa_Init(void);
void App_Vofa_Proc(void);

#endif
