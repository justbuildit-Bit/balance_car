#ifndef APP_VOFA_H
#define APP_VOFA_H
#include "stm32f10x.h"                  // Device header

/*
 * 串口波形输出（VOFA+ / FireWater 协议）
 * ------------------------------------------------------------
 * 作用：把姿态解算的中间量按固定周期从串口发出去，用 VOFA+ 画成曲线。
 *       输出五个通道，叠在一张波形图里就能直观看出滤波效果：
 *
 *   ch0  加速度计直接解算的角度  —— 噪声大，抖动明显          [°]
 *   ch1  互补滤波后的角度        —— app_MPU6050 里原有的算法  [°]
 *   ch2  卡尔曼滤波后的角度      —— 本例的融合结果            [°]
 *   ch3  陀螺零偏标定值 gx_offset —— 上电标定测到的 x 轴零偏   [°/s]
 *   ch4  卡尔曼残余零偏估计      —— 扣掉 ch3 之后仍在修正的量  [°/s]
 *
 * ch3/ch4 的单位是 °/s，和角度不同量纲，建议只看右侧数值读数；
 * 想画曲线就另开一个波形图控件绑定这两路，否则会把角度曲线压扁。
 *
 * 数据帧格式（FireWater）：
 *   angle:<ch0>,<ch1>,<ch2>,<ch3>,<ch4>\r\n
 *   冒号是数据起点，逗号分隔通道，换行结束一帧。
 *   APP_VOFA_PROBE=1 时再追加 ch5/ch6/ch7（控制环探针），共 8 通道。
 *
 * 用法：main 里初始化一次 App_Vofa_Init()，
 *       然后在 while(1) 中调用 App_Vofa_Proc()（内部自己控周期）。
 * ------------------------------------------------------------
 */

/* 控制环探针通道（定位极限环用）
 *   0 = 只发姿态 5 通道（默认）
 *   1 = 追加三个探针通道：
 *       ch5  θ_ref   速度环输出 = 倒立摆外环设定值          [°]
 *       ch6  ẋ       速度环反馈（轮速 + 车体姿态补偿）       [cm/s]
 *       ch7  θ̇       角速度环反馈                           [°/s]
 * 判读方法：稳态摆动时看 ch5 是否和 ch2 同频、同量级摆动 ——
 *   同频同量级  → 速度环在回路里，摆动由外环驱动；
 *   幅度很小    → 外环基本不参与，摆动落在角度环/内环。 */
#ifndef APP_VOFA_PROBE
#define APP_VOFA_PROBE       1
#endif

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
