#include "app_vofa.h"
#include "usart.h"
#include "task.h"
#include "app_kalman.h"
#include "app_MPU6050.h"

#if APP_VOFA_ENABLE

#if   (APP_VOFA_PORT == 1)
  #define VOFA_USART      USART1
#elif (APP_VOFA_PORT == 2)
  #define VOFA_USART      USART2
#else
  #error "APP_VOFA_PORT 只能取 1 或 2；USART3 已经被遥控(app_rc)占用"
#endif

/* ---------------- 定点数格式化（不用 printf 的 %f） ----------------
 * 原因：Keil MicroLIB 和 newlib-nano 对浮点格式化支持不一致，
 *       直接用 %f 容易在一端编过、另一端输出乱码。
 *       这里把角度乘以 100 变成整数，手工拼小数点，两边行为完全一致。
 * ------------------------------------------------------------------ */
static char *Vofa_PutStr(char *p, const char *s)
{
	while (*s != '\0')
	{
		*p++ = *s++;
	}
	return p;
}

static char *Vofa_PutFixed(char *p, float v)
{
	int32_t  scaled;
	uint32_t mag, ip, fp;
	char     tmp[8];
	uint8_t  n = 0;

	/* 角度不可能超过 ±999.99°，这里只是防止异常值把整数撑爆 */
	if (v >  999.99f) v =  999.99f;
	if (v < -999.99f) v = -999.99f;

	if (v >= 0.0f)
	{
		scaled = (int32_t)(v * 100.0f + 0.5f);
	}
	else
	{
		scaled = (int32_t)(v * 100.0f - 0.5f);
	}

	if (scaled < 0)
	{
		*p++ = '-';
		mag  = (uint32_t)(-scaled);
	}
	else
	{
		mag = (uint32_t)scaled;
	}

	ip = mag / 100u;        // 整数部分
	fp = mag % 100u;        // 小数部分

	if (ip == 0u)
	{
		*p++ = '0';
	}
	else
	{
		while (ip > 0u)
		{
			tmp[n++] = (char)('0' + (ip % 10u));
			ip /= 10u;
		}
		while (n > 0u)
		{
			*p++ = tmp[--n];
		}
	}

	*p++ = '.';
	*p++ = (char)('0' + (fp / 10u));
	*p++ = (char)('0' + (fp % 10u));

	return p;
}

#endif /* APP_VOFA_ENABLE */


//
//@简介：初始化波形输出串口
//@说明：会覆盖该串口原有的配置。如果 APP_VOFA_PORT 选的是 2，
//       就不要在 main 里再调 App_USART2_Init()，以这里的波特率为准。
//
void App_Vofa_Init(void)
{
#if APP_VOFA_ENABLE
	GPIO_InitTypeDef  gpio = {0};
	USART_InitTypeDef uart = {0};

#if (APP_VOFA_PORT == 1)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

	gpio.GPIO_Pin   = GPIO_Pin_9;               // USART1_Tx
	gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &gpio);

	gpio.GPIO_Pin  = GPIO_Pin_10;               // USART1_Rx
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &gpio);
#else
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	gpio.GPIO_Pin   = GPIO_Pin_2;               // USART2_Tx
	gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &gpio);

	gpio.GPIO_Pin  = GPIO_Pin_3;                // USART2_Rx
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &gpio);
#endif

	uart.USART_BaudRate            = APP_VOFA_BAUD;
	uart.USART_Mode                = USART_Mode_Tx;   // 只发不收
	uart.USART_Parity              = USART_Parity_No;
	uart.USART_StopBits            = USART_StopBits_1;
	uart.USART_WordLength          = USART_WordLength_8b;
	uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(VOFA_USART, &uart);

	USART_Cmd(VOFA_USART, ENABLE);
#endif
}


//
//@简介：按固定周期发送一帧姿态数据（FireWater 协议）
//@说明：在 main 的 while(1) 里调用，内部用 PERIODIC 控制频率，
//       必须在 App_MPU6050_Proc() 和 App_Kalman_Proc() 之后调用，
//       否则发出去的是上一周期的数据。
//
void App_Vofa_Proc(void)
{
#if APP_VOFA_ENABLE
	char  buf[80];
	char *p = buf;

	PERIODIC(APP_VOFA_PERIOD_MS)

	p = Vofa_PutStr(p, "angle:");
	p = Vofa_PutFixed(p, App_Kalman_GetAccelPitch());   // ch0 加速度计原始
	*p++ = ',';
	p = Vofa_PutFixed(p, App_MPU6050_GetPitch());       // ch1 互补滤波
	*p++ = ',';
	p = Vofa_PutFixed(p, App_Kalman_GetPitch());        // ch2 卡尔曼
	*p++ = ',';
	p = Vofa_PutFixed(p, App_Kalman_GetGyroOffset());   // ch3 零偏标定值  [°/s]
	*p++ = ',';
	p = Vofa_PutFixed(p, App_Kalman_GetPitchBias());    // ch4 卡尔曼残余零偏  [°/s]
	*p++ = '\r';
	*p++ = '\n';

	My_USART_SendBytes(VOFA_USART, (const uint8_t *)buf, (uint16_t)(p - buf));
#endif
}
