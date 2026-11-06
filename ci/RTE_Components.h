#ifndef RTE_COMPONENTS_H
#define RTE_COMPONENTS_H

/**
  ******************************************************************************
  * @file    RTE_Components.h
  * @brief   Keil RTE 生成的组件宏（GCC 编译校验用替身）
  *
  * 正式的 RTE_Components.h 由 Keil MDK 在 build 时自动生成到工程目录，不入库。
  * 命令行（arm-none-eabi-gcc / make）编译时需要它来决定 stm32f10x_conf.h
  * 里哪些外设头文件被包含，所以这里放一份等价的静态版本，只给 CI 使用。
  *
  * 如果以后在 Keil 里新勾选了外设模块，记得同步这里，否则 CI 会缺头文件。
  ******************************************************************************
  */

#define RTE_DEVICE_STDPERIPH_FRAMEWORK

#define RTE_DEVICE_STDPERIPH_ADC
#define RTE_DEVICE_STDPERIPH_BKP
#define RTE_DEVICE_STDPERIPH_CAN
#define RTE_DEVICE_STDPERIPH_CEC
#define RTE_DEVICE_STDPERIPH_CRC
#define RTE_DEVICE_STDPERIPH_DAC
#define RTE_DEVICE_STDPERIPH_DBGMCU
#define RTE_DEVICE_STDPERIPH_DMA
#define RTE_DEVICE_STDPERIPH_EXTI
#define RTE_DEVICE_STDPERIPH_FLASH
#define RTE_DEVICE_STDPERIPH_FSMC
#define RTE_DEVICE_STDPERIPH_GPIO
#define RTE_DEVICE_STDPERIPH_I2C
#define RTE_DEVICE_STDPERIPH_IWDG
#define RTE_DEVICE_STDPERIPH_PWR
#define RTE_DEVICE_STDPERIPH_RCC
#define RTE_DEVICE_STDPERIPH_RTC
#define RTE_DEVICE_STDPERIPH_SDIO
#define RTE_DEVICE_STDPERIPH_SPI
#define RTE_DEVICE_STDPERIPH_TIM
#define RTE_DEVICE_STDPERIPH_USART
#define RTE_DEVICE_STDPERIPH_WWDG

#endif /* RTE_COMPONENTS_H */
