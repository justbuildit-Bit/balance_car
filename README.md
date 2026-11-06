# balance_car

基于 STM32F103C8 的两轮自平衡车固件，Keil MDK 工程。

## 硬件

| 模块 | 型号 / 说明 |
|---|---|
| MCU | STM32F103C8T6（Cortex-M3，64KB Flash / 20KB RAM） |
| 姿态传感器 | MPU6050（I2C） |
| 显示 | OLED（I2C，支持软件 / 硬件 I2C 两套驱动） |
| 电机 | 带编码器的直流减速电机 + PWM 驱动 |
| 其他 | 按键、电池电压检测（ADC）、USART2、遥控输入 |

## 代码结构

```
user/                应用层，每个模块遵循 Xxx_Init / Xxx_Proc 的写法
  main.c             外设初始化 + 主循环调度
  app_control.c      直立与运动控制，对外提供速度 / 转向设定接口
  app_kalman.c       卡尔曼滤波姿态解算
  app_MPU6050.c      IMU 数据读取 + 互补滤波（保留用于对照）
  app_motor.c        电机输出
  app_encoder.c      编码器测速
  PID.c / PID.h      通用 PID（Kp/Ki/Kd + 设定值 + 输出限幅）
  app_pwm.c          电机 PWM
  app_bat.c          电池电压采集
  app_button.c       按键
  app_usart2.c       串口
  app_rc.c           遥控输入
my_lib/              通用驱动：oled、i2c、si2c、button、pid、qmath、delay
std_periph_driver/   ST 标准外设库
startup/             启动文件与启动汇编
test/                硬件模块自测代码（在 main 中默认注释掉，需要时单独打开）
```

主循环里 `App_Kalman_Proc()` 必须排在 `App_MPU6050_Proc()` 之后，否则读到的是上一周期的数据；上电后 `App_Kalman_GyroCalibrate(300)` 让车体静止约 0.3s 完成陀螺仪零偏标定。

## 编译

1. Keil MDK 打开 `balance_car.uvprojx`
2. Target 选 `default`，直接 Build
3. `清理.cmd` 可一键清除 `Objects/`、`Listings/`、`DebugConfig/`

## 关于字库

OLED 使用的中文字模由 `my_lib/font/` 下的 TTF 文件生成，字体文件和转换工具体积较大且涉及版权，**未纳入本仓库**（见 `.gitignore`）。编译不受影响，因为生成好的字模头文件 `my_lib/oled_default_font.h` 已经入库。若要重新生成字库，本地自备字体后使用 `ttf2c` / `bdf2c`。
