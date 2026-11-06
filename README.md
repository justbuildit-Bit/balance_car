# balance_car

基于 STM32F103C8 的两轮自平衡车固件，Keil MDK 工程。

![编译校验](https://github.com/justbuildit-Bit/balance_car/actions/workflows/build.yml/badge.svg)

## 效果展示

> 📷 **实车与实测数据**（占位，待补充）

| 内容 | 说明 |
|---|---|
| 实车照片 | 整车俯视图 / 侧面图 |
| 直立演示 | 松手后稳定直立的动图或短视频 |
| 姿态曲线 | MPU6050 原始数据与卡尔曼滤波后角度的对比曲线 |
| 参数整定 | PID 三个环的最终参数与整定过程记录 |

素材放到 `docs/images/` 下，然后把这里的占位替换成：

```markdown
![整车](docs/images/car.jpg)
![直立演示](docs/images/demo.gif)
```


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

正式固件用 Keil MDK：

1. Keil MDK 打开 `balance_car.uvprojx`
2. Target 选 `default`，直接 Build
3. `清理.cmd` 可一键清除 `Objects/`、`Listings/`、`DebugConfig/`

### GCC 编译校验（CI）

仓库另配了一份 `Makefile`，用 `arm-none-eabi-gcc` 把每个源文件编译成 `.o`，**只做编译校验，不链接、不产出固件**——固件的编译和下载仍然走 Keil。每次 push 和 PR 都会由 GitHub Actions 自动跑一遍，保证提交的代码是可编译的。

本地跑（需要先装好 ARM 交叉编译工具链）：

```bash
make        # 编译全部源文件到 build/
make clean  # 清理
```

为了能在 GCC 下编过，工程做了三处兼容：

- `std_periph_driver/inc/cmsis_gcc.h` — CMSIS 5.6 的 GCC 支持头（原工程只有 ARMCC 版本）
- `my_lib/compiler_compat.h` — 把 ARMCC 关键字 `__weak` 映射为 `__attribute__((weak))`
- `ci/RTE_Components.h` — Keil 自动生成的 RTE 组件宏的静态替身

Keil 编译完全不受影响，这三处对 ARMCC 都是空操作。

## 关于字库

OLED 使用的中文字模由 `my_lib/font/` 下的 TTF 文件生成，字体文件和转换工具体积较大且涉及版权，**未纳入本仓库**（见 `.gitignore`）。编译不受影响，因为生成好的字模头文件 `my_lib/oled_default_font.h` 已经入库。若要重新生成字库，本地自备字体后使用 `ttf2c` / `bdf2c`。

## 许可证

本仓库以 MIT 许可证发布，详见 [LICENSE](LICENSE)。

其中 `my_lib/` 下部分通用驱动（i2c、si2c、usart 等）源自公开教程的示例代码，文件头保留了原作者署名；`std_periph_driver/` 为 ST 官方标准外设库，`cmsis_gcc.h` 来自 ARM CMSIS，各自遵循其原始许可。商用或再分发前请自行确认相应条款。

