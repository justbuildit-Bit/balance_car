# balance_car

基于 STM32F103C8 的两轮自平衡车固件，Keil MDK 工程。

![编译校验](https://github.com/justbuildit-Bit/balance_car/actions/workflows/build.yml/badge.svg)

## 效果展示

![整车](docs/images/car.jpg)

双轮差速结构，车体上方为两节锂电池供电，前部是主控板。

![实车直立演示](docs/images/demo.gif)

实车直立行走，通过手机 App 遥控运行。上面是压缩后的预览（260px / 6fps / 6s），[**完整视频**](docs/images/demo.mp4) 为 720×1280、23 秒。

### 姿态解算：滤波效果实测

![姿态曲线](docs/images/attitude.jpg)

VOFA+ 实测（车体直立、电机通电保持平衡的工况），把三路俯仰角画在同一坐标系里对照：

| 曲线 | 内容 | 峰峰值 | 现象 |
|---|---|---|---|
| 粉 | 加速度计 `atan2` 直接解算 | **约 1.0°** | 电机振动和车体微动直接变成角度毛刺，尖峰密且幅度大 |
| 绿 | 互补滤波（α=0.952，时间常数 0.1s） | 约 0.2° | 明显平滑，但每个振动尖峰都还能透进来，且整体比卡尔曼低约 0.3° |
| 紫 | 卡尔曼滤波 | **< 0.1°** | 几乎一条直线，同时仍跟得上车体慢变的倾角趋势 |

这三个现象都能解释清楚，不是"调出来的好看"：

1. **降噪是两级叠加的**。互补滤波先把加速度计噪声低通掉一部分；卡尔曼在此基础上还有一层——`KALMAN_ACC_GATE` 用 `|a|²` 判据（0.7g ~ 1.3g）把振动尖峰对应的量测整体丢弃，只做预测不做校正。粉色曲线上那些尖峰落在这个窗口之外，所以紫色曲线几乎不受影响。这是它比绿色平得多的直接原因。
2. **0.3° 的稳态偏差，账其实记在上电标定值上**。`App_Kalman_GyroCalibrate(300)` 测出 x 轴零偏 `gx_offset`，但这个值只在 `App_Kalman_Proc()` 里被扣掉（`gx = GetGx() - gx_offset`）；`app_MPU6050.c` 的互补滤波仍然用原始 `gx`。于是零偏 b 被持续积分，稳态偏差就是 b·τ（τ = α·dt/(1-α) = 0.1 s）。按图上 0.28° 的差反推 b ≈ 2.8°/s —— **这个数可以直接用 `ch3` 打出来对账**，若 `ch3` 落在 -2.8 附近，则 -2.8 × 0.1 ≈ -0.28°，账就闭环了。卡尔曼把零偏当作第二个状态量在线估计并扣掉（`ch4` 就是扣完之后仍在修正的残余量），所以它没有这个偏差。控制环用的是 `App_Kalman_GetPitch()`，因此绿色那条线偏低并**不影响平衡**。
3. **对比要在同一时刻看**。三路数据在同一个 `App_Vofa_Proc()` 里、同一周期发出，没有采样时差，所以曲线之间的横向错位就是纯粹的相位差，不是通信延迟造成的假象。

| 内容 | 状态 |
|---|---|
| 整车照片 | ✅ [docs/images/car.jpg](docs/images/car.jpg) |
| 实车演示视频 | ✅ [docs/images/demo.mp4](docs/images/demo.mp4) |
| 姿态曲线 | ✅ [docs/images/attitude.jpg](docs/images/attitude.jpg) |
| 参数整定 | 待补充（各 PID 环的最终参数与整定记录） |


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
  app_vofa.c         串口波形输出（VOFA+ FireWater 协议），发原始/互补/卡尔曼三路角度 + 两路零偏量
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

## 调试与可视化（VOFA+）

固件把姿态解算的中间量按固定周期从串口发出去，用 [VOFA+](https://www.vofa.plus/) 就能把「加速度计原始角度」和「滤波后角度」画在同一张波形图里，滤波效果一眼可见。

**接线**（默认走 USART2，PA2/PA3 空闲可用）：

```
STM32 PA2 (USART2_TX) → USB-TTL 的 RX
STM32 GND             → USB-TTL 的 GND
```

**VOFA+ 设置**：数据引擎选 **FireWater**，波特率 **115200**，8 数据位 / 无校验 / 1 停止位。然后从左侧拖一个「波形图」控件，通道按顺序对应：

| 通道 | 含义 | 曲线特征 |
|---|---|---|
| `ch0` | 加速度计直接解算的角度（`App_Kalman_GetAccelPitch`） | 噪声大，上下抖动明显 |
| `ch1` | 互补滤波角度（`App_MPU6050_GetPitch`，原有算法） | 平滑，但振动尖峰仍会透进来，且因未扣陀螺零偏存在固定偏置 |
| `ch2` | 卡尔曼滤波角度（`App_Kalman_GetPitch`） | 平滑且滞后更小，还能估出陀螺零偏 |
| `ch3` | 上电标定的陀螺零偏 `x` 轴分量（`App_Kalman_GetGyroOffset`），**°/s** | 一条水平线，只有上电标定后才有值 |
| `ch4` | 卡尔曼残余零偏估计（`App_Kalman_GetPitchBias`），**°/s** | 上电后几十秒内收敛到 0 附近 |

前三路的实测对照见上文 [姿态解算：滤波效果实测](#姿态解算滤波效果实测)。**后两路是角度偏差的"账本"**：`ch1` 比 `ch2` 低多少，理论上就等于 `ch3 × 0.1s`（0.1s 是互补滤波的时间常数）。因为单位是 °/s 而不是 °，直接画在同一张图里会把角度曲线压扁，看右侧数值读数就行；要看曲线另拖一个波形图控件绑定 `ch3`/`ch4`。

实际发出的数据帧长这样（50Hz）：

```
angle:1.58,1.03,1.31,-2.84,0.07
      └ch0┘ │   │    └ch3┘ └ch4┘  ← 后两路单位是 °/s
           ch1 ch2
```

**相关配置**都在 `user/app_vofa.h`：

```c
#define APP_VOFA_ENABLE      1       // 发布固件时改成 0，编译后完全不占用 CPU 和串口
#define APP_VOFA_PORT        2       // 2 = USART2(PA2/PA3)，1 = USART1(PA9/PA10)
#define APP_VOFA_BAUD        115200  // 必须和 VOFA+ 一致
#define APP_VOFA_PERIOD_MS   20      // 20ms = 50Hz
```

两个实现细节值得说明：

- **不用 `printf("%f")`**：Keil MicroLIB 与 newlib-nano 对浮点格式化的支持不一致，直接用 `%f` 容易一端编过、另一端输出乱码。模块内部把角度乘 100 转成整数，手工拼小数点，两种工具链行为完全一致。
- **发送放在两个滤波之后**：`App_Vofa_Proc()` 必须排在 `App_MPU6050_Proc()` 和 `App_Kalman_Proc()` 之后，否则发出去的是上一周期的数据。



## 关于字库

OLED 使用的中文字模由 `my_lib/font/` 下的 TTF 文件生成，字体文件和转换工具体积较大且涉及版权，**未纳入本仓库**（见 `.gitignore`）。编译不受影响，因为生成好的字模头文件 `my_lib/oled_default_font.h` 已经入库。若要重新生成字库，本地自备字体后使用 `ttf2c` / `bdf2c`。

## 许可证

本仓库以 MIT 许可证发布，详见 [LICENSE](LICENSE)。

其中 `my_lib/` 下部分通用驱动（i2c、si2c、usart 等）源自公开教程的示例代码，文件头保留了原作者署名；`std_periph_driver/` 为 ST 官方标准外设库，`cmsis_gcc.h` 来自 ARM CMSIS，各自遵循其原始许可。商用或再分发前请自行确认相应条款。

