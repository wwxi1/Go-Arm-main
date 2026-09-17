# 硬件说明与引脚速查

> 原理图文件：仓库根目录 `H7.pdf`。本文件只做软件侧引脚速查，实际设计以原理图为准。

## 1. 板载资源

| 功能 | 引脚 | 说明 |
| --- | --- | --- |
| LED1 | PB3 | 高电平点亮 |
| LED2 | PB4 | 高电平点亮 |
| LED3 | PB5 | 高电平点亮 |
| LED4 | PB6 | 高电平点亮 |
| Buzzer | PB0 | 高电平鸣响 |

## 2. FDCAN

| 外设 | RX | TX | 模板默认用途 |
| --- | --- | --- | --- |
| FDCAN1 | PD0 | PD1 | 用户自定义 CAN |
| FDCAN2 | PB12 | PB13 | VESC + ZDrive 第一路 |
| FDCAN3 | PF6 | PF7 | DJI M2006/M3508 + ZDrive 第二路(拆分时) |

- 按经典 CAN 使用(CAN2.0、1 Mbps nominal,无 BRS);
- 全局滤波 + Start 在 `Motor/src/FD_Canqueue.c` 的 `CAN_Start()`;
- 接收:中断入队 → TIM2 中断统一出队分发;发送:入队 → TIM2 中断 1 kHz 出队。

## 3. UART

| 外设 | 默认波特率 | DMA RX | 模板默认用途 |
| --- | --- | --- | --- |
| USART1 | 115200 | 有 | 预留 |
| USART2 | 115200 | 无 | 预留(要 DMA 需在 CubeMX 加) |
| USART3 | 115200 | 有 | 预留 |
| UART4 | 115200 | 有 | 预留 |
| USART6 | 115200 | 有 | 预留 |
| UART7 | 4000000 | 有 | Unitree GO-M8010-6 RS485 |
| UART9 | 115200 | 有 | VOFA+ 打波调试输出(JustFloat) |

接收统一走 `HAL_UARTEx_ReceiveToIdle_DMA()`,在 `main.c` USER CODE 2 启动,
回调在 `Application/src/IQRhandler.c`,具体启用方式见 `porting_guide.md` 第 5 节。

> UART7 当前接宇树 GO 电机，RS485 半双工。该电机带绝对编码器，驱动默认不上电自动清零，上电前需确认机械零位并调用 `UnitreeMotor_SetZero(id)`。

> UART9(PD14/PD15,115200)接 VOFA+ 打波数据,USB-TTL 接 PC 需共地。
> 上位机协议选 **JustFloat**,通道数 6;API 见 [vofa_plus.md](vofa_plus.md)。

## 4. 定时器

| 外设 | 模板用途 |
| --- | --- |
| TIM1 | HAL 时基，1 kHz |
| TIM2 | 电机/控制定时器，1 kHz(报文发送 1 kHz,func counter ÷5 = 200 Hz) |
| TIM3/4/5 | 备用，1 kHz,在 IQRhandler.c 加分支使用 |

## 5. 供电与时钟

- MCU：STM32H723ZETx
- 电源：LDO 供电（`PWR_LDO_SUPPLY`）
- HSE：按 CubeMX 配置，PLL 后系统时钟以 `SystemClock_Config()` 为准
- FDCAN 时钟：PLL2，`PeriphCommonClock_Config()` 中配置
