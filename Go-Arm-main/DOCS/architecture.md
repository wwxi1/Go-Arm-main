# 27RC_Proj_Template 架构说明

> 架构以团队习惯为准,优先上手难度低,
> 简略代码规范见 [coding_standard.md](coding_standard.md)。

## 1. 目录结构

```text
27RC_Proj_Template/
├── 27RC_Proj_Template.ioc     CubeMX 工程(FDCAN/UART/TIM/任务都在这里配)
├── Core/                      CubeMX 生成,只改 USER CODE 区域
├── Drivers/                   CubeMX 生成(HAL/CMSIS)
├── Middlewares/               CubeMX 生成(FreeRTOS)
├── MDK-ARM/                   Keil 工程(uvprojx/startup/sct 平铺在这里)
├── Motor/                     电机与 CAN 队列
│   ├── inc/                   DJmotor.h VescMotor.h ZDrive.h UnitreeMotor.h FD_Canqueue.h
│   └── src/
├── Communication/             对外数据通信 / 数据输出
│   ├── Bluetooth/             bluetooth.h/c(帧层+Queue)、bluetooth_protocol.h/c(Payload 协议层)
│   └── VOFA/                  vofa.h/c(VOFA+ 打波)
├── Application/               任务函数、中断回调、业务逻辑、聚合头
│   ├── inc/                   includes.h IQRhandler.h myostasks.h bluetooth_serv.h
│   └── src/                   IQRhandler.c myostasks.c bluetooth_serv.c
├── BSP/                       板级资源(LED/蜂鸣器)
├── Algorithm/                 纯算法(PID/滤波/数学)
└── Config/                    配置宏(motor_config/board_config/app_config)
```

约定:

- 用户模块目录统一"大写模块名 + 小写 `inc/` `src/`"(Communication 内按通信子模块各建一个目录)。
- Keil include path 已包含各模块 inc 目录与 `Communication/Bluetooth`、`Communication/VOFA`,
  代码里**不写相对路径**,直接 `#include "ZDrive.h"`。新增模块时在 Keil 工程里加 inc 目录即可。
- 用户文件习惯先 `#include "includes.h"`(标准库 + RAM 段宏 + 电机开关)。
- CubeMX 生成文件只改 USER CODE 区域,重新生成不会丢代码。

## 2. 中断与电机时序(核心)

所有 HAL 回调强定义集中在 [Application/src/IQRhandler.c](../Application/src/IQRhandler.c)
(CubeMX 生成的是 `__weak`,重新生成不冲突)。main.c 里 CubeMX 生成的
`HAL_TIM_PeriodElapsedCallback` 在 USER CODE 里转发到
`TIM_PeriodElapsedCallback()`。

```text
TIM2 中断 (1 kHz)
 ├─ CAN_DequeueTx(CAN1_Txqueue)          → CAN1 用户通信出队
 ├─ ZdriveDequeue(bus1/bus2)             → ZDrive 总线出队(1000 Hz 恒定发送)
 └─ counter 每 5 次 (200 Hz) → DJmotor_Func / VescFunc / ZdriveFunc

FDCAN RX 中断
 └─ 各电机反馈帧直接解析(不进软件 RX 队列):
      ZDrive: Zdrive_IsOurs? → ZdriveReceive
      DJI:    DJmotor_Receive;VESC: VescReceiveData_CAN2
      CAN1 用户通信帧在这里直接解析

UART 空闲中断 (HAL_UARTEx_RxEventCallback)
 └─ 数据可用 → 协议处理写在这里 → 重新启动 DMA 接收

 UART 发送完成中断 (HAL_UART_TxCpltCallback)
  ├─ RS-485用于软件收发切换
  └─ 转发给 VOFA_DMA_TransmitCpltCallback 清 DMA 忙标志(打波发送)
 
```

电机时序约定:

- 电机报文发送固定在 **1000 Hz**(TIM2 中断里出队,CAN1 通用出队、
  ZDrive 总线由 `ZdriveDequeue(bus)` 出队,命名归驱动所有);
- 电机 func 更新固定在 **200 Hz**(TIM2 1 kHz 用 counter ÷5);
- DJI 电机若需要 1 kHz 电流环,把 `DJmotor_Func()` 挪到 IQRhandler.c 的 1 kHz 区即可;
- func 里只做计算和入队,真正发帧由下一次 TIM2 中断统一出队;
- 各电机(ZDrive/DJI/VESC)反馈帧都在 **FDCAN 接收中断里直接解析**,
  不经过软件 RX 队列(FDCAN 与 TIM2 同为优先级 5,同优先级互不抢占,
  写电机状态与 func 读电机状态天然互斥)—— 反馈时效最好,少一跳缓冲;

## 3. FreeRTOS 约定

- 任务**在 CubeMX 里创建**(.ioc → freertos.c),用户代码不建任务;
- 任务函数在 freertos.c 里是 `__weak` 占位,业务实现在
  [Application/src/myostasks.c](../Application/src/myostasks.c) 强定义覆盖;
- 模板自带 `LEDTask`(四灯流水示例)与 `VOFASendTask`(VOFA+ 打波,10 ms,
  见 [vofa_plus.md](vofa_plus.md));
- 不提供信号量/队列的封装服务,需要时直接调 CMSIS-OS2 API。

## 4. 电机接口

每类电机独立 API(无统一电机接口),由 IQRhandler 在定时器中断里调度:

```text
ZDrive:  ZdriveInit / ZdriveDequeue / Zdrive_IsOurs /
         ZdriveReceive / ZdriveFunc / ZdriveSet / ZdriveAsk /
         ZdriveParamConfig / ZdriveSetPVT ...
         统一 set:ZdriveSet 按 set_code(命令码)内部完成单位换算、
         配置命令读回(Vel_Limit/Acc_Acu/Acc_Dec 自动 Ask);PVT 双值
         帧独立为 ZdriveSetPVT
         通信端命名对齐:入队 ZdriveEnqueue、1kHz 恒定出队 ZdriveDequeue、
         接收过滤 Zdrive_IsOurs + 解析 ZdriveReceive(反馈帧在 FDCAN 中断直解析)
DJI:     DJmotor_Init / DJmotor_Receive / DJmotor_Func / DJmotor_SpeedMode / DJmotor_PositionMode ...
VESC:    VescInit / VescReceiveData_CAN2 / VescFunc / VescRPM_Mode / VescPosition_Mode ...
Unitree: UnitreeMotor_Init / UnitreeMotor_Func / UnitreeMotor_UART_RxHandler / UnitreeMotor_Enable / UnitreeMotor_Disable / UnitreeMotor_SetZero
```

- 开关:`Config/motor_config.h` 里 `USE_DJ / USE_VESC / USE_ZMDR / USE_UNITREE`,置 0 不编译对应驱动;
- 数量/总线/减速比:同上,`Config/motor_config.h`;
- 发送方式:ZDrive 走 `ZdriveEnqueue` 入队(内部按帧 ID 解析总线,统一
  `CAN_Enqueue`),VESC 直接入队 `CANx_Txqueue`;DJI 直接写 TX FIFO;
  ZDrive 总线出队由 `ZdriveDequeue(bus)` 负责(TIM2 1 kHz 每 tick 每总线 1 帧);
  接收无软件队列(RX 队列已移除),反馈帧全部在 FDCAN 接收中断里直解析;
- `ZdriveParamConfig(id, param)` 覆盖指定电机 param;param 里 PID 拆为
  `kpPos/kiPos/kpVel/kiVel` 四项,哪一项改动就自动调 `ZdriveSet` 下发对应 PID 命令
  (PID_POS_P/PID_POS_I/PID_VEL_P/PID_VEL_I)。
- Unitree GO-M8010-6 为绝对编码器电机，驱动默认**不上电自动清零**；上电/首次使能前应先在机械零位调用 `UnitreeMotor_SetZero(id)`。

## 5. VOFA+ 打波调试

上位机 [VOFA+](https://www.vofa.plus/),协议 **JustFloat**:
每帧 6 通道 × float32(小端)+ 帧尾 `00 00 80 7F`,共 28 字节,
经 UART9(PD14/PD15,115200)DMA 发出。模块在 `Communication/VOFA/vofa.c`,
配置宏在 `Config/app_config.h`(`APP_VOFA_ENABLE` / `APP_VOFA_TASK_PERIOD_MS` / `BOARD_VOFA_UART`)。

```c
/* 发送任务里:先填通道,再统一发送(模板 VOFASendTask 10 ms 一次) */
VOFA_Channel_Update(0, VOFA_TYPE_UINT8, &cnt);        /* 任意整型/浮点,自动转 float */
VOFA_Channel_Update(1, VOFA_TYPE_FLOAT, &angle_deg);
VOFA_Update();                                        /* DMA 发送,忙时自动跳过 */
```

发送完成中断已由 `HAL_UART_TxCpltCallback` 转发,用户无需挂回调。
完整 API、上位机设置与注意事项见 [vofa_plus.md](vofa_plus.md)。

## 6. 内存布局(MDK-ARM/stm32h723zxx_flash.sct)

| 段 | 宏 | 地址 | 用途 |
| --- | --- | --- | --- |
| DTCM | (默认) | 0x20000400 起 | 默认数据/栈,前 1 KB 是重定向的向量表 |
| .RAM_D1 | `__RAM_D1_` | 0x24000000(AXI SRAM, D-Cache) | 大块计算数据 |
| .RAM_D2 | `__RAM_D2_` | 0x30000000(SRAM1/2/3) | DMA 可达缓冲区(UART 接收缓冲在这) |
| .RAM_D3 | `__RAM_D3_` | 0x38000000(SRAM4) | 只有 BDMA 可达,按需使用 |

> 注意:DMA1/DMA2 无法访问 SRAM4(D3),UART DMA 缓冲必须放 D2。
> 启用 D-Cache 后,DMA 接收完先 `SCB_InvalidateDCache_by_Addr()`,
> DMA 发送前先 `SCB_CleanDCache_by_Addr()`。

## 7. 启动流程

```text
main()
 ├─ CubeMX: HAL_Init / SystemClock / MX_GPIO / MX_DMA / MX_FDCAN / MX_UART / MX_TIM
 ├─ USER CODE 2(平铺,参考工程习惯):
 │    Bsp_BoardInit();  BspBuzzer_StartupBeep();
 │    CAN_InitSendQueue();  CAN_Start();          ← 队列初始化 + FDCAN 启动/滤波
 │    DJmotor_Init();  VescInit();  ZdriveInit();  UnitreeMotor_Init();
 │    HAL_TIM_Base_Start_IT(&htim2);              ← 1 kHz 电机/控制定时器
 │    各 UART HAL_UARTEx_ReceiveToIdle_DMA(...)
 ├─ osKernelInitialize() → MX_FREERTOS_Init() → osKernelStart()
 └─ LEDTask 跑流水灯,其余业务在中断里跑
```


## 8. 蓝牙串口模块(Bluetooth/)

上位机/调试用的蓝牙串口通信,见 [bluetooth.md](bluetooth.md)。
- 硬件:USART6(PC6=TX, PC7=RX, 115200),串口号在 `Config/board_config.h` 的
  `BOARD_BLUETOOTH_UART`;
- 职责分层:UART 层(CubeMX + IQRhandler 回调)→ Bluetooth 帧层(队列/组帧/CRC,
  `Communication/Bluetooth` 的 bluetooth.h/c)→ Bluetooth 协议层(宏驱动的定长
  Payload 编解码,`Communication/Bluetooth` 的 bluetooth_protocol.h,BT_Proto_*
  逐字段访问函数)→ 业务层(`Bluetooth_Send()` / `Bluetooth_Receive()` + 自己的
  解析/打包函数);
- 并发原语:数据用基础 FreeRTOS Queue(`xQueueCreate/Send/SendFromISR/Receive`),
  中断→接收任务的唤醒用计数信号量(`xSemaphoreCreateCounting/GiveFromISR/Take`);
  收发任务在模块内部 `Bluetooth_Init()` 里创建(main.c 启动时调用);
- 中断里只搬数据 + 发信号,包头/包尾/校验解析全在接收任务里;
- Payload 协议最小示例(消息 Pack/Parse 写法)见 `Application` 的 bluetooth_serv.c。
