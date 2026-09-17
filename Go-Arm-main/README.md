# 27RC_Proj_Template — STM32H7 通用电控工程模板

基于 CubeMX 生成的 STM32H723ZETx + FreeRTOS(CMSIS-OS2) 工程,按团队习惯整理,
目标是覆盖 80% 场景、上手难度低。架构参考 `R2_Chassis-chassis_main`。

## 已移植内容

| 模块 | 位置/说明 |
| --- | --- |
| LED | `BSP/bsp_led.c`,PB3~PB6,带流水灯 |
| Buzzer | `BSP/bsp_buzzer.c`,PB0,开机音/报警音 |
| 数学库 | `Algorithm/`,mathFunc/pid/vector/filter/ring_buffer |
| FDCAN 队列 | `Motor/FD_Canqueue.c`,从 R2 底盘工程移植,含启动/滤波/队列保护 |
| DJI M2006/M3508 | `Motor/DJmotor.c`,级联位置-速度-电流 PID |
| VESC | `Motor/VescMotor.c`,速度/位置/电流/占空比/手刹 |
| ZDrive | `Motor/ZDrive.c`,电流/速度/位置/PVT 及参数查询 |
| Unitree GO-M8010-6 | `Motor/src/UnitreeMotor.c`,UART7 单路 RS485,绝对编码器,上电默认不自动清零 |
| 中断回调 | `Application/IQRhandler.c`,TIM/FDCAN/UART 回调集中 |
| 任务 | CubeMX 创建,`Application/myostasks.c` 覆盖弱函数 |
| VOFA+ 打波 | `Communication/VOFA/vofa.c`,JustFloat 协议,UART9 输出,API 见 [DOCS/vofa_plus.md](DOCS/vofa_plus.md) |
| 蓝牙串口 | `Communication/Bluetooth/`,帧层 bluetooth.c + Payload 协议层 bluetooth_protocol.c,USART6,见 [DOCS/bluetooth.md](DOCS/bluetooth.md) |

## 目录结构

```text
27RC_Proj_Template/
├─ Motor/            电机驱动 + CAN 队列
├─ Communication/    对外数据通信:Bluetooth(帧层 + Payload 协议层)、VOFA 打波
├─ Application/      任务函数、中断回调、业务逻辑、includes.h 聚合头
├─ BSP/              板级支持:LED、Buzzer
├─ Algorithm/        纯算法库:PID、滤波、数学工具
├─ Config/           配置宏:board_config / motor_config / app_config
├─ Core/             CubeMX 生成代码(只改 USER CODE 区域)
├─ Middlewares/      FreeRTOS、CMSIS DSP
├─ MDK-ARM/          Keil 工程(uvprojx / startup / sct 平铺)
└─ DOCS/             架构说明、移植指南、硬件说明、C 语言规范
```

## 快速上手

1. 打开 `MDK-ARM/27RC_Proj_Template.uvprojx`,编译下载。
2. 命令行编译(可选):`powershell -File Tools\build.ps1`(增量)/ `-Rebuild`(全量),
   日志在 `Tools\logs\`,详见 [Tools/README.md](Tools/README.md)。
3. 默认行为:
   - 上电蜂鸣器响两声;
   - LED 流水灯运行在 CubeMX 创建的 `LEDTask`;
   - TIM2 中断 1 kHz:电机报文发送 + 接收分发;func 更新 200 Hz(counter ÷5)。
4. 电机配置:
   - 开关与数量/总线/减速比:[Config/motor_config.h](Config/motor_config.h)
     (`USE_DJ / USE_VESC / USE_ZMDR / USE_UNITREE` 开关也在这里);
   - FDCAN2:VESC + ZDrive 第一路;FDCAN3:DJI + ZDrive 第二路(拆分时);FDCAN1:预留给用户;
     ZDrive 拆分:`MOTOR_ZDRIVE_SPLIT_COUNT` = 0 不拆分,= n 则 ID 1..n 走第一路、其余走第二路。
5. 电机默认失能:ZDrive 用驱动自身的 `Zdrive_Disable` 模式(无 Enable 标志),
   初始化完成后置 `Begin = true`,200 Hz 的 Func 会自动完成模式切换并下发控制帧;
   DJI 用 `MODE = DJ_Disable` 失能(协议无 disable 态,发 0 电流);
   VESC 仍用 `Enable = true` 使能。
   - Unitree GO-M8010-6 带绝对编码器，驱动默认**不在上电时自动清零**；上电/首次使能前应在机械零位调用 `UnitreeMotor_SetZero(id)`，否则位置控制会以电机出厂绝对零位为参考。

```c
Zmotor[0].Begin  = true;              /* 初始化完成后置位 */
Zmotor[0].mode   = Zdrive_Postion;    /* 停止:mode = Zdrive_Disable */
Zmotor[0].valSetNow.pos_deg = 90.0f;    /* 目标角度 90° */
```

5. VOFA+ 打波(默认开启,`APP_VOFA_ENABLE = 1`):
   - 数据出口 UART9(PD14/PD15,115200),接 USB-TTL 到电脑并共地;
   - 模板在 `VOFASendTask`(10 ms)里示例:通道 0 打一个递增计数;
   - 自己的数据用 `VOFA_Channel_Update(ch, type, &val)` 填进缓存,
     再调 `VOFA_Update()` 统一发出,详见 [DOCS/vofa_plus.md](DOCS/vofa_plus.md)。

## CubeMX 重新生成后的注意事项

- 只修改 CubeMX 生成文件的 `USER CODE` 区域,模板已按此原则接入;
- 若 CubeMX 覆盖 `MDK-ARM/27RC_Proj_Template.uvprojx`,需要重新添加
  `Motor`、`Application`、`BSP`、`Algorithm` 四个 Group、
  各模块 inc 目录的 include path,并把链接器 ScatterFile 指回
  `.\stm32h723zxx_flash.sct`(见 `DOCS/porting_guide.md` 第 9 节)。

## 文档

| 文档 | 内容 |
| --- | --- |
| [DOCS/README.md](DOCS/README.md) | 文档索引 |
| [DOCS/architecture.md](DOCS/architecture.md) | 目录结构、中断/电机时序、FreeRTOS 约定、内存布局 |
| [DOCS/porting_guide.md](DOCS/porting_guide.md) | 移植记录、新增电机/UART/TIM/任务指南 |
| [DOCS/hardware_notes.md](DOCS/hardware_notes.md) | 板载资源与引脚速查 |
| [DOCS/vofa_plus.md](DOCS/vofa_plus.md) | VOFA+ 打波:协议/接线/API 与上位机配置 |
| [DOCS/coding_standard.md](DOCS/coding_standard.md) | C 语言规范摘要 |
| [项目知识补全清单.md](项目知识补全清单.md) | 嵌入式 + 运动控制知识点清单：RS485 DMA 双缓冲原理、机械臂控制方式、扩展发散 |
