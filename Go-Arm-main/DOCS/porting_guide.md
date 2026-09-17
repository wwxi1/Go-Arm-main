# 移植与二次开发指南

## 1. 从 R2_Chassis-chassis_main 移植的内容

> Keil 工程已把 `Options for Target → Target → Use MicroLIB` 关闭（`useUlib=0`），
> 以正常链接 `sinf/cosf/sqrtf` 等标准 math 函数。CubeMX 重新生成后请再次确认。

| 源文件 | 目标文件 | 修改点 |
| --- | --- | --- |
| `math/inc/mathFunc.h` | `Algorithm/inc/mathFunc.h` | 去掉 `arm_math.h` 依赖，补齐 PI 定义 |
| `math/inc/pid.h` | `Algorithm/inc/pid.h` | 去掉 `includes.h` 依赖 |
| `math/src/*.c` | `Algorithm/src/*.c` | 修正向量内积和微分项符号 |
| `Motor/src/DJmotor.c` | `Motor/src/DJmotor.c` | 总线句柄按 `MOTOR_DJI_CAN_BUS` 选择；发送后清零电流设定 |
| `Motor/src/VescMotor.c` | `Motor/src/VescMotor.c` | 使用全局 `CANx_Txqueue`，TIM2 中断统一出队发送 |
| `Motor/src/ZDrive.c` | `Motor/src/ZDrive.c` | 接收加 ID 越界保护，发送加队列满保护 |
| `Motor/src/my_Unitree.c` | `Motor/src/UnitreeMotor.c` | 4 路 RS485 改为单路 UART7；协议层并入 `UnitreeMotor.h`；配置统一到 `motor_config.h` |
| `Master/src/FD_Canqueue.c` | `Motor/src/FD_Canqueue.c` | `CAN_DequeueTx` 发送失败保留帧；`CAN_Start()` 做启动/滤波 |
| `Master/src/LED.c` | `BSP/src/bsp_led.c` + `BSP/src/bsp_buzzer.c` | LED 与 Buzzer 拆分 |
| `Master/src/IQRhandler.c` | `Application/src/IQRhandler.c` | 去掉项目专用协议，留模板框架；func 频率 100 Hz → 200 Hz |

## 2. 修改硬件引脚

1. 在 CubeMX 中修改 GPIO/外设并重新生成。
2. 同步修改 `Config/board_config.h`：
   ```c
   #define BOARD_LED1_PORT   GPIOB
   #define BOARD_LED1_PIN    GPIO_PIN_3
   ```
3. 检查 `Core/Inc/main.h` 中 CubeMX 生成的 `LEDx_PIN` 定义是否一致。
4. 不要修改 `BSP/src/bsp_led.c` 中的逻辑，硬件差异只应出现在 Config 和 CubeMX。

## 3. 修改电机总线 / 开关

开关和数量/总线都在 `Config/motor_config.h`：

```c
#define USE_DJ   1                    /* 1 = 编译并使用,0 = 不编译 */
#define USE_VESC 0
#define USE_ZMDR 1
#define USE_UNITREE 1

#define MOTOR_DJI_COUNT    4U
#define MOTOR_DJI_CAN_BUS  2          /* 0=FDCAN1, 1=FDCAN2, 2=FDCAN3 */
#define MOTOR_VESC_CAN_BUS 1
#define MOTOR_ZDRIVE_COUNT       8U
#define MOTOR_ZDRIVE_SPLIT_COUNT 0U   /* 0=不拆分;n=前 n 个 ID 走第一路 */
#define MOTOR_ZDRIVE_CAN_BUS_1   1U   /* 第一路:FDCAN2 */
#define MOTOR_ZDRIVE_CAN_BUS_2   2U   /* 第二路:FDCAN3 */
#define MOTOR_UNITREE_COUNT           3U
#define MOTOR_UNITREE_UART            7U
#define MOTOR_UNITREE_REDUCTION_RATIO 6.33f
```

CAN 总线引脚由 CubeMX 生成，通常：
- FDCAN1: PD0/PD1
- FDCAN2: PB12/PB13
- FDCAN3: PF6/PF7

> **Unitree GO-M8010-6 零位注意**：该电机带绝对编码器，驱动默认**不上电自动清零**。上电/首次使能前必须先在机械零位调用 `UnitreeMotor_SetZero(id)`，否则位置环会以电机出厂绝对零位为参考。

## 4. 新增一类电机

1. 在 `Motor/` 下建 `MyMotor.h/.c`(参考 ZDrive.c 的写法:Init / Receive / Func);
2. `main.c` USER CODE 2 里加 `MyMotorInit();`;
3. `Application/src/IQRhandler.c` 里:
   - 在 `HAL_FDCAN_RxFifo0/1Callback` 对应总线的分支里加
     `MyMotorReceive(...)`(接收不走软件队列,反馈帧在 FDCAN 中断里
     直接解析;Receive 内部必须按帧 ID 过滤,不是自己的帧直接丢弃);
   - TIM2 的 200 Hz 区加 `MyMotorFunc()`(用 `#if USE_XXX` 包住,
     开关加进 includes.h);
4. Keil 工程把 `my_motor.c` 加进 `Motor` 组。

## 5. 新增 UART 业务

模板已在 `main.c` USER CODE 2 启动各串口 DMA 空闲接收,
处理入口在 `Application/src/IQRhandler.c` 的 `HAL_UARTEx_RxEventCallback`:

```c
if (huart->Instance == USART2)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)UART2_RxBuffer, Size);
    /* 在这里加 USART2 的协议处理 */
    HAL_UART_DMAStop(&huart2);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_RxBuffer, UART_RX_BUFFER_SIZE);
}
```

步骤:
1. 在 `Application/inc/IQRhandler.h` 声明新缓冲区 `UART2_RxBuffer`(带 `__RAM_D2_`);
2. 在 `IQRhandler.c` 定义它;
3. `main.c` USER CODE 2 启动接收(USART2 需要在 CubeMX 里配 DMA);
4. 在 `HAL_UARTEx_RxEventCallback` 加分支处理。

> H7 开了 D-Cache:DMA 接收完先 `SCB_InvalidateDCache_by_Addr()`,
> DMA 发送前先 `SCB_CleanDCache_by_Addr()`。DMA 缓冲放 D2 段可省心。

> **调试打波**:UART9 已接 VOFA+ 模块(JustFloat 协议),在发送任务里
> `VOFA_Channel_Update()` 填通道 + `VOFA_Update()` 发送即可,
> API 与上位机配置见 [vofa_plus.md](vofa_plus.md)。

## 6. 新增 TIM 业务

TIM3/4/5 已由 CubeMX 配好 1 kHz(备用),直接:

1. `Application/src/IQRhandler.c` 的 `TIM_PeriodElapsedCallback` 加分支:
   ```c
   else if (htim->Instance == TIM3)
   {
       /* 你的 1 kHz 逻辑 */
   }
   ```
2. `main.c` USER CODE 2 里 `HAL_TIM_Base_Start_IT(&htim3);`

频率不够用就改 CubeMX 里的分频/周期。中断里不要做长时间计算。

## 7. 新增任务

1. CubeMX → Middleware → FreeRTOS → Tasks 里创建任务(名称、栈、优先级);
2. 重新生成后,在 freertos.c 生成的 `__weak` 任务函数 USER CODE 里写业务,
   或者在 `Application/src/myostasks.c` 写同名强定义覆盖(推荐,文件清爽)。
   `VOFASendTask` 就是按这个流程建的(模板示例,10 ms 发一帧 VOFA+ 数据)。

## 8. RAM 段使用

`includes.h` 提供 `__RAM_D1_/__RAM_D2_/__RAM_D3_` 宏,对应
`MDK-ARM/stm32h723zxx_flash.sct` 里的区域:

```c
__RAM_D2_ ALIGN_32B uint8_t my_dma_buf[64];   /* DMA 可达 */
__RAM_D1_ float big_array[4096];              /* 大块计算数据(AXI) */
```

新增区域时同步改 sct 文件。注意:DMA1/DMA2 到不了 SRAM4(D3)。

## 9. CubeMX 重新生成后的检查清单

- [ ] `main.c` USER CODE Includes / USER CODE 2 / Callback 1 仍存在;
- [ ] `freertos.c` USER CODE 区域仍存在(任务在 CubeMX 里配置);
- [ ] `LEDTask` 栈大小至少 `64 * 4` 字节;
- [ ] Keil 工程 include path 包含 `../Motor/inc;../Application/inc;../BSP/inc;../Algorithm/inc;../Config;../Communication/Bluetooth;../Communication/VOFA`;
- [ ] 链接器:使用 `.\stm32h723zxx_flash.sct`(Use Memory Layout from Target Dialog 不勾选)。
