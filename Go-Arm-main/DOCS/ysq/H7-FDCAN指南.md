# H7 板卡 FDCAN 使用指南

> 适用板卡：STM32H723（本项目 `Go-Arm` 模板）
> 本文覆盖：芯片配置、接收链路（过滤器 / 开启接收 / 中断回调解包 / RxHeader）、发送链路（TxHeader / 不同 HAL 发送函数的芯片配置、用法与优缺点）。
> 所有库文件路径均为**相对本文件**（`DOCS/ysq/`）的相对路径，点开即可跳转。

---

## 0. 文件地图（相对路径）

| 作用 | 文件 |
| --- | --- |
| FDCAN 外设初始化（`MX_FDCANx_Init` / MspInit） | [Core/Src/fdcan.c](../../Core/Src/fdcan.c) |
| FDCAN 句柄声明与初始化原型 | [Core/Inc/fdcan.h](../../Core/Inc/fdcan.h) |
| FDCAN 中断向量入口 → `HAL_FDCAN_IRQHandler` | [Core/Src/stm32h7xx_it.c](../../Core/Src/stm32h7xx_it.c) |
| 启动顺序（`MX_FDCANx_Init` / `CAN_InitSendQueue` / `CAN_Start`） | [Core/Src/main.c](../../Core/Src/main.c) |
| 接收中断回调解包（`HAL_FDCAN_RxFifo0/1Callback`） | [Application/src/IQRhandler.c](../../Application/src/IQRhandler.c) |
| 发送队列 + TxHeader 组装（`CAN_DequeueTx`） | [Motor/src/FD_Canqueue.c](../../Motor/src/FD_Canqueue.c) |
| 发送队列数据结构定义 | [Motor/inc/FD_Canqueue.h](../../Motor/inc/FD_Canqueue.h) |
| HAL 结构体 / 枚举 / 函数原型（**权威定义**） | [Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_fdcan.h](../../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_fdcan.h) |
| HAL FDCAN 实现 | [Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c](../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c) |

---

## 1. 概述

H723 提供 3 路 FDCAN，本项目三路全部启用，统一跑**经典 CAN 1 Mbps**（不启用 CAN-FD、不启用 BRS）：

| 实例 | RX 引脚 | TX 引脚 | GPIO 复用 | MessageRAMOffset |
| --- | --- | --- | --- | --- |
| FDCAN1 | PD0 | PD1 | `GPIO_AF9_FDCAN1` | 0 |
| FDCAN2 | PB12 | PB13 | `GPIO_AF9_FDCAN2` | 850 |
| FDCAN3 | PF6 | PF7 | `GPIO_AF2_FDCAN3` | 1700 |

- FDCAN 内核时钟由 **PLL2** 提供，`RCC_FDCANCLKSOURCE_PLL2`，频率 **40 MHz**（`.ioc` 里 `RCC.FDCANFreq_Value=40000000`）。
- 时间量子 = 1 / 40 MHz = **25 ns**。

> 三路共用同一个 PLL2 时钟源，但每路有自己的 MessageRAM 偏移量（H723 FDCAN 共 2560 字 RAM，三路按偏移分段，互不重叠）。

---

## 2. 芯片配置

### 2.1 时钟与波特率

时钟配置在 [main.c](../../Core/Src/main.c) 的 `PeriphCommonClock_Config()`：

```c
PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
PeriphClkInitStruct.PLL2.PLL2M = 2;
PeriphClkInitStruct.PLL2.PLL2N = 16;
PeriphClkInitStruct.PLL2.PLL2P = 2;
PeriphClkInitStruct.PLL2.PLL2R = 2;
PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
```

波特率由 `FDCAN_InitTypeDef` 里的标称位时间参数决定：

```
内核时钟     = 40 MHz
时间量子 tq  = 25 ns
位时间       = 1 + NominalTimeSeg1 + NominalTimeSeg2
            = 1 + 29 + 10 = 40 tq
波特率       = 40 MHz / 40 = 1 Mbps
```

对应 `MX_FDCANx_Init` 里的关键字段（三路相同）：

```c
hfdcan1.Init.NominalPrescaler = 1;      // 40MHz / 1 = 40MHz，再除以位时间分频
hfdcan1.Init.NominalSyncJumpWidth = 5;  // 同步跳变宽度
hfdcan1.Init.NominalTimeSeg1 = 29;      // 时间段 1
hfdcan1.Init.NominalTimeSeg2 = 10;      // 时间段 2
```

> 数据位时间参数（`DataPrescaler` / `DataTimeSeg1` / `DataTimeSeg2`）仅在 CAN-FD 数据段变速时有效；本项目用经典帧，它们的值不影响波特率。

### 2.2 `FDCAN_InitTypeDef` 初始化字段

定义见 [stm32h7xx_hal_fdcan.h:59](../../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_fdcan.h#L59)。本项目 [fdcan.c](../../Core/Src/fdcan.c) 中 `MX_FDCAN1_Init()` 的完整配置如下，逐字段说明：

```c
hfdcan1.Instance = FDCAN1;
hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;   // 经典 CAN 模式（不使能 FD）
hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;            // 正常模式（非回环/监听）
hfdcan1.Init.AutoRetransmission = DISABLE;        // 关闭自动重发（机器人常用：旧帧不阻塞新帧）
hfdcan1.Init.TransmitPause = DISABLE;             // 发送暂停
hfdcan1.Init.ProtocolException = DISABLE;         // 协议异常处理
// —— 标称位时间（决定 1 Mbps）——
hfdcan1.Init.NominalPrescaler = 1;
hfdcan1.Init.NominalSyncJumpWidth = 5;
hfdcan1.Init.NominalTimeSeg1 = 29;
hfdcan1.Init.NominalTimeSeg2 = 10;
// —— 数据位时间（FD 变速段，经典帧下不用）——
hfdcan1.Init.DataPrescaler = 1;
hfdcan1.Init.DataSyncJumpWidth = 5;
hfdcan1.Init.DataTimeSeg1 = 29;
hfdcan1.Init.DataTimeSeg2 = 10;
// —— Message RAM 分配 ——
hfdcan1.Init.MessageRAMOffset = 0;                // FDCAN1 从 RAM 0 开始
hfdcan1.Init.StdFiltersNbr = 16;                  // 标准帧过滤器数量
hfdcan1.Init.ExtFiltersNbr = 16;                  // 扩展帧过滤器数量
hfdcan1.Init.RxFifo0ElmtsNbr = 20;                // RX FIFO0 元素个数
hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;// RX FIFO0 每元素数据字节数
hfdcan1.Init.RxFifo1ElmtsNbr = 20;
hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
hfdcan1.Init.RxBuffersNbr = 0;                    // 专用接收 Buffer 个数（本项目不用）
hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
hfdcan1.Init.TxEventsNbr = 0;                     // 发送事件 FIFO 个数（本项目不用）
hfdcan1.Init.TxBuffersNbr = 0;                    // 专用发送 Buffer 个数（用 FIFO/Queue 时填 0）
hfdcan1.Init.TxFifoQueueElmtsNbr = 16;            // TX FIFO/Queue 元素个数
hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION; // TX 用 FIFO 模式
hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;     // TX 每元素数据字节数
if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) Error_Handler();
```

三路差异只在 `MessageRAMOffset`（0 / 850 / 1700）和 `TxFifoQueueElmtsNbr`（FDCAN3 为 20，其余 16）。

### 2.3 MspInit：GPIO / 时钟 / NVIC

`HAL_FDCAN_Init` 会回调 `HAL_FDCAN_MspInit`（见 [fdcan.c:178](../../Core/Src/fdcan.c#L178)），完成三件事：

```c
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle)
{
    // 1) 外设时钟（三路共用一个计数器，仅第一次 Enable，避免重复开关）
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if (HAL_RCC_FDCAN_CLK_ENABLED == 1) { __HAL_RCC_FDCAN_CLK_ENABLE(); }

    // 2) GPIO 复用（以 FDCAN1 为例，PD0=RX, PD1=TX）
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // 3) 中断（IT0 + IT1 两条中断线都打开，优先级 5）
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
}
```

中断向量入口在 [stm32h7xx_it.c](../../Core/Src/stm32h7xx_it.c)，每个 `FDCANx_IT0/IT1_IRQHandler` 都只调用 `HAL_FDCAN_IRQHandler(&hfdcanx)`，由 HAL 分发到回调。

### 2.4 启动流程

完整顺序见 [main.c:130](../../Core/Src/main.c#L130) 附近：

```c
MX_FDCAN1_Init();   // 外设 + 波特率 + MessageRAM 配置
MX_FDCAN2_Init();
MX_FDCAN3_Init();
...
CAN_InitSendQueue();  // 初始化软件发送队列（见 FD_Canqueue.c）
CAN_Start();          // 配置过滤器 + 覆盖模式 + 通知 + HAL_FDCAN_Start
```

`HAL_FDCAN_Init` 只做硬件初始化，**并不会启动接收/发送**；必须显式调用 `HAL_FDCAN_Start`（本项目统一放在 `CAN_Start()` 里，见 [FD_Canqueue.c:36](../../Motor/src/FD_Canqueue.c#L36)）。

---

## 3. 接收链路配置

接收链路 = **过滤器配置 → 开启接收（Start + 中断通知）→ 中断回调 → `HAL_FDCAN_GetRxMessage` 解包**。

### 3.1 过滤器

FDCAN 有 3 类过滤：**标准帧过滤器 / 扩展帧过滤器 / 全局（不匹配帧）过滤器**。本项目采用“**全局过滤放行到 FIFO0 + 软件在中断里按 ID 二次过滤**”的简单方案。

#### (1) 全局过滤器

在 [FD_Canqueue.c:36](../../Motor/src/FD_Canqueue.c#L36) 的 `CAN_Start()` 里对三路统一配置：

```c
HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0,  // 不匹配的标准帧 → FIFO0
                             FDCAN_ACCEPT_IN_RX_FIFO0,            // 不匹配的扩展帧 → FIFO0
                             FDCAN_REJECT_REMOTE,                 // 拒绝标准远程帧
                             FDCAN_REJECT_REMOTE);                // 拒绝扩展远程帧
```

含义：**所有**数据帧（无论是否命中过滤器）都进 FIFO0；只有远程帧被丢弃。这样硬件层面“来者不拒”，具体是不是自己关心的帧由中断里的驱动 `Receive` 函数判断。

#### (2) 单条过滤器 `FDCAN_FilterTypeDef`

定义见 [stm32h7xx_hal_fdcan.h:181](../../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_fdcan.h#L181)。若需要硬件精确过滤（降低中断频率、按 ID 分 FIFO），用 `HAL_FDCAN_ConfigFilter`：

```c
FDCAN_FilterTypeDef sFilter = {0};
sFilter.IdType       = FDCAN_STANDARD_ID;          // 标准帧
sFilter.FilterIndex  = 0;                          // 标准帧过滤器编号 0~127
sFilter.FilterType   = FDCAN_FILTER_RANGE;         // 范围过滤：FilterID1~FilterID2
sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;    // 命中后存入 FIFO0
sFilter.FilterID1    = 0x201;                      // 范围起点（如 DJI 反馈 0x201~0x208）
sFilter.FilterID2    = 0x208;                      // 范围终点
HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter);
```

`FilterType` 可选：

| 值 | 含义 |
| --- | --- |
| `FDCAN_FILTER_RANGE` | 区间匹配 `[FilterID1, FilterID2]` |
| `FDCAN_FILTER_MASK` | 掩码匹配：`FilterID1 = ID`，`FilterID2 = 掩码` |
| `FDCAN_FILTER_RANGE_NO_EIDM` | 仅扩展帧，范围匹配且不用 EIDM 掩码 |

`FilterConfig` 决定命中后的去向：`FDCAN_FILTER_TO_RXFIFO0`（普通 FIFO0）、`FDCAN_FILTER_TO_RXFIFO1`、`FDCAN_FILTER_TO_RXBUFFER`（专用 Buffer）等。

### 3.2 开启接收：`Start` + 中断通知

`HAL_FDCAN_Start` 只是让外设进入工作状态，**接收中断还需要单独激活通知**。本项目在 `CAN_Start()` 里：

```c
HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE); // FIFO0 满时覆盖旧帧
HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);        // 新消息中断
HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0U);        // FIFO1 也开（暂未用）
HAL_FDCAN_Start(&hfdcan1);
```

- `FDCAN_RX_FIFO_OVERWRITE`：FIFO 满时**覆盖最旧帧**；机器人控制场景优先保证新数据，不阻塞总线。
- `FDCAN_IT_RX_FIFO0_NEW_MESSAGE`：FIFO0 有新消息时触发 `HAL_FDCAN_RxFifo0Callback`。

> 顺序要点：先配过滤器/覆盖/通知，**最后** `HAL_FDCAN_Start`。

### 3.3 中断回调解包

回调强定义在 [IQRhandler.c:109](../../Application/src/IQRhandler.c#L109)（CubeMX 生成的是 `__weak`，这里强定义不会冲突）：

```c
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        FDCAN_RxHeaderTypeDef Rxheader;
        uint8_t Rx_data[8] = {0};

        // 从 FIFO0 取出一帧：填 Rxheader + 数据，同时清中断标志
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &Rxheader, Rx_data);

        if (hfdcan == &hfdcan1)      { /* CAN1：用户通信 / ZDrive(总线0) */ }
        else if (hfdcan == &hfdcan2) { /* CAN2：ZDrive(1) / VESC / DJI */ }
        else if (hfdcan == &hfdcan3) { /* CAN3：ZDrive(2) / Arm / VESC / DJI */ }
    }
    else if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL)
    {
        // FIFO 满：读一帧丢弃最旧，避免卡死
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, NULL, NULL);
    }
}
```

**解包套路**：

1. 判断 `RxFifo0ITs` 中 `FDCAN_IT_RX_FIFO0_NEW_MESSAGE` 位，有消息才读。
2. 调 `HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &Rxheader, Rx_data)`：
   - 传 `&Rxheader` / `Rx_data` 正常取帧；
   - 传 `NULL` / `NULL` 则只清标志不取数据（用于 FIFO 满时丢弃）。
3. 用 `Rxheader.Identifier` / `IdType` / `RxFrameType` 做软件二次过滤，分发到对应驱动（本项目里 `ZdriveReceive` / `DJmotor_Receive` / `VescReceiveData_CAN2` / `Arm_Receive` 内部都再按 ID 过滤）。

### 3.4 `FDCAN_RxHeaderTypeDef` 结构体字段

定义见 [stm32h7xx_hal_fdcan.h:272](../../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_fdcan.h#L272)。接收解包时最常用字段：

| 字段 | 类型 | 说明 | 常用值 / 用法 |
| --- | --- | --- | --- |
| `Identifier` | uint32_t | 帧 ID | 标准帧 0~0x7FF，扩展帧 0~0x1FFFFFFF |
| `IdType` | uint32_t | ID 类型 | `FDCAN_STANDARD_ID` / `FDCAN_EXTENDED_ID` |
| `RxFrameType` | uint32_t | 帧类型 | `FDCAN_DATA_FRAME`（数据帧）/ `FDCAN_REMOTE_FRAME` |
| `DataLength` | uint32_t | 数据长度码 | `FDCAN_DLC_BYTES_4`=4，`FDCAN_DLC_BYTES_8`=8 等 |
| `ErrorStateIndicator` | uint32_t | 错误状态指示 | `FDCAN_ESI_ACTIVE` 等 |
| `BitRateSwitch` | uint32_t | 是否 BRS 变速 | `FDCAN_BRS_OFF` / `FDCAN_BRS_ON` |
| `FDFormat` | uint32_t | 经典 / FD 格式 | `FDCAN_CLASSIC_CAN` / `FDCAN_FD_CAN` |
| `RxTimestamp` | uint32_t | 帧起始时刻的时间戳计数器值 | 0~0xFFFF |
| `FilterIndex` | uint32_t | 命中的过滤器编号 | 未命中全局过滤时无意义 |
| `IsFilterMatchingFrame` | uint32_t | 是否命中过滤器 | 0=命中，1=未命中（走了全局过滤） |

本项目在接收里只用了 `Identifier` / `IdType` / `DataLength` / `RxFrameType` 四个字段，其余按需读取。

---

## 4. 发送链路配置

### 4.1 `FDCAN_TxHeaderTypeDef` 结构体字段

定义见 [stm32h7xx_hal_fdcan.h:232](../../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_fdcan.h#L232)。发送前需要逐字段填充：

| 字段 | 说明 | 本项目取值（经典 8 字节数据帧） |
| --- | --- | --- |
| `Identifier` | 帧 ID | 由协议决定（如 DJI `0x200`/`0x1FF`，ZDrive `id \| code<<4`） |
| `IdType` | ID 类型 | `FDCAN_STANDARD_ID`（DJI/ZDrive）或 `FDCAN_EXTENDED_ID`（VESC） |
| `TxFrameType` | 帧类型 | `FDCAN_DATA_FRAME` |
| `DataLength` | 数据长度码 | `FDCAN_DLC_BYTES_8`（8 字节） |
| `ErrorStateIndicator` | 错误状态指示 | `FDCAN_ESI_ACTIVE` |
| `BitRateSwitch` | 是否 BRS | `FDCAN_BRS_OFF` |
| `FDFormat` | 经典 / FD | `FDCAN_CLASSIC_CAN` |
| `TxEventFifoControl` | 发送事件 FIFO 控制 | `FDCAN_NO_TX_EVENTS`（不记录发送事件） |
| `MessageMarker` | 消息标记（写入 Tx 事件 FIFO 用） | `0` |

组装示例见 [FD_Canqueue.c:74](../../Motor/src/FD_Canqueue.c#L74) 的 `CAN_DequeueTx`：

```c
FDCAN_TxHeaderTypeDef tx_message;
tx_message.TxFrameType         = FDCAN_DATA_FRAME;
tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
tx_message.BitRateSwitch       = FDCAN_BRS_OFF;
tx_message.FDFormat            = FDCAN_CLASSIC_CAN;
tx_message.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
tx_message.MessageMarker       = 0;
tx_message.IdType              = queue->FDCAN_DataSend[queue->Front].IDE;
tx_message.Identifier          = queue->FDCAN_DataSend[queue->Front].ID;
tx_message.DataLength          = queue->FDCAN_DataSend[queue->Front].DLC;
```

> 直接发送的写法见 [DJmotor.c:212](../../Motor/src/DJmotor.c#L212) 的 `DJmotor_CurrentTransmit`，字段一致。

### 4.2 不同类型的 HAL 发送函数

HAL 提供两类发送 API，对应两种硬件发送资源，**在初始化阶段就决定用哪种**：

| 发送函数 | 用到的硬件资源 | 初始化关键字段 |
| --- | --- | --- |
| `HAL_FDCAN_AddMessageToTxFifoQ` | TX FIFO / Queue | `TxFifoQueueElmtsNbr` + `TxFifoQueueMode` |
| `HAL_FDCAN_AddMessageToTxBuffer` | 专用 TX Buffer | `TxBuffersNbr`（+ 可选 `TxEventsNbr`） |

函数原型见 [stm32h7xx_hal_fdcan.h:2055](../../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_fdcan.h#L2055)。

#### (1) `HAL_FDCAN_AddMessageToTxFifoQ` —— FIFO / Queue 发送（本项目使用）

芯片配置（[fdcan.c](../../Core/Src/fdcan.c)）：

```c
hfdcan1.Init.TxBuffersNbr = 0;                  // 不用专用 Buffer
hfdcan1.Init.TxFifoQueueElmtsNbr = 16;          // FIFO 深度 16
hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;  // FIFO 模式
hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
```

使用方式：

```c
HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_message, tx_data);
```

- `TxFifoQueueMode` 两种：
  - `FDCAN_TX_FIFO_OPERATION`：**先入先出**，发送顺序 = 入队顺序。
  - `FDCAN_TX_QUEUE_OPERATION`：**按 CAN ID 优先级仲裁**（ID 越小越先发，模拟 CAN 总线仲裁）。
- 返回值 `HAL_OK` 表示入队成功；`HAL_ERROR` 表示 TX FIFO 已满（本项目 [CAN_DequeueTx](../../Motor/src/FD_Canqueue.c#L74) 里满则保留帧、下周期重试）。

**优点**：

- 一次调用即完成入队 + 自动发送，硬件按序仲裁，无需手动管理 Buffer 状态。
- FIFO 满时立即返回错误，便于上层做“保留重试 / 丢弃”策略。
- 适合本项目“定时器中断里 1 kHz 恒定出队”的节奏。

**缺点**：

- FIFO/Queue 模式下无法为单帧指定“一次性/单次发送”属性，重发行为由 `AutoRetransmission` 全局决定。
- Queue 模式按 ID 仲裁，可能改变你预期的发送顺序（ID 小的帧一直优先，ID 大的可能饿死）。
- 深度受 `TxFifoQueueElmtsNbr`（≤32）限制。

#### (2) `HAL_FDCAN_AddMessageToTxBuffer` —— 专用 TX Buffer 发送

芯片配置（需把 `TxBuffersNbr` 改成 > 0）：

```c
hfdcan1.Init.TxBuffersNbr = 4;   // 分配 4 个专用发送 Buffer（0~3）
// TxFifoQueueElmtsNbr 可相应减少或归 0（总 RAM 预算要平衡）
```

使用方式分两步（先写 Buffer，再触发发送）：

```c
// 1) 写入指定 Buffer（此时不立即发送）
HAL_FDCAN_AddMessageToTxBuffer(&hfdcan1, &tx_message, tx_data, 0);

// 2) 显式触发 Buffer 0 的发送请求
HAL_FDCAN_EnableTxBufferRequest(&hfdcan1, 0);
```

> 原型见 [stm32h7xx_hal_fdcan.h:2057](../../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_fdcan.h#L2057)（`AddMessageToTxBuffer`）与 [:2059](../../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_fdcan.h#L2059)（`EnableTxBufferRequest`）。

**优点**：

- **独立控制**：每个 Buffer 是独立硬件元素，可分别配置单次/周期发送、触发时机。
- 可对同一 Buffer 重复写新数据“覆盖”再触发，适合高频周期帧（如电机电流帧）。
- 适合需要精确调度、每个 ID 固定 Buffer 的场景（类似 STM32F4 bxCAN 的邮箱用法）。

**缺点**：

- 需要两步操作（写 + 触发），遗漏 `EnableTxBufferRequest` 会“写了但发不出去”。
- 需自行管理 Buffer 分配与发送状态（`HAL_FDCAN_IsTxBufferMessagePending` 查询是否还在发送）。
- 只有 ≤32 个 Buffer，且和 TX FIFO/Queue、RX 等共享 MessageRAM 预算，配置复杂、易溢出。

#### 选型小结

| 场景 | 推荐 |
| --- | --- |
| 多路电机 / 多帧复用、定时器统一出队、要“满则重试” | `HAL_FDCAN_AddMessageToTxFifoQ`（FIFO 模式，本项目方案） |
| 少量高频周期帧、每个 ID 固定通道、要精确覆盖旧帧 | `HAL_FDCAN_AddMessageToTxBuffer` |
| 要严格按 CAN 仲裁优先级发送 | `AddMessageToTxFifoQ` + `FDCAN_TX_QUEUE_OPERATION` |

---

## 5. 本项目完整收发流程小结

```
初始化:
  MX_FDCAN1/2/3_Init()          -> 波特率 1Mbps + MessageRAM + HAL_FDCAN_Init + MspInit(GPIO/NVIC)
  CAN_InitSendQueue()           -> 软件发送队列 Front/Rear 归零, Canx 绑定 FDCANx
  CAN_Start()                   -> 全局过滤放行 FIFO0 + FIFO 覆盖 + 激活新消息中断 + HAL_FDCAN_Start

发送:
  驱动入队 ZdriveEnqueue/VESC 直接写队列  ->  TIM2@1kHz 中断 CAN_DequeueTx 出队
                                        ->  组装 TxHeader -> HAL_FDCAN_AddMessageToTxFifoQ

接收:
  FDCANx_IT0/IT1_IRQHandler -> HAL_FDCAN_IRQHandler -> HAL_FDCAN_RxFifo0Callback
                            -> HAL_FDCAN_GetRxMessage 取 RxHeader + 8 字节数据
                            -> 按 Rxheader.Identifier/IdType 分发到 ZdriveReceive / DJmotor_Receive / VescReceiveData_CAN2 / Arm_Receive
```

> 本项目接收**不走软件队列**，在 FDCAN 中断里直接解析；FDCAN 与 TIM2 同为优先级 5，同优先级互不抢占，故接收写电机状态与 TIM2 读状态天然互斥，无需加锁。
