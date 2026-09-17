# VOFA+ 打波调试说明

[VOFA+](https://www.vofa.plus/) 是一款串口上位机调试软件,支持波形/表格/示波器显示。
本工程在 `Communication/VOFA/vofa.c` 提供基于 **JustFloat** 协议的发送模块:

- 一帧最多 **6 个通道**(`VOFA_CHANNEL_NUM`),每通道 4 字节 float32(小端);
- 帧尾固定 `00 00 80 7F`(即 0x7F800000 的小端字节序);
- 整帧 `4 * (6 + 1) = 28` 字节,通过 UART9 DMA 发出。

## 1. 硬件接线

| 项 | 值 |
| --- | --- |
| 出口串口 | UART9(`huart9`) |
| 引脚 | PD14 = RX,PD15 = TX(USB-TTL 的 RX/TX 交叉接) |
| 波特率 | 115200(在 CubeMX 的 UART9 里改) |
| 连接 | USB-TTL → PC,必须**共地** |

> 出口串口通过宏 `BOARD_VOFA_UART` 指定(见下),换串口只需改这个宏。

## 2. 软件配置(Config/app_config.h)

```c
#define APP_VOFA_ENABLE        1           /* 1 = 编译并使用 VOFA+ 模块 */
#define APP_VOFA_TASK_PERIOD_MS  10U        /* 模板发送任务周期(ms) */
#define BOARD_VOFA_UART        (huart9)    /* VOFA+ debug data output */
```

- `APP_VOFA_ENABLE = 0` 时不编译发送函数(头文件里函数声明被 `#if` 包住),
  模板任务 `VOFA_SendTask` 也会休眠停发;
- `BOARD_VOFA_UART` 可以是任意已配 DMA 发送的 UART 句柄。

## 3. API 参考(Communication/VOFA/vofa.h)

### 数据类型

```c
typedef enum {
    VOFA_TYPE_FLOAT,    /* 32 位浮点,原样写入 */
    VOFA_TYPE_INT32,    /* 以下整型都先转成 float 再写入 */
    VOFA_TYPE_UINT32,
    VOFA_TYPE_INT16,
    VOFA_TYPE_UINT16,
    VOFA_TYPE_INT8,
    VOFA_TYPE_UINT8
} VOFA_DataType_t;
```

### VOFA_Channel_Update(channel, type, data)

```c
bool VOFA_Channel_Update(uint8_t channel, VOFA_DataType_t type, void *data);
```

- 把 `data` 指向的值写入内部缓存,`channel` 取值 `0 ~ VOFA_CHANNEL_NUM-1`(默认 0~5);
- 写的是**缓存**,不会立即发送,数据要等 `VOFA_Update()` 才出;
- 返回 `false` 表示通道号越界或类型非法(入参错误),不改变缓存。

### VOFA_Update()

```c
void VOFA_Update(void);
```

- 把缓存里 6 个通道的数据打包(6 × float32 + 帧尾)经 DMA 发出;
- 内部双缓冲 + 忙检查:上一次 DMA 还没发完时本次调用**直接跳过**,不会排队积压;
- 建议放在一个 FreeRTOS 任务里按固定周期调用(模板的 `VOFA_SendTask` 就是 10 ms 一次);
- **不要在中断里调用**(内部有 memcpy + DMA 启动,耗时不确定)。

### VOFA_DMA_TransmitCpltCallback(huart)

```c
void VOFA_DMA_TransmitCpltCallback(UART_HandleTypeDef *huart);
```

- UART 发送完成中断回调,用于清 DMA 忙标志;
- 已由 `Application/src/IQRhandler.c` 的 `HAL_UART_TxCpltCallback` 统一转发,
  换串口也不需要再改回调,一般**无需用户调用**。

## 4. 使用示例

模板任务 `VOFA_SendTask`(`Application/src/myostasks.c`,CubeMX 创建,10 ms 周期):

```c
void VOFA_SendTask(void *argument)
{
    for (;;)
    {
        static uint8_t Func_cnt = 0;
        Func_cnt++;
        /* 把要看的量填进通道(类型和指针任意组合,自动转 float) */
        VOFA_Channel_Update(0, VOFA_TYPE_UINT8, &Func_cnt);
        VOFA_Channel_Update(1, VOFA_TYPE_FLOAT, &some_float_var);
        /* 统一发送,忙时自动跳过 */
        VOFA_Update();
        osDelay(APP_VOFA_TASK_PERIOD_MS);
    }
}
```

## 5. VOFA+ 上位机设置

1. 打开 VOFA+ → **数据源**:选择 UART9 对应的串口号;
2. 波特率 **115200**,数据位 8,停止位 1,无校验;
3. **协议**:选择 **JustFloat**(帧尾自动识别为 `00 00 80 7F`);
4. 通道数 **6**,打开波形/示波器界面即可看到各通道曲线。

## 6. 注意事项

- **通道数**:改大 `VOFA_CHANNEL_NUM` 会同时改 `VOFA_PACKET_SIZE`,上位机通道数自动一致;
- **发送频率**:10 ms(100 Hz)是模板默认,波特率 115200 时 28 字节/帧绰绰有余;
  想提频先确认 DMA 忙标志,提高波特率可支撑更高频率;
- **D-Cache**:发送缓冲在 `__RAM_D1_`(AXI SRAM)且做了 32 字节对齐,
  `VOFA_Update()` 内部已 `SCB_CleanDCache_by_Addr()`;
- **多任务写通道**:`VOFA_Channel_Update` 只写缓存、无锁,不要多任务写同一通道;
- **UART9 RX 空闲中断**:IQRhandler.c 的 `HAL_UARTEx_RxEventCallback` 里
  UART9 分支已启动 DMA 空闲接收,不影响发送功能。
