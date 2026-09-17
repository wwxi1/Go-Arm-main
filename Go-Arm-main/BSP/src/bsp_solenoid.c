/**
 * @file bsp_solenoid.c
 * @date 2026-08-21
 *
 * @brief 电磁阀控制驱动。
 *
 * 设计思路：每个通道共用 SDA 和 CLK 两根 GPIO，按串行方式输出 4 位状态，
 * 低 4 位数据对应 4 个电磁阀的开/闭状态。该驱动不依赖 CubeMX 的串口配置，
 * 直接通过 GPIO 的输出脉冲模拟了一个简单的串行寄存器写入时序。
 */
#include "bsp_solenoid.h"

/*
 * 3 个电磁阀控制通道的缓存状态。
 * 每个通道独立保存对应 GPIO 端口、数据线和时钟线，以及最近写入的值。
 */
static Solenoid_t solenoid_Channel1 = {0};
static Solenoid_t solenoid_Channel2 = {0};
static Solenoid_t solenoid_Channel3 = {0};

/*
 * @brief 初始化单个电磁阀通道的 GPIO 映射与默认状态。
 * @param solenoid 目标通道结构体
 * @param gpio_port GPIO端口
 * @param gpio_pin_sda 数据线
 * @param gpio_pin_clk 时钟线
 *
 * data_prve 初始值设为 0xF0，含义是“默认缓存一个非零值”，用于避免第一次
 * 调用时被错误判定为与旧值相同。实际状态在后续 SolenoidValve_On 中被覆盖。
 */
static void _SolenoidValve_Channel_Init(Solenoid_t *solenoid, GPIO_TypeDef *gpio_port, uint16_t gpio_pin_sda, uint16_t gpio_pin_clk)
{
    solenoid->gpio_port = gpio_port;
    solenoid->gpio_pin_sda = gpio_pin_sda;
    solenoid->gpio_pin_clk = gpio_pin_clk;
    solenoid->data_prve = 0xF0;
}

/*
 * @brief 初始化指定通道。
 * @param usart_channel 要初始化的电磁阀通道编号
 *
 * 逻辑：
 * 1. 根据通道编号绑定对应 GPIO 端口和引脚。
 * 2. 使能相关 GPIO 时钟。
 * 3. 将 SDA / CLK 先拉低，避免电平浮动。
 * 4. 配置为推挽输出模式。
 * 5. 发送一个默认命令 0，确保所有电磁阀关闭。
 */
void SolenoidValve_Init(Solenoid_Channel usart_channel)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    switch (usart_channel)
    {
    case Solenoid_Channel1:
        _SolenoidValve_Channel_Init(&solenoid_Channel1, BOARD_SOLENOID_PORT1, BOARD_SOLENOID_SDA, BOARD_SOLENOID_CLK);
        __HAL_RCC_GPIOA_CLK_ENABLE();
        HAL_GPIO_WritePin(BOARD_SOLENOID_PORT1, BOARD_SOLENOID_SDA | BOARD_SOLENOID_CLK, GPIO_PIN_RESET);
        GPIO_InitStruct.Pin = BOARD_SOLENOID_SDA | BOARD_SOLENOID_CLK;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        break;
    case Solenoid_Channel2:
        _SolenoidValve_Channel_Init(&solenoid_Channel2, BOARD_SOLENOID_PORT2, BOARD_SOLENOID_SDA2, BOARD_SOLENOID_CLK2);
        __HAL_RCC_GPIOD_CLK_ENABLE();
        HAL_GPIO_WritePin(BOARD_SOLENOID_PORT2, BOARD_SOLENOID_SDA2 | BOARD_SOLENOID_CLK2, GPIO_PIN_RESET);
        GPIO_InitStruct.Pin = BOARD_SOLENOID_SDA2 | BOARD_SOLENOID_CLK2;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
        break;
    case Solenoid_Channel3:
        _SolenoidValve_Channel_Init(&solenoid_Channel3, BOARD_SOLENOID_PORT3, BOARD_SOLENOID_SDA3, BOARD_SOLENOID_CLK3);
        __HAL_RCC_GPIOD_CLK_ENABLE();
        HAL_GPIO_WritePin(BOARD_SOLENOID_PORT3, BOARD_SOLENOID_SDA3 | BOARD_SOLENOID_CLK3, GPIO_PIN_RESET);
        GPIO_InitStruct.Pin = BOARD_SOLENOID_SDA3 | BOARD_SOLENOID_CLK3;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
        break;
    default:
        break;
    }
    SolenoidValve_On(usart_channel, 0);
}

/*
 * @brief 将控制字按串行时序写入对应通道。
 * @param solenoid 目标通道状态对象
 * @param data 指向待写入的命令字节指针
 *
 * 逻辑：
 * 1. 如果新命令与上次相同，则无需重复发送，避免反复写寄存器带来的抖动。
 * 2. 取出最高位（bit7 -> bit3）依次写入 SDA。
 * 3. 每写一个 bit 就产生一次 CLK 上升沿/下降沿，以驱动外部寄存器或移位锁存。
 * 4. 额外再送一个时钟脉冲，确保最后一位被稳定锁存。
 *
 * 实际控制字在 SolenoidValve_On 中被截断为低 4 位：
 * 0000 4321
 * 其中 4、3、2、1 表示 4 个电磁阀的开关状态。
 */
static void _SolenoidValve_Register_Update(Solenoid_t *solenoid, uint8_t *data)
{
    if (*data == solenoid->data_prve)
        return;
    solenoid->data_prve = *data;
    for (int i = 0; i < 4; i++)
    {
        if ((*data & 0x08) == 0x08)
        {
            HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_sda, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_sda, GPIO_PIN_RESET);
        }
        *data <<= 1;
        HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_clk, GPIO_PIN_SET);
        HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_clk, GPIO_PIN_RESET);
    }
    HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_clk, GPIO_PIN_SET);
    HAL_GPIO_WritePin(solenoid->gpio_port, solenoid->gpio_pin_clk, GPIO_PIN_RESET);
}

/*
 * @brief 对指定通道发出电磁阀控制命令。
 * @param usart_channel 要控制的通道
 * @param cmd 控制码，低4位有效
 *
 * 命令举例：
 * - 0x00：全关闭
 * - 0x01：第1个电磁阀开启，其他关闭
 * - 0x0F：4个电磁阀全部开启
 *
 * 这里取 cmd & 0x0f 的意义是只保留低 4 位，避免高位干扰电磁阀状态。
 */
void SolenoidValve_On(Solenoid_Channel usart_channel, uint8_t cmd)
{
    uint8_t data = cmd & 0x0f;
    switch (usart_channel)
    {
    case Solenoid_Channel1:
        _SolenoidValve_Register_Update(&solenoid_Channel1, &data);
        break;
    case Solenoid_Channel2:
        _SolenoidValve_Register_Update(&solenoid_Channel2, &data);
        break;
    case Solenoid_Channel3:
        _SolenoidValve_Register_Update(&solenoid_Channel3, &data);
        break;
    default:
        break;
    }
}

/*
 * @brief 读取指定通道最后一次写入的控制值。
 * @param usart_channel 通道编号
 * @return 上一次写入的 4 位命令值
 *
 * 该函数本质上返回缓存值 data_prve，用于调试、状态回读或和目标状态对比。
 */
uint8_t SolenoidValve_Read(Solenoid_Channel usart_channel)
{
    uint8_t data = 0;
    switch (usart_channel)
    {
    case Solenoid_Channel1:
        data = solenoid_Channel1.data_prve;
        break;
    case Solenoid_Channel2:
        data = solenoid_Channel2.data_prve;
        break;
    case Solenoid_Channel3:
        data = solenoid_Channel3.data_prve;
        break;
    default:
        break;
    }
    return data;
}