/**
 * @file    UnitreeMotor.c
 * @brief   Unitree GO-M8010-6 RS485 motor driver.
 *
 * Ported from the 4-bus sample to the current single-UART7 RS485 hardware.
 * UART7 is initialized with HAL_RS485Ex_Init(), so DE direction is handled by
 * the peripheral; no manual GPIO toggling is required.
 */
#include "includes.h"
#include "UnitreeMotor.h"
#include "usart.h"
#include "crc_ccitt.h"
#include "ring_buffer.h"
#include "kinematics.h"

#if USE_UNITREE

#define UNITREE_2PI 6.28318f
#define UNITREE_SATURATE(_IN, _MIN, _MAX) \
    {                                     \
        if ((_IN) <= (_MIN))              \
            (_IN) = (_MIN);               \
        else if ((_IN) >= (_MAX))         \
            (_IN) = (_MAX);               \
    }

UnitreeMotor Unitree_motors[UNITREE_MOTOR_NUM];
__RAM_D1_ ALIGN_32B uint8_t Unitree_UART7_RxBuffer[UNITREE_RX_BUFFER_SIZE] = {0};
static __RAM_D1_ ALIGN_32B uint8_t s_unitree_uart7_rx_buf2[UNITREE_RX_BUFFER_SIZE] = {0};
static RingBuffer_t s_unitree_rx_queue;
static uint8_t s_unitree_rx_queue_buf[UNITREE_RX_BUFFER_SIZE];
static uint8_t s_unitree_frame[UNITREE_FRAME_LENGTH];
static uint8_t s_unitree_frame_len = 0U;
static bool s_unitree_frame_started = false;

static bool set_cur_init = true;

static bool s_unitree_initialized = false;
static uint8_t s_unitree_tx_index = 0U;
static UnitreeMotorData_t s_unitree_rx_data;
static __RAM_D1_ ALIGN_32B UnitreeMotorCmd_t s_unitree_tx_cmd;

/*
 * @brief  根据配置选择当前宇树电机使用的串口实例。
 * @return 对应的 UART 句柄；若配置不合法则返回 NULL。
 */
static UART_HandleTypeDef *UnitreeMotor_GetUart(void)
{
#if (MOTOR_UNITREE_UART == 4)
    return &huart4;
#elif (MOTOR_UNITREE_UART == 7)
    return &huart7;
#elif (MOTOR_UNITREE_UART == 9)
    return &huart9;
#else
    return NULL;
#endif
}

/*
 * @brief  设置 RS485 收发器进入接收模式。
 *        该脚本用于让总线从从机端接收电机回包。
 */
static inline void Unitree_SetRxMode(void)
{
    UART_HandleTypeDef *uart = UnitreeMotor_GetUart();

    if (uart == NULL)
    {
        return;
    }

    HAL_GPIO_WritePin(RS_485_U7_GPIO_Port, RS_485_U7_Pin, GPIO_PIN_RESET);
}

/*
 * @brief  设置 RS485 收发器进入发送模式。
 *        在发送控制命令前切换到发送方向，避免总线冲突。
 */
static inline void Unitree_SetTxMode(void)
{
    UART_HandleTypeDef *uart = UnitreeMotor_GetUart();

    if (uart == NULL)
    {
        return;
    }

    HAL_GPIO_WritePin(RS_485_U7_GPIO_Port, RS_485_U7_Pin, GPIO_PIN_SET);
}

/*
 * @brief  启动 UART DMA 双缓冲空闲接收，参照 R1_Arm zip 的双缓冲写法。
 */
static void UnitreeMotor_UART_DoubleBufferStart(UART_HandleTypeDef *uart)
{
    if (uart == NULL || uart->hdmarx == NULL)
    {
        return;
    }

    uart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    uart->RxEventType = HAL_UART_RXEVENT_IDLE;
    uart->RxXferSize = UNITREE_RX_BUFFER_SIZE;
    SET_BIT(uart->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(uart, UART_IT_IDLE);
    HAL_DMAEx_MultiBufferStart(uart->hdmarx, (uint32_t)&uart->Instance->RDR,
                               (uint32_t)Unitree_UART7_RxBuffer,
                               (uint32_t)s_unitree_uart7_rx_buf2,
                               UNITREE_RX_BUFFER_SIZE);
}

/*
 * @brief  初始化宇树电机驱动相关全局状态和各电机对象。
 * @details
 *  1. 清零所有电机结构体；
 *  2. 设置默认控制参数和 ID；
 *  3. 标记电机未使能，零点偏移量清零；
 *  4. 启动 UART7 的 DMA 空闲接收，准备接收回包。
 */
void UnitreeMotor_Init(void)
{
    UnitreeMotorCmd_t default_cmd;

    memset(&default_cmd, 0, sizeof(default_cmd));
    default_cmd.mode = UNITREE_MOTOR_MODE_IDLE;
    default_cmd.kp = MOTOR_UNITREE_DEFAULT_KP;
    default_cmd.kd = MOTOR_UNITREE_DEFAULT_KW;

    for (uint32_t i = 0U; i < UNITREE_MOTOR_NUM; i++)
    {
        memset(&Unitree_motors[i], 0, sizeof(Unitree_motors[i]));
        Unitree_motors[i].cmd = default_cmd;
        Unitree_motors[i].cmd.id = (uint16_t)i;
        Unitree_motors[i].data.id = (uint8_t)i;
        Unitree_motors[i].enable = false;
        Unitree_motors[i].set_zero = true;
        Unitree_motors[i].zero_offset = 0.0f;
        Unitree_motors[i].begin = true;
    }
    Unitree_motors[0].zero_pos = ARM_U1_ZERO_POS;
    Unitree_motors[1].zero_pos = ARM_U2_ZERO_POS;

    s_unitree_tx_index = 0U;
    s_unitree_initialized = true;

    RingBuffer_Init(&s_unitree_rx_queue, s_unitree_rx_queue_buf, sizeof(s_unitree_rx_queue_buf));
    s_unitree_frame_len = 0U;
    s_unitree_frame_started = false;
    UnitreeMotor_UART_DoubleBufferStart(UnitreeMotor_GetUart());
    Unitree_SetRxMode();
}

/*
 * @brief  将上层控制命令编码为宇树电机协议帧。
 * @param  cmd 需要发送的控制命令结构体。
 * @details
 *  该函数将速度、位置、转矩、KP/KD 等数据按宇树协议格式打包，
 *  并在尾部追加 CRC16 校验，以满足电机串口通信要求。
 */
void UnitreeMotor_EncodeFrame(UnitreeMotorCmd_t *cmd)
{
    UnitreeMotorControlFrame_t *frame;
    uint8_t *packet;
    int16_t tor_des = 0;
    int16_t spd_des = 0;
    int32_t pos_des = 0;
    int16_t kp = 0;
    int16_t kd = 0;

    if (cmd == NULL)
    {
        return;
    }

    frame = &cmd->frame;
    packet = (uint8_t *)frame;

    UNITREE_SATURATE(cmd->id, 0, 15);
    UNITREE_SATURATE(cmd->mode, 0, 7);
    UNITREE_SATURATE(cmd->kp, 0.0f, 25.599f);
    UNITREE_SATURATE(cmd->kd, 0.0f, 25.599f);
    UNITREE_SATURATE(cmd->torque, -127.99f, 127.99f);
    UNITREE_SATURATE(cmd->speed, -804.00f, 804.00f);
    UNITREE_SATURATE(cmd->position, -411774.0f, 411774.0f);

    /* 协议实测为转子侧量:上层关节语义需在此做减速比换算 */
    tor_des = (int16_t)(cmd->torque * UNITREE_GEAR_RATIO * 256.0f);
    spd_des = (int16_t)((cmd->speed * UNITREE_GEAR_RATIO) / UNITREE_2PI * 256.0f);
    pos_des = (int32_t)((cmd->position * UNITREE_GEAR_RATIO) / UNITREE_2PI * 32768.0f);
    kp = (int16_t)(cmd->kp / 25.6f * 32768.0f);
    kd = (int16_t)(cmd->kd / 25.6f * 32768.0f);

    packet[0] = 0xFE;
    packet[1] = 0xEE;
    packet[2] = (uint8_t)((cmd->id & 0x0FU) | ((cmd->mode & 0x07U) << 4));
    packet[3] = (uint8_t)(tor_des & 0xFF);
    packet[4] = (uint8_t)((tor_des >> 8) & 0xFF);
    packet[5] = (uint8_t)(spd_des & 0xFF);
    packet[6] = (uint8_t)((spd_des >> 8) & 0xFF);
    packet[7] = (uint8_t)(pos_des & 0xFF);
    packet[8] = (uint8_t)((pos_des >> 8) & 0xFF);
    packet[9] = (uint8_t)((pos_des >> 16) & 0xFF);
    packet[10] = (uint8_t)((pos_des >> 24) & 0xFF);
    packet[11] = (uint8_t)(kp & 0xFF);
    packet[12] = (uint8_t)((kp >> 8) & 0xFF);
    packet[13] = (uint8_t)(kd & 0xFF);
    packet[14] = (uint8_t)((kd >> 8) & 0xFF);

    {
        uint16_t crc = crc_ccitt(0, packet,
                                 sizeof(*frame) - sizeof(frame->crc16));
        packet[15] = (uint8_t)(crc & 0xFF);
        packet[16] = (uint8_t)((crc >> 8) & 0xFF);
    }
}

/*
 * @brief  解析一帧宇树电机回包，并转换成统一的工程量纲。
 * @param  data 存放解析结果的结构体。
 * @details
 *  校验帧头和 CRC；提取 ID、模式、温度、错误状态、转矩、速度、位置，
 *  再把转子侧协议值换算到统一的关节语义单位，便于上层控制使用。
 */
void UnitreeMotor_DecodeFrame(UnitreeMotorData_t *data)
{
    const UnitreeMotorFeedbackFrame_t *frame;
    const uint8_t *packet;
    uint16_t rx_crc = 0U;
    uint16_t status_force = 0U;
    int16_t speed = 0;
    int16_t torque = 0;
    int32_t pos = 0;

    if (data == NULL)
    {
        return;
    }

    frame = &data->frame;
    packet = (const uint8_t *)frame;

    if (packet[0] != 0xFD || packet[1] != 0xEE)
    {
        data->correct = 0;
        return;
    }

    status_force = (uint16_t)packet[12] | ((uint16_t)packet[13] << 8U);

    data->calc_crc = crc_ccitt(0, packet,
                               sizeof(*frame) - sizeof(frame->crc16));
    rx_crc = (uint16_t)packet[14] | ((uint16_t)packet[15] << 8U);

    if (rx_crc != data->calc_crc)
    {
        memset(&data->frame, 0, sizeof(data->frame));
        data->correct = 0;
        data->bad_msg++;
        return;
    }

    data->id = packet[2] & 0x0FU;
    data->mode = (packet[2] >> 4) & 0x07U;
    data->temperature = (int)(int8_t)packet[11];
    data->error = (int)(status_force & 0x0007U);

    torque = (int16_t)((uint16_t)packet[3] | ((uint16_t)packet[4] << 8U));
    speed = (int16_t)((uint16_t)packet[5] | ((uint16_t)packet[6] << 8U));
    pos = (int32_t)((uint32_t)packet[7] |
                    ((uint32_t)packet[8] << 8U) |
                    ((uint32_t)packet[9] << 16U) |
                    ((uint32_t)packet[10] << 24U));

    /* 协议回包实测为转子侧量:转换到上层统一的关节语义 */
    data->torque = ((float)torque) / 256.0f * UNITREE_GEAR_RATIO;
    data->speed = (((float)speed / 256.0f) * UNITREE_2PI) / UNITREE_GEAR_RATIO;
    data->position = (UNITREE_2PI * ((float)pos) / 32768.0f) / UNITREE_GEAR_RATIO;
    data->foot_force = (int)((status_force >> 3U) & 0x0FFFU);
    data->correct = 1;
}

/*
 * @brief  从环形队列中取出完整 16 字节反馈帧并解析。
 */
static void UnitreeMotor_ParseRxQueue(void)
{
    uint8_t byte;

    while (RingBuffer_Pop(&s_unitree_rx_queue, &byte))
    {
        if (!s_unitree_frame_started)
        {
            if (byte == 0xFD)
            {
                s_unitree_frame[0] = 0xFD;
                s_unitree_frame_len = 1U;
                s_unitree_frame_started = true;
            }
            continue;
        }

        if (s_unitree_frame_len == 1U)
        {
            if (byte == 0xEE)
            {
                s_unitree_frame[1] = 0xEE;
                s_unitree_frame_len = 2U;
            }
            else if (byte == 0xFD)
            {
                s_unitree_frame[0] = 0xFD;
                s_unitree_frame_len = 1U;
            }
            else
            {
                s_unitree_frame_started = false;
                s_unitree_frame_len = 0U;
            }
            continue;
        }

        s_unitree_frame[s_unitree_frame_len++] = byte;
        if (s_unitree_frame_len >= UNITREE_FRAME_LENGTH)
        {
            UnitreeMotorData_t *rx_data = &s_unitree_rx_data;

            memset(rx_data, 0, sizeof(*rx_data));
            memcpy(&rx_data->frame, s_unitree_frame, sizeof(rx_data->frame));
            UnitreeMotor_DecodeFrame(rx_data);

            if (rx_data->correct == 1 && rx_data->id < UNITREE_MOTOR_NUM)
            {
                uint32_t bad_msg = Unitree_motors[rx_data->id].data.bad_msg;
                Unitree_motors[rx_data->id].data = *rx_data;
                Unitree_motors[rx_data->id].data.bad_msg = bad_msg;
                Unitree_motors[rx_data->id].data.position -=
                    Unitree_motors[rx_data->id].zero_offset;
            }

            s_unitree_frame_started = false;
            s_unitree_frame_len = 0U;
        }
    }
}

/*
 * @brief  UART 双缓冲空闲中断入口。
 * @param  data  保留参数，实际数据从双缓冲中读取。
 * @param  size  本次空闲前接收到的字节数。
 */
void UnitreeMotor_UART_RxHandler(const uint8_t *data, uint16_t size)
{
    UART_HandleTypeDef *uart = UnitreeMotor_GetUart();
    uint8_t *src;
    uint32_t i;

    (void)data;
    if (uart == NULL || uart->hdmarx == NULL)
    {
        return;
    }

    if (((((DMA_Stream_TypeDef *)uart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
    {
        __HAL_DMA_DISABLE(uart->hdmarx);
        ((DMA_Stream_TypeDef *)uart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(uart->hdmarx, UNITREE_RX_BUFFER_SIZE);
        src = Unitree_UART7_RxBuffer;
    }
    else
    {
        __HAL_DMA_DISABLE(uart->hdmarx);
        ((DMA_Stream_TypeDef *)uart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
        __HAL_DMA_SET_COUNTER(uart->hdmarx, UNITREE_RX_BUFFER_SIZE);
        src = s_unitree_uart7_rx_buf2;
    }

    if (size > UNITREE_RX_BUFFER_SIZE)
    {
        size = UNITREE_RX_BUFFER_SIZE;
    }

    SCB_InvalidateDCache_by_Addr((uint32_t *)src, size);
    for (i = 0U; i < size; i++)
    {
        RingBuffer_Push(&s_unitree_rx_queue, src[i]);
    }

    __HAL_DMA_ENABLE(uart->hdmarx);
    UnitreeMotor_ParseRxQueue();
}

void UnitreeMotor_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart != UnitreeMotor_GetUart())
    {
        return;
    }
    Unitree_SetRxMode();
}

/*
 * @brief  向指定电机发送一帧控制命令。
 * @param  motor 目标电机对象。
 * @details
 *  仅在该电机处于 begin 状态、UART 空闲且允许发送时才发送；
 *  发送前会把目标位置叠加当前零点偏移，用于矫正相对零点控制。
 */
static void UnitreeMotor_SendCommand(UnitreeMotor *motor)
{
    UART_HandleTypeDef *uart = UnitreeMotor_GetUart();

    if (motor == NULL || uart == NULL || !motor->begin)
    {
        return;
    }

    /* UART7 的 RX DMA 长期处于接收状态,因此用 gState(TX 状态)判断是否可发送 */
    if (uart->gState != HAL_UART_STATE_READY)
    {
        return;
    }
    Unitree_SetTxMode();

    s_unitree_tx_cmd = motor->cmd;
    s_unitree_tx_cmd.position += motor->zero_offset; /* 用户目标 + 零位偏移 */
    UnitreeMotor_EncodeFrame(&s_unitree_tx_cmd);

    SCB_CleanDCache_by_Addr((uint32_t *)&s_unitree_tx_cmd.frame,
                            sizeof(s_unitree_tx_cmd.frame));
    HAL_UART_Transmit_DMA(uart, (uint8_t *)&s_unitree_tx_cmd.frame,
                          sizeof(s_unitree_tx_cmd.frame));
}

/*
 * @brief  对指定电机执行零点校准。
 * @param  motor_id 目标电机编号，-1 表示全部电机。
 * @details
 *  记录当前读取到的角度作为偏移量，并将本电机的目标位置和反馈位置清零，
 *  从而后续控制以零点为基准执行。
 */
static void UnitreeMotor_SetZero(int motor_id)
{
    if (motor_id < 0 || motor_id >= UNITREE_MOTOR_NUM)
    {
        for (uint32_t i = 0U; i < UNITREE_MOTOR_NUM; i++)
        {
            Unitree_motors[i].zero_offset += Unitree_motors[i].data.position;
            // Unitree_motors[i].cmd.position = Unitree_motors[i].zero_pos;
            // Unitree_motors[i].data.position = Unitree_motors[i].zero_pos;
            Unitree_motors[i].cmd.position = 0.f;
            Unitree_motors[i].data.position = 0.f;
        }
        return;
    }

    Unitree_motors[motor_id].zero_offset += Unitree_motors[motor_id].data.position;
    Unitree_motors[motor_id].cmd.position = 0.0f;
    Unitree_motors[motor_id].data.position = 0.0f;
}

/*
 * @brief  使能指定电机（或全部电机）。
 * @param  motor_id 电机编号，-1 表示全部。
 */
static void UnitreeMotor_Enable(int motor_id)
{
    if (motor_id < 0 || motor_id >= UNITREE_MOTOR_NUM)
    {
        for (uint32_t i = 0U; i < UNITREE_MOTOR_NUM; i++)
        {
            Unitree_motors[i].enable = true;
        }
        return;
    }

    Unitree_motors[motor_id].enable = true;
}

/*
 * @brief  禁用指定电机（或全部电机）。
 * @param  motor_id 电机编号，-1 表示全部。
 * @details
 *  禁用后该电机进入空闲模式，不再持续发送 FOC 控制命令。
 */
static void UnitreeMotor_Disable(int motor_id)
{
    if (motor_id < 0 || motor_id >= UNITREE_MOTOR_NUM)
    {
        for (uint32_t i = 0U; i < UNITREE_MOTOR_NUM; i++)
        {
            Unitree_motors[i].enable = false;
        }
        return;
    }

    Unitree_motors[motor_id].enable = false;
}

/*
 * @brief  宇树电机主循环控制函数。
 * @details
 *  1. 根据 enable 状态切换电机模式：使能时走 FOC，未使能时空闲；
 *  2. 若设置了零点校准标志，则执行零点校准；
 *  3. 轮询发送各电机命令，采用 round-robin 的方式分时发送。
 */
void UnitreeMotor_Func(void)
{
    if (!s_unitree_initialized)
    {
        return;
    }

    for (uint32_t i = 0U; i < UNITREE_MOTOR_NUM; i++)
    {
        UnitreeMotor *motor = &Unitree_motors[i];
        if (!set_cur_init && motor->data.correct)
        {
            motor->cmd.position = motor->data.position;
            set_cur_init = true;
        }

        if (motor->enable)
        {
            motor->cmd.mode = UNITREE_MOTOR_MODE_FOC;
        }
        else
        {
            motor->cmd.mode = UNITREE_MOTOR_MODE_IDLE;
        }

        if (motor->set_zero && motor->data.correct)
        {
            UnitreeMotor_SetZero((int)i);
            motor->set_zero = false;
        }
    }

    UnitreeMotor_SendCommand(&Unitree_motors[s_unitree_tx_index]);

    s_unitree_tx_index++;
    if (s_unitree_tx_index >= UNITREE_MOTOR_NUM)
    {
        s_unitree_tx_index = 0U;
    }
}

#endif /* USE_UNITREE */
