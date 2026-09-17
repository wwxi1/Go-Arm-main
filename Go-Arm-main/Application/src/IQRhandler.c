/**
 * @file    IQRhandler.c
 * @brief   模板中断回调集中地(参考工程习惯:所有 HAL 回调强定义在一个文件)。
 *
 * 时序约定:
 *   TIM2 @ 1kHz  →  CAN1 用户通信出队 + ZdriveDequeue(ZDrive 总线出队,
 *                   电机报文 1000Hz 恒定发送)
 *                   counter 每 5 次(200Hz)调用各电机 Func
 *                   (接收不走软件队列,全部在 FDCAN 中断里直接解析)
 *   FDCAN RX IRQ →  各电机反馈帧直接解析:ZDrive(Zdrive_IsOurs +
 *                   ZdriveReceive)、DJI(DJmotor_Receive)、VESC
 *                   (VescReceiveData_CAN2);FDCAN 与 TIM2 同为优先级 5,
 *                   同优先级互不抢占,与 TIM2 里的 func 天然互斥
 *   UART idle IRQ → 数据可用,处理完后重新启动 DMA 接收
 *
 * 这里的所有回调都是强定义(CubeMX 生成的是 __weak),重新生成代码不会冲突。
 * main.c 里 CubeMX 生成的 HAL_TIM_PeriodElapsedCallback 在 USER CODE 中转发到
 * 本文件的 TIM_PeriodElapsedCallback()。
 */
#include "includes.h"
#include "IQRhandler.h"
#include "motor_config.h"
#include "ZDrive.h"
#include "DJmotor.h"
#include "VescMotor.h"
#include "UnitreeMotor.h"
#include "vofa.h"
#include "board_config.h"
#include "bluetooth.h"
#include "seize_sky.h"
#include "holding_jaw.h"

static uint8_t Func_cnt = 0;
static uint8_t tim_count =0;

__RAM_D2_ ALIGN_32B uint8_t UART1_RxBuffer[UART_RX_BUFFER_SIZE] = {0};
__RAM_D2_ ALIGN_32B uint8_t UART3_RxBuffer[UART_RX_BUFFER_SIZE] = {0};
__RAM_D2_ ALIGN_32B uint8_t UART4_RxBuffer[UART_RX_BUFFER_SIZE] = {0};
__RAM_D2_ ALIGN_32B uint8_t UART6_RxBuffer[UART_RX_BUFFER_SIZE] = {0};
__RAM_D2_ ALIGN_32B uint8_t UART9_RxBuffer[UART_RX_BUFFER_SIZE] = {0};

/* main.c 的 HAL_TIM_PeriodElapsedCallback 转发到这里 */
void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        /* 1000Hz:发送队列出队,保持电机报文 1kHz 恒定发送。
           CAN1 留给用户通信,通用出队;ZDrive 所在总线由 ZdriveDequeue
           出队(命名归驱动所有)。拆分关闭时 CAN3 出队是空操作(队列可能
           承载 DJI/其他);ZDrive 若配在 CAN1,需注意此处会双重出队。 */

      
         Arm_State_Update();




        CAN_DequeueTx(&CAN1_Txqueue);
#if USE_ZMDR
        ZdriveDequeue((uint8_t)MOTOR_ZDRIVE_CAN_BUS_1);
        ZdriveDequeue((uint8_t)MOTOR_ZDRIVE_CAN_BUS_2);
#else
        CAN_DequeueTx(&CAN2_Txqueue);
        CAN_DequeueTx(&CAN3_Txqueue);
#endif

        /* 1000Hz:接收不再经过软件队列 —— 各电机反馈帧已在 FDCAN
           接收中断里直接解析(见 HAL_FDCAN_RxFifo0/1Callback) */

        
#if USE_UNITREE
        UnitreeMotor_Func();
#endif
        /* 200Hz:func 更新,counter 分频(1kHz ÷ 5)。
           DJI 电机若需要 1kHz 电流环,把 DJmotor_Func() 挪到上面的 1kHz 区即可 */
#if USE_DJ
        DJmotor_Func();
#endif       
        if (++Func_cnt >= 5)
        {
            Func_cnt = 0;
#if USE_VESC
            VescFunc();
#endif
#if USE_ZMDR
            ZdriveFunc();
#endif

        }

    tim_count++;
    if(tim_count>=20)
    {
        tim_count=0;
        Arm_Transmit();
    }




    }
    /* TIM3/4/5 是备用定时器,需要的话在这里加分支,并在 main.c 里启动 */
}

/* FDCAN 接收中断:各电机驱动的反馈帧全部在这里直接解析,不再经过软件
   RX 队列。FDCAN 与 TIM2 同为优先级 5,Cortex-M 同优先级互不抢占,
   Receive 写电机状态与 TIM2 里 Func 读电机状态天然互斥,无需加锁。
   每个驱动的 Receive 内部都按帧 ID 过滤,不是自己的帧自然丢弃。 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    static uint8_t s_full_conter = 0U;
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        FDCAN_RxHeaderTypeDef Rxheader;
        uint8_t Rx_data[8] = {0};
        /* 读取 FIFO0 的新消息,并清除中断标志 */
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &Rxheader, Rx_data);

        if (hfdcan == &hfdcan1)
        {
           



#if USE_ZMDR
            ZdriveReceive(Rxheader, Rx_data, 0U);
#endif
            /* CAN1 预留给用户通信,其余帧在这里直接解析 */
        }
        else if (hfdcan == &hfdcan2)
        {
#if USE_ZMDR
            ZdriveReceive(Rxheader, Rx_data, 1U);
#endif
#if USE_VESC && (MOTOR_VESC_CAN_BUS == 1U)
            VescReceiveData_CAN2(Rxheader, Rx_data);
            return;
#endif
#if USE_DJ && (MOTOR_DJI_CAN_BUS == 1U)
            DJmotor_Receive(Rxheader, Rx_data);
            return;
#endif
        }
        else if (hfdcan == &hfdcan3)
        {

            
            Arm_Receive(Rxheader,Rx_data);



#if USE_ZMDR
            ZdriveReceive(Rxheader, Rx_data, 2U);
#endif
#if USE_VESC && (MOTOR_VESC_CAN_BUS == 2U)
            VescReceiveData_CAN2(Rxheader, Rx_data);
            return;
#endif
#if USE_DJ && (MOTOR_DJI_CAN_BUS == 2U)
            DJmotor_Receive(Rxheader, Rx_data);
            return;
#endif
        }
    }
    else if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL)
    {
        /* FIFO0 满了,清除中断标志,丢弃最旧的帧 */
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, NULL, NULL);
        s_full_conter++;
    }
}
// FIFO1 暂时不启用
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    static uint8_t s_full_conter = 0U;
    if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)
    {
        FDCAN_RxHeaderTypeDef Rxheader;
        uint8_t Rx_data[8] = {0};
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &Rxheader, Rx_data);
        if (hfdcan == &hfdcan1)
        {
        }
        else if (hfdcan == &hfdcan2)
        {
        }
        else if (hfdcan == &hfdcan3)
        {
        }
    }
    else if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_FULL)
    {
        s_full_conter++;
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, NULL, NULL);
    }
}

/* UART 空闲中断:Size 字节可用,处理完重新启动 DMA 接收 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // if (huart->Instance == USART1)
    // {
    //     SCB_InvalidateDCache_by_Addr((uint32_t *)UART1_RxBuffer, Size);
    //     /* 在这里加 USART1 的协议处理 */
    //     HAL_UART_DMAStop(&huart1);
    //     HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_RxBuffer, UART_RX_BUFFER_SIZE);
    // }
     if (huart->Instance == USART3)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)UART3_RxBuffer, Size);
        /* 在这里加 USART3 的协议处理 */
        HAL_UART_DMAStop(&huart3);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_RxBuffer, UART_RX_BUFFER_SIZE);
    }
    else if (huart->Instance == UART4)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)UART4_RxBuffer, Size);
        /* 在这里加 UART4 的协议处理 */
        HAL_UART_DMAStop(&huart4);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART4_RxBuffer, UART_RX_BUFFER_SIZE);
    }
    else if (huart->Instance == USART6)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)UART6_RxBuffer, Size);
        Bluetooth_UartRxEvent(UART6_RxBuffer, Size);   /* 蓝牙模块:搬数据 + 通知解析任务 */
        /* 空闲中断后 DMA 已停,必须重新启动接收(与其它串口一致),否则只收得到第一帧 */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, UART6_RxBuffer, UART_RX_BUFFER_SIZE);
    }
    else if (huart->Instance == UART7)
    {
#if USE_UNITREE && (MOTOR_UNITREE_UART == 7)
        UnitreeMotor_UART_RxHandler(Unitree_UART7_RxBuffer, Size);
#endif
    }
    else if (huart->Instance == UART9)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)UART9_RxBuffer, Size);
        /* 在这里加 UART9 的协议处理 */
        HAL_UART_DMAStop(&huart9);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart9, UART9_RxBuffer, UART_RX_BUFFER_SIZE);
    }
}

/* UART 错误回调:清错误标志并重新启动 DMA 接收 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->ErrorCode & HAL_UART_ERROR_ORE)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
    }
    if (huart->ErrorCode & HAL_UART_ERROR_FE)
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);
    }

    // if (huart->Instance == USART1)
    //     HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_RxBuffer, UART_RX_BUFFER_SIZE);
    if (huart->Instance == USART3)
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_RxBuffer, UART_RX_BUFFER_SIZE);
    else if (huart->Instance == UART4)
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART4_RxBuffer, UART_RX_BUFFER_SIZE);
    else if (huart->Instance == USART6)
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, UART6_RxBuffer, UART_RX_BUFFER_SIZE);
    else if (huart->Instance == UART9)
        HAL_UARTEx_ReceiveToIdle_DMA(&huart9, UART9_RxBuffer, UART_RX_BUFFER_SIZE);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7)
    {
#if USE_UNITREE && (MOTOR_UNITREE_UART == 7)
        UnitreeMotor_UART_TxCpltCallback(huart);
#endif
    }
    if (huart->Instance == BOARD_BLUETOOTH_UART.Instance)
    {
        Bluetooth_UartTxCplt();
    }
    VOFA_DMA_TransmitCpltCallback(huart);
}
