/**
 * @file    FD_Canqueue.h
 * @brief   Lightweight FDCAN TX queue ported from R2 chassis.
 *
 * 只负责发送:驱动入队(ZdriveEnqueue / VESC 直接入队),TIM2 中断里按
 * 1kHz 恒定出队。接收不走软件队列 —— 各电机反馈帧在 FDCAN 接收中断里
 * 直接解析(见 IQRhandler.c 的 HAL_FDCAN_RxFifo0/1Callback)。
 */
#ifndef FD_CANQUEUE_H
#define FD_CANQUEUE_H

#include <stdbool.h>
#include "main.h"
#include "fdcan.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define FDCAN_QUEUESIZE 12U

    typedef struct
    {
        uint32_t ID;
        uint8_t DLC;
        uint32_t IDE;
        uint8_t Data[8];
        bool InConGrpFlag;
    } FDCAN_DataStruct;

    typedef struct
    {
        uint8_t Front;
        uint8_t Rear;
        FDCAN_GlobalTypeDef *Canx;
        FDCAN_DataStruct FDCAN_DataSend[FDCAN_QUEUESIZE];
    } FDCAN_SendQueueType;

    extern FDCAN_SendQueueType CAN1_Txqueue;
    extern FDCAN_SendQueueType CAN2_Txqueue;
    extern FDCAN_SendQueueType CAN3_Txqueue;

    void CAN_InitSendQueue(void);
    /** FDCAN 启动:全局滤波 + FIFO 覆盖 + Start + RX 通知(.ioc 未开 Start,统一在这里做) */
    void CAN_Start(void);
    bool CAN_Queue_IfEmpty(FDCAN_SendQueueType *queue);
    bool CAN_Queue_IfFull(FDCAN_SendQueueType *queue);
    bool CAN_DequeueTx(FDCAN_SendQueueType *queue);
    void CAN_Enqueue(FDCAN_SendQueueType *queue, FDCAN_RxHeaderTypeDef Rxheader, uint8_t Rxdata[]);
    void HeaderPrepare(uint32_t sendCode, uint32_t datalen, FDCAN_RxHeaderTypeDef *rxheader);

#ifdef __cplusplus
}
#endif

#endif /* FD_CANQUEUE_H */
