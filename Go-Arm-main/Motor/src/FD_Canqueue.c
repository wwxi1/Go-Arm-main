/**
 * @file    FD_Canqueue.c
 * @brief   FDCAN TX queue implementation, ported and slightly hardened.
 *
 * 纯发送队列:ZdriveEnqueue / VESC 入队,TIM2 中断 CAN_DequeueTx 出队。
 * 接收不再经过软件队列,反馈帧在 FDCAN 接收中断里直接解析。
 */
#include "includes.h"
#include "FD_Canqueue.h"
#include <string.h>

FDCAN_SendQueueType CAN1_Txqueue;
FDCAN_SendQueueType CAN2_Txqueue;
FDCAN_SendQueueType CAN3_Txqueue;

static bool s_can_queue_initialized;

void CAN_InitSendQueue(void)
{
    if (s_can_queue_initialized)
    {
        return; /* do not reset queues that may already contain frames */
    }

    CAN1_Txqueue.Front = CAN1_Txqueue.Rear = 0;
    CAN2_Txqueue.Front = CAN2_Txqueue.Rear = 0;
    CAN3_Txqueue.Front = CAN3_Txqueue.Rear = 0;

    CAN1_Txqueue.Canx = FDCAN1;
    CAN2_Txqueue.Canx = FDCAN2;
    CAN3_Txqueue.Canx = FDCAN3;

    s_can_queue_initialized = true;
}

void CAN_Start(void)
{
    /* 全局滤波:标准帧和扩展帧都进 FIFO0,拒绝远程帧 */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0U);
    HAL_FDCAN_Start(&hfdcan1);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0U);
    HAL_FDCAN_Start(&hfdcan2);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan3, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0U);
    HAL_FDCAN_Start(&hfdcan3);
}

bool CAN_Queue_IfEmpty(FDCAN_SendQueueType *queue)
{
    return (queue->Front == queue->Rear);
}

bool CAN_Queue_IfFull(FDCAN_SendQueueType *queue)
{
    return (((uint16_t)queue->Rear + 1U) % FDCAN_QUEUESIZE == queue->Front);
}

bool CAN_DequeueTx(FDCAN_SendQueueType *queue)
{
    FDCAN_TxHeaderTypeDef tx_message;
    uint8_t tx_data[8] = {0};

    if (CAN_Queue_IfEmpty(queue))
    {
        return false;
    }

    tx_message.TxFrameType = FDCAN_DATA_FRAME;
    tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_message.BitRateSwitch = FDCAN_BRS_OFF;
    tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_message.MessageMarker = 0;
    tx_message.IdType = queue->FDCAN_DataSend[queue->Front].IDE;
    tx_message.Identifier = queue->FDCAN_DataSend[queue->Front].ID;
    tx_message.DataLength = queue->FDCAN_DataSend[queue->Front].DLC;
    if (tx_message.DataLength > 8U)
    {
        tx_message.DataLength = 8U; /* template uses classic CAN 8-byte frames */
    }

    memcpy(tx_data, queue->FDCAN_DataSend[queue->Front].Data,
           (size_t)tx_message.DataLength * sizeof(uint8_t));

    HAL_StatusTypeDef status = HAL_ERROR;

    if (queue->Canx == FDCAN1)
    {
        status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_message, tx_data);
    }
    else if (queue->Canx == FDCAN2)
    {
        status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_message, tx_data);
    }
    else if (queue->Canx == FDCAN3)
    {
        status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx_message, tx_data);
    }

    if (status != HAL_OK)
    {
        return false; /* keep the frame in queue, retry next period */
    }

    queue->Front = (uint8_t)(((uint16_t)queue->Front + 1U) % FDCAN_QUEUESIZE);
    return true;
}

void CAN_Enqueue(FDCAN_SendQueueType *queue, FDCAN_RxHeaderTypeDef Rxheader, uint8_t Rxdata[])
{
    if (CAN_Queue_IfFull(queue))
    {
        return;
    }

    queue->FDCAN_DataSend[queue->Rear].DLC = (Rxheader.DataLength > 8U) ? 8U : (uint8_t)Rxheader.DataLength;
    queue->FDCAN_DataSend[queue->Rear].ID = Rxheader.Identifier;
    queue->FDCAN_DataSend[queue->Rear].IDE = Rxheader.IdType;
    memcpy(queue->FDCAN_DataSend[queue->Rear].Data, Rxdata,
           (size_t)queue->FDCAN_DataSend[queue->Rear].DLC * sizeof(uint8_t));

    queue->Rear = (uint8_t)(((uint16_t)queue->Rear + 1U) % FDCAN_QUEUESIZE);
}

void HeaderPrepare(uint32_t sendCode, uint32_t datalen, FDCAN_RxHeaderTypeDef *rxheader)
{
    rxheader->IdType = FDCAN_EXTENDED_ID;
    rxheader->Identifier = sendCode;
    rxheader->DataLength = datalen;
}
