#include "vofa.h"

#define VOFA_PACKET_SIZE (4 * (VOFA_CHANNEL_NUM + 1))

// 调试数据包结构体
typedef struct _vofa_msg
{
    float datach[VOFA_CHANNEL_NUM];
    uint8_t tail[4];
} VOFATxMsgTypedef;

static VOFATxMsgTypedef VofaTxPack = {{0}, {VOFA_SUFFIX1, VOFA_SUFFIX2, VOFA_SUFFIX3, VOFA_SUFFIX4}};
static __RAM_D1_ ALIGN_32B uint8_t vofa_buffer[VOFA_PACKET_SIZE][2] = {0};
static bool vofa_buffer_num = 0;
static volatile bool vofa_dma_busy = 0;

bool VOFA_Channel_Update(uint8_t channel, VOFA_DataType_t type, void *data)
{
    if (channel >= VOFA_CHANNEL_NUM)
    {
        return false;
    }

    switch (type)
    {
    case VOFA_TYPE_FLOAT:
        VofaTxPack.datach[channel] = *(float *)data;
        break;
    case VOFA_TYPE_INT32:
        VofaTxPack.datach[channel] = (float)(*(int32_t *)data);
        break;
    case VOFA_TYPE_UINT32:
        VofaTxPack.datach[channel] = (float)(*(uint32_t *)data);
        break;
    case VOFA_TYPE_INT16:
        VofaTxPack.datach[channel] = (float)(*(int16_t *)data);
        break;
    case VOFA_TYPE_UINT16:
        VofaTxPack.datach[channel] = (float)(*(uint16_t *)data);
        break;
    case VOFA_TYPE_INT8:
        VofaTxPack.datach[channel] = (float)(*(int8_t *)data);
        break;
    case VOFA_TYPE_UINT8:
        VofaTxPack.datach[channel] = (float)(*(uint8_t *)data);
        break;
    default:
        return false;
    }

    return true;
}

void VOFA_DMA_TransmitCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == BOARD_VOFA_UART.Instance)
    {
        vofa_dma_busy = 0;
    }
}

void VOFA_Update(void)
{
    if (vofa_dma_busy)
    {
        return;
    }
    uint8_t *sending_vofa_buffer=(vofa_buffer[vofa_buffer_num]);
    vofa_buffer_num = !vofa_buffer_num;

    memcpy(sending_vofa_buffer, &VofaTxPack, (VOFA_PACKET_SIZE));
    SCB_CleanDCache_by_Addr((uint32_t *)(sending_vofa_buffer), (VOFA_PACKET_SIZE));
    HAL_UART_Transmit_DMA(&BOARD_VOFA_UART, sending_vofa_buffer, (VOFA_PACKET_SIZE));
}