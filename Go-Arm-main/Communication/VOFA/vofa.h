#ifndef VOFA_PLUS_H
#define VOFA_PLUS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "includes.h"
#include "vector.h"
#include "app_config.h"
#include "usart.h"

#define VOFA_CHANNEL_NUM 6

// 调试数据的后缀
#define VOFA_SUFFIX1 0x00
#define VOFA_SUFFIX2 0x00
#define VOFA_SUFFIX3 0x80
#define VOFA_SUFFIX4 0x7F

    // 支持的数据类型
    typedef enum
    {
        VOFA_TYPE_FLOAT,
        VOFA_TYPE_INT32,
        VOFA_TYPE_UINT32,
        VOFA_TYPE_INT16,
        VOFA_TYPE_UINT16,
        VOFA_TYPE_INT8,
        VOFA_TYPE_UINT8
    } VOFA_DataType_t;

#if APP_VOFA_ENABLE
    // VOFA 通信相关函数

    /*
    在这里更新 VOFA 通信数据, channel 为通道号, type 为数据类型, data 为数据指针.
    数据会写入内部缓存, 需要调用 VOFA_Update() 才会发送出去.
    */
    bool VOFA_Channel_Update(uint8_t channel, VOFA_DataType_t type, void *data);
    /*
    推荐放在一个freertos任务中,以合适的频率调用它.
    */
    void VOFA_Update(void);

    /*
    用于注册uart的 发送完成中断 回调函数
    */
    void VOFA_DMA_TransmitCpltCallback(UART_HandleTypeDef *huart);

#endif /* APP_VOFA_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* VOFA_PLUS_H */