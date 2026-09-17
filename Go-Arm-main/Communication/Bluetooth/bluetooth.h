/**
 * @file    bluetooth.h
 * @brief   蓝牙串口通信模块(FreeRTOS 基础 Queue 版)。
 *
 * 职责分层:
 *   UART 层  : 只负责 DMA 收发与 IDLE 中断(CubeMX 生成 + IQRhandler.c 回调)
 *   本模块   : 协议帧(包头/包尾/校验,CRC16 或单字节校验和可选)、RX/TX Queue、收发任务
 *   业务层   : 只调用 Bluetooth_Init() / Bluetooth_Send() / Bluetooth_Receive()
 *
 * 业务层不需要知道 UART、DMA、IDLE、CRC、缓冲区的存在。
 * 蓝牙只传一种数据:业务层拿到 payload 后按自己的约定解析,模块不关心内容。
 *
 * 用法(在业务任务里):
 *   // 发送:组一个消息,丢给模块,剩下的事模块管
 *   uint8_t data[2] = {0x01, 0x02};
 *   Bluetooth_Send(data, 2);
 *
 *   // 接收:从 RX Queue 取一帧,超时返回 false
 *   BluetoothFrame_t frame;
 *   if (Bluetooth_Receive(&frame, 100)) {
 *       // frame.payload[0 .. frame.length-1] 就是本帧数据
 *   }
 */
#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>
#include <stdbool.h>

#include "bluetooth_protocol.h" /* BT_PROTO_* 访问函数 + BT_PROTO_PAYLOAD_SIZE */

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* 配置(按需修改;收发双方必须一致)                                        */
/* ------------------------------------------------------------------ */
#define BT_PAYLOAD_LENGTH       0U      /* 线上帧包长(收发双方约定一致,编译期定死):
                                           0 = 帧里不含包长字段,包头后直接跟包体;
                                           1..N = 帧里带 1 字节包长字段,包体固定为该字节数 */
#define BT_RX_QUEUE_SIZE        8U      /* 业务层接收队列深度(帧数) */
#define BT_TX_QUEUE_SIZE        8U      /* 发送队列深度(帧数) */
#define BT_SEND_TIMEOUT_MS      100U    /* Bluetooth_Send() 入队最长等待 */
#define BT_TASK_STACK_WORDS     128U    /* 收发任务栈大小(字) */
#define BT_RX_TASK_PRIORITY     3U      /* 接收任务优先级(数值越大越高) */
#define BT_TX_TASK_PRIORITY     4U      /* 发送任务优先级 */

/* ------------------------------------------------------------------ */
/* 帧结构:Queue 里直接存整个结构体(按值拷贝),业务层不用管内存/生命周期     */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t  payload[BT_PROTO_PAYLOAD_SIZE];    /* 包体数据 */
    uint16_t length;                          /* 包体有效字节数(1 .. 最大) */
} BluetoothFrame_t;

/* ------------------------------------------------------------------ */
/* API(业务层只用这三个)                                                 */
/* ------------------------------------------------------------------ */
void Bluetooth_Init(void);
bool Bluetooth_Send(const uint8_t *payload, uint16_t length);
bool Bluetooth_Receive(BluetoothFrame_t *frame, uint32_t timeout_ms);

/* 以下两个函数由 UART 中断回调调用(IQRhandler.c),业务层不要碰 */
void Bluetooth_UartRxEvent(const uint8_t *data, uint16_t size);
void Bluetooth_UartTxCplt(void);

#ifdef __cplusplus
}
#endif

#endif /* BLUETOOTH_H */
