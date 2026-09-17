/**
 * @file    bluetooth_serv.h
 * @brief   蓝牙业务消息层(Application 层):板子与上位机之间的命令/状态收发。
 *
 * 收发各用一个独立结构体,命名直接体现方向:
 *   接收(RX):BT_Cmd_t      —— 板子收到的"命令"(上位机 → 板子)
 *   发送(TX):BT_Status_t   —— 板子反馈的"状态"(板子 → 上位机)
 *
 * 两个结构体当前复用 bluetooth_protocol.h 的同一套宏布局(同一份字节),
 * 只是字段含义不同;若真实协议里命令/状态字段类型数量不同,
 * 再给协议层加两套宏(每方向一套)。
 *
 * 业务层(用户)要写的就是 bluetooth_serv.c 里的 Pack(状态)/Parse(命令),
 * 其余是快速调用样板。
 */
#ifndef BLUETOOTH_SERV_H
#define BLUETOOTH_SERV_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* 服务开关:1 = 在 VOFA_SendTask 里周期发状态 + 收命令(联调用,默认关)。   */
/* ------------------------------------------------------------------ */
#define BT_SERV_ENABLE 1U

/* ------------------------------------------------------------------ */
/* 板子接收到的"命令"(字段顺序与上位机协议一致)                          */
/* ------------------------------------------------------------------ */
typedef struct {
    bool     enable;    /* flag 0: 使能 */
    bool     auto_mode; /* flag 1: 自动模式 */
    bool     calib;     /* flag 2: 标定 */
    bool     stop;      /* flag 3: 急停 */
    bool     slow;      /* flag 4: 低速 */
    bool     fast;      /* flag 5: 高速 */
    bool     reset;     /* flag 6: 复位 */
    uint16_t speed;     /* 目标速度 */
    uint16_t angle;     /* 目标角度 */
    float    vx;        /* 目标速度值 */
} BT_Cmd_t;

/* ------------------------------------------------------------------ */
/* 板子发送的"状态"反馈(字段顺序与上位机协议一致)                         */
/* ------------------------------------------------------------------ */
typedef struct {
    bool     online;   /* flag 0: 在线 */
    bool     ready;    /* flag 1: 就绪 */
    bool     fault;    /* flag 2: 故障 */
    bool     limited;  /* flag 3: 限幅 */
    bool     stall;    /* flag 4: 堵转 */
    bool     brake;    /* flag 5: 刹车 */
    bool     estop;    /* flag 6: 急停生效 */
    uint16_t rpm;      /* 实际转速 */
    uint16_t vbus;     /* 母线电压(0.1V) */
    float    current;  /* 实际电流 */
} BT_Status_t;

/* ------------------------------------------------------------------ */
/* 快速调用框架                                                         */
/* ------------------------------------------------------------------ */
/* 发送状态(板子 → 上位机):BT_Status_t → 打包 → Bluetooth_Send()(入 TX 队列)。
   返回 Bluetooth_Send 的结果(入队是否成功)。 */
bool BT_Serv_Status_Send(const BT_Status_t *status);

/* 接收命令(上位机 → 板子):Bluetooth_Receive() → 解析 → BT_Cmd_t。
   timeout_ms 同 Bluetooth_Receive(0 = 非阻塞);没收到帧返回 false。 */
bool BT_Serv_Cmd_Receive(BT_Cmd_t *cmd, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BLUETOOTH_SERV_H */
