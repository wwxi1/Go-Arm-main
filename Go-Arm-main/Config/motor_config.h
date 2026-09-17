/**
 * @file    motor_config.h
 * @brief   电机清单与 CAN 总线分配,电机开关也在这里。
 *
 * 参考工程的总线约定:
 *   - FDCAN2:CAN2 总线,VESC + ZDrive 第一路
 *   - FDCAN3:CAN3 总线,DJI M2006/M3508 + ZDrive 第二路(拆分时)
 *   - FDCAN1:CAN1 总线,预留给用户通信
 */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ------------------------------------------------------------------ */
/* 电机驱动开关:1 = 编译并使用,0 = 不编译                               */
/* ------------------------------------------------------------------ */
#define USE_DJ 1
#define USE_VESC 0 // 未验证,不要启用
#define USE_ZMDR 0
#define USE_UNITREE 1

/* ------------------------------------------------------------------ */
/* DJI M2006 / M3508                                                    */
/* ------------------------------------------------------------------ */
#define MOTOR_DJI_COUNT 4U  /* 必须为 4 或 8(DJI CAN 打包要求) */
#define MOTOR_DJI_CAN_BUS 1 /* 0=FDCAN1,1=FDCAN2,2=FDCAN3 */

#define MOTOR_M2006_COUNT 0U
#define MOTOR_M3508_COUNT 4U
#define MOTOR_M2006_REDUCTION_RATIO 36U
#define MOTOR_M3508_REDUCTION_RATIO 19.20320855f

/* ------------------------------------------------------------------ */
/* VESC                                                                 */
/* ------------------------------------------------------------------ */
#define MOTOR_VESC_COUNT 4U
#define MOTOR_VESC_CAN_BUS 1 /* FDCAN2               */
#define MOTOR_VESC_POLE_PAIRS 7U

/* ------------------------------------------------------------------ */
/* ZDrive                                                              */
/* 总线拆分:SPLIT_COUNT = 0 → 全部走第一路 CAN;= n → ID 1..n 走第一路,  */
/* ID n+1..COUNT 走第二路。                                              */
/* NOTE: ZDrive 的帧 ID = motor_id | (op_code<<4),低 4 位 ID 空间 1..N   */
/* 与 DJI 反馈 ID 0x201..0x204 的低 4 位重叠,Zdrive_IsOurs 无法区分,     */
/* 原则上两者不同总线。                                                */
/* ------------------------------------------------------------------ */
#define MOTOR_ZDRIVE_COUNT 1U       /* 最多控 8 个电机 */
#define MOTOR_ZDRIVE_SPLIT_COUNT 4U /* 0=不拆分;n=前 n 个 ID 走第一路 */
#define MOTOR_ZDRIVE_CAN_BUS_1 1U   /* 第一路:FDCAN2 */
#define MOTOR_ZDRIVE_CAN_BUS_2 2U   /* 第二路:FDCAN3 */
#define MOTOR_ZDRIVE_BUS_RETRANS_CNT 2 //调用出队函数时,单BUS连续发送的次数

/* ------------------------------------------------------------------ */
/* Unitree GO-M8010-6 (RS485)                                          */
/* 当前硬件只有 UART7 一路 RS485,同一总线上电机 ID 从 0 开始。          */
/* 电机带绝对编码器，驱动默认不上电自动清零；上电后需按机械零位 SetZero。*/
/* ------------------------------------------------------------------ */
#define MOTOR_UNITREE_COUNT 2U
#define MOTOR_UNITREE_UART 7U
#define MOTOR_UNITREE_REDUCTION_RATIO 6.33f
#define RS_485_U7_GPIO_Port GPIOG
#define RS_485_U7_Pin GPIO_PIN_1
#define UNITREE_MOTOR_NUM MOTOR_UNITREE_COUNT
#define UNITREE_RX_BUFFER_SIZE 64U
#define MOTOR_UNITREE_DEFAULT_KP 4.8f
#define MOTOR_UNITREE_DEFAULT_KW 0.024f

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONFIG_H */
