/**
 * @file    VescMotor.h
 * @brief   VESC CAN driver, ported from R2 chassis.
 */
#ifndef VESCMOTOR_H
#define VESCMOTOR_H

#include <stdbool.h>
#include "main.h"
#include "mathFunc.h"
#include "FD_Canqueue.h"
#include "motor_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USE_VESCNUM     MOTOR_VESC_COUNT

typedef enum {
    VESC_RPM = 0,
    VESC_POSITION,
    VESC_CURRENT,
    VESC_DUTY,
    VESC_HANDBRAKE
} VescMode_t;

typedef enum {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_FILL_RX_BUFFER,
    CAN_PACKET_FILL_RX_BUFFER_LONG,
    CAN_PACKET_PROCESS_RX_BUFFER,
    CAN_PACKET_PROCESS_SHORT_BUFFER,
    CAN_PACKET_STATUS,
    CAN_PACKET_SET_CURRENT_REL,
    CAN_PACKET_SET_CURRENT_BRAKE_REL,
    CAN_PACKET_SET_CURRENT_HANDBRAKE,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
    CAN_PACKET_STATUS_2,
    CAN_PACKET_STATUS_3,
    CAN_PACKET_STATUS_4,
    CAN_PACKET_PING,
    CAN_PACKET_PONG,
    CAN_PACKET_DETECT_APPLY_ALL_FOC,
    CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
    CAN_PACKET_CONF_CURRENT_LIMITS,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
    CAN_PACKET_CONF_CURRENT_LIMITS_IN,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
    CAN_PACKET_CONF_FOC_ERPMS,
    CAN_PACKET_CONF_STORE_FOC_ERPMS,
    CAN_PACKET_STATUS_5
} VescCanPacketId_t;

typedef struct {
    float current;
    float speed;
    float angle;
    float duty;
    volatile int16_t handbrakeCurrent;
    float angleABS;
} VescValue;

typedef struct {
    bool timeoutflag;
    bool stuckflag;
} VescStatus;

typedef struct {
    bool    timeoutCheck;
    bool    StuckCheck;
    bool    releaseWhenStuck;
    uint8_t CurrentLimit;
} VecsLimit;

typedef struct {
    uint32_t lastRxTime;
    uint32_t TimeoutTick;
    uint32_t stuckCount;
    volatile u16 angleNow;
    volatile u16 anglePre;
    volatile s16 distance;
    volatile int32_t position;
} VescArgum;

typedef struct {
    volatile bool Enable;
    volatile bool Begin;
    uint8_t  mode;
    uint8_t  PolePairs;
    VescValue valSet;
    VescValue valNow;
    VecsLimit limit;
    VescStatus statusflag;
    VescArgum  argum;
} VescMOTOR;

#if USE_VESC
extern VescMOTOR Vescmotor[USE_VESCNUM];

void VescReceiveData_CAN2(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data);
void VescInit(void);
void VescFunc(void);
void VescPosition_Mode(uint8_t controlID, float position);
void VescRPM_Mode(uint8_t controlID, float speed);
void VescCurrent_Mode(uint8_t controlID, float current);
void VescDuty_Mode(uint8_t controlID, float duty);
void VescBrake_Mode(uint8_t controlID, float handbrake);
void VescStuck_Check(uint8_t id);
void VescTimeOut(uint8_t id);
#endif /* USE_VESC */

#ifdef __cplusplus
}
#endif

#endif /* VESCMOTOR_H */
