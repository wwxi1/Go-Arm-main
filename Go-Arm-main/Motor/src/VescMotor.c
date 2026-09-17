/**
 * @file    VescMotor.c
 * @brief   VESC CAN driver, ported from R2 chassis.
 */
#include "includes.h"
#include "VescMotor.h"
#include "fdcan.h"

#if USE_VESC

VescMOTOR Vescmotor[USE_VESCNUM];

/* 使用全局 TX 队列,TIM2 中断统一出队发送 */
#if MOTOR_VESC_CAN_BUS == 1
static FDCAN_SendQueueType *Vesc_queue = &CAN2_Txqueue;
#elif MOTOR_VESC_CAN_BUS == 2
static FDCAN_SendQueueType *Vesc_queue = &CAN3_Txqueue;
#else
static FDCAN_SendQueueType *Vesc_queue = &CAN1_Txqueue;
#endif

void VescInit(void)
{
    VecsLimit limit;
    VescValue valset;

    limit.CurrentLimit = 30;
    limit.releaseWhenStuck = true;
    limit.StuckCheck = true;
    limit.timeoutCheck = true;

    valset.current = 0.0f;
    valset.duty = 0.1f;
    valset.speed = 0.0f;
    valset.handbrakeCurrent = 5;

    /* Vesc_queue already points to the global CANx_Txqueue selected by
       MOTOR_VESC_CAN_BUS; CAN_InitSendQueue()(main.c 里调用)设置其 Canx 字段。 */

    for (uint32_t i = 0; i < USE_VESCNUM; i++) {
        Vescmotor[i].limit = limit;
        Vescmotor[i].valSet = valset;
        Vescmotor[i].mode = VESC_POSITION;
        Vescmotor[i].Begin = false;
        Vescmotor[i].Enable = false;
        Vescmotor[i].PolePairs = MOTOR_VESC_POLE_PAIRS;
        Vescmotor[i].argum.anglePre = 0;
    }
}

void VescReceiveData_CAN2(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data)
{
    int32_t index = 0;
    int32_t id = (int32_t)(Rxheader.Identifier & 0xFU) - 1;

    if ((id < 0) || (id >= (int32_t)USE_VESCNUM)) {
        return;
    }

    if (Rxheader.IdType == FDCAN_STANDARD_ID && Rxheader.RxFrameType == FDCAN_DATA_FRAME) {
        uint32_t packet_id = Rxheader.Identifier >> 8U;

        if (packet_id == CAN_PACKET_STATUS) {
            Vescmotor[id].valNow.speed = buffer_32_to_float(Rx_data, 1e0f, &index) /
                                         (float)Vescmotor[id].PolePairs;
            Vescmotor[id].valNow.current = buffer_16_to_float(Rx_data, 1e1f, &index);
            Vescmotor[id].valNow.angle = buffer_16_to_float(Rx_data, 1e1f, &index);
            /* VESC angle bytes are big-endian; keep Rx_data unchanged so other
               callbacks see the original frame. */
            Vescmotor[id].argum.angleNow = (u16)(((uint16_t)Rx_data[6] << 8U) | Rx_data[7]);

            Vescmotor[id].argum.distance = (s16)(Vescmotor[id].argum.angleNow -
                                                  Vescmotor[id].argum.anglePre);
            Vescmotor[id].argum.anglePre = Vescmotor[id].argum.angleNow;

            if (ABS(Vescmotor[id].argum.distance) > 1800) {
                Vescmotor[id].argum.distance = (s16)(Vescmotor[id].argum.distance - 3600);
            }
            Vescmotor[id].argum.position += Vescmotor[id].argum.distance;
            Vescmotor[id].valNow.angleABS = (float)Vescmotor[id].argum.position / 10.0f;
        } else if (packet_id == CAN_PACKET_STATUS_4) {
            index += 6;
            Vescmotor[id].valNow.angle = buffer_16_to_float(Rx_data, 5e1f, &index);
        } else {
            return;
        }

        Vescmotor[id].Begin = true;
        Vescmotor[id].argum.lastRxTime = 0;
    }
}

void VescPosition_Mode(uint8_t controlID, float position)
{
    int32_t send_index = 0;

    if (CAN_Queue_IfFull(Vesc_queue)) {
        return;
    }

    buffer_append_int32(Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].Data,
                        (int32_t)(position * 1e6f), &send_index);
    Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].ID = 0xf0000000U | controlID |
                                                       ((uint32_t)CAN_PACKET_SET_POS << 8U);
    Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].IDE = FDCAN_EXTENDED_ID;
    Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].DLC = (uint8_t)send_index;
    Vesc_queue->Rear = (uint8_t)(((uint16_t)Vesc_queue->Rear + 1U) % FDCAN_QUEUESIZE);
}

void VescRPM_Mode(uint8_t controlID, float speed)
{
    int32_t send_index = 0;

    if (CAN_Queue_IfFull(Vesc_queue)) {
        return;
    }

    buffer_append_int32(Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].Data,
                        (int32_t)speed, &send_index);
    Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].ID = 0xf0000000U | controlID |
                                                       ((uint32_t)CAN_PACKET_SET_RPM << 8U);
    Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].IDE = FDCAN_EXTENDED_ID;
    Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].DLC = (uint8_t)send_index;
    Vesc_queue->Rear = (uint8_t)(((uint16_t)Vesc_queue->Rear + 1U) % FDCAN_QUEUESIZE);
}

void VescCurrent_Mode(uint8_t controlID, float current)
{
    int32_t send_index = 0;

    if (CAN_Queue_IfFull(Vesc_queue)) {
        return;
    }

    buffer_append_int32(Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].Data,
                        (int32_t)(current * 1e3f), &send_index);
    Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].ID = 0xf0000000U | controlID |
                                                       ((uint32_t)CAN_PACKET_SET_CURRENT << 8U);
    Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].IDE = FDCAN_EXTENDED_ID;
    Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].DLC = (uint8_t)send_index;
    Vesc_queue->Rear = (uint8_t)(((uint16_t)Vesc_queue->Rear + 1U) % FDCAN_QUEUESIZE);
}

void VescDuty_Mode(uint8_t controlID, float duty)
{
    int32_t send_index = 0;

    if (CAN_Queue_IfFull(Vesc_queue)) {
        return;
    }

    buffer_append_int32(Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].Data,
                        (int32_t)(duty * 100000.0f), &send_index);
    Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].ID = 0xf0000000U | controlID |
                                                       ((uint32_t)CAN_PACKET_SET_DUTY << 8U);
    Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].IDE = FDCAN_EXTENDED_ID;
    Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].DLC = (uint8_t)send_index;
    Vesc_queue->Rear = (uint8_t)(((uint16_t)Vesc_queue->Rear + 1U) % FDCAN_QUEUESIZE);
}

void VescBrake_Mode(uint8_t controlID, float handbrake)
{
    int32_t send_index = 0;

    if (CAN_Queue_IfFull(Vesc_queue)) {
        return;
    }

    buffer_append_int32(Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].Data,
                        (int32_t)(handbrake * 1e3f), &send_index);
    Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].ID = 0xf0000000U | controlID |
                                                       ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8U);
    Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].IDE = FDCAN_EXTENDED_ID;
    Vesc_queue->FDCAN_DataSend[Vesc_queue->Rear].DLC = (uint8_t)send_index;
    Vesc_queue->Rear = (uint8_t)(((uint16_t)Vesc_queue->Rear + 1U) % FDCAN_QUEUESIZE);
}

void VescTimeOut(uint8_t id)
{
    if ((id >= USE_VESCNUM) || !Vescmotor[id].Enable) {
        return;
    }

    if (Vescmotor[id].argum.lastRxTime++ > 1000U) {
        if (Vescmotor[id].argum.TimeoutTick++ > 50U) {
            Vescmotor[id].statusflag.timeoutflag = true;
        } else {
            Vescmotor[id].statusflag.timeoutflag = false;
        }
    }
}

void VescStuck_Check(uint8_t id)
{
    if (id >= USE_VESCNUM) {
        return;
    }

    if (Vescmotor[id].valNow.current > 45.0f && Vescmotor[id].Enable) {
        if (Vescmotor[id].argum.stuckCount++ > 3000U) {
            Vescmotor[id].statusflag.stuckflag = true;
            if (Vescmotor[id].limit.releaseWhenStuck) {
                Vescmotor[id].Enable = false;
            }
        } else {
            Vescmotor[id].statusflag.timeoutflag = false;
        }
    }
}

void VescFunc(void)
{
    for (uint32_t i = 0; i < USE_VESCNUM; i++) {
        if (Vescmotor[i].Enable) {
            if (Vescmotor[i].limit.timeoutCheck) {
                VescTimeOut((uint8_t)i);
            }
            if (Vescmotor[i].limit.StuckCheck) {
                VescStuck_Check((uint8_t)i);
            }
            if (Vescmotor[i].Begin) {
                switch (Vescmotor[i].mode) {
                case VESC_RPM:
                    VescRPM_Mode((uint8_t)(i + 1U),
                                 Vescmotor[i].valSet.speed * (float)Vescmotor[i].PolePairs);
                    break;
                case VESC_POSITION:
                    VescPosition_Mode((uint8_t)(i + 1U), Vescmotor[i].valSet.angle);
                    break;
                case VESC_CURRENT:
                    VescCurrent_Mode((uint8_t)(i + 1U), Vescmotor[i].valSet.current);
                    break;
                case VESC_DUTY:
                    VescDuty_Mode((uint8_t)(i + 1U), Vescmotor[i].valSet.duty);
                    break;
                case VESC_HANDBRAKE:
                    VescBrake_Mode((uint8_t)(i + 1U), (float)Vescmotor[i].valSet.handbrakeCurrent);
                    break;
                default:
                    break;
                }
            } else {
                VescBrake_Mode((uint8_t)(i + 1U), (float)Vescmotor[i].valSet.handbrakeCurrent);
            }
        } else {
            VescCurrent_Mode((uint8_t)(i + 1U), 0.0f);
        }
    }
}
#endif /* USE_VESC */
