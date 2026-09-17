/**
 * @file    bluetooth_serv.c
 * @brief   蓝牙业务消息层实现(接口见 bluetooth_serv.h)。
 *
 * 数据流:
 *   发送状态:BT_Status_t → BT_Status_Pack → Bluetooth_Send() → TX Queue → 帧层
 *   接收命令:Bluetooth_Receive() → frame.payload → BT_Cmd_Parse → BT_Cmd_t
 *
 * 业务层(用户)要写的就是下面 Pack/Parse 里的字段映射,
 * 其余是固定样板。字段类型/数量改 bluetooth_protocol.h 顶部的宏即可。
 */
#include "bluetooth_serv.h"

#include "bluetooth.h"          /* Bluetooth_Send / Bluetooth_Receive */
#include "bluetooth_protocol.h" /* BT_Proto_* 访问函数 + BT_PROTO_PAYLOAD_SIZE */

/* ================================================================== */
/* 用户实现区:按上位机协议逐字段解析/打包(业务层唯一要写的地方)。         */
/* 索引顺序必须与 bluetooth_protocol.h 的 BT_PROTO_*_NUM 一致。         */
/* ================================================================== */
/* 接收:命令帧解析(RX) */
static void BT_Cmd_Parse(const uint8_t *payload, BT_Cmd_t *cmd)
{
    uint8_t bool_cnt = 0U;
    uint8_t u16_cnt  = 0U;
    uint8_t f32_cnt  = 0U;
    cmd->enable    = BT_Proto_ReadBool(payload, bool_cnt++);
    cmd->auto_mode = BT_Proto_ReadBool(payload, bool_cnt++);
    cmd->calib     = BT_Proto_ReadBool(payload, bool_cnt++);
    cmd->stop      = BT_Proto_ReadBool(payload, bool_cnt++);
    cmd->slow      = BT_Proto_ReadBool(payload, bool_cnt++);
    cmd->fast      = BT_Proto_ReadBool(payload, bool_cnt++);
    cmd->reset     = BT_Proto_ReadBool(payload, bool_cnt++);
    cmd->speed     = BT_Proto_ReadU16(payload, u16_cnt++);
    cmd->angle     = BT_Proto_ReadU16(payload, u16_cnt++);
    cmd->vx        = BT_Proto_ReadFloat(payload, f32_cnt++);
}

/* 发送:状态帧打包(TX) */
static void BT_Status_Pack(uint8_t *payload, const BT_Status_t *status)
{
    uint8_t bool_cnt = 0U;
    uint8_t u16_cnt  = 0U;
    uint8_t f32_cnt  = 0U;
    BT_Proto_WriteBool(payload, bool_cnt++, status->online);
    BT_Proto_WriteBool(payload, bool_cnt++, status->ready);
    BT_Proto_WriteBool(payload, bool_cnt++, status->fault);
    BT_Proto_WriteBool(payload, bool_cnt++, status->limited);
    BT_Proto_WriteBool(payload, bool_cnt++, status->stall);
    BT_Proto_WriteBool(payload, bool_cnt++, status->brake);
    BT_Proto_WriteBool(payload, bool_cnt++, status->estop);
    BT_Proto_WriteU16(payload, u16_cnt++, status->rpm);
    BT_Proto_WriteU16(payload, u16_cnt++, status->vbus);
    BT_Proto_WriteFloat(payload, f32_cnt++, status->current);
}

/* ================================================================== */
/* 固定样板:框架层调用入口,一般不用改                                   */
/* ================================================================== */
bool BT_Serv_Status_Send(const BT_Status_t *status)
{
    uint8_t payload[BT_PROTO_PAYLOAD_SIZE];

    BT_Status_Pack(payload, status);
    return Bluetooth_Send(payload, BT_PROTO_PAYLOAD_SIZE);
}

bool BT_Serv_Cmd_Receive(BT_Cmd_t *cmd, uint32_t timeout_ms)
{
    BluetoothFrame_t frame;

    if (Bluetooth_Receive(&frame, timeout_ms) != true)
    {
        return false;
    }

    BT_Cmd_Parse(frame.payload, cmd);
    return true;
}
