/**
 * @file    bluetooth_protocol.h
 * @brief   蓝牙 Payload 协议层:宏驱动的定长编解码(单帧类型场景)。
 *
 * 设计前提:实际工程只会用**一种**蓝牙帧,字段类型与数量固定,
 * 由下面的宏按上位机协议约定填好。本层根据宏推导出定长布局,
 * 提供逐字段的读写访问函数;业务层唯一要做的是写解析函数,
 * 把每个字段赋给业务结构体成员,不碰位移/按位与/字节偏移。
 *
 * 布局(编译期固定,类型按协议顺序分组):
 *   [0, BOOL_BYTES)         : bool 打包区(LSB 在前,第 1 个 bool 在 bit0)
 *   [BOOL_BYTES, +U8_NUM)   : u8
 *   [.., +2*U16_NUM)        : u16(小端)
 *   [.., +4*U32_NUM)        : u32(小端)
 *   [.., +4*FLOAT_NUM)      : float(小端)
 *
 * 某类型数量为 0 时,该类型全部读写函数被 #if 取消编译,不占代码。
 * 调用方保证:payload 长度 >= BT_PROTO_PAYLOAD_SIZE,idx 在数量范围内。
 */
#ifndef BLUETOOTH_PROTOCOL_H
#define BLUETOOTH_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h> /* memcpy(仅 float 位拷贝) */

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* 协议定义:按上位机约定填写每种字段的数量(0 = 该类型不出现)。            */
/* 可在包含本头文件之前自行 #define 覆盖(如放 Config/app_config.h)。     */
/* ------------------------------------------------------------------ */
#ifndef BT_PROTO_BOOL_NUM
#define BT_PROTO_BOOL_NUM   7U   /* bool 数量 */
#endif
#ifndef BT_PROTO_U8_NUM
#define BT_PROTO_U8_NUM     0U   /* u8 数量 */
#endif
#ifndef BT_PROTO_U16_NUM
#define BT_PROTO_U16_NUM    2U   /* u16 数量 */
#endif
#ifndef BT_PROTO_U32_NUM
#define BT_PROTO_U32_NUM    0U   /* u32 数量 */
#endif
#ifndef BT_PROTO_FLOAT_NUM
#define BT_PROTO_FLOAT_NUM  1U   /* float 数量 */
#endif

/* ------------------------------------------------------------------ */
/* 由数量推导的定长布局(编译期常量,勿手动改)                             */
/* ------------------------------------------------------------------ */
#define BT_PROTO_BOOL_BYTES   ((BT_PROTO_BOOL_NUM + 7U) / 8U)   /* bool 打包区字节数 */
#define BT_PROTO_U8_OFFSET    (BT_PROTO_BOOL_BYTES)
#define BT_PROTO_U16_OFFSET   (BT_PROTO_U8_OFFSET + BT_PROTO_U8_NUM)
#define BT_PROTO_U32_OFFSET   (BT_PROTO_U16_OFFSET + 2U * BT_PROTO_U16_NUM)
#define BT_PROTO_FLOAT_OFFSET (BT_PROTO_U32_OFFSET + 4U * BT_PROTO_U32_NUM)

/* payload 总字节数(定长) */
#define BT_PROTO_PAYLOAD_SIZE (BT_PROTO_FLOAT_OFFSET + 4U * BT_PROTO_FLOAT_NUM)

/* ------------------------------------------------------------------ */
/* 字段访问函数:每个类型独立 #if,数量为 0 时整段不编译                    */
/* ------------------------------------------------------------------ */

#if BT_PROTO_BOOL_NUM > 0U
/* 第 idx 个 bool(0 起),LSB 在前:第 1 个 = 字节 bit0,第 8 个 = bit7 */
static inline bool BT_Proto_ReadBool(const uint8_t *payload, uint8_t idx)
{
    return ((payload[idx >> 3] >> (idx & 7U)) & 1U) != 0U;
}

static inline void BT_Proto_WriteBool(uint8_t *payload, uint8_t idx, bool value)
{
    if (value)
    {
        payload[idx >> 3] |= (uint8_t)(1U << (idx & 7U));
    }
    else
    {
        payload[idx >> 3] &= (uint8_t)~(1U << (idx & 7U));
    }
}
#endif /* BT_PROTO_BOOL_NUM > 0 */

#if BT_PROTO_U8_NUM > 0U
static inline uint8_t BT_Proto_ReadU8(const uint8_t *payload, uint8_t idx)
{
    return payload[BT_PROTO_U8_OFFSET + idx];
}

static inline void BT_Proto_WriteU8(uint8_t *payload, uint8_t idx, uint8_t value)
{
    payload[BT_PROTO_U8_OFFSET + idx] = value;
}
#endif /* BT_PROTO_U8_NUM > 0 */

#if BT_PROTO_U16_NUM > 0U
/* 第 idx 个 u16(0 起),小端 */
static inline uint16_t BT_Proto_ReadU16(const uint8_t *payload, uint8_t idx)
{
    uint16_t off = (uint16_t)(BT_PROTO_U16_OFFSET + 2U * idx);
    return (uint16_t)((uint16_t)payload[off] | ((uint16_t)payload[off + 1U] << 8));
}

static inline void BT_Proto_WriteU16(uint8_t *payload, uint8_t idx, uint16_t value)
{
    uint16_t off = (uint16_t)(BT_PROTO_U16_OFFSET + 2U * idx);
    payload[off]       = (uint8_t)(value & 0xFFU);
    payload[off + 1U]  = (uint8_t)(value >> 8);
}
#endif /* BT_PROTO_U16_NUM > 0 */

#if BT_PROTO_U32_NUM > 0U
static inline uint32_t BT_Proto_ReadU32(const uint8_t *payload, uint8_t idx)
{
    uint16_t off = (uint16_t)(BT_PROTO_U32_OFFSET + 4U * idx);
    return (uint32_t)payload[off]
         | ((uint32_t)payload[off + 1U] << 8)
         | ((uint32_t)payload[off + 2U] << 16)
         | ((uint32_t)payload[off + 3U] << 24);
}

static inline void BT_Proto_WriteU32(uint8_t *payload, uint8_t idx, uint32_t value)
{
    uint16_t off = (uint16_t)(BT_PROTO_U32_OFFSET + 4U * idx);
    payload[off]       = (uint8_t)(value & 0xFFU);
    payload[off + 1U]  = (uint8_t)((value >> 8) & 0xFFU);
    payload[off + 2U]  = (uint8_t)((value >> 16) & 0xFFU);
    payload[off + 3U]  = (uint8_t)((value >> 24) & 0xFFU);
}
#endif /* BT_PROTO_U32_NUM > 0 */

#if BT_PROTO_FLOAT_NUM > 0U
/* 第 idx 个 float(0 起),IEEE754 单精度,小端 */
static inline float BT_Proto_ReadFloat(const uint8_t *payload, uint8_t idx)
{
    uint16_t off = (uint16_t)(BT_PROTO_FLOAT_OFFSET + 4U * idx);
    uint32_t bits = (uint32_t)payload[off]
                  | ((uint32_t)payload[off + 1U] << 8)
                  | ((uint32_t)payload[off + 2U] << 16)
                  | ((uint32_t)payload[off + 3U] << 24);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static inline void BT_Proto_WriteFloat(uint8_t *payload, uint8_t idx, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    uint16_t off = (uint16_t)(BT_PROTO_FLOAT_OFFSET + 4U * idx);
    payload[off]       = (uint8_t)(bits & 0xFFU);
    payload[off + 1U]  = (uint8_t)((bits >> 8) & 0xFFU);
    payload[off + 2U]  = (uint8_t)((bits >> 16) & 0xFFU);
    payload[off + 3U]  = (uint8_t)((bits >> 24) & 0xFFU);
}
#endif /* BT_PROTO_FLOAT_NUM > 0 */

#ifdef __cplusplus
}
#endif

#endif /* BLUETOOTH_PROTOCOL_H */
