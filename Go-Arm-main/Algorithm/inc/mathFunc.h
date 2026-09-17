/**
 * @file    mathFunc.h
 * @brief   Small math/encode-decode helpers ported from the R2 chassis project.
 *
 * The original file depended on arm_math.h only for the PI definition.
 * It is removed here so the Algorithm layer stays pure libc.
 */
#ifndef MATHFUNC_H
#define MATHFUNC_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ABS(x) (((x) > 0) ? (x) : (-(x)))
#define GetSign(x) (((x) > 0) ? 1 : -1)

#define PI 3.14159265358979f
#define Pi 3.1415926f

    typedef int8_t s8;
    typedef uint8_t u8;
    typedef int16_t s16;
    typedef uint16_t u16;
    typedef int32_t s32;
    typedef uint32_t u32;

#define EncodeS32Data(f, buff) \
    {                          \
        *(int32_t *)buff = *f; \
    }
#define DecodeS32Data(f, buff) \
    {                          \
        *f = *(int32_t *)buff; \
    }
#define EncodeS16Data(f, buff) \
    {                          \
        *(s16 *)buff = *f;     \
    }
#define DecodeS16Data(f, buff) \
    {                          \
        *f = *(s16 *)buff;     \
    }
#define EncodeU16Data(f, buff) \
    {                          \
        *(u16 *)buff = *f;     \
    }
#define DecodeU16Data(f, buff) \
    {                          \
        *f = *(u16 *)buff;     \
    }

    /* 短换算函数放头文件 static inline:各编译单元直接内联,无调用开销 */
    static inline float N2DEG(float n)
    {
        return n * 360.0f;
    }

    static inline float DEG2N(float deg)
    {
        return deg / 360.0f;
    }

    static inline float DEG2RAD(float angle)
    {
        return angle / 180.0f * PI;
    }

    static inline float RAD2DEG(float angle)
    {
        return angle / PI * 180.0f;
    }

    void ChangeDataByte(uint8_t *p1, uint8_t *p2);
    float buffer_32_to_float(const uint8_t *buffer, float scale, int32_t *index);
    float buffer_16_to_float(const uint8_t *buffer, float scale, int32_t *index);
    int32_t get_s32_from_buffer(const uint8_t *buffer, int32_t *index);
    int16_t get_s16_from_buffer(const uint8_t *buffer, int32_t *index);
    void buffer_append_int32(uint8_t *buffer, int32_t source, int32_t *index);
    void buffer_append_int16(uint8_t *buffer, int16_t source, int32_t *index);
    double cvtFloat2Double(float n1, float n2);

    float uint2float(int x_int, float x_min, float x_max, int bits);
    u16 float2uint(float x, float x_min, float x_max, uint8_t bits);
    float Lerp(float start, float end, float t);
    void Rotate(float *x, float *y, float x0, float y0, float a);
    static inline float Clamp(float value, float min, float max)
    {
        if (value < min)
            return min;
        else if (value > max)
            return max;
        else
            return value;
    }
    static inline float ClampPeak(float value, float abs_max)
    {
        if (value > abs_max)
            return abs_max;
        else if (value < -abs_max)
            return -abs_max;
        else
            return value;
    }

    s16 MSG_Byte2Int16(uint8_t *buff, uint8_t i);
    int MSG_Byte2Int32(uint8_t *buff, uint8_t i);
    void MSG_Int162Byte(s16 data, uint8_t *buff, uint8_t i);
    void MSG_Int322Byte(int data, uint8_t *buff, uint8_t i);
    float Quintic_Traj(float time,float total_time);

#ifdef __cplusplus
}
#endif

#endif /* MATHFUNC_H */
