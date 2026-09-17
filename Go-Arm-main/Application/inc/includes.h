/**
 * @file    includes.h
 * @brief   模板聚合头:标准库 + 常用宏。
 *
 * 用户代码文件习惯上先 include 这个头,再 include 自己需要的模块头。
 * 电机开关 USE_DJ / USE_VESC / USE_ZMDR / USE_UNITREE 在 Config/motor_config.h。
 */
#ifndef __INCLUDES_H__
#define __INCLUDES_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "mathFunc.h"

/* RAM 段放置宏,配合 MDK-ARM/stm32h723zxx_flash.sct 使用 */
#define __RAM_D1_ __attribute__((section(".RAM_D1")))
#define __RAM_D2_ __attribute__((section(".RAM_D2")))
#define __RAM_D3_ __attribute__((section(".RAM_D3")))

#define ALIGN_32B __attribute__((aligned(32)))

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDES_H__ */
