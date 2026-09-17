/**
 * @file    tasks.h
 * @brief   任务函数声明。
 *
 * 任务本体在 CubeMX(.ioc → Core/Src/freertos.c)里创建,任务函数在
 * freertos.c 里是 __weak 占位,这里声明的强定义会覆盖它们。
 */
#ifndef TASKS_H
#define TASKS_H

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 强定义覆盖 CubeMX 的弱函数 Alarm_Task() */
void Alarm_Task(void *argument);
void VOFA_SendTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASKS_H */
