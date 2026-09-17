/**
 * @file    IQRhandler.h
 * @brief   中断回调集中声明:定时器调度、UART 接收缓冲区。
 *
 * 回调函数的强定义在 Application/src/IQRhandler.c 里,CubeMX 生成的是
 * __weak 版本,重新生成代码不会冲突。
 */
#ifndef __IQRHANDLER_H__
#define __IQRHANDLER_H__

#include "includes.h"
#include "tim.h"
#include "FD_Canqueue.h"

/* UART 接收缓冲区,放 D2 段(0x30000000,DMA 可达且不经 D-Cache) */
#define UART_RX_BUFFER_SIZE 64U

extern __RAM_D2_ ALIGN_32B uint8_t UART1_RxBuffer[UART_RX_BUFFER_SIZE];
extern __RAM_D2_ ALIGN_32B uint8_t UART3_RxBuffer[UART_RX_BUFFER_SIZE];
extern __RAM_D2_ ALIGN_32B uint8_t UART4_RxBuffer[UART_RX_BUFFER_SIZE];
extern __RAM_D2_ ALIGN_32B uint8_t UART6_RxBuffer[UART_RX_BUFFER_SIZE];
extern __RAM_D2_ ALIGN_32B uint8_t UART9_RxBuffer[UART_RX_BUFFER_SIZE];

/* main.c 的 HAL_TIM_PeriodElapsedCallback 会转发到这里 */
void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* __IQRHANDLER_H__ */
