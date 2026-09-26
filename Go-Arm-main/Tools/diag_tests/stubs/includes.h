#ifndef DIAG_TEST_INCLUDES_H
#define DIAG_TEST_INCLUDES_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#define __RAM_D1_
#define ALIGN_32B __attribute__((aligned(32)))
typedef struct { uint32_t Instance; } UART_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
extern UART_HandleTypeDef huart9;
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
void SCB_CleanDCache_by_Addr(uint32_t *p, int32_t size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, uint8_t *p, uint16_t size);
#endif
