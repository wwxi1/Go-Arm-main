/**
 * @file    bsp_led.h
 * @brief   LED BSP interface.
 */
#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BspLed_Init(void);
void BspLed_On(uint8_t index);
void BspLed_Off(uint8_t index);
void BspLed_Toggle(uint8_t index);
void BspLed_Set(uint8_t index, uint8_t on);
void BspLed_AllOff(void);
void BspLed_Flow(uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LED_H */
