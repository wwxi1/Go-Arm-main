/**
 * @file    bsp_buzzer.h
 * @brief   Buzzer BSP interface.
 */
#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BspBuzzer_Init(void);
void BspBuzzer_On(void);
void BspBuzzer_Off(void);
void BspBuzzer_Toggle(void);

/** Blocking startup beep: on-off-on-off, 50 ms each. */
void BspBuzzer_StartupBeep(void);

/** Blocking alarm: beep times cycles of on_ms/off_ms. */
void BspBuzzer_Alarm(uint8_t times, uint32_t on_ms, uint32_t off_ms);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BUZZER_H */
