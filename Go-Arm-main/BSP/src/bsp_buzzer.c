/**
 * @file    bsp_buzzer.c
 * @brief   Buzzer BSP implementation.
 */
#include "bsp_buzzer.h"
#include "board_config.h"
#include "main.h"

void BspBuzzer_Init(void)
{
    BspBuzzer_Off();
}

void BspBuzzer_On(void)
{
    HAL_GPIO_WritePin(BOARD_BUZZER_PORT, BOARD_BUZZER_PIN, BOARD_BUZZER_ACTIVE_LEVEL);
}

void BspBuzzer_Off(void)
{
    HAL_GPIO_WritePin(BOARD_BUZZER_PORT, BOARD_BUZZER_PIN,
                      (BOARD_BUZZER_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void BspBuzzer_Toggle(void)
{
    HAL_GPIO_TogglePin(BOARD_BUZZER_PORT, BOARD_BUZZER_PIN);
}

void BspBuzzer_StartupBeep(void)
{
    BspBuzzer_On();
    HAL_Delay(50);
    BspBuzzer_Off();
    HAL_Delay(50);
    BspBuzzer_On();
    HAL_Delay(50);
    BspBuzzer_Off();
    HAL_Delay(50);
}

void BspBuzzer_Alarm(uint8_t times, uint32_t on_ms, uint32_t off_ms)
{
    for (uint8_t i = 0; i < times; i++) {
        BspBuzzer_On();
        HAL_Delay(on_ms);
        BspBuzzer_Off();
        HAL_Delay(off_ms);
    }
}
