/**
 * @file    bsp_led.c
 * @brief   LED BSP implementation (PB3..PB6 on 27RC_Proj_Template).
 */
#include "bsp_led.h"
#include "board_config.h"
#include "main.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} BspLed_Device_t;

static BspLed_Device_t s_led_device[BOARD_LED_NUM] = {
    { BOARD_LED1_PORT, BOARD_LED1_PIN },
    { BOARD_LED2_PORT, BOARD_LED2_PIN },
    { BOARD_LED3_PORT, BOARD_LED3_PIN },
    { BOARD_LED4_PORT, BOARD_LED4_PIN },
};

void BspLed_Init(void)
{
    BspLed_AllOff();
}

void BspLed_On(uint8_t index)
{
    if (index < BOARD_LED_NUM) {
        HAL_GPIO_WritePin(s_led_device[index].port, s_led_device[index].pin, GPIO_PIN_SET);
    }
}

void BspLed_Off(uint8_t index)
{
    if (index < BOARD_LED_NUM) {
        HAL_GPIO_WritePin(s_led_device[index].port, s_led_device[index].pin, GPIO_PIN_RESET);
    }
}

void BspLed_Toggle(uint8_t index)
{
    if (index < BOARD_LED_NUM) {
        HAL_GPIO_TogglePin(s_led_device[index].port, s_led_device[index].pin);
    }
}

void BspLed_Set(uint8_t index, uint8_t on)
{
    if (on != 0U) {
        BspLed_On(index);
    } else {
        BspLed_Off(index);
    }
}

void BspLed_AllOff(void)
{
    for (uint8_t i = 0; i < BOARD_LED_NUM; i++) {
        BspLed_Off(i);
    }
}

void BspLed_Flow(uint32_t period_ms)
{
    for (uint8_t i = 0; i < BOARD_LED_NUM; i++) {
        BspLed_On(i);
        HAL_Delay(period_ms);
        BspLed_Off(i);
        HAL_Delay(period_ms);
    }
}
