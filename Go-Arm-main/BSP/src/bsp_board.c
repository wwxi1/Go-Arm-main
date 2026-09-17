/**
 * @file    bsp_board.c
 * @brief   板级 BSP 初始化。
 */
#include "bsp_board.h"
#include "bsp_led.h"
#include "bsp_buzzer.h"

void Bsp_BoardInit(void)
{
    BspLed_Init();
    BspBuzzer_Init();
}
