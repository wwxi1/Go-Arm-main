/**
 * @file    board_config.h
 * @brief   Board-level hardware mapping for 27RC_Proj_Template.
 *
 * All pin/port/peripheral assignments used by BSP and Driver layers are
 * collected here.  If the PCB changes, only this file and CubeMX need to be
 * updated.
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "main.h"
#include "tim.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ------------------------------------------------------------------ */
/* LED                                                                  */
/* ------------------------------------------------------------------ */
#define BOARD_LED_NUM 4U

#define BOARD_LED1_PORT GPIOB
#define BOARD_LED1_PIN GPIO_PIN_3
#define BOARD_LED2_PORT GPIOB
#define BOARD_LED2_PIN GPIO_PIN_4
#define BOARD_LED3_PORT GPIOB
#define BOARD_LED3_PIN GPIO_PIN_5
#define BOARD_LED4_PORT GPIOB
#define BOARD_LED4_PIN GPIO_PIN_6

/* ------------------------------------------------------------------ */
/* Buzzer                                                               */
/* ------------------------------------------------------------------ */
#define BOARD_BUZZER_PORT GPIOB
#define BOARD_BUZZER_PIN GPIO_PIN_0
#define BOARD_BUZZER_ACTIVE_LEVEL GPIO_PIN_SET

/* ------------------------------------------------------------------ */
/* Timer service                                                        */
/* ------------------------------------------------------------------ */
#define BOARD_CONTROL_TIMER (&htim2) /* 1 kHz, used for control task */
#define BOARD_SLOW_TIMER (&htim3)    /* spare, configured by CubeMX  */

/* ------------------------------------------------------------------ */
/* UART service                                                        */
/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/* Solenoid valve service                                              */
/* ------------------------------------------------------------------ */
#define BOARD_SOLENOID_PORT1 GPIOA
#define BOARD_SOLENOID_SDA GPIO_PIN_9
#define BOARD_SOLENOID_CLK GPIO_PIN_10

#define BOARD_SOLENOID_PORT2 GPIOD
#define BOARD_SOLENOID_SDA2 GPIO_PIN_5
#define BOARD_SOLENOID_CLK2 GPIO_PIN_6

#define BOARD_SOLENOID_PORT3 GPIOD
#define BOARD_SOLENOID_SDA3 GPIO_PIN_8
#define BOARD_SOLENOID_CLK3 GPIO_PIN_9

#ifdef __cplusplus
}
#endif

#endif /* BOARD_CONFIG_H */
