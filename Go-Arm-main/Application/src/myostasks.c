/**
 * @file    tasks.c
 * @brief   任务函数实现(强定义覆盖 freertos.c 里的 __weak 占位)。
 *
 * 新建任务的标准流程:
 *   1. CubeMX 里创建任务(生成 freertos.c 的任务创建代码和 __weak 任务函数);
 *   2. 在 CubeMX 任务函数的 USER CODE 里写业务,或者在本文件写强定义覆盖。
 */
#include "includes.h"
#include "myostasks.h"
#include "app_config.h"
#include "board_config.h"

#include "bsp_led.h"
#include "vofa.h"
#include "bluetooth_serv.h" /* 含 BT_SERV_ENABLE 开关,默认 0 */
#include "UnitreeMotor.h"

/* CubeMX 默认任务 LEDTask:四灯流水 */
void Alarm_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        for (uint8_t i = 0; i < BOARD_LED_NUM; i++)
        {
            BspLed_On(i);
            osDelay(APP_LED_TASK_PERIOD_MS);
            BspLed_Off(i);
            osDelay(APP_LED_TASK_PERIOD_MS);
        }
    }
}

void VOFA_SendTask(void *argument)
{
    (void)argument;

    for (;;)
    {   
        static uint8_t Func_cnt = 0;
        Func_cnt++;
        VOFA_Channel_Update(0, VOFA_TYPE_FLOAT, (void *)&(Unitree_motors[0].data.position));
#if BT_SERV_ENABLE
        /* 蓝牙业务联调(默认关,BT_SERV_ENABLE=1 开启):
           周期发一条"状态",并把收到的"命令"解析出来。
           链路:Status → Pack → Frame → TX Queue;
                 RX Queue → Frame → Cmd_Parse → BT_Cmd_t */
        static BT_Status_t status = {.online = true, .ready = true,
                                     .rpm = 1000U, .vbus = 500U, .current = 1.5f};
        (void)BT_Serv_Status_Send(&status);
        BT_Cmd_t cmd;
        (void)BT_Serv_Cmd_Receive(&cmd, 0);
#endif
        #if APP_VOFA_ENABLE
        VOFA_Update();
        osDelay(APP_VOFA_TASK_PERIOD_MS);
        #else
        osDelay(99999);
        #endif
    }
}