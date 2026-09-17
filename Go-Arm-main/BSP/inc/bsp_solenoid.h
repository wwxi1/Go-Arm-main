/**
 * @file bsp_solenoid.h
 * @date 2026-08-21
 */

//  配置：GPIO输出 利用队里F4板上的串口的4Pin口
// （RX为SCL  TX为SDA）
// 一个电磁阀板控制4个电磁阀，需要4位
// （分高四位或者低四位）
#ifndef __BSP_SOLENOID_H
#define __BSP_SOLENOID_H

#include "main.h"
#include "board_config.h"


/*
 * 电磁阀控制器状态结构体：
 * gpio_port   -> 当前通道使用的GPIO端口
 * gpio_pin_sda-> 串行数据线，负责发送控制位
 * gpio_pin_clk-> 时钟线，每发一位数据就产生一个时钟脉冲
 * data_prve   -> 上一次写入的寄存器值，用于做去抖/状态缓存和读取回显
 */
typedef struct Solenoid_t
{
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin_sda;
    uint16_t gpio_pin_clk;
    uint8_t data_prve;
} Solenoid_t;

/*
 * 3个独立的电磁阀控制通道
 * 1/2/3 对应不同GPIO组，每个通道可控制4个电磁阀
 */
typedef enum
{
    Solenoid_Channel1 = 1,
    Solenoid_Channel2,
    Solenoid_Channel3,
} Solenoid_Channel;

/*
 * 初始化指定通道的GPIO、状态变量和默认输出
 */
void SolenoidValve_Init(Solenoid_Channel usart_channel);

/*
 * 向指定通道发送 4 位控制命令，低 4 位对应 4 个电磁阀状态
 * 命令格式：bit3 bit2 bit1 bit0 -> valve4 valve3 valve2 valve1
 */
void SolenoidValve_On(Solenoid_Channel usart_channel, uint8_t cmd);

/*
 * 读取指定通道上一次写入的命令值，便于状态回读或调试
 */
uint8_t SolenoidValve_Read(Solenoid_Channel usart_channel);
#endif /* __BSP_SOLENOID_H */