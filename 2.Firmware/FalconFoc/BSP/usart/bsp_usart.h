/********************************************************************************
 * @file        : bsp_usart.h
 * @author      : Zhang jia ming
 * @brief       : USART外设驱动文件
 * @version     : V1.0
 * @date        : 2024 - 11 - 12
 *
 * @details:
 *  - USART外设驱动文件,提供注册、发送、接收等功能
 *
 * @note:
 *  - 支持 DMA/IT/BLOCKING 三种传输模式
 *
 * @history:
 *  V1.0:
 *    - 修改注释，完善功能
 *
 * Copyright (c) 2024 Zhang jia ming. All rights reserved.
 ********************************************************************************/

#ifndef BSP_USART_H
#define BSP_USART_H

#include <stdint.h>
#include "usart.h"
#include "main.h"

#define DEVICE_USART_CNT 5     // 板子分配串口数
#define USART_RXBUFF_LIMIT 256 // 如果协议需要更大的buff,请修改这里

/**
 * @brief 模块回调函数,用于解析协议
 */
typedef void (*usart_module_callback)();

/**
 * @brief 发送模式枚举
 */
#pragma pack(1)
typedef enum
{
    USART_TRANSFER_NONE = 0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

/**
 * @brief 串口实例结构体
 */
typedef struct
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT]; // 预先定义的最大buff大小,如果太小请修改USART_RXBUFF_LIMIT
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
} USARTInstance;

/**
 * @brief 串口初始化配置结构体
 */
typedef struct
{
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
} USART_Init_Config_s;
#pragma pack()

/**
 * @brief          启动串口服务,会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 * @todo           串口服务会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *                 可能还要将此函数修改为extern,使得module可以控制串口的启停
 *
 * @param[in]      _instance: 模块拥有的串口实例
 * @retval         note
 */
void USARTServiceInit(USARTInstance *_instance);

/**
 * @brief          注册USART实例
 * @param[in]      USART_config: USART配置结构体
 * @retval         USART实例指针
 */
USARTInstance *USARTRegister(USART_Init_Config_s *USART_config);

/**
 * @brief          USART发送数据
 * @attention      在短时间内连续调用此接口,若采用IT/DMA会导致上一次的发送未完成而新的发送取消.
 *                 因此,若希望连续使用DMA/IT进行发送,请配合USARTIsReady()使用,或自行为你的module实现一个发送队列和任务.
 * @todo           是否考虑为USARTInstance增加发送队列以进行连续发送?
 *
 * @param[in]      _instance: 模块拥有的串口实例
 * @param[in]      send_buf:  待发送数据缓冲区
 * @param[in]      send_size: 待发送数据长度
 * @param[in]      mode:      传输模式
 * @retval         USART实例指针
 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode);

/**
 * @brief          判断串口是否准备好,用于连续或异步的IT/DMA发送
 * @param[in]      _instance: 要判断的串口实例
 * @retval         ready:1-准备好,0-未准备好
 */
uint8_t USARTIsReady(USARTInstance *_instance);

#endif // BSP_USART_H
