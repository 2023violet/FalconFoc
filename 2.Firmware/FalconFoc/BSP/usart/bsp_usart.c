/********************************************************************************
 * @file        : bsp_usart.c
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

#include "bsp_usart.h"
#include "bsp_log.h"
#include <stdlib.h>
#include <string.h>

/* usart service instance, modules' info would be recoreded here using USARTRegister() */
/* usart服务实例,所有注册了usart的模块信息会被保存在这里 */
static uint8_t idx;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

/**
 * @brief          启动串口服务,会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 * @todo           串口服务会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *                 可能还要将此函数修改为extern,使得module可以控制串口的启停
 *
 * @param[in]      _instance: 模块拥有的串口实例
 * @retval         note
 */
void USARTServiceInit(USARTInstance *_instance)
{
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size);
    // 关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
    // 这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
    // 我们只希望处理第一种和第三种情况,因此直接关闭DMA半传输中断
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}

/**
 * @brief          注册USART实例
 * @param[in]      USART_config: USART配置结构体
 * @retval         USART实例指针
 */
USARTInstance *USARTRegister(USART_Init_Config_s *USART_config)
{
    if (idx >= DEVICE_USART_CNT) // 超过最大实例数
    {
        while (1)
            LOGERROR("[bsp_usart] USART exceed max instance count!");
    }

    for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
    {
        if (usart_instance[i]->usart_handle == USART_config->usart_handle)
        {
            while (1)
                LOGERROR("[bsp_usart] USART instance already registered!");
        }
    }

    USARTInstance *usart   = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(usart, 0, sizeof(USARTInstance));

    usart->usart_handle    = USART_config->usart_handle;
    usart->recv_buff_size  = USART_config->recv_buff_size;
    usart->module_callback = USART_config->module_callback;

    usart_instance[idx++] = usart;
    USARTServiceInit(usart);
    return usart;
}

/**
 * @brief          USART发送数据
 * @attention      在短时间内连续调用此接口,若采用IT/DMA会导致上一次的发送未完成而新的发送取消.
 *                 因此,若希望连续使用DMA/IT进行发送,请配合USARTIsReady()使用,或自行为你的module实现一个发送队列和任务.
 * @todo           是否考虑为USARTInstance增加发送队列以进行连续发送?
 *
 * @param[in]      _instance: 模块拥有的串口实例
 * @param[in]      send_buf:  待发送数据缓冲区
 * @param[in]      send_size: 待发送数据长度
 * @param[in]      mode:     传输模式
 * @retval         USART实例指针
 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        break;
    case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;
    default:
        //        while (1)
        //            ; // illegal mode! check your code context! 检查定义instance的代码上下文,可能出现指针越界
        break;
    }
}

/**
 * @brief          判断串口是否准备好,用于连续或异步的IT/DMA发送
 * @param[in]      _instance: 要判断的串口实例
 * @retval         ready:1-准备好,0-未准备好
 */
uint8_t USARTIsReady(USARTInstance *_instance)
{
    if (_instance->usart_handle->gState | HAL_UART_STATE_BUSY_TX)
        return 0;
    else
        return 1;
}

/**
 * @brief          USART中断回调函数（每次dma/idle中断发生时，都会调用此函数.对于每个uart实例会调用对应的回调进行进一步的处理）
 * @attention      通过__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
 *                 这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
 *                 我们只希望处理，因此直接关闭DMA半传输中断第一种和第三种情况
 *
 * @param[in]      huart: 发生中断的串口
 * @param[in]      Size: 此次接收到的总数居量,暂时没用
 * @retval         note
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i)
    { // find the instance which is being handled
        if (huart == usart_instance[i]->usart_handle)
        { // call the callback function if it is not NULL
            if (usart_instance[i]->module_callback != NULL)
            {
                usart_instance[i]->module_callback();
                memset(usart_instance[i]->recv_buff, 0, Size); // 接收结束后清空buffer,对于变长数据是必要的
            }
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            return; // break the loop
        }
    }
}

/**
 * @brief          USART错误回调函数（当串口发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收）
 * @attention      最常见的错误:奇偶校验/溢出/帧错误
 *
 * @param[in]      huart: 发生错误的串口
 * @retval         note
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            LOGWARNING("[bsp_usart] USART error callback triggered, instance idx [%d]", i);
            return;
        }
    }
}
