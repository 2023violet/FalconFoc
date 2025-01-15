/********************************************************************************
 * @file        : log.h
 * @author      : Zhang jia ming
 * @brief       : 该文件实现了日志记录功能，包括日志初始化、不同级别日志输出等功能。
 * @version     : V1.0
 * @date        : 2025 - 1 - 11
 *
 * @details:
 *  - 本文件定义了日志记录相关的函数，通过 `LogInit` 函数初始化日志系统，使其与特定的串口实例相关联。
 *  - `LOG_PROTO` 函数是核心的日志输出函数，它根据传入的日志级别（如 `LOG_DEBUG`、`LOG_INFO`、`LOG_WARNING`、`LOG_ERROR`），
 *    格式化日志信息，并通过串口以DMA模式发送出去。
 *  - 同时，通过宏定义 `LOGDEBUG`、`LOGINFO`、`LOGWARNING` 和 `LOGERROR` 为不同级别的日志输出提供了便捷的调用方式。
 *
 * @note:
 *  - 在使用本日志模块前，需确保相关串口驱动及配置已正确初始化，特别是与 `log_usart_instance` 相关的串口配置。
 *  - 日志缓冲区大小由 `LOG_BUFFER_SIZE` 宏定义，当前设置为1024字节。若日志信息过长，可能会导致数据截断，使用时需注意。
 *  - 对于 `USARTSend` 函数，假设其已正确实现且在DMA模式下能可靠地发送数据。若该函数返回错误，当前代码仅预留了错误处理位置，
 *    实际应用中需根据需求完善错误处理逻辑。
 *
 * @history:
 *  V1.0:
 *    - 修改注释，完善功能。对函数功能、使用注意事项等进行了详细说明，提高代码的可读性和可维护性。
 *
 * Copyright (c) 2025 Zhang jia ming. All rights reserved.
 ********************************************************************************/
#ifndef BSP_LOG_H
#define BSP_LOG_H

#include <stdarg.h>
#include <stdbool.h>
#include "bsp_usart.h"

#define LOG_BUFFER_SIZE 256 // 最大缓冲区

/**
 * @brief 日志级别
 */
typedef enum
{
  LOG_DEBUG = 0,
  LOG_INFO,
  LOG_WARNING,
  LOG_ERROR,
} LOG_LEVEL;

/**
 * @brief 日志初始化
 */
void LogInit(UART_HandleTypeDef *log_config);

/**
 * @brief 日志输出函数
 */
void LOG_PROTO(const char *fmt, LOG_LEVEL level, const char *file,
               int line, const char *func, ...);

// debug level
#define LOGDEBUG(fmt, ...) LOG_PROTO(fmt, LOG_DEBUG, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
// information level
#define LOGINFO(fmt, ...) LOG_PROTO(fmt, LOG_INFO, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
// warning level
#define LOGWARNING(fmt, ...) LOG_PROTO(fmt, LOG_WARNING, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
// error level
#define LOGERROR(fmt, ...) LOG_PROTO(fmt, LOG_ERROR, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)

#endif
