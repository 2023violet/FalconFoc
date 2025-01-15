/********************************************************************************
 * @file        : daemon.h
 * @author      : Zhang jia ming
 * @brief       : daemon驱动文件
 * @version     : V1.0
 * @date        : 2024 - 11 - 17
 *
 * @details:
 *  - daemon模块用于管理各个模块的状态,包括是否在线,是否需要喂狗
 *
 * @note:
 *  - 支持多实例，每个实例可以有自己的owner_id,reload_count,init_count,callback等参数
 *
 * @history:
 *  V1.0:
 *    - 修改注释，完善功能
 *
 * Copyright (c) 2024 Zhang jia ming. All rights reserved.
 ********************************************************************************/

#ifndef DAEMON_H
#define DAEMON_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define DAEMON_MX_CNT 64

/**
 * @brief 模块离线处理函数指针
 */
typedef void (*offline_callback)(void *);

/**
 * @brief daemon实例结构体
 */
typedef struct daemon_ins
{
    uint16_t reload_count;     // 重载值
    offline_callback callback; // 异常处理函数,当模块发生异常时会被调用
    uint16_t temp_count;       // 当前值,减为零说明模块离线或异常
    void *owner_id;            // daemon实例的地址,初始化的时候填入
} DaemonInstance;

/**
 * @brief daemon初始化配置
 */
typedef struct
{
    uint16_t reload_count;     // 实际上这是app唯一需要设置的值?
    uint16_t init_count;       // 上线等待时间,有些模块需要收到主控的指令才会反馈报文,或pc等需要开机时间
    offline_callback callback; // 异常处理函数,当模块发生异常时会被调用
    void *owner_id;            // id取拥有daemon的实例的地址,如DJIMotorInstance*,cast成void*类型
} Daemon_Init_Config_s;

/**
 * @brief          注册守护实例
 * @param[in]      DAEMON_config: 守护实例配置
 * @retval         守护实例
 */
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *DAEMON_config);

/**
 * @brief          "喂狗"函数
 * @param[in]      instance: 守护实例配置
 * @retval         note
 */
void DaemonReload(DaemonInstance *instance);

/**
 * @brief          确认模块是否离线
 * @param[in]      instance: 守护实例配置
 * @retval         note
 */
uint8_t DaemonIsOnline(DaemonInstance *instance);

/**
 * @brief          "喂狗"任务
 * @attention       放入rtos中,会给每个daemon实例的temp_count按频率进行递减操作.
 *                  如果模块成功接受数据或成功操作则会重载temp_count的值为reload_count.
 *
 * @param[in]      note
 * @retval         note
 */
void DaemonTask(void);

#endif // DAEMON_H
