/********************************************************************************
 * @file        : daemon.c
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

#include "daemon.h"

// 用于保存所有的daemon instance
static DaemonInstance *daemon_instances[DAEMON_MX_CNT] = {NULL};
static uint8_t idx; // 用于记录当前的daemon instance数量,配合回调使用

/**
 * @brief          注册守护实例
 * @param[in]      DAEMON_config: 守护实例配置
 * @retval         守护实例
 */
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *DAEMON_config)
{
    DaemonInstance *Daemon_instance = (DaemonInstance *)malloc(sizeof(DaemonInstance));
    memset(Daemon_instance, 0, sizeof(DaemonInstance));

    Daemon_instance->owner_id       = DAEMON_config->owner_id;
    Daemon_instance->reload_count   = DAEMON_config->reload_count == 0 ? 100 : DAEMON_config->reload_count; // 默认值为100
    Daemon_instance->callback       = DAEMON_config->callback;
    Daemon_instance->temp_count     = DAEMON_config->init_count == 0 ? 100 : DAEMON_config->init_count; // 默认值为100,初始计数

    Daemon_instance->temp_count     = DAEMON_config->reload_count;
    daemon_instances[idx++]         = Daemon_instance;
    return Daemon_instance;
}

/**
 * @brief          "喂狗"函数
 * @param[in]      instance: 守护实例配置
 * @retval         note
 */
void DaemonReload(DaemonInstance *instance)
{
    instance->temp_count = instance->reload_count;
}

/**
 * @brief          确认模块是否离线
 * @param[in]      instance: 守护实例配置
 * @retval         note
 */
uint8_t DaemonIsOnline(DaemonInstance *instance)
{
    return instance->temp_count > 0;
}

/**
 * @brief          "喂狗"任务
 * @attention       放入rtos中,会给每个daemon实例的temp_count按频率进行递减操作.
 *                  如果模块成功接受数据或成功操作则会重载temp_count的值为reload_count.
 *
 * @param[in]      note
 * @retval         note
 */
void DaemonTask(void)
{
    DaemonInstance *dins; // 提高可读性同时降低访存开销
    for (size_t i = 0; i < idx; ++i)
    {
        dins = daemon_instances[i];
        if (dins->temp_count > 0) // 如果计数器还有值,说明上一次喂狗后还没有超时,则计数器减一
            dins->temp_count--;
        else if (dins->callback) // 等于零说明超时了,调用回调函数(如果有的话)
            dins->callback(dins->owner_id);
        // module内可以将owner_id强制类型转换成自身类型从而调用特定module的offline callback
        // @todo 为蜂鸣器/led等增加离线报警的功能,非常关键!
    }
}
