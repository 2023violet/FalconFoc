/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

// 系统库
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "tim.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "usb_device.h"

// 用户库
#include "robot.h"
#include "motor_task.h"
#include "cmd_task.h"
#include "led.h"
#include "vofa.h"

void StartDefaultTask(void const *argument);
void StartGuardTask(void const *argument);
void StartCustomTask(void const *argument);

/**
 * @brief 调试线程
 */
__attribute__((noreturn)) void StartDefaultTask(void const *argument)
{
    static float default_start;
    static float default_dt;
    MX_USB_Device_Init();
    for (;;)
    {
        default_start = DWT_GetTimeline_ms();
        vofa_start();
        default_dt = DWT_GetTimeline_ms() - default_start;
        if (default_dt > 1)
            LOGERROR("[freeRTOS] DEFAULT Task is being DELAY! dt = [%f]", default_dt);
        osDelay(1);
    }
}

/**
 * @brief 守护线程
 *
 * LED蓝灯代表无错误正常运行
 * LED紫灯代表正在校准
 * LED红灯代表故障
 *
 */
__attribute__((noreturn)) void StartGuardTask(void const *argument)
{
    static float guard_start;
    static float guard_dt;
    for (;;)
    {
        guard_start = DWT_GetTimeline_ms();
        MotorGuardTask(&motor_data);
        TempResultTask(&motor_data);
        guard_dt = DWT_GetTimeline_ms() - guard_start;
        if (guard_dt > 5)
            LOGERROR("[freeRTOS] DEFAULT Task is being DELAY! dt = [%f]", guard_dt);
        osDelay(5);
    }
}

/**
 * @brief 自定义功能线程（oled之类的）
 */
__attribute__((noreturn)) void StartCustomTask(void const *argument)
{
    static float custom_start;
    static float custom_dt;
    for (;;)
    {
        custom_start = DWT_GetTimeline_ms();
        RobotCMDTask();
        custom_dt = DWT_GetTimeline_ms() - custom_start;
        if (custom_dt > 2)
            LOGERROR("[freeRTOS] DEFAULT Task is being DELAY! dt = [%f]", custom_dt);
        osDelay(2);
    }
}
