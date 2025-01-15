#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"
#include "cmd_task.h"
#include "bsp_log.h"

void RobotInit(void)
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();

    DWT_Delay(0.016f); // MT6816上电的16ms无输出数据
    BSPInit();         // 初始化DWT
    LogInit(&huart1);  // 初始化日志
    RobotCMDInit();    // 初始化FDCAN
    adc_bsp_init();    // 初始化ADC，配置ADC硬件
    Foc_Pwm_Start();   // foc启动PWM

    __enable_irq(); // 初始化完成,开启中断
}

void RobotTask(void)
{
}
