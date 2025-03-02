#include "motor_task.h"

#define ADJUST_EN 0 // 调参使能

/**
 * @brief ADC1 电流采样完成回调函数
 *
 * 电流采样频率20khz
 *
 * @param hadc 传入的 ADC1
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // 更新事件触发器
    UNUSED(motor_data.components.current->hadc);
    // 检查完成转换的 ADC 是否为 ADC1
    if (hadc == &ADC_HSPI)
    {
#if ADJUST_EN
        GetMotorADC1PhaseCurrent(&motor_data);
        GetMotor_Angle(motor_data.components.encoder);
#else
        GetMotorADC1PhaseCurrent(&motor_data);
        GetMotor_Angle(motor_data.components.encoder);
        MotorStateTask(&motor_data);
#endif
    }
}
