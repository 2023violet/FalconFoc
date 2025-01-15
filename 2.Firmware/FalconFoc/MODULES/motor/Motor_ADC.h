#ifndef _Motor_ADC_H__ // 防止重复包含头文件
#define _Motor_ADC_H__

#include "general_def.h"
#include "bsp_adc.h" // 包含基础 ADC 驱动的头文件

#define ADC_HSPI (hadc1) // 获取SPI句柄;

// 定义 ADC 偏置结构体，用于存储电流传感器的偏置值
typedef struct
{
    ADC_HandleTypeDef *hadc;    // ADC 句柄
    float Temp_Result;          // 温度
    float Ia_offset;            // Ia 偏置
    float Ib_offset;            // Ib 偏置
    float Ic_offset;            // Ic 偏置
    float current_offset_sum_a; // Ia 偏置和
    float current_offset_sum_b; // Ib 偏置和
    float current_offset_sum_c; // Ic 偏置和
} CURRENT_DATA;

extern CURRENT_DATA current_data;

/* adc通道名字定义，根据自己的使用来命名 */
typedef enum
{
    adc1_ch1 = 0,
    adc1_ch2 = 1,
    adc1_ch3 = 2,
    adc1_ch4 = 3
} adc1_num;

typedef enum
{
    adc2_ch12 = 0,
} adc2_num;

void GetTempNtc(uint16_t value_adc, float *value_temp);
uint16_t adc1_median_filter(uint8_t channel);
uint16_t adc1_avg_filter(uint8_t channel);
uint16_t adc2_median_filter(uint8_t channel);
uint16_t adc2_avg_filter(uint8_t channel);
#endif // _Motor_ADC_H__
