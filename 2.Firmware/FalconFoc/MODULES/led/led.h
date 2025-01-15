#ifndef LED_H
#define LED_H

#include "general_def.h"
#include "tim.h"

/*这里是计算所得CCR的宏定义*/
#define CODE_1 (140) // 1码定时器计数次数
#define CODE_0 (70)  // 0码定时器计数次数

#define LED_MAX_NUM 1 // LED数量宏定义，这里我使用一个LED

/*建立一个定义单个LED三原色值大小的结构体*/
typedef struct
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
} RGB_Color_TypeDef;

void RGB_SetColor(uint8_t LedId, RGB_Color_TypeDef Color); // 给一个LED装载24个颜色数据码（0码和1码）
void Reset_Load(void);                                     // 该函数用于将数组最后24个数据变为0，代表RESET_code
void RGB_SendArray(void);                                  // 发送LED数据
void RGB_DisplayColor(RGB_Color_TypeDef color);            // 显示指定颜色
void RGB_DisplayColorById(uint8_t color_id);               // 根据颜色编号显示颜色

#endif
