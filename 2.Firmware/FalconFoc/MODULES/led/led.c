#include "led.h" // 包含 LED 控制相关的头文件

/* 静态颜色常量定义 */
const RGB_Color_TypeDef RED = {255, 0, 0};        // 红色
const RGB_Color_TypeDef GREEN = {0, 255, 0};      // 绿色
const RGB_Color_TypeDef BLUE = {0, 0, 255};       // 蓝色
const RGB_Color_TypeDef SKY = {0, 255, 255};      // 天蓝色
const RGB_Color_TypeDef MAGENTA = {255, 0, 220};  // 品红色
const RGB_Color_TypeDef YELLOW = {127, 216, 0};   // 黄色
const RGB_Color_TypeDef ORANGE = {127, 106, 0};   // 橙色
const RGB_Color_TypeDef BLACK = {0, 0, 0};        // 黑色
const RGB_Color_TypeDef WHITE = {255, 255, 255};  // 白色
const RGB_Color_TypeDef PURPLE = {128, 0, 128};   // 紫色
const RGB_Color_TypeDef BROWN = {165, 42, 42};    // 棕色
const RGB_Color_TypeDef GRAY = {128, 128, 128};   // 灰色
const RGB_Color_TypeDef PINK = {255, 192, 203};   // 粉色
const RGB_Color_TypeDef GOLD = {255, 215, 0};     // 金色
const RGB_Color_TypeDef SILVER = {192, 192, 192}; // 银色

/* LED数据缓冲区 */
uint32_t led_buf[LED_MAX_NUM + 1][24]; // LED_MAX_NUM + 1 是为了存储最后一行的 RESET 信号

/**
 * @brief 设定单个 RGB LED 的颜色
 *
 * 此函数根据给定的 LED 序号和颜色设置对应 LED 的颜色数据。
 *
 * @param LedId LED 序号，范围从 0 到 LED_MAX_NUM
 * @param Color 颜色结构体，包含 RGB 三个分量
 */
void RGB_SetColor(uint8_t LedId, RGB_Color_TypeDef Color)
{
    if (LedId > LED_MAX_NUM) // 检查 LED 序号是否越界
        return;              // 防止数组越界

    // 将颜色数据转换为 PWM 信号格式，存储到 led_buf 中
    for (uint8_t i = 0; i < 8; i++)
    {
        // 设置 G（绿色）分量
        led_buf[LedId][i]      = (Color.G & (1 << (7 - i))) ? CODE_1 : CODE_0;
        // 设置 R（红色）分量
        led_buf[LedId][8 + i]  = (Color.R & (1 << (7 - i))) ? CODE_1 : CODE_0;
        // 设置 B（蓝色）分量
        led_buf[LedId][16 + i] = (Color.B & (1 << (7 - i))) ? CODE_1 : CODE_0;
    }
}

/**
 * @brief 设定最后一行的 RESET 码
 *
 * 此函数用于将 LED 数据缓冲区的最后一行清零，以发送 RESET 信号。
 */
void Reset_Load(void)
{
    memset(led_buf[LED_MAX_NUM], 0, sizeof(led_buf[LED_MAX_NUM])); // 清空最后一行数据
}

/**
 * @brief 发送 LED 数据
 *
 * 此函数通过 DMA 启动 PWM 输出，将 LED 数据发送到硬件。
 */
void RGB_SendArray(void)
{
    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t *)led_buf, (LED_MAX_NUM + 1) * 24); // 启动 PWM 输出
}

/**
 * @brief 显示指定颜色
 *
 * 此函数将指定颜色应用到所有 LED。
 *
 * @param color 颜色结构体，包含 RGB 三个分量
 */
void RGB_DisplayColor(RGB_Color_TypeDef color)
{
    for (uint16_t i = 0; i < LED_MAX_NUM; i++) // 遍历所有 LED
    {
        RGB_SetColor(i, color); // 设置每个 LED 的颜色
    }
    Reset_Load();    // 设置 RESET 信号
    RGB_SendArray(); // 发送数据到 LED
}

/**
 * @brief 根据颜色编号显示颜色
 *
 * 此函数根据给定的颜色编号显示对应的颜色。
 *
 * @param color_id 颜色编号，范围从 0 到 14（增加了新的颜色）
 */
void RGB_DisplayColorById(uint8_t color_id)
{
    RGB_Color_TypeDef color; // 定义一个颜色结构体

    // 根据颜色编号选择对应的颜色
    switch (color_id)
    {
    case 0:
        color = RED; // 红色
        break;
    case 1:
        color = GREEN; // 绿色
        break;
    case 2:
        color = BLUE; // 蓝色
        break;
    case 3:
        color = SKY; // 天蓝色
        break;
    case 4:
        color = MAGENTA; // 品红色
        break;
    case 5:
        color = YELLOW; // 黄色
        break;
    case 6:
        color = ORANGE; // 橙色
        break;
    case 7:
        color = BLACK; // 黑色
        break;
    case 8:
        color = WHITE; // 白色
        break;
    case 9:
        color = PURPLE; // 紫色
        break;
    case 10:
        color = BROWN; // 棕色
        break;
    case 11:
        color = GRAY; // 灰色
        break;
    case 12:
        color = PINK; // 粉色
        break;
    case 13:
        color = GOLD; // 金色
        break;
    case 14:
        color = SILVER; // 银色
        break;
    default:
        color = BLACK; // 默认颜色为黑色
        break;
    }

    RGB_DisplayColor(color); // 调用显示颜色函数
}
