#include "mt6816_encoder.h"

/**
 * @brief 编码器初始化函数
 *
 * 此函数用于初始化编码器相关参数和 SPI 外设。
 *
 * @param None
 */
ENCODER_DATA encoder_data = {
    .hspi                 = &MT6816_SPI_Get_HSPI, // 指向 SPI 外设的指针
    .CS_Port              = SPI1_CS_GPIO_Port,    // 片选引脚的端口
    .CS_Pin               = SPI1_CS_Pin,          // 片选引脚的引脚号
    .pole_pairs           = MPTOR_P,              // 设置极对数为电机参数
    .encoder_offset       = MOTOR_OFFSET,         // 设置电角度偏移量
    .dir                  = MOTOR_DIRECTION,      // 设置编码器旋转方向为顺时针
    .raw                  = 0,                    // 初始化原始计数值为0
    .cnt                  = 0,                    // 计数器
    .theta_acc            = 0.01f,                // 设置角度累加步长为0.01弧度
    .count_in_cpr_        = 0,                    // 原始值（整数型）
    .shadow_count_        = 0,                    // 原始值累加计数（整数型）
    .pos_estimate_counts_ = 0.0f,                 // 原始值累加计数（浮点型）
    .vel_estimate_counts_ = 0.0f,                 // 原始值转速
    .pos_cpr_counts_      = 0.0f,                 // 原始值（浮点型）
    .pos_estimate_        = 0.0f,                 // 圈数
    .vel_estimate_        = 0.0f,                 // 加速度rad/s
    .pos_cpr_             = 0.0f,                 // 0-1
    .phase_               = 0.0f,                 // 电角度
    .interpolation_       = 0.0f,                 // 插值系数
    .elec_angle           = 0.0f,                 // 电角度
    .mec_angle            = 0.0f,                 // 机械角度
    .calib_valid          = false,                // 表示校准的数据是否有效
};

/**
 * @brief 将角度归一化到 [0, 2π] 范围内
 *
 * 此函数将给定的角度值归一化到 [0, 2π] 范围内。
 *
 * @param angle 要归一化的角度
 */
float normalize_angle(float angle)
{
    float a = fmod(angle, M_2PI);    // 使用取余运算进行归一化
    return a >= 0 ? a : (a + M_2PI); // 确保返回值在 [0, 2π] 范围内
}

/**
 * @brief 发送与接收数据
 *
 * 此函数通过 SPI 接口发送和接收一个字节的数据。
 *
 * @param txdata 指向要发送的数据的指针
 * @param rxdata 指向接收数据的指针
 */
bool MT6816_read_write_byte(uint8_t *txdata, uint8_t *rxdata)
{
    MT6816_SPI_CS_L();
    bool transmitResult = (HAL_OK == HAL_SPI_Transmit(&MT6816_SPI_Get_HSPI, txdata, 1, MT6816_MAX_DELAY));
    bool receiveResult  = (HAL_OK == HAL_SPI_Receive(&MT6816_SPI_Get_HSPI, rxdata, 1, MT6816_MAX_DELAY));
    MT6816_SPI_CS_H();
    return transmitResult && receiveResult;
}

/**
 * @brief 从 MT6816 编码器读取原始角度数据
 *
 * 此函数读取 MT6816 编码器的原始角度值
 *
 * @param encoder 指向包含电机状态和控制数据的结构体指针
 */
bool mt6816_read_raw(ENCODER_DATA *encoder)
{
    const uint8_t tx[] = {MT6816_Angle_Reg, MT6816_Warning_Reg};
    uint8_t rx[2];
    uint8_t h_count;
    uint16_t rawAngle;

    bool Result_One = MT6816_read_write_byte((uint8_t *)&tx[0], (uint8_t *)&rx[0]);
    bool Result_Two = MT6816_read_write_byte((uint8_t *)&tx[1], (uint8_t *)&rx[1]);

    if (!Result_One || !Result_Two)
    {
        goto TIMEOUT;
    }

    if (encoder->rx_err_count)
    {
        encoder->rx_err_count--;
    }

    // 计算原始角度
    rawAngle = ((rx[0] & 0xFF) << 8) | (rx[1] & 0xFF);

    // 奇偶校验
    h_count = 0;
    for (uint8_t j = 0; j < 16; j++)
    {
        if (rawAngle & (0x01 << j))
        {
            h_count++;
        }
    }
    if (h_count & 0x01)
    {
        goto CHECK_ERR;
    }

    encoder->angle = rawAngle >> 2;

    if (encoder->check_err_count)
    {
        encoder->check_err_count--;
    }

    return true;

CHECK_ERR:

    return false;

TIMEOUT:

    MT6816_SPI_CS_H();

    if (encoder->rx_err_count < 0xFF)
    {
        encoder->rx_err_count++;
    }

    return false;
}

/**
 * @brief 将读取的原始值转化成实际角度和电角度
 *
 * 此函数将读取的原始值转化成实际角度和电角度。
 *
 * @param encoder 指向包含电机状态和控制数据的结构体指针
 */
void GetMotor_Angle(ENCODER_DATA *encoder)
{
    static const float pll_kp_ = 2.0f * ENCODER_PLL_BANDWIDTH;
    static const float pll_ki_ = 0.25f * SQ(pll_kp_);
    static const float snap_threshold = 0.5f * CURRENT_MEASURE_PERIOD * pll_ki_;
    if (mt6816_read_raw(encoder))
    {
        if (encoder->dir == CW)
        {
            encoder->raw = encoder->angle;
        }
        else
        {
            encoder->raw = (ENCODER_CPR - encoder->angle);
        }
    }

    // offset compensation
    int off_1      = encoder->offset_lut[encoder->raw >> 7];
    int off_2      = encoder->offset_lut[((encoder->raw >> 7) + 1) % 128];
    int off_interp = off_1 + ((off_2 - off_1) * (encoder->raw - ((encoder->raw >> 7) << 7)) >> 7); // Interpolate between lookup table entries
    int cnt        = encoder->raw - off_interp;                                                           // Correct for nonlinearity with lookup table from calibration
    if (cnt > ENCODER_CPR)
    {
        cnt -= ENCODER_CPR;
    }
    else if (cnt < 0)
    {
        cnt += ENCODER_CPR;
    }
    encoder->cnt   = cnt;

    int delta_enc  = encoder->cnt - encoder->count_in_cpr_;
    delta_enc      = mod(delta_enc, ENCODER_CPR);
    if (delta_enc > ENCODER_CPR_DIV)
    {
        delta_enc -= ENCODER_CPR;
    }

    encoder->shadow_count_ += delta_enc;
    encoder->count_in_cpr_ += delta_enc;
    encoder->count_in_cpr_  = mod(encoder->count_in_cpr_, ENCODER_CPR);

    encoder->count_in_cpr_  = encoder->cnt;

    //// run pll (for now pll is in units of encoder counts)
    // Predict current pos
    encoder->pos_estimate_counts_ += CURRENT_MEASURE_PERIOD * encoder->vel_estimate_counts_;
    encoder->pos_cpr_counts_      += CURRENT_MEASURE_PERIOD * encoder->vel_estimate_counts_;
    // discrete phase detector
    float delta_pos_counts         = (float)(encoder->shadow_count_ - (int32_t)floor(encoder->pos_estimate_counts_));
    float delta_pos_cpr_counts     = (float)(encoder->count_in_cpr_ - (int32_t)floor(encoder->pos_cpr_counts_));
    delta_pos_cpr_counts           = wrap_pm(delta_pos_cpr_counts, ENCODER_CPR_DIV);
    // pll feedback
    encoder->pos_estimate_counts_ += CURRENT_MEASURE_PERIOD * pll_kp_ * delta_pos_counts;
    encoder->pos_cpr_counts_      += CURRENT_MEASURE_PERIOD * pll_kp_ * delta_pos_cpr_counts;
    encoder->pos_cpr_counts_       = fmodf_pos(encoder->pos_cpr_counts_, ENCODER_CPR_F);
    encoder->vel_estimate_counts_ += CURRENT_MEASURE_PERIOD * pll_ki_ * delta_pos_cpr_counts;
    bool snap_to_zero_vel          = false;
    if (ABS(encoder->vel_estimate_counts_) < snap_threshold) // 100
    {
        encoder->vel_estimate_counts_ = 0.0f; // align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    // Outputs from Encoder for Controller
    float pos_cpr_last     = encoder->pos_cpr_;
    encoder->pos_estimate_ = encoder->pos_estimate_counts_ / ENCODER_CPR_F;
    encoder->vel_estimate_ = encoder->vel_estimate_counts_ / ENCODER_CPR_F;

    encoder->pos_estimate_ = encoder->pos_estimate_counts_ / ENCODER_CPR_F;
    encoder->vel_estimate_ = encoder->vel_estimate_counts_ / ENCODER_CPR_F;
    encoder->pos_cpr_      = encoder->pos_cpr_counts_ / ENCODER_CPR_F;
    float delta_pos_cpr    = wrap_pm(encoder->pos_cpr_ - pos_cpr_last, 0.5f);

    //// run encoder count interpolation
    int32_t corrected_enc  = encoder->count_in_cpr_ - encoder->encoder_offset;
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel)
    {
        encoder->interpolation_ = 0.5f;
        // reset interpolation if encoder edge comes
        // TODO: This isn't correct. At high velocities the first phase in this count may very well not be at the edge.
    }
    else if (delta_enc > 0)
    {
        encoder->interpolation_ = 0.0f;
    }
    else if (delta_enc < 0)
    {
        encoder->interpolation_ = 1.0f;
    }
    else
    {
        // Interpolate (predict) between encoder counts using vel_estimate,
        encoder->interpolation_ += CURRENT_MEASURE_PERIOD * encoder->vel_estimate_counts_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (encoder->interpolation_ > 1.0f)
            encoder->interpolation_ = 1.0f;
        if (encoder->interpolation_ < 0.0f)
            encoder->interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + encoder->interpolation_;

    //// compute electrical phase
    float elec_rad_per_enc = encoder->pole_pairs * M_2PI * (1.0f / ENCODER_CPR_F);
    float ph               = elec_rad_per_enc * interpolated_enc;
    encoder->phase_        = wrap_pm_pi(ph); // 电角度
    encoder->mec_angle     = normalize_angle(corrected_enc * (360.0f / ENCODER_CPR_F) * (M_PI / 180.0f));
    encoder->elec_angle    = normalize_angle(encoder->pole_pairs * encoder->mec_angle); // 电角度
}

/**
 * @brief 角度自增函数
 *
 * 根据电机的旋转方向和速度，计算并更新电机的电角度。
 *
 * @param encoder 指向电机数据结构的指针
 */
void Theta_ADD(ENCODER_DATA *encoder)
{
    encoder->elec_angle    += encoder->theta_acc;
    if (encoder->elec_angle > M_2PI)
        encoder->elec_angle = 0.0f;
    else if (encoder->elec_angle < 0)
        encoder->elec_angle = M_2PI;
}
