#include "BLDCMotor.h"

// 全局变量，包含电机相关数据
FOC_DATA foc_data = {
    .inv_vbus                = INVBATVEL,                 // 反馈电压
    .vd_set                  = 0.0f,                      // d 轴电压设置
    .vq_set                  = 0.0f,                      // q 轴电压设置
    .id_set                  = 0.0f,                      // d 轴电流设置
    .iq_set                  = MOTOR_INPUT_CURRENT,       // q 轴电流设置
    .i_d_filt                = 0.0f,                      // 低通滤波后的 d 轴电流
    .i_q_filt                = 0.0f,                      // 低通滤波后的 q 轴电流
    .i_bus                   = 0.0f,                      // 总电流
    .i_bus_filt              = 0.0f,                      // 低通滤波后的总电流
    .power_filt              = 0.0f,                      // 低通滤波后的总功率
    .current_ctrl_integral_d = 0.0f,                      // d 轴电流环积分值
    .current_ctrl_integral_q = 0.0f,                      // q 轴电流环积分值
    .current_ctrl_p_gain     = MOTOR_CURRENT_CTRL_P_GAIN, // 电流环增益，在校准过程中自动计算得出，也可自行设置 (Auto)
    .current_ctrl_i_gain     = MOTOR_CURRENT_CTRL_I_GAIN, // 电流环积分增益，在校准过程中自动计算得出，也可自行设置 (Auto)
};

void FOC_reset(FOC_DATA *foc)
{
    foc->i_d                 = 0.0f;
    foc->i_q                 = 0.0f;
    foc->current_ctrl_p_gain = 0.0f;
    foc->current_ctrl_i_gain = 0.0f;
}

/**
 * @brief 启动PWM输出
 *
 * 启动定时器通道1、2、3的PWM输出，用于控制电机三相电流。
 *
 * @param
 */
void Foc_Pwm_Start(void)
{
    set_dtc_a(PWM_ARR >> 1); // 设置通道1的占空比
    set_dtc_b(PWM_ARR >> 1); // 设置通道2的占空比
    set_dtc_c(PWM_ARR >> 1); // 设置通道3的占空比

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief 停止PWM输出
 *
 * 停止定时器通道1、2、3的PWM输出，用于控制电机三相电流。
 *
 * @param
 */
void Foc_Pwm_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

void Foc_Pwm_LowSides(void)
{
    /* Set all duty to 0% */
    set_dtc_a(0); // 设置通道1的占空比
    set_dtc_b(0); // 设置通道2的占空比
    set_dtc_c(0); // 设置通道3的占空比

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/**
 * @brief 设置PWM占空比
 *
 * 根据输入值设置三个通道的PWM占空比，用于控制电机转速和转向。
 *
 * @param dtc_a 通道1的占空比值（0-100%）
 * @param dtc_b 通道2的占空比值（0-100%）
 * @param dtc_c 通道3的占空比值（0-100%）
 */
void SetPwm(FOC_DATA *foc)
{
    // 设置各个通道对应寄存器比较值，实现PWM输出调整
    set_dtc_a((uint16_t)(foc->dtc_a * PWM_ARR)); // 设置通道1的占空比
    set_dtc_b((uint16_t)(foc->dtc_b * PWM_ARR)); // 设置通道2的占空比
    set_dtc_c((uint16_t)(foc->dtc_c * PWM_ARR)); // 设置通道3的占空比
}

/**
 * @brief 计算正弦和余弦值
 *
 * 计算电角度 theta 对应的 sin 和 cos 值
 *
 * @param foc 指向电机数据结构的指针
 */
void Sin_Cos_Val(FOC_DATA *foc)
{
    foc->sin_val = arm_sin_f32(foc->theta); // 计算正弦值
    foc->cos_val = arm_cos_f32(foc->theta); // 计算余弦值
}

/**
 * @brief Clarke变换
 *
 * Clarke变换将三相电流转换为 Alpha-Beta 坐标系下的电流
 *
 * @param foc 指向电机数据结构的指针
 */
void Clarke(FOC_DATA *foc)
{
    foc->i_alpha = foc->i_a;                            // α轴电流等于a相电流
    foc->i_beta  = (foc->i_b - foc->i_c) * ONE_BY_SQRT3; // β轴电流计算，使用√3的倒数进行归一化
}

/**
 * @brief 逆Clarke变换
 *
 * 逆Clarke变换将 Alpha-Beta 坐标系下的电压转换为三相电压
 *
 * @param foc 指向电机数据结构的指针
 */
void Inv_clarke(FOC_DATA *foc)
{
    foc->v_a = foc->v_alpha;                                  // a相电压等于α轴电压
    foc->v_b = -0.5f * foc->v_alpha + _SQRT3_2 * foc->v_beta; // b相电压计算
    foc->v_c = -0.5f * foc->v_alpha - _SQRT3_2 * foc->v_beta; // c相电压计算
}

/**
 * @brief Park变换
 *
 * Park变换将 Alpha-Beta 坐标系下的电流转换为 dq 坐标系下的电流，转换成了两个恒定的直流分量
 *
 * @param foc 指向电机数据结构的指针
 */
void Park(FOC_DATA *foc)
{
    foc->i_d = foc->i_alpha * foc->cos_val + foc->i_beta * foc->sin_val; // d轴电流计算
    foc->i_q = foc->i_beta * foc->cos_val - foc->i_alpha * foc->sin_val; // q轴电流计算
}

/**
 * @brief 逆Park变换
 *
 * 反Park变换将 dq 坐标系下的电压转换为 Alpha-Beta 坐标系下的电压
 *
 * @param foc 指向电机数据结构的指针
 */
void Inv_Park(FOC_DATA *foc)
{
    foc->v_alpha = (foc->v_d * foc->cos_val - foc->v_q * foc->sin_val); // α轴电压计算
    foc->v_beta  = (foc->v_d * foc->sin_val + foc->v_q * foc->cos_val);  // β轴电压计算
}

/**
 * @brief 均值零序分量注入
 *
 * 对α-β坐标系的电压进行均值零序分量注入，以便进行SVPWM调制
 * 电压角度是垂直于转子角度的，n代表的是合成电压的扇区，不是转子所在的扇区
 *
 * @param foc 指向电机数据结构的指针
 */
void Svpwm_Midpoint(FOC_DATA *foc)
{
    // 根据母线电压调整α和β轴电压
    foc->v_alpha = foc->inv_vbus * foc->v_alpha;
    foc->v_beta  = foc->inv_vbus * foc->v_beta;

    float va     = foc->v_alpha;                                  // 保存α轴电压
    float vb     = -0.5f * foc->v_alpha + _SQRT3_2 * foc->v_beta; // 计算b相电压
    float vc     = -0.5f * foc->v_alpha - _SQRT3_2 * foc->v_beta; // 计算c相电压

    float vmax   = max(max(va, vb), vc); // 找到最大电压值
    float vmin   = min(min(va, vb), vc); // 找到最小电压值

    float vcom   = (vmax + vmin) * 0.5f; // 计算零序分量

    // 计算各相的占空比调整值，确保在0到1之间
    foc->dtc_a   = ((vcom - va) + 0.5f), 0.0f, 1.0f;
    foc->dtc_b   = ((vcom - vb) + 0.5f), 0.0f, 1.0f;
    foc->dtc_c   = ((vcom - vc) + 0.5f), 0.0f, 1.0f;
}

/**
 * @brief 扇区判断
 *
 * 根据α-β坐标系中的电压值判断当前所在扇区，并计算各相的占空比
 *
 * @param foc 指向电机数据结构的指针
 */
void Svpwm_Sector(FOC_DATA *foc)
{
    float ta    = 0.0f, tb = 0.0f, tc = 0.0f;                    // 各相占空比时间初始化
    float k     = (TS * _SQRT3) * foc->inv_vbus;                 // 根据母线电压计算k值
    float va    = foc->v_beta;                                   // β轴电压
    float vb    = (_SQRT3 * foc->v_alpha - foc->v_beta) * 0.5f;  // b相电压计算
    float vc    = (-_SQRT3 * foc->v_alpha - foc->v_beta) * 0.5f; // c相电压计算
    int a       = (va > 0.0f) ? 1 : 0;                           // 判断a相是否大于零，结果为1或0
    int b       = (vb > 0.0f) ? 1 : 0;                           // 判断b相是否大于零，结果为1或0
    int c       = (vc > 0.0f) ? 1 : 0;                           // 判断c相是否大于零，结果为1或0
    int sextant = (c << 2) + (b << 1) + a;                       // 根据a、b、c相的状态确定扇区

    switch (sextant)
    {
    case SECTOR_3: // 扇区3
    {
        float t4 = k * vb;
        float t6 = k * va;
        float t0 = (TS - t4 - t6) * 0.5f; // 零矢量的作用时间

        ta       = t4 + t6 + t0; // 计算a相占空比时间
        tb       = t6 + t0;      // 计算b相占空比时间
        tc       = t0;           // c相占空比时间为t0
    }
    break;

    case SECTOR_1: // 扇区1
    {
        float t6 = -k * vc;
        float t2 = -k * vb;
        float t0 = (TS - t2 - t6) * 0.5f;

        ta       = t6 + t0;      // a相占空比时间计算
        tb       = t2 + t6 + t0; // b相占空比时间计算
        tc       = t0;           // c相占空比时间为t0
    }
    break;

    case SECTOR_5: // 扇区5
    {
        float t2 = k * va;
        float t3 = k * vc;
        float t0 = (TS - t2 - t3) * 0.5f;

        ta       = t0;           // a相占空比时间为t0
        tb       = t2 + t3 + t0; // b相占空比时间计算
        tc       = t3 + t0;      // c相占空比时间计算
    }
    break;

    case SECTOR_4: // 扇区4
    {
        float t1 = -k * va;
        float t3 = -k * vb;
        float t0 = (TS - t1 - t3) * 0.5f;

        ta       = t0;           // a相占空比时间为t0
        tb       = t3 + t0;      // b相占空比时间计算
        tc       = t1 + t3 + t0; // c相占空比时间计算
    }
    break;

    case SECTOR_6: // 扇区6
    {
        float t1 = k * vc;
        float t5 = k * vb;
        float t0 = (TS - t1 - t5) * 0.5f;

        ta       = t5 + t0;      // a相占空比时间计算
        tb       = t0;           // b相占空比时间为t0
        tc       = t1 + t5 + t0; // c相占空比时间计算
    }
    break;

    case SECTOR_2: // 扇区2
    {
        float t4 = -k * vc;
        float t5 = -k * va;
        float t0 = (TS - t4 - t5) * 0.5f;

        ta       = t4 + t5 + t0; // a相占空比时间计算
        tb       = t0;           // b相占空比时间为t0
        tc       = t5 + t0;      // c相占空比时间计算
    }
    break;

    default:
        break; // 默认情况，不做处理
    }

    // 更新各相的占空比调整值，反转以适应SVPWM调制方式，使其在[0,1]范围内。
    foc->dtc_a   = (1.0f - ta), 0.0f, 1.0f;
    foc->dtc_b   = (1.0f - tb), 0.0f, 1.0f;
    foc->dtc_c   = (1.0f - tc), 0.0f, 1.0f;
}

/**
 * @brief FOC用户函数
 *
 * 此函数用于包装FOC的一些操作步骤
 *
 * @param motor 指向电机数据结构的指针
 */
void commonFOCOperations(FOC_DATA *foc)
{
    Sin_Cos_Val(foc);
    Clarke(foc);
    Park(foc);
}

void commonInverseFOCOperations(FOC_DATA *foc)
{
    Inv_Park(foc);
    Inv_clarke(foc);
    Svpwm_Sector(foc);
    SetPwm(foc);
}
void setPhaseVoltage(FOC_DATA *foc, float Vd_set, float Vq_set, float phase)
{
    foc->v_q   = Vq_set;
    foc->v_d   = Vd_set;
    foc->theta = phase;
    Sin_Cos_Val(foc);
    commonInverseFOCOperations(foc);
}

void FOC_voltage(FOC_DATA *foc, float Vd_set, float Vq_set, float phase)
{
    // Clarke transform
    float i_alpha, i_beta;
    clarke_transform(foc->i_a, foc->i_b, foc->i_c, &i_alpha, &i_beta);

    // Park transform
    float i_d, i_q;
    park_transform(i_alpha, i_beta, phase, &i_d, &i_q);

    // Used for report
    foc->i_q = i_q;
    UTILS_LP_FAST(foc->i_q_filt, foc->i_q, 0.01f);
    foc->i_d = i_d;
    UTILS_LP_FAST(foc->i_d_filt, foc->i_d, 0.01f);

    // Modulation
    float V_to_mod = 1.0f / (foc->vbus * 2.0f / 3.0f);
    float mod_d    = V_to_mod * Vd_set;
    float mod_q    = V_to_mod * Vq_set;

    // Vector modulation saturation, lock integrator if saturated
    float mod_scalefactor = 0.9f * _SQRT3_2 / sqrtf(SQ(mod_d) + SQ(mod_q));
    if (mod_scalefactor < 1.0f)
    {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
    }

    // Inverse park transform
    float mod_alpha;
    float mod_beta;
    inverse_park(mod_d, mod_q, phase, &mod_alpha, &mod_beta);

    // SVM
    if (0 == svm(mod_alpha, mod_beta, &foc->dtc_a, &foc->dtc_b, &foc->dtc_c))
    {
        SetPwm(foc);
    }
}

void FOC_current(FOC_DATA *foc, float Id_set, float Iq_set, float phase, float phase_vel)
{
    // Clarke transform
    float i_alpha, i_beta;
    clarke_transform(foc->i_a, foc->i_b, foc->i_c, &i_alpha, &i_beta);

    // Park transform
    float i_d, i_q;
    park_transform(i_alpha, i_beta, phase, &i_d, &i_q);

    float mod_to_V = foc->vbus * 2.0f / 3.0f;
    float V_to_mod = 1.0f / mod_to_V;

    // Apply PI control
    float Ierr_d = Id_set - i_d;
    float Ierr_q = Iq_set - i_q;
    float mod_d  = V_to_mod * (foc->current_ctrl_integral_d + Ierr_d * foc->current_ctrl_p_gain);
    float mod_q  = V_to_mod * (foc->current_ctrl_integral_q + Ierr_q * foc->current_ctrl_p_gain);

    // Vector modulation saturation, lock integrator if saturated
    float mod_scalefactor = 0.9f * _SQRT3_2 / sqrtf(SQ(mod_d) + SQ(mod_q));
    if (mod_scalefactor < 1.0f)
    {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
        foc->current_ctrl_integral_d *= 0.99f;
        foc->current_ctrl_integral_q *= 0.99f;
    }
    else
    {
        foc->current_ctrl_integral_d += Ierr_d * (foc->current_ctrl_i_gain * CURRENT_MEASURE_PERIOD);
        foc->current_ctrl_integral_q += Ierr_q * (foc->current_ctrl_i_gain * CURRENT_MEASURE_PERIOD);
    }

    // Inverse park transform
    float mod_alpha, mod_beta;
    float pwm_phase = phase + phase_vel * CURRENT_MEASURE_PERIOD;
    inverse_park(mod_d, mod_q, pwm_phase, &mod_alpha, &mod_beta);

    // Used for reportZ
    foc->i_q        = i_q;
    UTILS_LP_FAST(foc->i_q_filt, foc->i_q, 0.01f);
    foc->i_d        = i_d;
    UTILS_LP_FAST(foc->i_d_filt, foc->i_d, 0.01f);
    foc->i_bus      = (mod_d * i_d + mod_q * i_q);
    UTILS_LP_FAST(foc->i_bus_filt, foc->i_bus, 0.01f);
    foc->power_filt = foc->vbus * foc->i_bus_filt;

    // SVM
    if (0 == svm(mod_alpha, mod_beta, &foc->dtc_a, &foc->dtc_b, &foc->dtc_c))
    {
        SetPwm(foc);
    }
}
