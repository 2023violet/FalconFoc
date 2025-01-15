#ifndef BLDCMotor_H
#define BLDCMotor_H

#include "general_def.h"
#include "tim.h"
#include "arm_math.h"
#include "mt6816_encoder.h"

// 电机类型
#define MOTOR_PM3510 0
#define MOTOR_DJI2312 1
#define MOTOR_X2216 0

#define N_BASE 3.0f               // 3S电池
#define BATVEL (4.0f * N_BASE)    // 电池电压
#define INVBATVEL (1.0f / BATVEL) // 电池电压的倒数

#define PRE_CALIBRATED 1 // 第一次运行时先设置0，上电后校准电机，获取了下面的参数后写在下面；设置为1，则直接使用下面的参数

#if MOTOR_X2216
// 如果是云台电机则不用相电感
#if PRE_CALIBRATED
#define MPTOR_P 7u               // 电机极对数
#define MOTOR_RS 0.08729776f     // 相电阻
#define MOTOR_LS 1.40520524e-05f // 相电感
#define MOTOR_FLUX 0.0f          // 磁链
#define MOTOR_OFFSET 2984        // 电机偏移量
#define MOTOR_DIRECTION CCW      // 电机方向
#else
#define MPTOR_P 0u              // 电机极对数
#define MOTOR_RS 0.0f           // 相电阻
#define MOTOR_LS 0.0f           // 相电感
#define MOTOR_FLUX 0.0f         // 磁链
#define MOTOR_OFFSET 0.0f       // 电机偏移量
#define MOTOR_DIRECTION UNKNOWN // 电机方向
#endif

// 电机参数相关定义 （梯度加减速不能太大，不然波形在减速时会出现断崖式下跌，位移步数短看不出来，拉大后很明显！！！）
#define MOTOR_INERTIA 0.001f                // 转动惯量 [A/(turn/s^2)]，含义为电机轴以1转每秒的加速度运转时需要提供的用于加速消耗的电流，需要根据电机轴重量和所带负载来进行调试。
#define MOTOR_CURRENT_RAMP_RATE 0.001f      // 电流爬升力矩 [Nm/s]
#define MOTOR_VEL_RAMP_RATE 170.0f          // 转速爬升速度 [(turn/s)/s]
#define MOTOR_TRAJ_VEL 170.0f               // 梯形轨迹控制模式下最大转速 [turn/s] (设定值应当小于或等于 MOTOR_VEL_LIMIT)
#define MOTOR_TRAJ_ACCEL 50.0f              // 梯形轨迹控制模式下加速度 [(turn/s)/s]
#define MOTOR_TRAJ_DECEL 50.0f              // 梯形轨迹控制模式下减速度 [(turn/s)/s]
#define MOTOR_VEL_LIMIT 175.0f              // 转速限制值 [turn/s] (turn/s = 最大转速rpm / 60) 一般设置为电机最大允许转速
#define MOTOR_VOLTAGE_LIMIT 12.0f           // 电机电压限制值 [V]
#define MOTOR_CURRENT_LIMIT 10.0f           // 电机电流限制值 [A] (该值限制的是实际运行中电机的相电流 foc->i_q_filt) 一般设置为电机最大允许电流
#define MOTOR_CURRENT_CTRL_P_GAIN 0.0f      // 电流环增益，在校准过程中自动计算得出，也可自行设置 (Auto)
#define MOTOR_CURRENT_CTRL_I_GAIN 0.0f      // 电流环积分增益，在校准过程中自动计算得出，也可自行设置 (Auto)
#define MOTOR_CURRENT_CTRL_BANDWIDTH 230.0f // 电流环带宽[rad/s]，范围 (2~60000)
#define MOTOR_STALL_CURRENT 10.5f           // 堵转电流，电机堵转时的电流
#define MOTOR_INPUT_CURRENT 5.0f            // 目标电流
#define MOTOR_INPUT_VELOCITY 50.0f          // 目标转速
#define MOTOR_INPUT_POSITION 0.0f           // 目标位置

// PID参数
#define MOTOR_IQ_PID_KP 0.0f
#define MOTOR_IQ_PID_KI 0.0f
#define MOTOR_IQ_PID_KD 0.0f
#define MOTOR_ID_PID_KP 0.0f
#define MOTOR_ID_PID_KI 0.0f
#define MOTOR_ID_PID_KD 0.0f
#define MOTOR_VEL_PID_KP 0.25f
#define MOTOR_VEL_PID_KI 0.0025f // 不可太大，否则会导致电机激烈抖动，以后想做无感时切换就不顺畅了
#define MOTOR_VEL_PID_KD 0.0f
#define MOTOR_POS_PID_KP 160.0f // 可以看回正时速度是否快准狠
#define MOTOR_POS_PID_KI 0.0f
#define MOTOR_POS_PID_KD 0.0f

/*单电流环时：      限幅值没有什么约束但是不能大于母线电压/根号3，否则会导致电流环输出超限
**电流速度环时：    因为要先经过速度环，然后再把速度环的值传给电流环，所以电流环的限幅为母线电压/根号3，速度环的限幅为最大电流值
**电流速度位置环时：把位置环的限幅设为电机最大转速即可，电流环的限幅为母线电压/根号3，速度环的限幅为最大电流值
**
**位置环的限幅不可过大，不然会有很大的过充一下子就报警告了！！！
*/
#define CURRENT_PID_MAX_OUT ((BATVEL) / _SQRT3)
#define IQ_PID_MAX_OUT CURRENT_PID_MAX_OUT
#define IQ_PID_MAX_IOUT CURRENT_PID_MAX_OUT
#define ID_PID_MAX_OUT CURRENT_PID_MAX_OUT
#define ID_PID_MAX_IOUT CURRENT_PID_MAX_OUT
#define VEL_PID_MAX_OUT MOTOR_CURRENT_LIMIT
#define VEL_PID_MAX_IOUT MOTOR_CURRENT_LIMIT
#define POS_PID_MAX_OUT (MOTOR_VEL_LIMIT / 1.3f)
#define POS_PID_MAX_IOUT (MOTOR_VEL_LIMIT / 1.3f)
#endif

#if MOTOR_DJI2312 // 电机类型
#if PRE_CALIBRATED
#define MPTOR_P 7u               // 电机极对数
#define MOTOR_RS 0.125786185f    // 相电阻
#define MOTOR_LS 2.37391487e-05f // 相电感
#define MOTOR_FLUX 0.0f          // 磁链
#define MOTOR_OFFSET 13360       // 电机偏移量
#define MOTOR_DIRECTION CCW      // 电机方向
#else
#define MPTOR_P 0u              // 电机极对数
#define MOTOR_RS 0.0f           // 相电阻
#define MOTOR_LS 0.0f           // 相电感
#define MOTOR_FLUX 0.0f         // 磁链
#define MOTOR_OFFSET 0.0f       // 电机偏移量
#define MOTOR_DIRECTION UNKNOWN // 电机方向
#endif

// 电机参数相关定义 （梯度加减速不能太大，不然波形在减速时会出现断崖式下跌，位移步数短看不出来，拉大后很明显！！！）
#define MOTOR_INERTIA 0.0001f               // 转动惯量 [A/(turn/s^2)]，含义为电机轴以1转每秒的加速度运转时需要提供的用于加速消耗的电流，需要根据电机轴重量和所带负载来进行调试。
#define MOTOR_CURRENT_RAMP_RATE 0.001f      // 电流爬升力矩 [Nm/s]
#define MOTOR_VEL_RAMP_RATE 160.0f          // 转速爬升速度 [(turn/s)/s]
#define MOTOR_TRAJ_VEL 160.0f               // 梯形轨迹控制模式下最大转速 [turn/s] (设定值应当小于或等于 MOTOR_VEL_LIMIT)
#define MOTOR_TRAJ_ACCEL 40.0f              // 梯形轨迹控制模式下加速度 [(turn/s)/s]
#define MOTOR_TRAJ_DECEL 40.0f              // 梯形轨迹控制模式下减速度 [(turn/s)/s]
#define MOTOR_VEL_LIMIT 168.0f              // 转速限制值 [turn/s] (turn/s = 最大转速rpm / 60) 一般设置为电机最大允许转速
#define MOTOR_VOLTAGE_LIMIT 12.0f           // 电机电压限制值 [V]
#define MOTOR_CURRENT_LIMIT 6.0f            // 电机电流限制值 [A] (该值限制的是实际运行中电机的相电流 foc->i_q_filt) 一般设置为电机最大允许电流
#define MOTOR_CURRENT_CTRL_P_GAIN 0.0f      // 电流环增益，在校准过程中自动计算得出，也可自行设置 (Auto)
#define MOTOR_CURRENT_CTRL_I_GAIN 0.0f      // 电流环积分增益，在校准过程中自动计算得出，也可自行设置 (Auto)
#define MOTOR_CURRENT_CTRL_BANDWIDTH 230.0f // 电流环带宽[rad/s]，范围 (2~60000)
#define MOTOR_STALL_CURRENT 6.5f            // 堵转电流，电机堵转时的电流
#define MOTOR_INPUT_CURRENT 5.0f            // 目标电流
#define MOTOR_INPUT_VELOCITY 0.0f           // 目标转速
#define MOTOR_INPUT_POSITION 0.0f           // 目标位置

// PID参数
#define MOTOR_IQ_PID_KP 0.0f
#define MOTOR_IQ_PID_KI 0.0f
#define MOTOR_IQ_PID_KD 0.0f
#define MOTOR_ID_PID_KP 0.0f
#define MOTOR_ID_PID_KI 0.0f
#define MOTOR_ID_PID_KD 0.0f
#define MOTOR_VEL_PID_KP 0.40f
#define MOTOR_VEL_PID_KI 0.0040f // 不可太大，否则会导致电机激烈抖动，以后想做无感时切换就不顺畅了
#define MOTOR_VEL_PID_KD 0.0f
#define MOTOR_POS_PID_KP 165.0f // 可以看回正时速度是否快准狠
#define MOTOR_POS_PID_KI 0.0f
#define MOTOR_POS_PID_KD 0.0f

/*单电流环时：      限幅值没有什么约束但是不能大于母线电压/根号3，否则会导致电流环输出超限
**电流速度环时：    因为要先经过速度环，然后再把速度环的值传给电流环，所以电流环的限幅为母线电压/根号3，速度环的限幅为最大电流值
**电流速度位置环时：把位置环的限幅设为电机最大转速即可，电流环的限幅为母线电压/根号3，速度环的限幅为最大电流值
**
**位置环的限幅不可过大，不然会有很大的过充一下子就报警告了！！！
*/
#define CURRENT_PID_MAX_OUT ((BATVEL) / _SQRT3)
#define IQ_PID_MAX_OUT CURRENT_PID_MAX_OUT
#define IQ_PID_MAX_IOUT CURRENT_PID_MAX_OUT
#define ID_PID_MAX_OUT CURRENT_PID_MAX_OUT
#define ID_PID_MAX_IOUT CURRENT_PID_MAX_OUT
#define VEL_PID_MAX_OUT MOTOR_CURRENT_LIMIT
#define VEL_PID_MAX_IOUT MOTOR_CURRENT_LIMIT
#define POS_PID_MAX_OUT (MOTOR_VEL_LIMIT / 1.3f)
#define POS_PID_MAX_IOUT (MOTOR_VEL_LIMIT / 1.3f)
#endif

#if MOTOR_PM3510 // 电机类型
#if PRE_CALIBRATED
#define MPTOR_P 11u              // 电机极对数
#define MOTOR_RS 1.72955227f     // 相电阻
#define MOTOR_LS 0.000661760569f // 相电感
#define MOTOR_FLUX 0.0f          // 磁链
#define MOTOR_OFFSET 7963        // 电机偏移量
#define MOTOR_DIRECTION CW       // 电机方向
#else
#define MPTOR_P 0u              // 电机极对数
#define MOTOR_RS 0.0f           // 相电阻
#define MOTOR_LS 0.0f           // 相电感
#define MOTOR_OFFSET 0.0f       // 电机偏移量
#define MOTOR_DIRECTION UNKNOWN // 电机方向
#endif

// 电机参数相关定义
#define MOTOR_INERTIA 0.0001f               // 转动惯量 [A/(turn/s^2)]，含义为电机轴以1转每秒的加速度运转时需要提供的用于加速消耗的电流，需要根据电机轴重量和所带负载来进行调试。
#define MOTOR_CURRENT_RAMP_RATE 0.001f      // 电流爬升力矩 [Nm/s]
#define MOTOR_VEL_RAMP_RATE 50.0f           // 转速爬升速度 [(turn/s)/s]
#define MOTOR_TRAJ_VEL 20.0f                // 梯形轨迹控制模式下最大转速 [turn/s] (设定值应当小于或等于 MOTOR_VEL_LIMIT)
#define MOTOR_TRAJ_ACCEL 5.0f               // 梯形轨迹控制模式下加速度 [(turn/s)/s]
#define MOTOR_TRAJ_DECEL 5.0f               // 梯形轨迹控制模式下减速度 [(turn/s)/s]
#define MOTOR_VEL_LIMIT 20.8f               // 转速限制值 [turn/s] (turn/s = 最大转速rpm / 60) 一般设置为电机最大允许转速
#define MOTOR_VOLTAGE_LIMIT 12.0f           // 电机电压限制值 [V]
#define MOTOR_CURRENT_LIMIT 0.6f            // 电机电流限制值 [A] (该值限制的是实际运行中电机的相电流 foc->i_q_filt) 一般设置为电机最大允许电流
#define MOTOR_CURRENT_CTRL_P_GAIN 0.0f      // 电流环增益，在校准过程中自动计算得出，也可自行设置 (Auto)
#define MOTOR_CURRENT_CTRL_I_GAIN 0.0f      // 电流环积分增益，在校准过程中自动计算得出，也可自行设置 (Auto)
#define MOTOR_CURRENT_CTRL_BANDWIDTH 230.0f // 电流环带宽[rad/s]，范围 (2~60000)
#define MOTOR_STALL_CURRENT 0.8f            // 堵转电流，电机堵转时的电流
#define MOTOR_INPUT_CURRENT 0.5f            // 目标电流
#define MOTOR_INPUT_VELOCITY 10.0f          // 目标转速
#define MOTOR_INPUT_POSITION 0.0f           // 目标位置

// PID参数
#define MOTOR_IQ_PID_KP 0.0f
#define MOTOR_IQ_PID_KI 0.0f
#define MOTOR_IQ_PID_KD 0.0f
#define MOTOR_ID_PID_KP 0.0f
#define MOTOR_ID_PID_KI 0.0f
#define MOTOR_ID_PID_KD 0.0f
#define MOTOR_VEL_PID_KP 0.3f
#define MOTOR_VEL_PID_KI 0.003f // 不可太大，否则会导致电机激烈抖动，以后想做无感时切换就不顺畅了
#define MOTOR_VEL_PID_KD 0.0f
#define MOTOR_POS_PID_KP 100.0f // 可以看回正时速度是否快准狠
#define MOTOR_POS_PID_KI 0.0f
#define MOTOR_POS_PID_KD 0.0f

#define CURRENT_PID_MAX_OUT ((BATVEL) / _SQRT3)
#define IQ_PID_MAX_OUT CURRENT_PID_MAX_OUT
#define IQ_PID_MAX_IOUT CURRENT_PID_MAX_OUT
#define ID_PID_MAX_OUT CURRENT_PID_MAX_OUT
#define ID_PID_MAX_IOUT CURRENT_PID_MAX_OUT
#define VEL_PID_MAX_OUT MOTOR_CURRENT_LIMIT
#define VEL_PID_MAX_IOUT MOTOR_CURRENT_LIMIT
#define POS_PID_MAX_OUT (MOTOR_VEL_LIMIT - 5.0f)
#define POS_PID_MAX_IOUT (MOTOR_VEL_LIMIT - 5.0f)

#endif

// Calib
#define CURRENT_MAX_CALIB 0.5f // 校准最大电流限制 (如果电机校准的时候顿挫感严重或者动一半就不动了，就需要调大这个值)
#define VOLTAGE_MAX_CALIB 2.0f // 校准最大电压限制(如果电机校准的时候，电机没有发出“哔”的声音(就像普通电调启动的声音一样)，就需要调大这个值)

#define OFFSET_LUT_NUM 128               // 偏移量LUT数
#define COGGING_MAP_NUM 3000             // 扇区数
#define CALB_SPEED M_2PI                 // 电机转速
#define MOTOR_POLE_PAIRS_MAX 20          // 最大检测极对数（可以增加但是要注意堆的大小分配）
#define SAMPLES_PER_PPAIR OFFSET_LUT_NUM // 极对数采样数

// Control
#define BATVEL_MAX_LIMIT (BATVEL * 1.2f)         // 电池过压报错阈值
#define BATVEL_MIN_LIMIT (BATVEL / 1.5f)         // 电池低压报错阈值
#define TORQUE_MAX_LIMIT 0.3f                    // 电机转矩常数限制
#define SPEED_MAX_LIMIT (MOTOR_VEL_LIMIT * 1.2f) // 超速限制
#define TEMP_MAX_LIMIT 65.0f                     // 红温限制

// 基础配置
#define TIMER1_CLK_MHz 168                                                                                         // 定时器时钟频率
#define PWM_FREQUENCY 20000                                                                                        // PWM频率20KHz
#define PWM_MEASURE_PERIOD (float)(1.0f / (float)PWM_FREQUENCY)                                                    // PWM周期
#define CURRENT_MEASURE_HZ PWM_FREQUENCY                                                                           // 电流频率
#define CURRENT_MEASURE_PERIOD (float)(1.0f / (float)CURRENT_MEASURE_HZ)                                           // 电流周期
#define PWM_PERIOD_CYCLES (uint16_t)((TIMER1_CLK_MHz * (uint32_t)1000000u / ((uint32_t)(PWM_FREQUENCY))) & 0xFFFE) // 定时器1的周期数
#define PWM_ARR (uint16_t)(PWM_PERIOD_CYCLES / 2u)                                                                 // 3500
#define VEL_POS_HZ 5000                                                                                            // 速度位置环频率
#define VEL_POS_PERIOD (float)(1.0f / (float)VEL_POS_HZ)                                                           // 速度位置环周期
#define CURRENT_CALIBRATION_DURATION 1000u                                                                         // 电流校准时间
#define DEADTIME_COMP 20u                                                                                          // 死区时间补偿
#define TS 1.0f                                                                                                    // 采样周期 (将占空比限制在0-1之间)

// 硬件配置
#define V_REG 1.65f                                                             // ADC参考电压
#define CURRENT_SHUNT_RES 0.001f                                                // 电流采样电阻
#define CURRENT_AMP_GAIN 50.0f                                                  // 电流放大倍数
#define VIN_R1 1000.0f                                                          // 电压采样分压电阻
#define VIN_R2 10000.0f                                                         // 电压采样分压电阻
#define FAC_CURRENT ((3.3f / 4095.0f) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN)) // 电流放大倍数
#define VOLTAGE_TO_ADC_FACTOR (((VIN_R2 + VIN_R1) / VIN_R1) * (3.3f / 4095.0f)) // 电压放大倍数

// 设置PWM占空比宏定义
#define set_dtc_a(value) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, value) // 设置A相PWM占空比
#define set_dtc_b(value) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, value) // 设置B相PWM占空比
#define set_dtc_c(value) __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, value) // 设置C相PWM占空比

#pragma pack(1)
// 扇区枚举
typedef enum
{
    SECTOR_1 = 1,
    SECTOR_2,
    SECTOR_3,
    SECTOR_4,
    SECTOR_5,
    SECTOR_6
} svpwm_sector_t;

typedef struct
{
    float vbus;     // 母线电压
    float inv_vbus; // 母线电压倒数，用于计算电流和电压
    float theta;    // 角度
    float i_q_filt, i_d_filt, i_bus, i_bus_filt, power_filt;

    float current_ctrl_integral_d, current_ctrl_integral_q; // Current error integrals
    float current_ctrl_p_gain;                              // (Auto)
    float current_ctrl_i_gain;                              // (Auto)
    // 扇区信息（用于SVPWM）
    int sector; // 当前扇区，用于SVPWM调制

    // 坐标系电压、电流分量
    float i_a; // A 相电流
    float i_b; // B 相电流
    float i_c; // C 相电流

    float v_a; // A 相电压
    float v_b; // B 相电压
    float v_c; // C 相电压

    float i_d; // D 坐标系电流（FOC中的直轴电流）
    float i_q; // Q 坐标系电流（FOC中的交轴电流）

    float v_d; // D 坐标系电压（FOC中的直轴电压） 控制扭矩
    float v_q; // Q 坐标系电压（FOC中的交轴电压） 弱磁控制

    float i_alpha; // Alpha 坐标系电流（Clarke变换后的电流）
    float i_beta;  // Beta  坐标系电流（Clarke变换后的电流）

    float v_alpha; // Alpha 坐标系电压（逆Clarke变换后的电压）
    float v_beta;  // Beta  坐标系电压（逆Clarke变换后的电压）

    float dtc_a; // A 相 PWM 占空比
    float dtc_b; // B 相 PWM 占空比
    float dtc_c; // C 相 PWM 占空比

    float vd_set; // d 轴电压设置
    float vq_set; // q 轴电压设置
    float id_set; // d 轴电流设置
    float iq_set; // q 轴电流设置

    float sin_val; // 电角度的正弦值，用于FOC计算
    float cos_val; // 电角度的余弦值，用于FOC计算
} FOC_DATA;
#pragma pack()

extern FOC_DATA foc_data;

// 函数声明部分
void FOC_reset(FOC_DATA *foc);                                                             // 重置FOC数据
void Foc_Pwm_LowSides(void);                                                               // 启动FOC 但占空比为零
void Foc_Pwm_Start(void);                                                                  // 启动FOC PWM
void Foc_Pwm_Stop(void);                                                                   // 停止FOC PWM
void SetPwm(FOC_DATA *foc);                                                                // 设置PWM占空比
void Sin_Cos_Val(FOC_DATA *foc);                                                           // 计算正弦和余弦值
void Clarke(FOC_DATA *foc);                                                                // Clarke变换
void Park(FOC_DATA *foc);                                                                  // Park变换
void Inv_clarke(FOC_DATA *foc);                                                            // 逆Clarke变换
void Inv_Park(FOC_DATA *foc);                                                              // 逆Park变换
void Svpwm_Midpoint(FOC_DATA *foc);                                                        // SVPWM调制
void Svpwm_Sector(FOC_DATA *foc);                                                          // SVPWM扇区选择
void commonFOCOperations(FOC_DATA *foc);                                                   // 通用FOC操作
void commonInverseFOCOperations(FOC_DATA *foc);                                            // 通用逆FOC操作
void setPhaseVoltage(FOC_DATA *foc, float Vd_set, float Vq_set, float phase);              // 设置相电压和相电流
void FOC_voltage(FOC_DATA *foc, float Vd_set, float Vq_set, float phase);                  // 电压控制
void FOC_current(FOC_DATA *foc, float Id_set, float Iq_set, float phase, float phase_vel); // 电流控制
#endif                                                                                     // BLDCMotor_H
