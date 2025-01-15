#ifndef FOCMOTOR_H
#define FOCMOTOR_H

#include "general_def.h"
#include "pid.h"
#include "BLDCMotor.h"
#include "Motor_ADC.h"
#include "mt6816_encoder.h"

/**
 * @brief 闭环类型,如果需要多个闭环,则使用或运算
 *        例如需要速度环和电流环: CURRENT_LOOP | SPEED_LOOP
 *
 *  typedef struct
 *  {
 *    uint8_t current_loop : 1; // 电流闭环
 *    uint8_t speed_loop : 1;   // 速度闭环
 *    uint8_t pos_loop : 1;     // 位置闭环
 *  } CONTROL_MODE;
 *
 *  可以使用位域来做也很方便
 *
 *  他哥的g431怎么不支持二进制字面量
 * typedef enum
 * {
 *     CONTROL_OPEN_LOOP = 0b0000,    // 速度开环
 *     CONTROL_CURRENT_LOOP = 0b0001, // 电流闭环
 *     CONTROL_SPEED_LOOP = 0b0010,   // 速度闭环
 *     CONTROL_POS_LOOP = 0b0100,     // 位置闭环
 *
 *     // 组合模式
 *     CONTROL_CURRENT_SPEED_LOOP = 0b0011,     // 电流闭环和速度闭环
 *     CONTROL_CURRENT_POS_LOOP = 0b0101,       // 电流闭环和位置闭环
 *     CONTROL_SPEED_POS_LOOP = 0b0110,         // 速度闭环和位置闭环
 *     CONTROL_CURRENT_SPEED_POS_LOOP = 0b0111, // 电流闭环、速度闭环和位置闭环
 * } CONTROL_MODE;
 */
typedef enum
{
    CONTROL_MODE_OPEN = 0,          // 速度开环
    CONTROL_MODE_TORQUE = 1,        // 力矩控制模式
    CONTROL_MODE_VELOCITY = 2,      // 速度控制模式
    CONTROL_MODE_POSITION = 3,      // 位置控制模式
    CONTROL_MODE_VELOCITY_RAMP = 4, // 速度梯度控制模式
    CONTROL_MODE_POSITION_RAMP = 5, // 位置梯度控制模式
} CONTROL_MODE;

/**
 * @brief 校准状态模式
 */
typedef enum
{
    SUB_STATE_IDLE = 0,  // 检测空闲状态
    CURRENT_CALIBRATING, // 电流校准状态
    RSLS_CALIBRATING,    // 电阻电感编码器校准状态
    FLUX_CALIBRATING,    // 磁链校准状态
} SUB_STATE;

typedef enum
{
    CS_STATE_IDLE = 0,
    CS_MOTOR_R_START,
    CS_MOTOR_R_LOOP,
    CS_MOTOR_R_END,
    CS_MOTOR_L_START,
    CS_MOTOR_L_LOOP,
    CS_MOTOR_L_END,
    CS_DIR_PP_START,
    CS_DIR_PP_LOOP,
    CS_DIR_PP_END,
    CS_ENCODER_START,
    CS_ENCODER_CW_LOOP,
    CS_ENCODER_CCW_LOOP,
    CS_ENCODER_END,
    CS_REPORT_OFFSET_LUT,
} CS_STATE;

/**
 * @brief 电机状态模式
 */
typedef enum
{
    STATE_MODE_IDLE = 0,  // 空闲模式
    STATE_MODE_DETECTING, // 检测模式
    STATE_MODE_RUNNING,   // 运行模式
    STATE_MODE_GUARD,     // 守护模式
} STATE_MODE;

/**
 * @brief 故障状态模式
 */
typedef enum
{
    FAULT_STATE_NORMAL = 0,       // 正常
    FAULT_STATE_OVER_CURRENT,     // 过流
    FAULT_STATE_OVER_VOLTAGE,     // 过压
    FAULT_STATE_UNDER_VOLTAGE,    // 欠压
    FAULT_STATE_OVER_TEMPERATURE, // 过温
    FAULT_STATE_SPEEDING,         // 超速
} FAULT_STATE;

/**
 * @brief 电机数据类型定义
 */
typedef struct
{
    FOC_DATA *foc;         // 电机数据
    ENCODER_DATA *encoder; // 编码器数据
    CURRENT_DATA *current; // 电流数据
} MOTOR_COMPIONENTS;

/**
 * @brief 电机参数类型定义
 */
typedef struct
{
    float Rs;   // 相电阻
    float Ls;   // 相电感
    float flux; // 磁链
} MOTOR_PARAMETERS;

typedef struct
{
    float inertia;              // [A/(turn/s^2)]
    float torque_ramp_rate;     // [Nm/s]
    float vel_ramp_rate;        // [(turn/s)/s]
    float traj_vel;             // [turn/s]
    float traj_accel;           // [(turn/s)/s]
    float traj_decel;           // [(turn/s)/s]
    float vel_limit;            // [turn/s]
    float current_limit;        // [A]
    float voltage_limit;        // [V]
    float current_ctrl_p_gain;  // (Auto)
    float current_ctrl_i_gain;  // (Auto)
    int current_ctrl_bandwidth; // [rad/s] Current loop bandwidth 100~2000

    float input_position; // 输入位置
    float input_velocity; // 输入速度
    float input_torque;   // 输入力矩
    float input_torque_q; // 输入Q轴力矩
    float input_torque_d; // 输入DD轴力矩

    float pos_setpoint;    // 位置设定值
    float vel_setpoint;    // 速度设定值
    float torque_setpoint; // 力矩设定值

    volatile bool input_updated; // 输入位置更新标志
} MOTOR_CONTROLLER;

/**
 * @brief 电机状态类型定义
 */
typedef struct
{
    STATE_MODE State_Mode;     // 运行状态
    CONTROL_MODE Control_Mode; // 闭环类型
    SUB_STATE Sub_State;       // 检测模式
    CS_STATE Cs_State;         // 检测流程
    FAULT_STATE Fault_State;   // 故障状态
} MOTOR_STATE;

typedef struct
{
    MOTOR_COMPIONENTS components; // 电机组件
    MOTOR_STATE state;            // 电机状态
    MOTOR_PARAMETERS parameters;  // 电机参数
    MOTOR_CONTROLLER Controller;  // 电机控制

    PidTypeDef IqPID;  // 控制电流环IQ的PID控制器
    PidTypeDef IdPID;  // 控制电流环ID的PID控制器
    PidTypeDef VelPID; // 控制电机速度的PID控制器
    PidTypeDef PosPID; // 控制电机位置的PID控制器
} MOTOR_DATA;

extern MOTOR_DATA motor_data;
void Init_Motor_No_Calib(MOTOR_DATA *motor);      // 不校准
void Init_Motor_Calib(MOTOR_DATA *motor);         // 校准
void GetMotorADC1PhaseCurrent(MOTOR_DATA *motor); // 获取电流
void TempResultTask(MOTOR_DATA *motor);           // 温度获取任务
void MotorStateTask(MOTOR_DATA *motor);           // FOC运行任务
void MotorGuardTask(MOTOR_DATA *motor);           // FOC守护任务
#endif                                            // FOCMOTOR_H
