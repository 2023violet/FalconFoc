#ifndef PID_H_
#define PID_H_

#include "general_def.h" // 包含通用定义的头文件

enum PID_MODE
{
  PID_POSITION = 0, // 位置式
  PID_DELTA         // 增量式
};

typedef struct
{
  uint8_t mode;

  float Kp;
  float Ki;
  float Kd;

  float max_out;  // 最大输出
  float max_iout; // 最大积分输出

  float set;
  float fdb;

  float out;
  float Pout;
  float Iout;
  float Dout;
  float Dbuf[3];  // 微分项 0最新 1上一次 2上上次
  float error[3]; // 误差项 0最新 1上一次 2上上次

} PidTypeDef;

/**
 * @brief          pid struct data init
 * @param[out]     pid: PID struct data point
 * @param[in]      mode: PID_POSITION: normal pid
 *                 PID_DELTA: delta pid
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid max out
 * @param[in]      max_iout: pid max iout
 * @retval         none
 */
/**
 * @brief          pid struct data init
 * @param[out]     pid: PID结构数据指针
 * @param[in]      mode: PID_POSITION:普通PID
 *                 PID_DELTA: 差分PID
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid最大输出
 * @param[in]      max_iout: pid最大积分输出
 * @retval         none
 */
extern void PID_Init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);

/**
 * @brief          pid calculate
 * @param[out]     pid: PID struct data point
 * @param[in]      ref: feedback data
 * @param[in]      set: set point
 * @retval         pid out
 */
/**
 * @brief          pid计算
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @retval         pid输出
 */
extern float PID_Calc(PidTypeDef *pid, float ref, float set);

/**
 * @brief          pid out clear
 * @param[out]     pid: PID struct data point
 * @retval         none
 */
/**
 * @brief          pid 输出清除
 * @param[out]     pid: PID结构数据指针
 * @retval         none
 */
extern void PID_clear(PidTypeDef *pid);

#endif /* CODE_INCLUDE_PID_H_ */
