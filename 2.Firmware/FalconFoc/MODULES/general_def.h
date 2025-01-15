#ifndef GENERAL_DEF_H
#define GENERAL_DEF_H

#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include <math.h>

typedef unsigned char bool_t; // 布尔类型

#define M_PI (3.14159265358979323846f)                   // 圆周率
#define M_2PI (6.28318530717958647692f)                  // 2倍圆周率
#define M_3PI_2 4.71238898038469f                        // 3/2倍圆周率
#define _SQRT3 (1.7320508075688772935f)                  // 3的平方根
#define _SQRT3_2 (0.86602540378443864f)                  // 3的平方根的一半
#define ONE_BY_SQRT2 (0.7071067811865475f)               // 1除以2的平方根
#define ONE_BY_SQRT3 (0.57735026919f)                    // 1除以3的平方根
#define TWO_BY_SQRT3 (1.15470053838f)                    // 2除以3的平方根
#define FOC_EANGLE_TO_ERADIN (0.0174532925199432957692f) // 角度值转化弧度制系数

#define SQ(x) ((x) * (x))                                                         // 求平方
#define NORM2_f(x, y) (sqrtf(SQ(x) + SQ(y)))                                      // 求模
#define DEG2RAD_f(deg) ((deg) * (float)(M_PI / 180.0f))                           // 角度值转化弧度制
#define RAD2DEG_f(rad) ((rad) * (float)(180.0f / M_PI))                           // 弧度值转化角度制
#define RPM2RADPS_f(rpm) ((rpm) * (float)((2.0f * M_PI) / 60.0f))                 // 转速值转化角速度值
#define RADPS2RPM_f(rad_per_sec) ((rad_per_sec) * (float)(60.0f / (2.0f * M_PI))) // 角速度值转化转速值

#define ABS(a) ((a > 0.0f) ? (a) : (-a))                   // 求绝对值
#define min(a, b) (((a) < (b)) ? (a) : (b))                // 求最小值
#define max(a, b) (((a) > (b)) ? (a) : (b))                // 求最大值
#define CLAMP(x, lower, upper) (min(upper, max(x, lower))) // 求值范围
#define SIGN(x) (((x) < 0.0f) ? -1.0f : 1.0f)              // 求符号
#define UTILS_LP_FAST(value, sample, filter_constant) (value -= (filter_constant) * (value - (sample)))
#define UTILS_LP_MOVING_AVG_APPROX(value, sample, N) UTILS_LP_FAST(value, sample, 2.0f / ((N) + 1.0f))

static inline int mod(int dividend, int divisor)
{
  int r = dividend % divisor;
  return (r < 0) ? (r + divisor) : r;
}

static inline void int_to_data(int val, uint8_t *data)
{
  data[0] = *(((uint8_t *)(&val)) + 0);
  data[1] = *(((uint8_t *)(&val)) + 1);
  data[2] = *(((uint8_t *)(&val)) + 2);
  data[3] = *(((uint8_t *)(&val)) + 3);
}

static inline int data_to_int(uint8_t *data)
{
  int tmp_int;
  *(((uint8_t *)(&tmp_int)) + 0) = data[0];
  *(((uint8_t *)(&tmp_int)) + 1) = data[1];
  *(((uint8_t *)(&tmp_int)) + 2) = data[2];
  *(((uint8_t *)(&tmp_int)) + 3) = data[3];
  return tmp_int;
}

static inline void float_to_data(float val, uint8_t *data)
{
  data[0] = *(((uint8_t *)(&val)) + 0);
  data[1] = *(((uint8_t *)(&val)) + 1);
  data[2] = *(((uint8_t *)(&val)) + 2);
  data[3] = *(((uint8_t *)(&val)) + 3);
}

static inline float data_to_float(uint8_t *data)
{
  float tmp_float;
  *(((uint8_t *)(&tmp_float)) + 0) = data[0];
  *(((uint8_t *)(&tmp_float)) + 1) = data[1];
  *(((uint8_t *)(&tmp_float)) + 2) = data[2];
  *(((uint8_t *)(&tmp_float)) + 3) = data[3];
  return tmp_float;
}

static inline void float4_to_data7(float val, uint8_t *data)
{
  data[4] = *(((uint8_t *)(&val)) + 0);
  data[5] = *(((uint8_t *)(&val)) + 1);
  data[6] = *(((uint8_t *)(&val)) + 2);
  data[7] = *(((uint8_t *)(&val)) + 3);
}

static inline uint32_t cpu_enter_critical(void)
{
  //    uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return 1;
}

static inline void cpu_exit_critical(uint32_t priority_mask)
{
  //__set_PRIMASK(priority_mask);
  __enable_irq();
}

static inline float fmodf_pos(float x, float y)
{
  float out = fmodf(x, y);
  if (out < 0.0f)
  {
    out += y;
  }
  return out;
}

static inline float wrap_pm(float x, float pm_range)
{
  return fmodf_pos(x + pm_range, 2.0f * pm_range) - pm_range;
}

static inline float wrap_pm_pi(float theta)
{
  return wrap_pm(theta, M_PI);
}

static inline void fast_sincos(float angle, float *sin, float *cos)
{
  // always wrap input angle to -PI..PI
  angle = wrap_pm_pi(angle);

  // compute sine
  if (angle < 0.0f)
  {
    *sin = 1.27323954f * angle + 0.405284735f * angle * angle;

    if (*sin < 0.0f)
    {
      *sin = 0.225f * (*sin * -*sin - *sin) + *sin;
    }
    else
    {
      *sin = 0.225f * (*sin * *sin - *sin) + *sin;
    }
  }
  else
  {
    *sin = 1.27323954f * angle - 0.405284735f * angle * angle;

    if (*sin < 0.0f)
    {
      *sin = 0.225f * (*sin * -*sin - *sin) + *sin;
    }
    else
    {
      *sin = 0.225f * (*sin * *sin - *sin) + *sin;
    }
  }

  // compute cosine: sin(x + PI/2) = cos(x)
  angle += 0.5f * M_PI;
  if (angle > M_PI)
  {
    angle -= 2.0f * M_PI;
  }

  if (angle < 0.0f)
  {
    *cos = 1.27323954f * angle + 0.405284735f * angle * angle;

    if (*cos < 0.0f)
    {
      *cos = 0.225f * (*cos * -*cos - *cos) + *cos;
    }
    else
    {
      *cos = 0.225f * (*cos * *cos - *cos) + *cos;
    }
  }
  else
  {
    *cos = 1.27323954f * angle - 0.405284735f * angle * angle;

    if (*cos < 0.0f)
    {
      *cos = 0.225f * (*cos * -*cos - *cos) + *cos;
    }
    else
    {
      *cos = 0.225f * (*cos * *cos - *cos) + *cos;
    }
  }
}

static inline void clarke_transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta)
{
  *Ialpha = Ia;
  *Ibeta = (Ib - Ic) * ONE_BY_SQRT3;
}

static inline void park_transform(float Ialpha, float Ibeta, float Theta, float *Id, float *Iq)
{
  float s, c;
  fast_sincos(Theta, &s, &c);
  *Id = Ialpha * c + Ibeta * s;
  *Iq = -Ialpha * s + Ibeta * c;
}

static inline void inverse_park(float mod_d, float mod_q, float Theta, float *mod_alpha, float *mod_beta)
{
  float s, c;
  fast_sincos(Theta, &s, &c);
  *mod_alpha = mod_d * c - mod_q * s;
  *mod_beta = mod_d * s + mod_q * c;
}

static inline int svm(float alpha, float beta, float *tA, float *tB, float *tC)
{
  int Sextant;

  if (beta >= 0.0f)
  {
    if (alpha >= 0.0f)
    {
      // quadrant I
      if (ONE_BY_SQRT3 * beta > alpha)
        Sextant = 2; // sextant v2-v3
      else
        Sextant = 1; // sextant v1-v2
    }
    else
    {
      // quadrant II
      if (-ONE_BY_SQRT3 * beta > alpha)
        Sextant = 3; // sextant v3-v4
      else
        Sextant = 2; // sextant v2-v3
    }
  }
  else
  {
    if (alpha >= 0.0f)
    {
      // quadrant IV
      if (-ONE_BY_SQRT3 * beta > alpha)
        Sextant = 5; // sextant v5-v6
      else
        Sextant = 6; // sextant v6-v1
    }
    else
    {
      // quadrant III
      if (ONE_BY_SQRT3 * beta > alpha)
        Sextant = 4; // sextant v4-v5
      else
        Sextant = 5; // sextant v5-v6
    }
  }

  switch (Sextant)
  {
  // sextant v1-v2
  case 1:
  {
    // Vector on-times
    float t1 = alpha - ONE_BY_SQRT3 * beta;
    float t2 = TWO_BY_SQRT3 * beta;

    // PWM timings
    *tA = (1.0f - t1 - t2) * 0.5f;
    *tB = *tA + t1;
    *tC = *tB + t2;
  }
  break;

  // sextant v2-v3
  case 2:
  {
    // Vector on-times
    float t2 = alpha + ONE_BY_SQRT3 * beta;
    float t3 = -alpha + ONE_BY_SQRT3 * beta;

    // PWM timings
    *tB = (1.0f - t2 - t3) * 0.5f;
    *tA = *tB + t3;
    *tC = *tA + t2;
  }
  break;

  // sextant v3-v4
  case 3:
  {
    // Vector on-times
    float t3 = TWO_BY_SQRT3 * beta;
    float t4 = -alpha - ONE_BY_SQRT3 * beta;

    // PWM timings
    *tB = (1.0f - t3 - t4) * 0.5f;
    *tC = *tB + t3;
    *tA = *tC + t4;
  }
  break;

  // sextant v4-v5
  case 4:
  {
    // Vector on-times
    float t4 = -alpha + ONE_BY_SQRT3 * beta;
    float t5 = -TWO_BY_SQRT3 * beta;

    // PWM timings
    *tC = (1.0f - t4 - t5) * 0.5f;
    *tB = *tC + t5;
    *tA = *tB + t4;
  }
  break;

  // sextant v5-v6
  case 5:
  {
    // Vector on-times
    float t5 = -alpha - ONE_BY_SQRT3 * beta;
    float t6 = alpha - ONE_BY_SQRT3 * beta;

    // PWM timings
    *tC = (1.0f - t5 - t6) * 0.5f;
    *tA = *tC + t5;
    *tB = *tA + t6;
  }
  break;

  // sextant v6-v1
  case 6:
  {
    // Vector on-times
    float t6 = -TWO_BY_SQRT3 * beta;
    float t1 = alpha + ONE_BY_SQRT3 * beta;

    // PWM timings
    *tA = (1.0f - t6 - t1) * 0.5f;
    *tC = *tA + t1;
    *tB = *tC + t6;
  }
  break;
  }

  // if any of the results becomes NaN, result_valid will evaluate to false
  int result_valid =
      *tA >= 0.0f && *tA <= 1.0f && *tB >= 0.0f && *tB <= 1.0f && *tC >= 0.0f && *tC <= 1.0f;

  return result_valid ? 0 : -1;
}

#endif // !GENERAL_DEF_H
