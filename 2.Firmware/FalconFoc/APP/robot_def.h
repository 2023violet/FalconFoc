#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "stdint.h"

#define FC_MOTOR_ID 0x001 // 电机ID

/**
 * @brief 电机CAN帧ID定义
 */
typedef enum
{
  FDCAN_M1_ID = 0x100, // 左前
  FDCAN_M2_ID = 0x1ff, // 右前
  FDCAN_M3_ID = 0x200, // 左后
  FDCAN_M4_ID = 0x2ff, // 右后
} FDCAN_ID_e;

typedef enum
{
  CMD_ID_GET_POSITION = 0x01,
  CMD_ID_GET_VELOCITY = 0x02,
  CMD_ID_GET_TORQUE = 0x03,
  CMD_ID_CLEAR_ERRORS = 0x04,
  CMD_ID_GET_ENALBED = 0x05,
  CMD_ID_GET_STOP = 0x06,
} Cmd_Id_e;

#pragma pack(1)
typedef struct
{
  int cmd_id;
  float cmd_rx_vel;
  float cmd_rx_pos;
} Cmd_Rx_s;

typedef struct
{
  int cmd_tx_state;
  float cmd_tx_vel;
  float cmd_tx_pos;
} Cmd_Tx_s;
#pragma pack()

#endif
