#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "stdint.h"

typedef enum
{
  CMD_ID_GET_POSITION = 0x01,
  CMD_ID_GET_VELOCITY = 0x02,
  CMD_ID_GET_TORQUE = 0x03,
  CMD_ID_CLEAR_ERRORS = 0x04,
} Cmd_Id_e;

#pragma pack(1)
typedef struct
{
  int cmd_id;
  float cmd_rx_data;
} Cmd_Rx_s;

typedef struct
{
  float cmd_tx_data;
} Cmd_Tx_s;
#pragma pack()

#endif
