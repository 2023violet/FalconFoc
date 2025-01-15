/********************************************************************************
 * @file        : cmd_task.c
 * @author      : Zhang jia ming
 * @brief       : 发布&订阅应用任务
 * @version     : V1.0
 * @date        : 2024 - 11 - 12
 *
 * @details:
 *  - 发布&订阅应用任务
 *
 * @note:
 *  - 无
 *
 * @history:
 *  V1.0:
 *    - 修改注释，完善功能
 *
 * Copyright (c) 2024 Zhang jia ming. All rights reserved.
 ********************************************************************************/

// app
#include "cmd_task.h"
#include "robot_def.h"
// module
#include "can_driver.h"
#include "FOCMotor.h"
// bsp
#include "arm_math.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"

static FDCANCommInstance *ins;
static Cmd_Rx_s cmd_rx;
static Cmd_Tx_s cmd_tx;
static float last_position = 0.0f;

void RobotCMDInit(void)
{
    FDCANComm_Init_Config_s config = {
        .can_config                = {
            .fdcan_handle          = &hfdcan1,
            .tx_id                 = 0x101,
            .rx_id                 = 0x201
            },
        .send_data_len             = sizeof(cmd_tx),
        .recv_data_len             = sizeof(cmd_rx)
        };
    ins = FDCANCommInit(&config);
}

void RobotCMDTask(void)
{
    static uint8_t loop_count;

    cmd_rx = *(Cmd_Rx_s *)FDCANCommGet(ins);

    if (cmd_rx.cmd_id == CMD_ID_GET_POSITION)
    {
        motor_data.state.Control_Mode = CONTROL_MODE_POSITION_RAMP;
        // 检查当前接收到的位置数据是否与上一次不同
        if (cmd_rx.cmd_rx_data != last_position)
        {
            motor_data.Controller.input_position = cmd_rx.cmd_rx_data;
            motor_data.Controller.input_updated  = true;
            last_position                        = cmd_rx.cmd_rx_data; 
        }
        else
        {
            motor_data.Controller.input_updated  = false;
        }
    }
    else if (cmd_rx.cmd_id == CMD_ID_GET_VELOCITY)
    {
        motor_data.state.Control_Mode = CONTROL_MODE_VELOCITY_RAMP;
        motor_data.Controller.input_velocity = cmd_rx.cmd_rx_data;
    }
    else if (cmd_rx.cmd_id == CMD_ID_GET_TORQUE)
    {
        motor_data.Controller.input_torque = cmd_rx.cmd_rx_data;
    }
    else if (cmd_rx.cmd_id == CMD_ID_CLEAR_ERRORS)
    {
        motor_data.state.State_Mode = STATE_MODE_IDLE;
        motor_data.state.Fault_State = FAULT_STATE_NORMAL;
    }

     if (++loop_count >= 2)
     {
         loop_count = 0;

		 //自行添加需要发送的内容
		 cmd_tx.cmd_tx_data = 0.0f;
		 FDCANCommSend(ins, (void *)&cmd_tx);
    }
}
