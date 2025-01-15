#include "vofa.h"
#include "usbd_cdc_if.h"
#include "motor_task.h"
#include "robot_def.h"

#define MAX_TXBUFFER_SIZE 512        // 定义发送缓冲区大小
#define MAX_RXBUFFER_SIZE 512        // 定义接收缓冲区大小
uint8_t send_buf[MAX_TXBUFFER_SIZE]; // 发送缓冲区
uint8_t receive_buf[MAX_RXBUFFER_SIZE];
uint16_t cnt = 0; // 当前缓冲区计数

/***********************************************************************
 * @brief:      vofa_start(void)
 * @param:      void
 * @retval:     void
 * @details:    发送数据给上位机
 ***********************************************************************/
void vofa_start(void)
{
    Vofa_Packet(); // 生成并发送数据包
}

/***********************************************************************
 * @brief:      vofa_transmit(const uint8_t* buf, uint16_t len)
 * @param:      buf: 数据缓冲区指针
 * @param:      len: 数据长度
 * @retval:     void
 * @details:    发送数据到上位机，使用 USART 或 USB
 ***********************************************************************/
#define USART_OR_CDC 1 // 0: 使用 USART 发送，1: 使用 USB 发送
void vofa_transmit(const uint8_t *buf, uint16_t len)
{
    if (len > 0 && len <= MAX_TXBUFFER_SIZE) // 检查长度
    {
#if USART_OR_CDC == 0
        memcpy(send_buf, buf, len);
        HAL_UART_Transmit_DMA(&huart3, send_buf, len); // 使用 DMA 发送数据
#elif USART_OR_CDC == 1
        CDC_Transmit_FS((uint8_t *)buf, len);
#endif
    }
}

/***********************************************************************
 * @brief:      vofa_send_data(uint8_t num, float data)
 * @param[in]:   num: 数据编号
 * @param[in]:   data: 浮点数据
 * @retval:     void
 * @details:    将浮点数据拆分成单字节并存储到发送缓冲区
 ***********************************************************************/
void vofa_send_data(uint8_t num, float data)
{
    if (cnt + 4 < MAX_TXBUFFER_SIZE)
    {                                  // 检查缓冲区是否足够
        send_buf[cnt++] = byte0(data); // 拆分浮点数为字节
        send_buf[cnt++] = byte1(data);
        send_buf[cnt++] = byte2(data);
        send_buf[cnt++] = byte3(data);
    }
}

/***********************************************************************
 * @brief:      vofa_sendframetail(void)
 * @param:      NULL
 * @retval:     void
 * @details:    给数据包发送帧尾并发送整个数据包
 ***********************************************************************/
void vofa_sendframetail(void)
{
    if (cnt + 4 < MAX_TXBUFFER_SIZE)
    {                           // 检查缓冲区是否足够
        send_buf[cnt++] = 0x00; // 添加帧尾
        send_buf[cnt++] = 0x00;
        send_buf[cnt++] = 0x80;
        send_buf[cnt++] = 0x7f;

        vofa_transmit(send_buf, cnt); // 发送整个数据包
        cnt = 0;                      // 重置计数器
    }
}

/***********************************************************************
 * @brief:      Vofa_Packet(void)
 * @param:      NULL
 * @retval:     void
 * @details:    生成并发送数据包
 ***********************************************************************/
void Vofa_Packet(void)
{
    vofa_send_data(0, motor_data.components.foc->i_a);
    vofa_send_data(1, motor_data.components.foc->i_b);
    vofa_send_data(2, motor_data.components.foc->i_c);
    vofa_send_data(3, motor_data.components.foc->i_q);
    vofa_send_data(4, motor_data.components.foc->i_d);
    vofa_send_data(5, motor_data.components.foc->iq_set);
    vofa_send_data(6, motor_data.components.foc->id_set);
    vofa_send_data(7, motor_data.components.encoder->vel_estimate_);
    vofa_send_data(8, motor_data.components.encoder->pos_estimate_);
    vofa_send_data(9, motor_data.components.encoder->phase_);
    vofa_send_data(10, motor_data.components.current->Temp_Result);
    vofa_send_data(11, motor_data.components.foc->vbus);
    vofa_sendframetail(); // 发送帧尾并传输数据包
}

/**USB信息处理**/
static float vofa_cmd_parse(uint8_t *cmdBuf, const char *arg)
{
    return atof((char *)cmdBuf + strlen(arg));
}

/**接收中断服务函数**/
void vofa_Receive(uint8_t *buf, uint16_t len)
{
    memcpy((char *)receive_buf, buf, len);

    char *recvStr = (char *)receive_buf;

    if (strstr(recvStr, "set_Iq="))
    {
        motor_data.components.foc->iq_set = vofa_cmd_parse(receive_buf, "set_Iq=");
    }

    if (strstr(recvStr, "set_Id="))
    {
        motor_data.components.foc->id_set = vofa_cmd_parse(receive_buf, "set_Id=");
    }

    if (strstr(recvStr, "set_torque="))
    {
        motor_data.Controller.input_torque = vofa_cmd_parse(receive_buf, "set_torque=");
    }

    if (strstr(recvStr, "set_vel="))
    {
        motor_data.Controller.input_velocity = vofa_cmd_parse(receive_buf, "set_vel=");
    }

    if (strstr(recvStr, "set_pos="))
    {
        motor_data.Controller.input_position = vofa_cmd_parse(receive_buf, "set_pos=");
        motor_data.Controller.input_updated = true;
    }

    if (strstr(recvStr, "set_current_ctrl_bw="))
    {
        motor_data.Controller.current_ctrl_bandwidth = vofa_cmd_parse(receive_buf, "set_current_ctrl_bw=");
    }

    if (strstr(recvStr, "set_vel_kp="))
    {
        motor_data.VelPID.Kp = vofa_cmd_parse(receive_buf, "set_vel_kp=");
    }

    if (strstr(recvStr, "set_vel_ki="))
    {
        motor_data.VelPID.Ki = vofa_cmd_parse(receive_buf, "set_vel_ki=");
    }

    if (strstr(recvStr, "set_pos_kp="))
    {
        motor_data.PosPID.Kp = vofa_cmd_parse(receive_buf, "set_pos_kp=");
    }

    if (strstr(recvStr, "set_ctrl_mode="))
    {
        float ctrlModeVal = vofa_cmd_parse(receive_buf, "set_ctrl_mode=");
        int ctrlModeInt = (int)ctrlModeVal;
        if (ctrlModeInt >= CONTROL_MODE_OPEN && ctrlModeInt <= CONTROL_MODE_POSITION_RAMP)
        {
            motor_data.state.Control_Mode = (CONTROL_MODE)ctrlModeInt;
        }
    }

    if (strstr(recvStr, "calib="))
    {
        float calib_enable = vofa_cmd_parse(receive_buf, "calib=");
        if (calib_enable == 1)
        {
            PID_clear(&motor_data.IqPID);
            PID_clear(&motor_data.IdPID);
            PID_clear(&motor_data.VelPID);
            PID_clear(&motor_data.PosPID);
            FOC_reset(motor_data.components.foc);
            Foc_Pwm_LowSides();
            motor_data.state.State_Mode = STATE_MODE_DETECTING;
            motor_data.state.Sub_State = CURRENT_CALIBRATING;
            Init_Motor_Calib(&motor_data);
        }
    }

    // if (strstr(recvStr, "motor_enable="))
    // {
    //     float motor_enable = vofa_cmd_parse(receive_buf, "motor_enable=");
    // }
}
