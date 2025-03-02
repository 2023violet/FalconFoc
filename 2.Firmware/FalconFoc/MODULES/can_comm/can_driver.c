#include "can_driver.h" // 包含 CAN 驱动的头文件
#include "string.h"     // 包含字符串处理函数的头文件
#include "stdlib.h"
#include "crc8.h"

/**
 * @brief 重置CAN comm的接收状态和buffer
 *
 * @param ins 需要重置的实例
 */
static void FDCANCommResetRx(FDCANCommInstance *ins)
{
  // 当前已经收到的buffer清零
  memset(ins->raw_recvbuf, 0, ins->cur_recv_len);
  ins->recv_state   = 0; // 接收状态重置
  ins->cur_recv_len = 0; // 当前已经收到的长度重置
}

/**
 * @brief fdcancomm的接收回调函数
 *
 * @param _instance
 */
static void FDCANCommRxCallback(FDCANInstance *_instance)
{
  // id可以将 FDCANCommInstance 的数据传递给 FDCANInstance 的回调函数。
  FDCANCommInstance *comm = (FDCANCommInstance *)_instance->id; // 注意写法,将fdcan instance的id强制转换为FDCANCommInstance*类型

  /* 当前接收状态判断 */
  if (_instance->rx_buff[0] == FDCAN_COMM_HEADER && comm->recv_state == 0) // 之前尚未开始接收且此次包里第一个位置是帧头
  {
    if (_instance->rx_buff[1] == comm->recv_data_len) // 如果这一包里的datalen也等于我们设定接收长度(这是因为暂时不支持动态包长)
    {
      comm->recv_state = 1; // 设置接收状态为1,说明已经开始接收
    }
    else
      return; // 直接跳过即可
  }

  if (comm->recv_state) // 已经收到过帧头
  {
    // 如果已经接收到的长度加上当前一包的长度大于总buf len,说明接收错误
    if (comm->cur_recv_len + _instance->rx_len > comm->recv_buf_len)
    {
      FDCANCommResetRx(comm);
      return; // 重置状态然后返回
    }

    // 直接把当前接收到的数据接到buffer后面
    memcpy(comm->raw_recvbuf + comm->cur_recv_len, _instance->rx_buff, _instance->rx_len);
    comm->cur_recv_len += _instance->rx_len;

    // 收完这一包以后刚好等于总buf len,说明已经收完了
    if (comm->cur_recv_len == comm->recv_buf_len)
    {
      // 如果buff里本tail的位置等于CAN_COMM_TAIL
      if (comm->raw_recvbuf[comm->recv_buf_len - 1] == FDCAN_COMM_TAIL)
      { // 通过校验,复制数据到unpack_data中
        if (comm->raw_recvbuf[comm->recv_buf_len - 2] == crc_8(comm->raw_recvbuf + 2, comm->recv_data_len))
        { // 数据量大的话考虑使用DMA
          memcpy(comm->unpacked_recv_data, comm->raw_recvbuf + 2, comm->recv_data_len);
          comm->update_flag = 1;           // 数据更新flag置为1
          DaemonReload(comm->comm_daemon); // 重载daemon,避免数据更新后一直不被读取而导致数据更新不及时
        }
      }
      FDCANCommResetRx(comm);
      return; // 重置状态然后返回
    }
  }
}

static void FDCANCommLostCallback(void *cancomm)
{
  FDCANCommInstance *comm = (FDCANCommInstance *)cancomm;
  FDCANCommResetRx(comm);
}

FDCANCommInstance *FDCANCommInit(FDCANComm_Init_Config_s *comm_config)
{
  FDCANCommInstance *ins = (FDCANCommInstance *)malloc(sizeof(FDCANCommInstance));
  memset(ins, 0, sizeof(FDCANCommInstance));

  ins->recv_data_len  = comm_config->recv_data_len;
  ins->recv_buf_len   = comm_config->recv_data_len + FDCAN_COMM_OFFSET_BYTES; // head + datalen + crc8 + tail
  ins->send_data_len  = comm_config->send_data_len;
  ins->send_buf_len   = comm_config->send_data_len + FDCAN_COMM_OFFSET_BYTES;
  ins->raw_sendbuf[0] = FDCAN_COMM_HEADER;          // head,直接设置避免每次发送都要重新赋值,下面的tail同理
  ins->raw_sendbuf[1] = comm_config->send_data_len; // datalen
  ins->raw_sendbuf[comm_config->send_data_len + FDCAN_COMM_OFFSET_BYTES - 1] = FDCAN_COMM_TAIL;
  // can instance的设置
  comm_config->can_config.id = ins;                                    // FDCANComm的实例指针作为FDCANInstance的id,回调函数中会用到
  comm_config->can_config.fdcan_module_callback = FDCANCommRxCallback; // 接收回调函数
  ins->can_ins = FDCANRegister(&comm_config->can_config);              // 注册CAN实例

  Daemon_Init_Config_s daemon_config = {
      .callback     = FDCANCommLostCallback,
      .owner_id     = (void *)ins,
      .reload_count = comm_config->daemon_count,
  };
  ins->comm_daemon  = DaemonRegister(&daemon_config);
  return ins;
}

void FDCANCommSend(FDCANCommInstance *instance, uint8_t *data)
{
  static uint8_t crc8;
  static uint8_t send_len;
  // 将data copy到raw_sendbuf中,计算crc8
  memcpy(instance->raw_sendbuf + 2, data, instance->send_data_len);
  crc8 = crc_8(data, instance->send_data_len);
  instance->raw_sendbuf[2 + instance->send_data_len] = crc8;

  // CAN单次发送最大为8字节,如果超过8字节,需要分包发送
  for (size_t i = 0; i < instance->send_buf_len; i += 8)
  { // 如果是最后一包,send len将会小于8,要修改CAN的txconf中的DLC位,调用bsp_can提供的接口即可
    send_len = instance->send_buf_len - i >= 8 ? 8 : instance->send_buf_len - i;
    FDCANSetDLC(instance->can_ins, send_len);
    memcpy(instance->can_ins->tx_buff, instance->raw_sendbuf + i, send_len);
    FDCANTransmit(instance->can_ins, 2);
  }
}

void *FDCANCommGet(FDCANCommInstance *instance)
{
  instance->update_flag = 0; // 读取后将更新flag置为0
  return instance->unpacked_recv_data;
}

uint8_t FDCANCommIsOnline(FDCANCommInstance *instance)
{
  return DaemonIsOnline(instance->comm_daemon);
}
