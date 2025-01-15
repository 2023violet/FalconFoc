#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"

/* can instance ptrs storage, used for recv callback */
// 在CAN产生接收中断会遍历数组,选出hcan和rxid与发生中断的实例相同的那个,调用其回调函数
// @todo: 后续为每个CAN总线单独添加一个can_instance指针数组,提高回调查找的性能
static FDCANInstance *fdcan_instance[FDCAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // 全局CAN实例索引,每次有新的模块注册会自增

/* ----------------two static function called by CANRegister()-------------------- */

/**
 * @brief CAN滤波器初始化
 *
 * @param _instance can instance owned by specific module
 */
static bool FDCANAddFilter(FDCANInstance *_instance)
{
  HAL_StatusTypeDef result;
  FDCAN_FilterTypeDef fdcan_filter_conf;

  fdcan_filter_conf.IdType       = FDCAN_STANDARD_ID;       // 标准ID
  fdcan_filter_conf.FilterIndex  = 0;                       // 滤波器索引
  fdcan_filter_conf.FilterType   = FDCAN_FILTER_RANGE;      // 滤波器类型
  fdcan_filter_conf.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器0关联到FIFO0
  fdcan_filter_conf.FilterID1    = 0x0000;                  // 为0就不会过滤任何id
  fdcan_filter_conf.FilterID2    = 0x0000;                  // 理由如上

  result = HAL_FDCAN_ConfigFilter(_instance->fdcan_handle, &fdcan_filter_conf);
  if (result != HAL_OK) // 滤波器初始化
  {
    return result == HAL_ERROR;
  }
  return result == HAL_OK;
}

/**
 * @brief 在第一个FDCAN实例初始化的时候会自动调用此函数,启动CAN服务
 *
 * @note 此函数会启动FDCAN1,开启FDCAN1的FIFO0 & FIFO1溢出通知
 *
 */
static bool FDCANServiceInit(void)
{
  HAL_StatusTypeDef result;
  result = HAL_FDCAN_Start(&hfdcan1);
  result |= HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  result |= HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
  return result == HAL_OK;
}

/* ----------------------- two extern callable function -----------------------*/

FDCANInstance *FDCANRegister(FDCAN_Init_Config_s *config)
{
  if (!idx)
  {
    FDCANServiceInit(); // 第一次注册,先进行硬件初始化
    LOGINFO("[bsp_can] CAN Service Init");
  }
  if (idx >= FDCAN_MX_REGISTER_CNT) // 超过最大实例数
  {
    while (1)
      LOGERROR("[bsp_can] CAN instance exceeded MAX num, consider balance the load of CAN bus");
  }
  for (size_t i = 0; i < idx; i++)
  { // 重复注册 | id重复
    if (fdcan_instance[i]->rx_id == config->rx_id && fdcan_instance[i]->fdcan_handle == config->fdcan_handle)
    {
      while (1)
        LOGERROR("[}bsp_can] CAN id crash ,tx [%d] or rx [%d] already registered", &config->tx_id, &config->rx_id);
    }
  }

  FDCANInstance *instance = (FDCANInstance *)malloc(sizeof(FDCANInstance)); // 分配空间
  memset(instance, 0, sizeof(FDCANInstance));                               // 分配的空间未必是0,所以要先清空
  // 进行发送报文的配置
  instance->txconf.Identifier          = config->tx_id;      // 发送id
  instance->txconf.IdType              = FDCAN_STANDARD_ID;  // 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
  instance->txconf.TxFrameType         = FDCAN_DATA_FRAME;   // 发送数据帧
  instance->txconf.DataLength          = 0x08;               // 默认发送长度为8
  instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // 错误状态指示器打开
  instance->txconf.BitRateSwitch       = FDCAN_BRS_OFF;      // 比特率切换关闭，不适用于经典CAN
  instance->txconf.FDFormat            = FDCAN_CLASSIC_CAN;  // 使用经典CAN
  instance->txconf.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // 不储存发送事件
  instance->txconf.MessageMarker       = 0x0;                // 消息标记0
  // 设置回调函数和接收发送id
  instance->fdcan_handle               = config->fdcan_handle;
  instance->tx_id                      = config->tx_id;
  instance->rx_id                      = config->rx_id;
  instance->fdcan_module_callback      = config->fdcan_module_callback;
  instance->id                         = config->id;

  FDCANAddFilter(instance);         // 添加CAN过滤器规则
  fdcan_instance[idx++] = instance; // 将实例保存到can_instance中

  return instance; // 返回can实例指针
}

/* @todo 目前似乎封装过度,应该添加一个指向tx_buff的指针,tx_buff不应该由CAN instance保存 */
/* 如果让CANinstance保存txbuff,会增加一次复制的开销 */
uint8_t FDCANTransmit(FDCANInstance *_instance, float timeout)
{
  static uint32_t busy_count;
  static volatile float wait_time __attribute__((unused)); // for cancel warning
  float dwt_start = DWT_GetTimeline_ms();
  while (HAL_FDCAN_GetTxFifoFreeLevel(_instance->fdcan_handle) == 0) // 等待邮箱空闲
  {
    if (DWT_GetTimeline_ms() - dwt_start > timeout) // 超时
    {
      LOGWARNING("[bsp_can] CAN MAILbox full! failed to add msg to mailbox. Cnt [%d]", busy_count);
      busy_count++;
      return 0;
    }
  }
  wait_time = DWT_GetTimeline_ms() - dwt_start;
  // tx_mailbox会保存实际填入了这一帧消息的邮箱,但是知道是哪个邮箱发的似乎也没啥用
  if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->fdcan_handle, &_instance->txconf, _instance->tx_buff) != HAL_OK)
  {
    LOGWARNING("[bsp_can] CAN bus BUS! cnt:%d", busy_count);
    busy_count++;
    return 0;
  }
  return 1; // 发送成功
}

void FDCANSetDLC(FDCANInstance *_instance, uint8_t length)
{
  // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
  if (length > 8 || length == 0) // 安全检查
  {
    while (1)
      LOGERROR("[bsp_can] CAN DLC error! check your code or wild pointer");
  }
  _instance->txconf.DataLength = length;
}

/* -----------------------belows are callback definitions--------------------------*/

/**
 * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
 *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
 *
 * @param _hcan
 * @param fifox passed to HAL_FDCAN_GetRxMessage() to get mesg from a specific fifo
 */
static void FDCANFIFOxCallback(FDCAN_HandleTypeDef *_hcan, uint32_t fifox)
{
  static FDCAN_RxHeaderTypeDef rxconf; // 同上
  uint8_t can_rx_buff[8];
  while (HAL_FDCAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
  {
    HAL_FDCAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff); // 从FIFO中获取数据
    for (size_t i = 0; i < idx; ++i)
    { // 两者相等说明这是要找的实例
      if (_hcan == fdcan_instance[i]->fdcan_handle && rxconf.Identifier == fdcan_instance[i]->rx_id)
      {
        if (fdcan_instance[i]->fdcan_module_callback != NULL) // 回调函数不为空就调用
        {
          fdcan_instance[i]->rx_len = rxconf.DataLength;                      // 保存接收到的数据长度
          memcpy(fdcan_instance[i]->rx_buff, can_rx_buff, rxconf.DataLength); // 消息拷贝到对应实例
          fdcan_instance[i]->fdcan_module_callback(fdcan_instance[i]);        // 触发回调进行数据解析和处理
        }
        return;
      }
    }
  }
}

/**
 * @brief 注意,STM32的两个CAN设备共享两个FIFO
 * 下面两个函数是HAL库中的回调函数,他们被HAL声明为__weak,这里对他们进行重载(重写)
 * 当FIFO0或FIFO1溢出时会调用这两个函数
 */
// 下面的函数会调用CANFIFOxCallback()来进一步处理来自特定CAN设备的消息

/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hfdcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
  }
}

/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hfdcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  {
    FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
  }
}
