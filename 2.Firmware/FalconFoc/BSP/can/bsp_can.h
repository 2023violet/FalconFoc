#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "general_def.h"
#include "main.h"
#include "fdcan.h"

// 最多能够支持的CAN设备数
#define FDCAN_MX_REGISTER_CNT 6 // 这个数量取决于CAN总线的负载
#define DEVICE_CAN_CNT 1        // 根据板子设定

typedef struct fdcaninstance
{
  FDCAN_HandleTypeDef *fdcan_handle; // can句柄
  FDCAN_TxHeaderTypeDef txconf;      // CAN报文发送配置
  uint32_t tx_id;                    // 发送id
  uint32_t tx_mailbox;               // CAN消息填入的邮箱号
  uint8_t tx_buff[8];                // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
  uint8_t rx_buff[8];                // 接收缓存,最大消息长度为8
  uint32_t rx_id;                    // 接收id
  uint8_t rx_len;                    // 接收长度,可能为0-8
  // 接收的回调函数,用于解析接收到的数据
  void (*fdcan_module_callback)(struct fdcaninstance *); // callback needs an instance to tell among registered ones
  void *id;                                              // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
} FDCANInstance;

/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
  FDCAN_HandleTypeDef *fdcan_handle;              // can句柄
  uint32_t tx_id;                                 // 发送id
  uint32_t rx_id;                                 // 接收id
  void (*fdcan_module_callback)(FDCANInstance *); // 处理接收数据的回调函数
  void *id;                                       // 拥有can实例的模块地址,用于区分不同的模块(如果有需要的话),如果不需要可以不传入
} FDCAN_Init_Config_s;

/**
 * @brief Register a module to CAN service,remember to call this before using a CAN device
 *        注册(初始化)一个can实例,需要传入初始化配置的指针.
 * @param config init config
 * @return CANInstance* can instance owned by module
 */
FDCANInstance *FDCANRegister(FDCAN_Init_Config_s *config);

/**
 * @brief 修改CAN发送报文的数据帧长度;注意最大长度为8,在没有进行修改的时候,默认长度为8
 *
 * @param _instance 要修改长度的can实例
 * @param length    设定长度
 */
void FDCANSetDLC(FDCANInstance *_instance, uint8_t length);

/**
 * @brief transmit mesg through CAN device,通过can实例发送消息
 *        发送前需要向CAN实例的tx_buff写入发送数据
 *
 * @attention 超时时间不应该超过调用此函数的任务的周期,否则会导致任务阻塞
 *
 * @param timeout 超时时间,单位为ms;后续改为us,获得更精确的控制
 * @param _instance* can instance owned by module
 */
uint8_t FDCANTransmit(FDCANInstance *_instance, float timeout);

#endif
