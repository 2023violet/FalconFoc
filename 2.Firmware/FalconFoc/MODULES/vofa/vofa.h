#ifndef VOFA_H
#define VOFA_H

#include "general_def.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include "FOCMotor.h"
#include "BLDCMotor.h"
#include "mt6816_encoder.h"
#include "Motor_ADC.h"

#define byte0(dw_temp) (*(char *)(&dw_temp))
#define byte1(dw_temp) (*((char *)(&dw_temp) + 1))
#define byte2(dw_temp) (*((char *)(&dw_temp) + 2))
#define byte3(dw_temp) (*((char *)(&dw_temp) + 3))

void vofa_start(void);
void vofa_send_data(uint8_t num, float data);
void vofa_sendframetail(void);
void Vofa_Packet(void);
void vofa_Receive(uint8_t *buf, uint16_t len);

#endif
