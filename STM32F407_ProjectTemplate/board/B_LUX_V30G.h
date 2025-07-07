#ifndef _B_LUX_V30G_V30_H_
#define _B_LUX_V30G_V30_H_

/*--------------------------ͷ�ļ�����--------------------------------*/
#include "stm32f4xx.h"
#include "DataType.h"

/*-----------------------------�ṹ�嶨��---------------------------------*/  

/*-----------------------------�궨��---------------------------------*/
// ע��STM32F4 ����ʹ�� GPIO_Mode_OUT + GPIO_OType_OD/PP��GPIO_Mode_IN + GPIO_PuPd ����

// SCL ��������

// ���� GPIOB �� GPIOC ��ʱ��
#define B_LUX_V30G_GPIOB_CLK_ENABLE()  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)
#define B_LUX_V30G_GPIOC_CLK_ENABLE()  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)

// ���� GPIOB10 Ϊ������� (SCL)
#define B_LUX_V30G_SCL0_O    {\
                              GPIO_InitTypeDef GPIO_ST; \
                              GPIO_ST.GPIO_Pin = GPIO_Pin_10; \
                              GPIO_ST.GPIO_Mode = GPIO_Mode_OUT; \
                              GPIO_ST.GPIO_Speed = GPIO_Speed_50MHz; \
                              GPIO_ST.GPIO_OType = GPIO_OType_PP; \
                              GPIO_ST.GPIO_PuPd = GPIO_PuPd_NOPULL; \
                              GPIO_Init(GPIOB, &GPIO_ST); }  // GPIOB10 �������

#define B_LUX_V30G_SCL0_H    GPIO_SetBits(GPIOB, GPIO_Pin_10)
#define B_LUX_V30G_SCL0_L    GPIO_ResetBits(GPIOB, GPIO_Pin_10)

// ���� GPIOB10 Ϊ���� (SCL)
#define B_LUX_V30G_SCL0_I    {\
                              GPIO_InitTypeDef GPIO_ST; \
                              GPIO_ST.GPIO_Pin = GPIO_Pin_10; \
                              GPIO_ST.GPIO_Mode = GPIO_Mode_IN; \
                              GPIO_ST.GPIO_Speed = GPIO_Speed_50MHz; \
                              GPIO_ST.GPIO_PuPd = GPIO_PuPd_DOWN; \
                              GPIO_Init(GPIOB, &GPIO_ST); }  // GPIOB10 ��������

#define B_LUX_V30G_SCL0_DAT  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)

// ���� GPIOB11 Ϊ������� (SDA)
#define B_LUX_V30G_SDA0_O    {\
                              GPIO_InitTypeDef GPIO_ST; \
                              GPIO_ST.GPIO_Pin = GPIO_Pin_11; \
                              GPIO_ST.GPIO_Mode = GPIO_Mode_OUT; \
                              GPIO_ST.GPIO_Speed = GPIO_Speed_50MHz; \
                              GPIO_ST.GPIO_OType = GPIO_OType_PP; \
                              GPIO_ST.GPIO_PuPd = GPIO_PuPd_NOPULL; \
                              GPIO_Init(GPIOB, &GPIO_ST); }  // GPIOB11 �������

#define B_LUX_V30G_SDA0_H    GPIO_SetBits(GPIOB, GPIO_Pin_11)
#define B_LUX_V30G_SDA0_L    GPIO_ResetBits(GPIOB, GPIO_Pin_11)

// ���� GPIOB11 Ϊ�������� (SDA)
#define B_LUX_V30G_SDA0_I    {\
                              GPIO_InitTypeDef GPIO_ST; \
                              GPIO_ST.GPIO_Pin = GPIO_Pin_11; \
                              GPIO_ST.GPIO_Mode = GPIO_Mode_IN; \
                              GPIO_ST.GPIO_Speed = GPIO_Speed_50MHz; \
                              GPIO_ST.GPIO_PuPd = GPIO_PuPd_NOPULL; \
                              GPIO_Init(GPIOB, &GPIO_ST); }  // GPIOB11 ��������

#define B_LUX_V30G_SDA0_DAT  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)

// ���� GPIOC13 Ϊ������� (EN)
#define B_LUX_V30G_EN_O      {\
                              GPIO_InitTypeDef GPIO_ST; \
                              GPIO_ST.GPIO_Pin = GPIO_Pin_13; \
                              GPIO_ST.GPIO_Mode = GPIO_Mode_OUT; \
                              GPIO_ST.GPIO_Speed = GPIO_Speed_50MHz; \
                              GPIO_ST.GPIO_OType = GPIO_OType_PP; \
                              GPIO_ST.GPIO_PuPd = GPIO_PuPd_NOPULL; \
                              GPIO_Init(GPIOC, &GPIO_ST); }  // GPIOC13 �������

#define B_LUX_V30G_EN_H      GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define B_LUX_V30G_EN_L      GPIO_ResetBits(GPIOC, GPIO_Pin_13)

// �豸��ַ
#define B_LUX_V30G_SlaveAddress 0x94
#define B_LUX_V30G_CONFIG_REG   0x05
																														
/*-----------------------------��������-------------------------------*/
vid B_LUX_V30G_Delay_nms(uint16 k);
uint8 B_LUX_V30G_Init(vid);

vid B_LUX_V30G_Single_Write(uint8 REG_Address);
uint8 B_LUX_V30G_Single_Read(uint8 REG_Address);
vid B_LUX_V30G_Multiple_read(vid);

vid B_LUX_V30G_Delay5us(vid);
vid B_LUX_V30G_Delay5ms(vid);
vid B_LUX_V30G_Start(vid);
vid B_LUX_V30G_Stop(vid);
vid B_LUX_V30G_SendACK(uint8 ack);
uint8 B_LUX_V30G_RecvACK(vid);
uint8 B_LUX_V30G_SendByte(uint8 dat);
uint8 B_LUX_V30G_RecvByte(vid);
uint8 B_LUX_V30G_WriteCfg(uint8 vRegAddr, uint8 vDat);
float LUX_filtering();
#endif
