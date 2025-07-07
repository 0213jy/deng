#ifndef _B_LUX_V30G_V30_H_
#define _B_LUX_V30G_V30_H_

/*--------------------------头文件引用--------------------------------*/
#include "stm32f4xx.h"
#include "DataType.h"

/*-----------------------------结构体定义---------------------------------*/  

/*-----------------------------宏定义---------------------------------*/
// 注：STM32F4 上需使用 GPIO_Mode_OUT + GPIO_OType_OD/PP，GPIO_Mode_IN + GPIO_PuPd 设置

// SCL 引脚配置

// 启用 GPIOB 和 GPIOC 的时钟
#define B_LUX_V30G_GPIOB_CLK_ENABLE()  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)
#define B_LUX_V30G_GPIOC_CLK_ENABLE()  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)

// 配置 GPIOB10 为推挽输出 (SCL)
#define B_LUX_V30G_SCL0_O    {\
                              GPIO_InitTypeDef GPIO_ST; \
                              GPIO_ST.GPIO_Pin = GPIO_Pin_10; \
                              GPIO_ST.GPIO_Mode = GPIO_Mode_OUT; \
                              GPIO_ST.GPIO_Speed = GPIO_Speed_50MHz; \
                              GPIO_ST.GPIO_OType = GPIO_OType_PP; \
                              GPIO_ST.GPIO_PuPd = GPIO_PuPd_NOPULL; \
                              GPIO_Init(GPIOB, &GPIO_ST); }  // GPIOB10 推挽输出

#define B_LUX_V30G_SCL0_H    GPIO_SetBits(GPIOB, GPIO_Pin_10)
#define B_LUX_V30G_SCL0_L    GPIO_ResetBits(GPIOB, GPIO_Pin_10)

// 配置 GPIOB10 为输入 (SCL)
#define B_LUX_V30G_SCL0_I    {\
                              GPIO_InitTypeDef GPIO_ST; \
                              GPIO_ST.GPIO_Pin = GPIO_Pin_10; \
                              GPIO_ST.GPIO_Mode = GPIO_Mode_IN; \
                              GPIO_ST.GPIO_Speed = GPIO_Speed_50MHz; \
                              GPIO_ST.GPIO_PuPd = GPIO_PuPd_DOWN; \
                              GPIO_Init(GPIOB, &GPIO_ST); }  // GPIOB10 下拉输入

#define B_LUX_V30G_SCL0_DAT  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)

// 配置 GPIOB11 为推挽输出 (SDA)
#define B_LUX_V30G_SDA0_O    {\
                              GPIO_InitTypeDef GPIO_ST; \
                              GPIO_ST.GPIO_Pin = GPIO_Pin_11; \
                              GPIO_ST.GPIO_Mode = GPIO_Mode_OUT; \
                              GPIO_ST.GPIO_Speed = GPIO_Speed_50MHz; \
                              GPIO_ST.GPIO_OType = GPIO_OType_PP; \
                              GPIO_ST.GPIO_PuPd = GPIO_PuPd_NOPULL; \
                              GPIO_Init(GPIOB, &GPIO_ST); }  // GPIOB11 推挽输出

#define B_LUX_V30G_SDA0_H    GPIO_SetBits(GPIOB, GPIO_Pin_11)
#define B_LUX_V30G_SDA0_L    GPIO_ResetBits(GPIOB, GPIO_Pin_11)

// 配置 GPIOB11 为浮空输入 (SDA)
#define B_LUX_V30G_SDA0_I    {\
                              GPIO_InitTypeDef GPIO_ST; \
                              GPIO_ST.GPIO_Pin = GPIO_Pin_11; \
                              GPIO_ST.GPIO_Mode = GPIO_Mode_IN; \
                              GPIO_ST.GPIO_Speed = GPIO_Speed_50MHz; \
                              GPIO_ST.GPIO_PuPd = GPIO_PuPd_NOPULL; \
                              GPIO_Init(GPIOB, &GPIO_ST); }  // GPIOB11 浮空输入

#define B_LUX_V30G_SDA0_DAT  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)

// 配置 GPIOC13 为推挽输出 (EN)
#define B_LUX_V30G_EN_O      {\
                              GPIO_InitTypeDef GPIO_ST; \
                              GPIO_ST.GPIO_Pin = GPIO_Pin_13; \
                              GPIO_ST.GPIO_Mode = GPIO_Mode_OUT; \
                              GPIO_ST.GPIO_Speed = GPIO_Speed_50MHz; \
                              GPIO_ST.GPIO_OType = GPIO_OType_PP; \
                              GPIO_ST.GPIO_PuPd = GPIO_PuPd_NOPULL; \
                              GPIO_Init(GPIOC, &GPIO_ST); }  // GPIOC13 推挽输出

#define B_LUX_V30G_EN_H      GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define B_LUX_V30G_EN_L      GPIO_ResetBits(GPIOC, GPIO_Pin_13)

// 设备地址
#define B_LUX_V30G_SlaveAddress 0x94
#define B_LUX_V30G_CONFIG_REG   0x05
																														
/*-----------------------------函数声明-------------------------------*/
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
