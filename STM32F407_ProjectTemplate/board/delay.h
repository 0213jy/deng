#ifndef _DELAY_H_H_H
#define _DELAY_H_H_H
#include "stm32f4xx.h"
#include "DataType.h"


/*-----------------------------�궨��---------------------------------*/
#define SYSTICK_OFF				SysTick->CTRL &= ~(0x01<<0);								//�ر�SysTick��ʱ��
#define SYSTICK_ON				SysTick->CTRL |= (0x01<<0);									//ʹ��SysTick��ʱ��

#define	STK_1MS						SysTick->LOAD = (72000&0xFFFFFF);						//���붨ʱ��Ƶֵ
#define STK_1US						SysTick->LOAD = (72&0xFFFFFF);							//΢�׶�ʱ��Ƶֵ

/*---------------------------�ⲿ��������-----------------------------*/

/*-----------------------------��������-------------------------------*/
void Systick_Init(void);																	
void SysTick_Handler(void);
void Stk_Delay_ms(uint16 vDly);
void Stk_Delay_us(uint16 vDly);

#endif