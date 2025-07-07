#include "delay.h"
#include "stm32f4xx.h"
#include "DataType.h"

volatile uint16 m_SystickCounter;																//系统滴答定时计数值 volatile确保不会被优化掉 （全局变量尽量加volatile）

void Systick_Init()
{
	//SysTick->CTRL &= ~(0x01<<2);									//选择外部时钟作为时钟源
	SysTick->CTRL |= (0x01<<2);											//选择内部时钟作为时钟源
	
	SysTick->LOAD = (720000&0xFFFFFF);								//外部时钟10ms定时 AHB时钟72Mhz/100 = 720000
	
	SysTick->CTRL |= (0x01<<1);											//开启Systick异常中断请求
	SysTick->CTRL &= ~(0x01<<0);										//关闭Systick
	
	NVIC_SetPriority(SysTick_IRQn, 0x0A);
}

void Stk_Delay_ms(uint16 vDly)
{
	m_SystickCounter = vDly;
	STK_1MS;
	SYSTICK_ON;
	while(m_SystickCounter!=0);
	SYSTICK_OFF;
}

void Stk_Delay_us(uint16 vDly)
{
	m_SystickCounter = vDly;
	STK_1US;
	SYSTICK_ON;
	while(m_SystickCounter!=0);
	SYSTICK_OFF;
}

//void SysTick_Handler()
//{
//	m_SystickCounter--;
//}