#ifndef __LED_H__
#define __LED_H__

#include "stm32f4xx.h"

void pwm_breathing_lamp(void);
void TIM_PWM_Init(TIM_TypeDef* TIMx, uint8_t channel, uint32_t period, uint32_t prescaler);
void PWM_GPIO_Config(void);
void All_PWM_Init(void);
void Update_PWM_From_Array(void);
void BreathingEffect(void);
void Light_Dark();
//void TIM6_Init(void);
#endif
