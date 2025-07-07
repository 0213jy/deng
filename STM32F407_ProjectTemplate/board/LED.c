#include "LED.h"
#include "stm32f4xx.h"
#include "B_LUX_V30G.h"
#include "board.h"
#include "bsp_uart.h"
#include <stdio.h>
#include <inttypes.h>
#include "DataType.h"
#include "LED.h"


extern uint8_t light_status[4][4];

/* 定义亮度数组 */
uint16_t pwm_brightness[4][4]={	700,700,700,700,
																700,700,700,700,
																700,700,700,700,
																700,700,700,700	};



/* 定义每个定时器的PWM初始化函数 */
void TIM_PWM_Init(TIM_TypeDef* TIMx, uint8_t channel, uint32_t period, uint32_t prescaler)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    switch (channel)
    {
        case 1: TIM_OC1Init(TIMx, &TIM_OCInitStructure); TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable); break;
        case 2: TIM_OC2Init(TIMx, &TIM_OCInitStructure); TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable); break;
        case 3: TIM_OC3Init(TIMx, &TIM_OCInitStructure); TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable); break;
        case 4: TIM_OC4Init(TIMx, &TIM_OCInitStructure); TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable); break;
    }

    TIM_Cmd(TIMx, ENABLE);

    // ? TIM1 是高级定时器，需要启用主输出
    if (TIMx == TIM1)
        TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void PWM_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

    // GPIOA: PA0~PA3, PA6~PA8
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                                GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // GPIOB: PB0, PB1, PB6~PB9
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |
                                GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // GPIOE: PE11, PE13, PE14
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOE, &GPIO_InitStruct);

    // 复用配置
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
}
/* 所有PWM初始化（周期：1000，分频：84） */
void All_PWM_Init(void)
{
    PWM_GPIO_Config();

    TIM_PWM_Init(TIM1, 1, 1000, 84);
    TIM_PWM_Init(TIM1, 2, 1000, 84);
    TIM_PWM_Init(TIM1, 3, 1000, 84);
    TIM_PWM_Init(TIM1, 4, 1000, 84);

    TIM_PWM_Init(TIM2, 1, 1000, 84);
    TIM_PWM_Init(TIM2, 2, 1000, 84);
    TIM_PWM_Init(TIM2, 3, 1000, 84);
    TIM_PWM_Init(TIM2, 4, 1000, 84);

    TIM_PWM_Init(TIM3, 1, 1000, 84);
    TIM_PWM_Init(TIM3, 2, 1000, 84);
    TIM_PWM_Init(TIM3, 3, 1000, 84);
    TIM_PWM_Init(TIM3, 4, 1000, 84);

    TIM_PWM_Init(TIM4, 1, 1000, 84);
    TIM_PWM_Init(TIM4, 2, 1000, 84);
    TIM_PWM_Init(TIM4, 3, 1000, 84);
    TIM_PWM_Init(TIM4, 4, 1000, 84);
}


void Update_PWM_From_Array(void)
{
    for (int row = 0; row < 4; row++)
    {
        for (int col = 0; col < 4; col++)
        {
            uint16_t value = light_status[row][col] ? pwm_brightness[row][col] : 0;

            switch (row)
            {
                case 0:
                    switch (col)
                    {
                        case 0: TIM_SetCompare1(TIM1, value); break;
                        case 1: TIM_SetCompare2(TIM1, value); break;
                        case 2: TIM_SetCompare3(TIM1, value); break;
                        case 3: TIM_SetCompare4(TIM1, value); break;
                    }
                    break;
                case 1:
                    switch (col)
                    {
                        case 0: TIM_SetCompare1(TIM2, value); break;
                        case 1: TIM_SetCompare2(TIM2, value); break;
                        case 2: TIM_SetCompare3(TIM2, value); break;
                        case 3: TIM_SetCompare4(TIM2, value); break;
                    }
                    break;
                case 2:
                    switch (col)
                    {
                        case 0: TIM_SetCompare1(TIM3, value); break;
                        case 1: TIM_SetCompare2(TIM3, value); break;
                        case 2: TIM_SetCompare3(TIM3, value); break;
                        case 3: TIM_SetCompare4(TIM3, value); break;
                    }
                    break;
                case 3:
                    switch (col)
                    {
                        case 0: TIM_SetCompare1(TIM4, value); break;
                        case 1: TIM_SetCompare2(TIM4, value); break;
                        case 2: TIM_SetCompare3(TIM4, value); break;
                        case 3: TIM_SetCompare4(TIM4, value); break;
                    }
                    break;
            }
        }
    }
}

//呼吸灯
void BreathingEffect(void)
{
    static int brightness = 0;
    static int direction = 10;  // 每次改变的步长

    brightness += direction;

    if (brightness >= 1000)
    {
        brightness = 1000;
        direction = -10;    // 开始变暗
    }
    else if (brightness <= 0)
    {
        brightness = 0;
        direction = 10;     // 开始变亮
    }

    // 设置所有灯相同亮度
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            pwm_brightness[i][j] = brightness;
        }
    }

    Update_PWM_From_Array();
}


//控制亮度
void Light_Dark()
{
    float LED = LUX_filtering();  // 获取光照强度
    int base_brightness = 0;
		uint8_t lux_buf[32];
		
    if (LED < 200)
    {
        // 所有灯设置为固定高亮度700
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                pwm_brightness[i][j] = 700;
            }
        }
    }
    else
    {
        // 将光照值映射为不超过700的亮度值
        base_brightness = (int)(LED * 0.1f);  // 经验系数，可调
				
        if (base_brightness > 700) base_brightness = 700;
        if (base_brightness < 50) base_brightness = 50;

        for (int row = 0; row < 4; row++)
        {
            int col_brightness = base_brightness + row * 200;
            if (col_brightness > 700) col_brightness = 700;
//						sprintf((char*)lux_buf, "lux;%d\r\n", col_brightness);
//						usart_send_string(BSP_USART3, lux_buf);
            for (int col = 0; col < 4; col++)
            {
                pwm_brightness[row][col] = col_brightness;
            }
        }
    }
}

//void TIM6_Init(void)
//{
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);  // 开启TIM6时钟

//    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;

//    // 时基配置：
//    // APB1 Timer Clock = 84MHz（通常）
//    // 设 Prescaler=8399 -> 84MHz/8400 = 10kHz
//    // Period = 19999 -> 10kHz/20000 = 0.5Hz（每2秒中断）
//    TIM_TimeBaseStructure.TIM_Prescaler = 8399;
//    TIM_TimeBaseStructure.TIM_Period = 19999;
//    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

//    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);  // 允许更新中断

//    // 中断优先级配置
//    NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);

//    TIM_Cmd(TIM6, ENABLE);  // 启动定时器
//}


//extern uint8_t MODE_FLAG;
//extern uint8_t ARR2[5];
//extern uint8_t ARR3[5];
//void TIM6_DAC_IRQHandler(void)
//{
//    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
//    {
//        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);  // 清除中断标志位

//        

//        if (MODE_FLAG == 1)
//        {
//        }
//        else if (MODE_FLAG == 2)
//        {
//            usart_send_array(BSP_USART1, ARR2, 5);
//        }
//        else if (MODE_FLAG == 3)
//        {
//            usart_send_array(BSP_USART1, ARR3, 5);
//        }

//        if (MODE_FLAG == 3)
//            Light_Dark();

//        Update_PWM_From_Array();
//        // === 你的逻辑代码结束 ===
//    }
//}















