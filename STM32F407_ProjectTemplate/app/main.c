/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：club.szlcsc.com
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 * 
 
 Change Logs:
 * Date           Author       Notes
 * 2024-03-07     LCKFB-LP    first version
 */
#include "board.h"
#include "bsp_uart.h"
#include <stdio.h>
#include "B_LUX_V30G.h"
#include <inttypes.h>
#include "DataType.h"
#include "LED.h"


extern uint8_t MODE_FLAG;
extern uint8_t ARR1[5];
extern uint8_t ARR2[5];
extern uint8_t ARR3[5];

uint8_t ARR4[6] = {0x55,0x05,0x03,0x00,0x32,0xDD};
int main(void)
{
	board_init();
	All_PWM_Init();
	uart1_init(115200U);
	uart3_init(115200U);
//	uart3_dma_tx_init();
	B_LUX_V30G_Init();
	static uint8_t last_mode_flag = 0;  // 保存上一次的 MODE_FLAG 值
	while(1)
	{
//			uart3_process_protocol();  // 接收处理协议，可能更新 MODE_FLAG
			// 检测 MODE_FLAG 是否变化
			if (MODE_FLAG != last_mode_flag)
			{
					last_mode_flag = MODE_FLAG;  // 更新上一次的值

					// 根据新的 MODE_FLAG 执行对应处理
					if (MODE_FLAG == 1)
					{
							usart_send_array(BSP_USART1, ARR1, 5);
					}
					else if (MODE_FLAG == 2)
					{
							usart_send_array(BSP_USART1, ARR2, 5);
					}
					else if (MODE_FLAG == 3)
					{
							usart_send_array(BSP_USART1, ARR3, 5);
							Light_Dark();  // 只在进入 mode 3 的一瞬执行一次
					}
			}
			if(MODE_FLAG == 3) 	
			{
			  	Light_Dark();
			}
//			send_all_lights_brightness();
			Update_PWM_From_Array();  // 每次循环都更新 PWM（不依赖 MODE_FLAG）
	}
	

}
