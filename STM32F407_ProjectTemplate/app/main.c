/*
 * ������������Ӳ�������������չ����Ӳ�����Ϲ���ȫ����Դ
 * �����������www.lckfb.com
 * ����֧�ֳ�פ��̳���κμ������⻶ӭ��ʱ����ѧϰ
 * ������̳��club.szlcsc.com
 * ��עbilibili�˺ţ������������塿���������ǵ����¶�̬��
 * ��������׬Ǯ���������й�����ʦΪ����
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
	static uint8_t last_mode_flag = 0;  // ������һ�ε� MODE_FLAG ֵ
	while(1)
	{
//			uart3_process_protocol();  // ���մ���Э�飬���ܸ��� MODE_FLAG
			// ��� MODE_FLAG �Ƿ�仯
			if (MODE_FLAG != last_mode_flag)
			{
					last_mode_flag = MODE_FLAG;  // ������һ�ε�ֵ

					// �����µ� MODE_FLAG ִ�ж�Ӧ����
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
							Light_Dark();  // ֻ�ڽ��� mode 3 ��һ˲ִ��һ��
					}
			}
			if(MODE_FLAG == 3) 	
			{
			  	Light_Dark();
			}
//			send_all_lights_brightness();
			Update_PWM_From_Array();  // ÿ��ѭ�������� PWM�������� MODE_FLAG��
	}
	

}
