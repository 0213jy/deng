/*
 * ������������Ӳ�������������չ����Ӳ�����Ϲ���ȫ����Դ
 * �����������www.lckfb.com
 * ����֧�ֳ�פ��̳���κμ������⻶ӭ��ʱ����ѧϰ
 * ������̳��club.szlcsc.com
 * ��עbilibili�˺ţ������������塿���������ǵ����¶�̬��
 * ��������׬Ǯ���������й�����ʦΪ����
 * 
 * �����־:
 * ����           ����       ��ע
 * 2024-03-07     LCKFB-LP   ����
 * 2025-05-15     Grok       ��Ӵ���3֧��
 * 2025-05-15     Grok       ��Ӵ���3�ж�֧��
 * 2025-05-15     Grok       ���Э�����֧��
 * 2025-05-15     Grok       �� light_status �滻Ϊ pwm_brightness
 */
 
#ifndef __BSP_UART_H__
#define __BSP_UART_H__
 
#include "stm32f4xx.h"
 
// ����1 ����
#define BSP_USART1_RCC       RCC_APB2Periph_USART1      // ����1 ʱ��
#define BSP_USART1_TX_RCC    RCC_AHB1Periph_GPIOA       // TX ����ʱ��
#define BSP_USART1_RX_RCC    RCC_AHB1Periph_GPIOA       // RX ����ʱ��
#define BSP_USART1           USART1                      // ����1
#define BSP_USART1_TX_PORT   GPIOA                      // TX ���Ŷ˿�
#define BSP_USART1_TX_PIN    GPIO_Pin_9                 // TX ����
#define BSP_USART1_RX_PORT   GPIOA                      // RX ���Ŷ˿�
#define BSP_USART1_RX_PIN    GPIO_Pin_10                // RX ����
#define BSP_USART1_AF        GPIO_AF_USART1             // ���ù���
#define BSP_USART1_TX_AF_PIN GPIO_PinSource9            // TX ��������Դ
#define BSP_USART1_RX_AF_PIN GPIO_PinSource10           // RX ��������Դ
 
// ����3 ����
#define BSP_USART3_RCC       RCC_APB1Periph_USART3      // ����3 ʱ��
#define BSP_USART3_TX_RCC    RCC_AHB1Periph_GPIOC       // TX ����ʱ��
#define BSP_USART3_RX_RCC    RCC_AHB1Periph_GPIOC       // RX ����ʱ��
#define BSP_USART3           USART3                      // ����3
#define BSP_USART3_TX_PORT   GPIOC                      // TX ���Ŷ˿�
#define BSP_USART3_TX_PIN    GPIO_Pin_10                // TX ����
#define BSP_USART3_RX_PORT   GPIOC                      // RX ���Ŷ˿�
#define BSP_USART3_RX_PIN    GPIO_Pin_11                // RX ����
#define BSP_USART3_AF        GPIO_AF_USART3             // ���ù���
#define BSP_USART3_TX_AF_PIN GPIO_PinSource10           // TX ��������Դ
#define BSP_USART3_RX_AF_PIN GPIO_PinSource11           // RX ��������Դ

// ����3 ���ջ�������С
#define UART3_RX_BUFFER_SIZE 5

// Э����ض���
#define PROTOCOL_HEAD       0x55    // Э����ͷ
#define PROTOCOL_TAIL       0xDD    // Э����β
#define TASK_LIGHT_CONTROL  0x01    // �ƿ�����
#define TASK_MODE_CONTROL   0x02    // ģʽ��������
#define TASK_BRIGHTNESS_CONTROL  0x03  // �����ȿ�������

// �ⲿ��������
extern uint8_t uart3_rx_buffer[UART3_RX_BUFFER_SIZE]; // ����3���ջ�����
extern volatile uint16_t uart3_rx_index;              // ����3��������
extern uint16_t pwm_brightness[4][4];                 // PWM�������� (0=��, 65535=��)
extern uint8_t display_mode;                          // ��ʾ��ģʽ (1=�ֶ�, 2=�Ͽ�, 3=��ϰ)
 
// �ⲿ��������
void uart1_init(uint32_t __Baud);                       // ��ʼ������1
void uart3_init(uint32_t __Baud);                       // ��ʼ������3
void usart_send_data(USART_TypeDef* USARTx, uint8_t ucch); // ���͵��ֽ�����
void usart_send_string(USART_TypeDef* USARTx, uint8_t *ucstr); // �����ַ���
void uart3_process_protocol(uint8_t *frame);                       // ������3����Э��
void usart_send_array(USART_TypeDef* USARTx, const uint8_t *data, uint16_t len);
void uart1_process_light_control(uint8_t *frame);
void uart3_dma_tx_init(void);
void usart3_dma_send_array(uint8_t *data, uint16_t len);
void send_all_lights_brightness(void);
#endif
  
	