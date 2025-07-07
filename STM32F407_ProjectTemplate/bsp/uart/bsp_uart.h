/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：club.szlcsc.com
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 * 
 * 变更日志:
 * 日期           作者       备注
 * 2024-03-07     LCKFB-LP   初版
 * 2025-05-15     Grok       添加串口3支持
 * 2025-05-15     Grok       添加串口3中断支持
 * 2025-05-15     Grok       添加协议解析支持
 * 2025-05-15     Grok       将 light_status 替换为 pwm_brightness
 */
 
#ifndef __BSP_UART_H__
#define __BSP_UART_H__
 
#include "stm32f4xx.h"
 
// 串口1 定义
#define BSP_USART1_RCC       RCC_APB2Periph_USART1      // 串口1 时钟
#define BSP_USART1_TX_RCC    RCC_AHB1Periph_GPIOA       // TX 引脚时钟
#define BSP_USART1_RX_RCC    RCC_AHB1Periph_GPIOA       // RX 引脚时钟
#define BSP_USART1           USART1                      // 串口1
#define BSP_USART1_TX_PORT   GPIOA                      // TX 引脚端口
#define BSP_USART1_TX_PIN    GPIO_Pin_9                 // TX 引脚
#define BSP_USART1_RX_PORT   GPIOA                      // RX 引脚端口
#define BSP_USART1_RX_PIN    GPIO_Pin_10                // RX 引脚
#define BSP_USART1_AF        GPIO_AF_USART1             // 复用功能
#define BSP_USART1_TX_AF_PIN GPIO_PinSource9            // TX 复用引脚源
#define BSP_USART1_RX_AF_PIN GPIO_PinSource10           // RX 复用引脚源
 
// 串口3 定义
#define BSP_USART3_RCC       RCC_APB1Periph_USART3      // 串口3 时钟
#define BSP_USART3_TX_RCC    RCC_AHB1Periph_GPIOC       // TX 引脚时钟
#define BSP_USART3_RX_RCC    RCC_AHB1Periph_GPIOC       // RX 引脚时钟
#define BSP_USART3           USART3                      // 串口3
#define BSP_USART3_TX_PORT   GPIOC                      // TX 引脚端口
#define BSP_USART3_TX_PIN    GPIO_Pin_10                // TX 引脚
#define BSP_USART3_RX_PORT   GPIOC                      // RX 引脚端口
#define BSP_USART3_RX_PIN    GPIO_Pin_11                // RX 引脚
#define BSP_USART3_AF        GPIO_AF_USART3             // 复用功能
#define BSP_USART3_TX_AF_PIN GPIO_PinSource10           // TX 复用引脚源
#define BSP_USART3_RX_AF_PIN GPIO_PinSource11           // RX 复用引脚源

// 串口3 接收缓冲区大小
#define UART3_RX_BUFFER_SIZE 5

// 协议相关定义
#define PROTOCOL_HEAD       0x55    // 协议针头
#define PROTOCOL_TAIL       0xDD    // 协议针尾
#define TASK_LIGHT_CONTROL  0x01    // 灯控任务
#define TASK_MODE_CONTROL   0x02    // 模式控制任务
#define TASK_BRIGHTNESS_CONTROL  0x03  // 灯亮度控制任务

// 外部变量声明
extern uint8_t uart3_rx_buffer[UART3_RX_BUFFER_SIZE]; // 串口3接收缓冲区
extern volatile uint16_t uart3_rx_index;              // 串口3接收索引
extern uint16_t pwm_brightness[4][4];                 // PWM亮度数组 (0=灭, 65535=亮)
extern uint8_t display_mode;                          // 显示屏模式 (1=手动, 2=上课, 3=自习)
 
// 外部函数声明
void uart1_init(uint32_t __Baud);                       // 初始化串口1
void uart3_init(uint32_t __Baud);                       // 初始化串口3
void usart_send_data(USART_TypeDef* USARTx, uint8_t ucch); // 发送单字节数据
void usart_send_string(USART_TypeDef* USARTx, uint8_t *ucstr); // 发送字符串
void uart3_process_protocol(uint8_t *frame);                       // 处理串口3接收协议
void usart_send_array(USART_TypeDef* USARTx, const uint8_t *data, uint16_t len);
void uart1_process_light_control(uint8_t *frame);
void uart3_dma_tx_init(void);
void usart3_dma_send_array(uint8_t *data, uint16_t len);
void send_all_lights_brightness(void);
#endif
  
	