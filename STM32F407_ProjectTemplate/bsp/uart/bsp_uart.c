#include "bsp_uart.h" 
#include "stdio.h"
#include <board.h>

// 串口3 接收缓冲区和索引
uint8_t uart3_rx_buffer[UART3_RX_BUFFER_SIZE];
volatile uint16_t uart3_rx_index = 0;

// 灯状态数组 (4x4, 0=灭, 1=亮)
uint8_t light_status[4][4] = {0};

// 显示屏模式 (1=手动, 2=上课, 3=自习)
uint8_t display_mode = 0;

/**
 * @brief  初始化串口1
 * @param  __Baud 波特率
 * @retval None
 */
void uart1_init(uint32_t __Baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;    

    RCC_AHB1PeriphClockCmd(BSP_USART1_TX_RCC, ENABLE); // 使能 TX 引脚时钟

    // 配置串口1引脚的复用功能
    GPIO_PinAFConfig(BSP_USART1_TX_PORT, BSP_USART1_TX_AF_PIN, BSP_USART1_AF);
    GPIO_PinAFConfig(BSP_USART1_RX_PORT, BSP_USART1_RX_AF_PIN, BSP_USART1_AF);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = BSP_USART1_TX_PIN;  // TX 引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;       // 复用模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // 高速
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // 推挽输出
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;       // 上拉
    GPIO_Init(BSP_USART1_TX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_USART1_RX_PIN;  // RX 引脚
    GPIO_Init(BSP_USART1_RX_PORT, &GPIO_InitStructure);
  
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(BSP_USART1_RCC, ENABLE);    // 使能串口1时钟

    USART_DeInit(BSP_USART1);                          // 复位串口1

    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate            = __Baud; // 波特率
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b; // 8位数据
    USART_InitStructure.USART_StopBits            = USART_StopBits_1; // 1位停止位
    USART_InitStructure.USART_Parity              = USART_Parity_No; // 无奇偶校验
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
    USART_Init(BSP_USART1, &USART_InitStructure);
    
    USART_ClearFlag(BSP_USART1, USART_FLAG_RXNE);      // 清除接收标志
    
    USART_Cmd(BSP_USART1, ENABLE);                     // 使能串口1

    USART_ITConfig(BSP_USART1, USART_IT_RXNE, ENABLE); // 使能接收中断
    
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn; // 串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 主优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1; // 次优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE; // 使能中断
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  初始化串口3
 * @param  __Baud 波特率
 * @retval None
 */
void uart3_init(uint32_t __Baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;    

    RCC_AHB1PeriphClockCmd(BSP_USART3_TX_RCC, ENABLE); // 使能 TX 引脚时钟

    // 配置串口3引脚的复用功能
    GPIO_PinAFConfig(BSP_USART3_TX_PORT, BSP_USART3_TX_AF_PIN, BSP_USART3_AF);
    GPIO_PinAFConfig(BSP_USART3_RX_PORT, BSP_USART3_RX_AF_PIN, BSP_USART3_AF);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = BSP_USART3_TX_PIN;  // TX 引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;       // 复用模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // 高速
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // 推挽输出
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;       // 上拉
    GPIO_Init(BSP_USART3_TX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BSP_USART3_RX_PIN;  // RX 引脚
    GPIO_Init(BSP_USART3_RX_PORT, &GPIO_InitStructure);
  
    USART_InitTypeDef USART_InitStructure;

    RCC_APB1PeriphClockCmd(BSP_USART3_RCC, ENABLE);    // 使能串口3时钟

    USART_DeInit(BSP_USART3);                          // 复位串口3

    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate            = __Baud; // 波特率
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b; // 8位数据
    USART_InitStructure.USART_StopBits            = USART_StopBits_1; // 1位停止位
    USART_InitStructure.USART_Parity              = USART_Parity_No; // 无奇偶校验
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
    USART_Init(BSP_USART3, &USART_InitStructure);
    
    USART_ClearFlag(BSP_USART3, USART_FLAG_RXNE);      // 清除接收标志
    
    USART_Cmd(BSP_USART3, ENABLE);                     // 使能串口3

    USART_ITConfig(BSP_USART3, USART_IT_RXNE, ENABLE); // 使能接收中断
    
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn; // 串口3中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 主优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1; // 次优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE; // 使能中断
    NVIC_Init(&NVIC_InitStructure);

    // 初始化接收缓冲区
    uart3_rx_index = 0;
    for (uint16_t i = 0; i < UART3_RX_BUFFER_SIZE; i++) {
        uart3_rx_buffer[i] = 0;
    }
}

void uart3_dma_tx_init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);  // 开启DMA1时钟

    DMA_InitTypeDef DMA_InitStructure;

    DMA_DeInit(DMA1_Stream3);  // USART3 TX -> DMA1_Stream3, Channel4

    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE); // 等待DMA不可用

    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)0;  // 暂不设置，使用时再配置
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = 0;  // 发送长度，使用时再配置
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA1_Stream3, &DMA_InitStructure);

    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);  // 使能USART3 TX的DMA请求
}

void usart3_dma_send_array(uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) return;

    // 1. 禁用 DMA
    DMA_Cmd(DMA1_Stream3, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);  // 等待 DMA 完全关闭

    // 2. 设置内存地址和长度
    DMA1_Stream3->M0AR = (uint32_t)data;                // 源地址：数组首地址
    DMA_SetCurrDataCounter(DMA1_Stream3, len);          // 设置传输长度

    // 3. 启动 DMA
    DMA_Cmd(DMA1_Stream3, ENABLE);
}


void send_all_lights_brightness(void)
{
    uint8_t tx_buf[6];

    for (uint8_t row = 0; row < 4; row++)
    {
        for (uint8_t col = 0; col < 4; col++)
        {
            uint16_t brightness = pwm_brightness[row][col];

            // 限制亮度在 100~1000 之间
            if (brightness < 0) brightness = 0;
            if (brightness > 1000) brightness = 1000;

            uint8_t light_id = row * 4 + col;

            tx_buf[0] = 0x55;                         // 帧头
            tx_buf[1] = 0x04;                         // 任务位
            tx_buf[2] = light_id;                     // 控制位（灯编号 0~15）
            tx_buf[3] = (brightness >> 8) & 0xFF;     // 数据1（高8位）
            tx_buf[4] = 0x00;            // 数据2（低8位）
            tx_buf[5] = 0xDD;                         // 帧尾

					  usart_send_array(BSP_USART3, tx_buf, 5);
					
					
//            usart3_dma_send_array(tx_buf, 6);

//            // 为避免DMA冲突，可以适当延时（建议根据波特率调整）
//            delay_ms(10);
//            while (DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_TCIF3) == RESET); // 等待发送完成
//            DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3); // 清除发送完成标志位
        }
    }
}



uint8_t uart_rx_buf[6];     // 接收缓冲
uint8_t uart_rx_index = 0;  // 当前接收索引
uint8_t uart_frame_ready = 0; // 完整帧标志位

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t byte = USART_ReceiveData(USART1);  // 读取接收到的字节

        switch (uart_rx_index)
        {
            case 0:
                if (byte == 0x55)
                {
                    uart_rx_buf[0] = byte;
                    uart_rx_index = 1;
                }
                break;
            case 1:
                uart_rx_buf[1] = byte;
                uart_rx_index = 2;
                break;
            case 2:
                uart_rx_buf[2] = byte;
                uart_rx_index = 3;
                break;
            case 3:
                uart_rx_buf[3] = byte;
                uart_rx_index = 4;
                break;
            case 4:
                if (byte == 0xDD)
                {
                    uart_rx_buf[4] = byte;
										uart1_process_light_control(uart_rx_buf);
                    uart_frame_ready = 1;  // 接收完成标志置位
                }
                uart_rx_index = 0;  // 无论结尾对不对都重置
                break;
            default:
                uart_rx_index = 0;
                break;
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);  // 清除中断标志
    }
}

/**
 * @brief  串口3中断服务函数
 * @param  None
 * @retval None
 */
void USART3_IRQHandler(void)
{
    static uint8_t frame[5];
    static uint8_t index = 0;

    if (USART_GetITStatus(BSP_USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t byte = (uint8_t)USART_ReceiveData(BSP_USART3);

        switch (index)
        {
            case 0:
                if (byte == PROTOCOL_HEAD)
                    frame[index++] = byte;  // 收到帧头
                break;

            case 1: case 2: case 3:
                frame[index++] = byte;     // 收任务+数据1+数据2
                break;

            case 4:
                frame[index++] = byte;     // 收帧尾
                if (frame[4] == PROTOCOL_TAIL)
                {
                    uart3_process_protocol(frame);
                }
                // 无论成功失败都重新开始
                index = 0;
                break;

            default:
                index = 0;
                break;
        }

        USART_ClearITPendingBit(BSP_USART3, USART_IT_RXNE);
    }
}

uint8_t ARR1[5] = {0x55,0x02,0x01,0x00,0xDD};
uint8_t ARR2[5] = {0x55,0x02,0x02,0x00,0xDD};
uint8_t ARR3[5] = {0x55,0x02,0x03,0x00,0xDD};

#define PROTOCOL_HEAD           0x55
#define PROTOCOL_TAIL           0xDD
#define TASK_LIGHT_CONTROL      0x01
#define TASK_SINGLE_BRIGHTNESS  0x04
#define TASK_COLUMN_BRIGHTNESS  0x05

void uart1_process_light_control(uint8_t *frame)
{
    if (frame[0] != PROTOCOL_HEAD)
        return;

    uint8_t task = frame[1];

    // === 情况1：5字节帧（尾在frame[4]）===
    if (frame[4] == PROTOCOL_TAIL)
    {
        if (task == TASK_LIGHT_CONTROL)
        {
            uint8_t data1 = frame[2];
            uint8_t data2 = frame[3];

            for (int i = 0; i < 8; ++i)
                light_status[i / 4][i % 4] = (data1 >> (7 - i)) & 0x01;

            for (int i = 0; i < 8; ++i)
                light_status[(i + 8) / 4][(i + 8) % 4] = (data2 >> (7 - i)) & 0x01;

            usart_send_array(BSP_USART3, frame, 5);
        }
    }
    // === 情况2：6字节帧（尾在frame[5]）===
    else if (frame[5] == PROTOCOL_TAIL)
    {
        uint8_t ctrl = frame[2];
        uint8_t high = frame[3];
        uint8_t low  = frame[4];
        uint16_t brightness = (high << 8) | low;
        if (brightness > 1000) brightness = 1000;

        switch (task)
        {
            case TASK_SINGLE_BRIGHTNESS:
            {
                uint8_t row = ctrl / 4;
                uint8_t col = ctrl % 4;
                pwm_brightness[row][col] = brightness;
                break;
            }

            case TASK_COLUMN_BRIGHTNESS:
            {
                if (ctrl < 4) // 只允许列号为 0~3
                {
                    for (int row = 0; row < 4; row++)
                    {
                        pwm_brightness[row][ctrl] = brightness;
                    }
                }
                break;
            }

            default:
                break;
        }

        usart_send_array(BSP_USART3, frame, 6);
    }
}

/**
 * @brief  处理串口3接收到的协议数据
 * @param  None
 * @retval None
 */
uint8_t MODE_FLAG = 0;
void uart3_process_protocol(uint8_t *frame)
{
    if (frame[0] != PROTOCOL_HEAD || frame[4] != PROTOCOL_TAIL)
        return;

    uint8_t task  = frame[1];
    uint8_t data1 = frame[2];
    uint8_t data2 = frame[3];

    // 回发原帧
    usart_send_array(BSP_USART3, frame, 5);

    switch (task)
    {
        case TASK_LIGHT_CONTROL:
        {
            // 控制灯状态（16bit控制16盏灯）
            for (int i = 0; i < 8; ++i)
                light_status[i / 4][i % 4] = (data1 >> (7 - i)) & 0x01;
            for (int i = 0; i < 8; ++i)
                light_status[(i + 8) / 4][(i + 8) % 4] = (data2 >> (7 - i)) & 0x01;
            break;
        }

        case TASK_MODE_CONTROL:
        {
            display_mode = data1;
            MODE_FLAG = data1;
            if (data1 == 1) usart_send_array(BSP_USART1, ARR1, 5);
            if (data1 == 2) usart_send_array(BSP_USART1, ARR2, 5);
            if (data1 == 3) usart_send_array(BSP_USART1, ARR3, 5);
            break;
        }

        case TASK_BRIGHTNESS_CONTROL:
        {
            uint16_t brightness = (data1 << 8) | data2;
            if (brightness > 1000) brightness = 1000;
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    pwm_brightness[i][j] = brightness;
                }
            }
            break;
        }

        default:
            break;
    }
}


/**
 * @brief  发送单字节数据
 * @param  USARTx 目标串口 (如 BSP_USART1 或 BSP_USART3)
 * @param  ucch 数据字节
 * @retval None
 */
void usart_send_data(USART_TypeDef* USARTx, uint8_t ucch)
{
    USART_SendData(USARTx, (uint8_t)ucch);
    
    while (RESET == USART_GetFlagStatus(USARTx, USART_FLAG_TXE)) {} // 等待发送完成
}

/**
 * @brief  发送字符串
 * @param  USARTx 目标串口 (如 BSP_USART1 或 BSP_USART3)
 * @param  ucstr 字符串指针
 * @retval None
 */
void usart_send_string(USART_TypeDef* USARTx, uint8_t *ucstr)
{   
    while (ucstr && *ucstr) // 检查指针和字符串是否有效
    {     
        usart_send_data(USARTx, *ucstr++);    
    }
}

/**
 * @brief  发送数据数组
 * @param  USARTx 目标串口 (如 BSP_USART1 或 BSP_USART3)
 * @param  data 数据数组指针
 * @param  len  数据长度
 * @retval None
 */
void usart_send_array(USART_TypeDef* USARTx, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) return;

    for (uint16_t i = 0; i < len; i++) {
        usart_send_data(USARTx, data[i]);
    }
}

#if !defined(__MICROLIB)
#if (__ARMCLIB_VERSION <= 6000000)
struct __FILE
{
    int handle;
};
#endif

FILE __stdout;

void _sys_exit(int x)
{
    x = x;
}
#endif

/**
 * @brief  重定向 printf 到串口1
 * @param  ch 字符
 * @param  f 文件指针
 * @retval 字符
 */
int fputc(int ch, FILE *f)
{
    USART_SendData(BSP_USART1, (uint8_t)ch);
    
    while (RESET == USART_GetFlagStatus(BSP_USART1, USART_FLAG_TXE)) {}
    
    return ch;
}
