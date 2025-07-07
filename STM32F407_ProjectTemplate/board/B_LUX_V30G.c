/*
参照代码由 "淮安市蓝控电子科技有限公司"提供，联系QQ:1746950285

传感器移植说明:  代码以IO模拟I2C实现传感器采集，需要移植文件 传感器型号对应的.h文件和.c两个文件、
DataType.h(可选) 
			.h文件(如：B_LUX_V30G.h等)：包含IO脚的宏定义 和 函数声明, 需要修改IO的宏定义，对应用户使用的IO
			.c文件(如: B_LUX_V30G.c等)：包含I2C模拟代码和传感器采集函数, 可能需要修改地方,两个延时函数的修过
			DataType.h							 ：类型定义 (可选)
*/
//***************************************
// B_LUX_V30G采集程序
//****************************************
#include "B_LUX_V30G.h"
#include "stm32f4xx.h"
#include "DataType.h"
#include <board.h>
#include <stdio.h>
#include <string.h> // for memset
#include <stdlib.h> // for qsort
/*---------------------------------------------------------------------
 功能描述: 延时纳秒 不同的工作环境,需要调整此函数
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
vid B_LUX_V30G_Delay_nms(uint16 k)	
{						
	delay_ms(k);
}					

/*---------------------------------------------------------------------
 功能描述: 延时5微秒  不同的工作环境,需要调整此函数
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
vid B_LUX_V30G_Delay5us()
{
	delay_us(5);
}

/*---------------------------------------------------------------------
 功能描述: 延时5毫秒  不同的工作环境,需要调整此函数
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
vid B_LUX_V30G_Delay5ms()
{
	B_LUX_V30G_Delay_nms(5);
}

/*---------------------------------------------------------------------
 功能描述: 起始信号
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
vid B_LUX_V30G_Start()
{
  B_LUX_V30G_SDA0_H;                         //拉高数据线
  B_LUX_V30G_SCL0_H;                         //拉高时钟线
  B_LUX_V30G_Delay5us();                     //延时
  B_LUX_V30G_SDA0_L;                         //产生下降沿
  B_LUX_V30G_Delay5us();                     //延时
  B_LUX_V30G_SCL0_L;                         //拉低时钟线
}

/*---------------------------------------------------------------------
 功能描述: 停止信号
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
vid B_LUX_V30G_Stop()
{
  B_LUX_V30G_SDA0_L;                         //拉低数据线
  B_LUX_V30G_SCL0_H;                         //拉高时钟线
  B_LUX_V30G_Delay5us();                     //延时
  B_LUX_V30G_SDA0_H;                         //产生上升沿
  B_LUX_V30G_Delay5us();                     //延时
  B_LUX_V30G_SCL0_L;
  B_LUX_V30G_Delay5us();
}

/*---------------------------------------------------------------------
 功能描述: 发送应答信号
 参数说明: ack - 应答信号(0:ACK 1:NAK)
 函数返回: 无
 ---------------------------------------------------------------------*/
vid B_LUX_V30G_SendACK(uint8 ack)
{
  if (ack&0x01)	B_LUX_V30G_SDA0_H;		//写应答信号
  else	B_LUX_V30G_SDA0_L;
  
  B_LUX_V30G_SCL0_H;                         //拉高时钟线
  B_LUX_V30G_Delay5us();                     //延时
  B_LUX_V30G_SCL0_L;                         //拉低时钟线
  B_LUX_V30G_SDA0_H;
  B_LUX_V30G_Delay5us();                     //延时
}

/*---------------------------------------------------------------------
 功能描述: 接收应答信号
 参数说明: 无
 函数返回: 返回应答信号
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_RecvACK()
{
  uint8 CY = 0x00;
  B_LUX_V30G_SDA0_H;
  
  B_LUX_V30G_SDA0_I;
  
  B_LUX_V30G_SCL0_H;                         //拉高时钟线
  B_LUX_V30G_Delay5us();                     //延时
  
  if (B_LUX_V30G_SDA0_DAT)
	  CY = 1;                 				 				//读应答信号
  
  B_LUX_V30G_Delay5us();                     //延时
  
  B_LUX_V30G_SCL0_L;                         //拉低时钟线
  
  B_LUX_V30G_SDA0_O;
  
  return CY;
}

/*---------------------------------------------------------------------
 功能描述: 向IIC总线发送一个字节数据
 参数说明: dat - 写字节
 函数返回: 无
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_SendByte(uint8 dat)
{
  uint8 i;
  
  for (i=0; i<8; i++)         						//8位计数器
  {
    if (dat&0x80)	B_LUX_V30G_SDA0_H;
    else	B_LUX_V30G_SDA0_L;                   //送数据口
    
    B_LUX_V30G_Delay5us();             		//延时
    B_LUX_V30G_SCL0_H;                		//拉高时钟线
    B_LUX_V30G_Delay5us();             		//延时
    B_LUX_V30G_SCL0_L;                		//拉低时钟线
    B_LUX_V30G_Delay5us();             		//延时
    dat <<= 1;              							//移出数据的最高位
  }
  
	return B_LUX_V30G_RecvACK();
}

/*---------------------------------------------------------------------
 功能描述: 从IIC总线接收一个字节数据
 参数说明: 无
 函数返回: 接收字节
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_RecvByte()
{
  uint8 i;
  uint8 dat = 0;
  B_LUX_V30G_SDA0_I;
  
  B_LUX_V30G_SDA0_H;                         	//使能内部上拉,准备读取数据,
  for (i=0; i<8; i++)         	        			//8位计数器
  {
    B_LUX_V30G_SCL0_H;                       	//拉高时钟线
    B_LUX_V30G_Delay5us();             				//延时
	
	if (B_LUX_V30G_SDA0_DAT)
		dat |= 1;              										//读数据 
	
    B_LUX_V30G_SCL0_L;                       	//拉低时钟线
    B_LUX_V30G_Delay5us();             				//延时
    
		if (i<7) dat <<= 1;
  }
  B_LUX_V30G_SDA0_O;
  
  return dat;
}

/*---------------------------------------------------------------------
 功能描述: 传感器复位
 参数说明: 无
 函数返回: 返回结果 0成功 大于0失败
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_Reset()
{
	uint8 vRval = 0x00;
	
	B_LUX_V30G_Start();                                //起始信号
  vRval += B_LUX_V30G_SendByte(0x00);    						 //发送通用设备地址+写信号
  vRval += B_LUX_V30G_SendByte(0xC0);								 //写复位命令
	vRval += B_LUX_V30G_SendByte(0x00);
  B_LUX_V30G_Stop();                                 //停止信号
	
	return vRval;
}

/*---------------------------------------------------------------------
 功能描述: 修改传感器设备地址
 参数说明: vAddr - 设备地址
 函数返回: 返回结果 0成功 大于0失败
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_SetDevAddr(uint8_t vAddr)
{
	uint8 vRval = 0x00;
	
	B_LUX_V30G_Start();                                //起始信号
  vRval += B_LUX_V30G_SendByte(0x00);   			  		 //发送通用设备地址+写信号
  vRval += B_LUX_V30G_SendByte(0xA0);								 //写设备地址寄存器
	vRval += B_LUX_V30G_SendByte(vAddr);
  B_LUX_V30G_Stop();                                 //停止信号
	
	return vRval;
}

/*---------------------------------------------------------------------
 功能描述: 传感器恢复默认配置
 参数说明: 无
 函数返回: 返回结果 0成功 大于0失败
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_DefCfg()
{
	uint8 vRval = 0x00;
	
	B_LUX_V30G_Start();                                //起始信号
	vRval += B_LUX_V30G_SendByte(0x00);    			   		 //发送通用设备地址+写信号
	vRval += B_LUX_V30G_SendByte(0xB0);								 //写恢复出厂设置命令
	vRval += B_LUX_V30G_SendByte(0x00);
	B_LUX_V30G_Stop();                                 //停止信号
	
	return vRval;
}

/*---------------------------------------------------------------------
 功能描述: 初始化光照传感器
 参数说明: 无
 函数返回: 无
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_Init()
{
	uint8 vRval = 0;
	B_LUX_V30G_GPIOB_CLK_ENABLE();
	B_LUX_V30G_GPIOC_CLK_ENABLE();
	
	B_LUX_V30G_SCL0_O;
	B_LUX_V30G_SDA0_O;
	B_LUX_V30G_EN_O;
	B_LUX_V30G_EN_H;
	B_LUX_V30G_SCL0_H;
	B_LUX_V30G_SDA0_H;
  
	B_LUX_V30G_Delay_nms(100);	                        //延时100ms
	
	//写配置参数, 配置传感器采集速度为6级（1级最快，数值越大采集速度越慢）
	vRval += B_LUX_V30G_WriteCfg(B_LUX_V30G_CONFIG_REG, 0x06);			
	B_LUX_V30G_Delay_nms(100);
	B_LUX_V30G_Reset();
  
	return vRval;
}



/*---------------------------------------------------------------------
 功能描述: 写寄存器数据 
 参数说明: vRegAddr - 寄存器地址
					 vDat - 寄存器数据
 函数返回: 返回结果 0成功 大于0失败
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_WriteCfg(uint8 vRegAddr, uint8 vDat)
{
	uint8 vRval = 0x00;
	
	B_LUX_V30G_Start();                                //起始信号
  vRval += B_LUX_V30G_SendByte(B_LUX_V30G_SlaveAddress+0);    //发送设备地址+读信号
  vRval += B_LUX_V30G_SendByte(vRegAddr);
  vRval += B_LUX_V30G_SendByte(vDat); 
  B_LUX_V30G_Stop();                                 //停止信号
	
	return vRval;
}

/*---------------------------------------------------------------------
 功能描述: CRC8校验
 参数说明: 	*vDat - 数据指针
						vLen - 校验数据长度
 函数返回:  返回CRC8校验值
 ---------------------------------------------------------------------*/
#define POLYNOMIAL 0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001
uint8 CheckCrc8(uint8 *vDat, uint8 vLen)
{
	uint8 vBit; // bit mask
	uint8 vCrc = 0xFF; // calculated checksum
	uint8 vByteCtr; // byte counter
	for(vByteCtr = 0; vByteCtr < vLen; vByteCtr++)
	{
		vCrc ^= (vDat[vByteCtr]);
		for(vBit = 8; vBit > 0; --vBit)
		{
			if(vCrc & 0x80) vCrc = (vCrc << 1) ^ POLYNOMIAL;
			else vCrc = (vCrc << 1);
		}
	}
	return vCrc;
}

/*---------------------------------------------------------------------
 功能描述: 光照读取函数
 参数说明: 无
 函数返回: 返回 0采集成功 大于0 失败
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_GetLux(uint32 *vLux)
{  
	uint8 vCrc8 = 0;
  uint8 vRval = 0;
  uint8 vBuf[5];                       							//接收数据缓存区  
  uint32 val32 = 0;  
  fp32 temp;
  uint8 i = 0;
  

  //---------------------------读程序
  B_LUX_V30G_Start();                                //起始信号
	vRval += B_LUX_V30G_SendByte(B_LUX_V30G_SlaveAddress+0);         //发送设备地址+写信号
  vRval += B_LUX_V30G_SendByte(0x00);
 
  B_LUX_V30G_Start();          	                     //起始信号
  vRval += B_LUX_V30G_SendByte(B_LUX_V30G_SlaveAddress+1);         //发送设备地址+读信号
  B_LUX_V30G_Delay_nms(10);                         //延时10ms
  for (i=0; i<5; i++)                           		//连续读取5个字节数据，存储中BUF
  {
    vBuf[i] = B_LUX_V30G_RecvByte();  
    if (i == 4)																			 //从第0个字节，开始判断最有一字节，返回NACK
    {
      B_LUX_V30G_SendACK(1);                         //最后一个数据需要回NOACK
    }
    else
    {		
      B_LUX_V30G_SendACK(0);                         //回应ACK
    }
  }
  B_LUX_V30G_Stop();                                 //停止信号
  
  //---------------------------
  
  
  /*
   //---------------------------写程序
  B_LUX_V30G_Start();                                //起始信号
  B_LUX_V30G_SendByte(B_LUX_V30G_SlaveAddress+0);    //发送设备地址+写信号
  B_LUX_V30G_SendByte(0x10);
  for (i=0; i<=112; i++)                             //连续写数据
  {
    B_LUX_V30G_SendByte(0x30+i); 
  }
  //B_LUX_V30G_SendByte(0x00); 
  B_LUX_V30G_Stop();                                 //停止信号
  //---------------------------
  */
  
  
	vCrc8 = CheckCrc8(vBuf, 4);												 //光照数据进行CRC8校验
	if (vCrc8 == vBuf[4])															 //判断CRC8数据校验
	{
		val32   = vBuf[3];
		val32 <<= 8;
		val32  |= vBuf[2];
		val32 <<= 8;
		val32  |= vBuf[1];
		val32 <<= 8;
		val32  |= vBuf[0];
  
		temp = (fp32)val32*1.4;												//半球透明外壳光照度矫正值
		//temp = (fp32)val32/0.4;												//半球白色外壳光照度矫正值
		*vLux = (uint32)(temp);
	}
	else
	{
		vRval = 0xF0;
	}
		
	return vRval;
} 

void BubbleSort(uint32 *arr, int length)
{
    int i, j;
    for (i = 0; i < length - 1; i++)
    {
        for (j = 0; j < length - 1 - i; j++)
        {
            if (arr[j] > arr[j + 1])
            {
                // 交换
                int temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

#define N 10
float LUX_filtering()
{
	uint32 arr[N];
	uint32 lux;
	uint8 i=0;
	while(i<10)
	{
		if(B_LUX_V30G_GetLux(&lux) == 0)
		{
			arr[i++] = lux;
		}
	}
	BubbleSort(arr,N);
	// 去掉最大和最小后求平均
  uint64_t sum = 0;
  for (i = 1; i < N - 1; i++)  // 去掉 arr[0] 最小值，arr[N-1] 最大值
  {
     sum += arr[i];
  }

  return ((uint32)(sum / (N - 2)))/0.4/1000;  // 返回平均值
}

	
	
