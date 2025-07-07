/*
���մ����� "���������ص��ӿƼ����޹�˾"�ṩ����ϵQQ:1746950285

��������ֲ˵��:  ������IOģ��I2Cʵ�ִ������ɼ�����Ҫ��ֲ�ļ� �������ͺŶ�Ӧ��.h�ļ���.c�����ļ���
DataType.h(��ѡ) 
			.h�ļ�(�磺B_LUX_V30G.h��)������IO�ŵĺ궨�� �� ��������, ��Ҫ�޸�IO�ĺ궨�壬��Ӧ�û�ʹ�õ�IO
			.c�ļ�(��: B_LUX_V30G.c��)������I2Cģ�����ʹ������ɼ�����, ������Ҫ�޸ĵط�,������ʱ�������޹�
			DataType.h							 �����Ͷ��� (��ѡ)
*/
//***************************************
// B_LUX_V30G�ɼ�����
//****************************************
#include "B_LUX_V30G.h"
#include "stm32f4xx.h"
#include "DataType.h"
#include <board.h>
#include <stdio.h>
#include <string.h> // for memset
#include <stdlib.h> // for qsort
/*---------------------------------------------------------------------
 ��������: ��ʱ���� ��ͬ�Ĺ�������,��Ҫ�����˺���
 ����˵��: ��	
 ��������: ��
 ---------------------------------------------------------------------*/
vid B_LUX_V30G_Delay_nms(uint16 k)	
{						
	delay_ms(k);
}					

/*---------------------------------------------------------------------
 ��������: ��ʱ5΢��  ��ͬ�Ĺ�������,��Ҫ�����˺���
 ����˵��: ��	
 ��������: ��
 ---------------------------------------------------------------------*/
vid B_LUX_V30G_Delay5us()
{
	delay_us(5);
}

/*---------------------------------------------------------------------
 ��������: ��ʱ5����  ��ͬ�Ĺ�������,��Ҫ�����˺���
 ����˵��: ��	
 ��������: ��
 ---------------------------------------------------------------------*/
vid B_LUX_V30G_Delay5ms()
{
	B_LUX_V30G_Delay_nms(5);
}

/*---------------------------------------------------------------------
 ��������: ��ʼ�ź�
 ����˵��: ��	
 ��������: ��
 ---------------------------------------------------------------------*/
vid B_LUX_V30G_Start()
{
  B_LUX_V30G_SDA0_H;                         //����������
  B_LUX_V30G_SCL0_H;                         //����ʱ����
  B_LUX_V30G_Delay5us();                     //��ʱ
  B_LUX_V30G_SDA0_L;                         //�����½���
  B_LUX_V30G_Delay5us();                     //��ʱ
  B_LUX_V30G_SCL0_L;                         //����ʱ����
}

/*---------------------------------------------------------------------
 ��������: ֹͣ�ź�
 ����˵��: ��	
 ��������: ��
 ---------------------------------------------------------------------*/
vid B_LUX_V30G_Stop()
{
  B_LUX_V30G_SDA0_L;                         //����������
  B_LUX_V30G_SCL0_H;                         //����ʱ����
  B_LUX_V30G_Delay5us();                     //��ʱ
  B_LUX_V30G_SDA0_H;                         //����������
  B_LUX_V30G_Delay5us();                     //��ʱ
  B_LUX_V30G_SCL0_L;
  B_LUX_V30G_Delay5us();
}

/*---------------------------------------------------------------------
 ��������: ����Ӧ���ź�
 ����˵��: ack - Ӧ���ź�(0:ACK 1:NAK)
 ��������: ��
 ---------------------------------------------------------------------*/
vid B_LUX_V30G_SendACK(uint8 ack)
{
  if (ack&0x01)	B_LUX_V30G_SDA0_H;		//дӦ���ź�
  else	B_LUX_V30G_SDA0_L;
  
  B_LUX_V30G_SCL0_H;                         //����ʱ����
  B_LUX_V30G_Delay5us();                     //��ʱ
  B_LUX_V30G_SCL0_L;                         //����ʱ����
  B_LUX_V30G_SDA0_H;
  B_LUX_V30G_Delay5us();                     //��ʱ
}

/*---------------------------------------------------------------------
 ��������: ����Ӧ���ź�
 ����˵��: ��
 ��������: ����Ӧ���ź�
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_RecvACK()
{
  uint8 CY = 0x00;
  B_LUX_V30G_SDA0_H;
  
  B_LUX_V30G_SDA0_I;
  
  B_LUX_V30G_SCL0_H;                         //����ʱ����
  B_LUX_V30G_Delay5us();                     //��ʱ
  
  if (B_LUX_V30G_SDA0_DAT)
	  CY = 1;                 				 				//��Ӧ���ź�
  
  B_LUX_V30G_Delay5us();                     //��ʱ
  
  B_LUX_V30G_SCL0_L;                         //����ʱ����
  
  B_LUX_V30G_SDA0_O;
  
  return CY;
}

/*---------------------------------------------------------------------
 ��������: ��IIC���߷���һ���ֽ�����
 ����˵��: dat - д�ֽ�
 ��������: ��
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_SendByte(uint8 dat)
{
  uint8 i;
  
  for (i=0; i<8; i++)         						//8λ������
  {
    if (dat&0x80)	B_LUX_V30G_SDA0_H;
    else	B_LUX_V30G_SDA0_L;                   //�����ݿ�
    
    B_LUX_V30G_Delay5us();             		//��ʱ
    B_LUX_V30G_SCL0_H;                		//����ʱ����
    B_LUX_V30G_Delay5us();             		//��ʱ
    B_LUX_V30G_SCL0_L;                		//����ʱ����
    B_LUX_V30G_Delay5us();             		//��ʱ
    dat <<= 1;              							//�Ƴ����ݵ����λ
  }
  
	return B_LUX_V30G_RecvACK();
}

/*---------------------------------------------------------------------
 ��������: ��IIC���߽���һ���ֽ�����
 ����˵��: ��
 ��������: �����ֽ�
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_RecvByte()
{
  uint8 i;
  uint8 dat = 0;
  B_LUX_V30G_SDA0_I;
  
  B_LUX_V30G_SDA0_H;                         	//ʹ���ڲ�����,׼����ȡ����,
  for (i=0; i<8; i++)         	        			//8λ������
  {
    B_LUX_V30G_SCL0_H;                       	//����ʱ����
    B_LUX_V30G_Delay5us();             				//��ʱ
	
	if (B_LUX_V30G_SDA0_DAT)
		dat |= 1;              										//������ 
	
    B_LUX_V30G_SCL0_L;                       	//����ʱ����
    B_LUX_V30G_Delay5us();             				//��ʱ
    
		if (i<7) dat <<= 1;
  }
  B_LUX_V30G_SDA0_O;
  
  return dat;
}

/*---------------------------------------------------------------------
 ��������: ��������λ
 ����˵��: ��
 ��������: ���ؽ�� 0�ɹ� ����0ʧ��
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_Reset()
{
	uint8 vRval = 0x00;
	
	B_LUX_V30G_Start();                                //��ʼ�ź�
  vRval += B_LUX_V30G_SendByte(0x00);    						 //����ͨ���豸��ַ+д�ź�
  vRval += B_LUX_V30G_SendByte(0xC0);								 //д��λ����
	vRval += B_LUX_V30G_SendByte(0x00);
  B_LUX_V30G_Stop();                                 //ֹͣ�ź�
	
	return vRval;
}

/*---------------------------------------------------------------------
 ��������: �޸Ĵ������豸��ַ
 ����˵��: vAddr - �豸��ַ
 ��������: ���ؽ�� 0�ɹ� ����0ʧ��
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_SetDevAddr(uint8_t vAddr)
{
	uint8 vRval = 0x00;
	
	B_LUX_V30G_Start();                                //��ʼ�ź�
  vRval += B_LUX_V30G_SendByte(0x00);   			  		 //����ͨ���豸��ַ+д�ź�
  vRval += B_LUX_V30G_SendByte(0xA0);								 //д�豸��ַ�Ĵ���
	vRval += B_LUX_V30G_SendByte(vAddr);
  B_LUX_V30G_Stop();                                 //ֹͣ�ź�
	
	return vRval;
}

/*---------------------------------------------------------------------
 ��������: �������ָ�Ĭ������
 ����˵��: ��
 ��������: ���ؽ�� 0�ɹ� ����0ʧ��
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_DefCfg()
{
	uint8 vRval = 0x00;
	
	B_LUX_V30G_Start();                                //��ʼ�ź�
	vRval += B_LUX_V30G_SendByte(0x00);    			   		 //����ͨ���豸��ַ+д�ź�
	vRval += B_LUX_V30G_SendByte(0xB0);								 //д�ָ�������������
	vRval += B_LUX_V30G_SendByte(0x00);
	B_LUX_V30G_Stop();                                 //ֹͣ�ź�
	
	return vRval;
}

/*---------------------------------------------------------------------
 ��������: ��ʼ�����մ�����
 ����˵��: ��
 ��������: ��
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
  
	B_LUX_V30G_Delay_nms(100);	                        //��ʱ100ms
	
	//д���ò���, ���ô������ɼ��ٶ�Ϊ6����1����죬��ֵԽ��ɼ��ٶ�Խ����
	vRval += B_LUX_V30G_WriteCfg(B_LUX_V30G_CONFIG_REG, 0x06);			
	B_LUX_V30G_Delay_nms(100);
	B_LUX_V30G_Reset();
  
	return vRval;
}



/*---------------------------------------------------------------------
 ��������: д�Ĵ������� 
 ����˵��: vRegAddr - �Ĵ�����ַ
					 vDat - �Ĵ�������
 ��������: ���ؽ�� 0�ɹ� ����0ʧ��
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_WriteCfg(uint8 vRegAddr, uint8 vDat)
{
	uint8 vRval = 0x00;
	
	B_LUX_V30G_Start();                                //��ʼ�ź�
  vRval += B_LUX_V30G_SendByte(B_LUX_V30G_SlaveAddress+0);    //�����豸��ַ+���ź�
  vRval += B_LUX_V30G_SendByte(vRegAddr);
  vRval += B_LUX_V30G_SendByte(vDat); 
  B_LUX_V30G_Stop();                                 //ֹͣ�ź�
	
	return vRval;
}

/*---------------------------------------------------------------------
 ��������: CRC8У��
 ����˵��: 	*vDat - ����ָ��
						vLen - У�����ݳ���
 ��������:  ����CRC8У��ֵ
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
 ��������: ���ն�ȡ����
 ����˵��: ��
 ��������: ���� 0�ɼ��ɹ� ����0 ʧ��
 ---------------------------------------------------------------------*/
uint8 B_LUX_V30G_GetLux(uint32 *vLux)
{  
	uint8 vCrc8 = 0;
  uint8 vRval = 0;
  uint8 vBuf[5];                       							//�������ݻ�����  
  uint32 val32 = 0;  
  fp32 temp;
  uint8 i = 0;
  

  //---------------------------������
  B_LUX_V30G_Start();                                //��ʼ�ź�
	vRval += B_LUX_V30G_SendByte(B_LUX_V30G_SlaveAddress+0);         //�����豸��ַ+д�ź�
  vRval += B_LUX_V30G_SendByte(0x00);
 
  B_LUX_V30G_Start();          	                     //��ʼ�ź�
  vRval += B_LUX_V30G_SendByte(B_LUX_V30G_SlaveAddress+1);         //�����豸��ַ+���ź�
  B_LUX_V30G_Delay_nms(10);                         //��ʱ10ms
  for (i=0; i<5; i++)                           		//������ȡ5���ֽ����ݣ��洢��BUF
  {
    vBuf[i] = B_LUX_V30G_RecvByte();  
    if (i == 4)																			 //�ӵ�0���ֽڣ���ʼ�ж�����һ�ֽڣ�����NACK
    {
      B_LUX_V30G_SendACK(1);                         //���һ��������Ҫ��NOACK
    }
    else
    {		
      B_LUX_V30G_SendACK(0);                         //��ӦACK
    }
  }
  B_LUX_V30G_Stop();                                 //ֹͣ�ź�
  
  //---------------------------
  
  
  /*
   //---------------------------д����
  B_LUX_V30G_Start();                                //��ʼ�ź�
  B_LUX_V30G_SendByte(B_LUX_V30G_SlaveAddress+0);    //�����豸��ַ+д�ź�
  B_LUX_V30G_SendByte(0x10);
  for (i=0; i<=112; i++)                             //����д����
  {
    B_LUX_V30G_SendByte(0x30+i); 
  }
  //B_LUX_V30G_SendByte(0x00); 
  B_LUX_V30G_Stop();                                 //ֹͣ�ź�
  //---------------------------
  */
  
  
	vCrc8 = CheckCrc8(vBuf, 4);												 //�������ݽ���CRC8У��
	if (vCrc8 == vBuf[4])															 //�ж�CRC8����У��
	{
		val32   = vBuf[3];
		val32 <<= 8;
		val32  |= vBuf[2];
		val32 <<= 8;
		val32  |= vBuf[1];
		val32 <<= 8;
		val32  |= vBuf[0];
  
		temp = (fp32)val32*1.4;												//����͸����ǹ��նȽ���ֵ
		//temp = (fp32)val32/0.4;												//�����ɫ��ǹ��նȽ���ֵ
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
                // ����
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
	// ȥ��������С����ƽ��
  uint64_t sum = 0;
  for (i = 1; i < N - 1; i++)  // ȥ�� arr[0] ��Сֵ��arr[N-1] ���ֵ
  {
     sum += arr[i];
  }

  return ((uint32)(sum / (N - 2)))/0.4/1000;  // ����ƽ��ֵ
}

	
	
