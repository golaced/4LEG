#include "sys.h"
#include "usart.h"	  

void uart_init(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 

}

void USART1_send_byte(uint8_t byte)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
	USART1->DR=byte;	
}
void USART1_Send(uint16_t *Buffer, uint8_t Length)
{
	uint16_t i=0;
	while(i<Length)
	{
		USART1_send_byte(Buffer[i++]);
	}
}

u8  direction1=0;
u8  direction2=0;
u8  direction3=0;
u8  direction4=0;
//给定速度
int v1_set=0;
int v2_set=0;
int v3_set=0;
int v4_set=0;



#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

 void Data_Receive_AnlPx4(u8 *data_buf,u8 num)
{
	
	vs16 rc_value_temp,rc_value_temp1;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		
		return;		//?D??sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//?D????矸
	if(*(data_buf+2)==0x01)//
  { 
			direction1=*(data_buf+4);
			v1_set=(float)((int16_t)(*(data_buf+5)<<8)|*(data_buf+6))/10.;
			direction2=*(data_buf+7);
			v2_set=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10.;
			direction3=*(data_buf+10);
			v3_set=(float)((int16_t)(*(data_buf+11)<<8)|*(data_buf+12))/10.;
			direction4=*(data_buf+13);
			v4_set=(float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/10.;
			kp=(float)((int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/10.;
			ki=(float)((int16_t)(*(data_buf+18)<<8)|*(data_buf+19))/10.;
			kd=(float)((int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/10.;
	}

}

u8 RxBuffer3[50];
u8 RxState3 = 0;
u8 RxBufferNum3 = 0;
u8 RxBufferCnt3 = 0;
u8 RxLen3 = 0;
static u8 _data_len3 = 0,_data_cnt3 = 0;
void USART1_IRQHandler(void)
{ // OSIntEnter();  
	u8 com_data;
	
	if(USART1->SR & USART_SR_ORE)//ORE??
	{
		com_data = USART1->DR;
	}

  //????
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//??????

		com_data = USART1->DR;
		if(RxState3==0&&com_data==0xAA)
		{
			RxState3=1;
			RxBuffer3[0]=com_data;
		}
		else if(RxState3==1&&com_data==0xAF)
		{
			RxState3=2;
			RxBuffer3[1]=com_data;
		}
		else if(RxState3==2&&com_data>0&&com_data<0XF1)
		{
			RxState3=3;
			RxBuffer3[2]=com_data;
		}
		else if(RxState3==3&&com_data<50)
		{
			RxState3 = 4;
			RxBuffer3[3]=com_data;
			_data_len3 = com_data;
			_data_cnt3 = 0;
		}
		else if(RxState3==4&&_data_len3>0)
		{
			_data_len3--;
			RxBuffer3[4+_data_cnt3++]=com_data;
			if(_data_len3==0)
				RxState3 = 5;
		}
		else if(RxState3==5)
		{
			RxState3 = 0;
			RxBuffer3[4+_data_cnt3]=com_data;
		    Data_Receive_AnlPx4(RxBuffer3,_data_cnt3+5);
		}
		else
			RxState3 = 0;
	
	}
}



// void Data_Receive_Anl1(u8 *data_buf,u8 num)
//{
//	u8 sum = 0;
//	u8 i;
//	//for( i=0;i<(num-1);i++)
//	//	sum += *(data_buf+i);
//	//u8 sum_in=*(data_buf+num-1);
//	//if(!(sum==*(data_buf+num-1)))		
//	//	return;
//	  if(USART_RX_STA&0x4000)
//		{				
//	     if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		 
//	     if(*(data_buf+2)==0x01)//
//       {  
//         direction1=*(data_buf+3);
//		     v1_set=*(data_buf+4);
//		     direction2=*(data_buf+5);
//		     v2_set=*(data_buf+6);
//		     direction3=*(data_buf+7);
//		     v3_set=*(data_buf+8);   
//		     direction4=*(data_buf+9);
//		     v4_set=*(data_buf+10);	
//	      }
//				USART_RX_STA=0;
//		}
//}

void send_out_D1(u16 temp1,u16 temp2,u16 temp3,u16 temp4,u16 temp5,u16 temp6,u16 temp7,u16 temp8)
{ uint8_t b;	
	uint16_t TX_DATA[20],i=0;
	TX_DATA[i++]=0xAA;
	TX_DATA[i++]=0xAF;
	TX_DATA[i++]=0x01;	
	TX_DATA[i++]=temp1;
  TX_DATA[i++]=temp2;
	TX_DATA[i++]=temp3;
  TX_DATA[i++]=temp4;
	TX_DATA[i++]=temp5;
  TX_DATA[i++]=temp6;
	TX_DATA[i++]=temp7;
  TX_DATA[i++]=temp8;
	b=0xAA+0xAF+0x01+temp1+temp2+temp3+temp4+temp5+temp6+temp7+temp8;
  TX_DATA[i++]=b;
	USART1_Send(TX_DATA,12);
}


