
#include "include.h"
#include "usart_fc.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "rc.h"

void Usart1_Init(u32 br_num)//-------UPload_board1
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART1, ENABLE); 


}
#define USE_MINI_BOARD 1
void Usart2_Init(u32 br_num)//--GOL-link
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	#if USE_MINI_BOARD
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	#else
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
  #endif
	

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART2, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART2, ENABLE); 
}


void Usart3_Init(u32 br_num)//-------GPS_board
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

	
   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART3, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART3, ENABLE); 
//	//使能发送（进入移位）中断
}


void Usart4_Init(u32 br_num)//-------SD_board
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

	

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(UART4, ENABLE);  //使能串口1 
	
	USART_ClearFlag(UART4, USART_FLAG_TC);
	

	//使能USART2接收中断
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(UART4, ENABLE); 

}



void Uart5_Init(u32 br_num)//-----odroid
{
	USART_InitTypeDef USART_InitStructure;
	//USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	
	//配置PC12作为UART5　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD2作为UART5　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置UART5
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(UART5, &USART_InitStructure);
	


	//使能UART5接收中断
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	//使能USART5
	USART_Cmd(UART5, ENABLE); 


}

void Uart6_Init(u32 br_num)//-----odroid
{
	USART_InitTypeDef USART_InitStructure;
	//USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
	
	//配置PC12作为UART5　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD2作为UART5　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	//配置UART5
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(USART6, &USART_InitStructure);
	


	//使能UART5接收中断
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	//使能USART5
	USART_Cmd(USART6, ENABLE); 
}

//original head
//void UsartSend_LEG1(uint8_t ch)
//{
//while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
//USART_SendData(USART2, ch); 
//}
//void UsartSend_LEG2(uint8_t ch)
//{
//while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
//USART_SendData(USART3, ch); 
//}
//void UsartSend_LEG3(uint8_t ch)
//{
//while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
//USART_SendData(UART4, ch); 
//}
//void UsartSend_LEG4(uint8_t ch)
//{
//while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
//USART_SendData(UART5, ch); 
//}

void UsartSend_LEG_DJ(uint8_t ch)
{
while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, ch); 
}

void UsartSend_LEG4(uint8_t ch)
{
while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
USART_SendData(USART2, ch); 
}
void UsartSend_LEG3(uint8_t ch)
{
while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); 
}
void UsartSend_LEG2(uint8_t ch)
{
while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
USART_SendData(UART4, ch); 
}
void UsartSend_LEG1(uint8_t ch)
{
while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
USART_SendData(UART5, ch); 
}
static void UsartSend_LEG_BUF(u8 *dataToSend , u8 length,u8 sel)
{
u16 i;
  for(i=0;i<length;i++)
	#if USE_DJ_CONTROL_BOARD
	UsartSend_LEG_DJ(dataToSend[i]);
	#else
   switch(sel){
	case 1:		 
	UsartSend_LEG1(dataToSend[i]);break;
	case 2:		 
	UsartSend_LEG2(dataToSend[i]);break;
	case 3:		 
	UsartSend_LEG3(dataToSend[i]);break;
	case 4:		 
	UsartSend_LEG4(dataToSend[i]);break;}
	#endif
		 
}

//#1P1500T100

char* my_itoa(int value,char *str,int radix)
{
	int sign = 0;
	//char *s = str;
	char ps[32];int i=0,j=0;
	memset(ps,0,32);
	
	if(value < 0)
	{
		sign = -1;
		value = -value;
	}
	do
	{
		if(value%radix>9)
			ps[i] = value%radix +'0'+7;
		else 
			ps[i] = value%radix +'0';
		i++;
	}while((value/=radix)>0);
	if(sign<0)
		ps[i] = '-';
	else
		i--;
	for(j=i;j>=0;j--)
	{
		str[i-j] = ps[j];
	}
	return str;
}
u16 test_leg[3]={1234,12,123};
void UsartSend_LEG_BUF_BUF(u8 sel)
{u8 i=0,j,cnt=0;
 char conver[4][5]={0}	;
 char conver_id[4][5]={0}	;
 u8 buf[40]={0};
 static u8 id_sel[5]={0};
 
if(leg[sel].sys.id&&leg[sel].leg_power){
  buf[cnt++]='#';
  #if USE_DJ_CONTROL_BOARD
	my_itoa(leg[sel].sys.pwm_id[id_sel[sel]],&conver_id[id_sel[sel]][0],10);	
 	for(j=0;j<4;j++)
		if(conver_id[id_sel[sel]][j]!=0)
			buf[cnt++]=conver_id[id_sel[sel]][j];
		else
			break;
  #else
  buf[cnt++]=(id_sel[sel])+1+48;
  #endif
  #if USE_DJ_CONTROL_BOARD
  buf[cnt++]=' '; 
  #endif
  buf[cnt++]='P';
	my_itoa(leg[sel].sys.PWM_OUT[id_sel[sel]],&conver[id_sel[sel]][0],10);	
	//my_itoa(test_leg[i],conver[i],10);	
	for(j=0;j<4;j++)
		if(conver[id_sel[sel]][j]!=0)
			buf[cnt++]=conver[id_sel[sel]][j];
		else
			break;
  #if USE_DJ_CONTROL_BOARD
  buf[cnt++]=' '; 
  #endif		
	buf[cnt++]='T';
  buf[cnt++]='1';
  buf[cnt++]=0x0d;
  buf[cnt++]=0x0a;	

  if(id_sel[sel]++>=4)	
		id_sel[sel]=0;
	
	//UsartSend_LEG_BUF(buf,cnt,sel);
}
}

void UsartSend_LEG_BUF_BUF_2(u8 sel)
{u8 i=0,j,l,cnt=0;
 char conver[4][5]={0}	;
 char conver_id[4][5]={0}	;
 u8 buf[15*5]={0};
 static u8 id_sel[5]={0};
 
if(leg[sel].sys.id&&leg[sel].leg_power){
	for(i=0;i<4;i++){
  buf[cnt++]='#';
	my_itoa(leg[sel].sys.pwm_id[i],&conver_id[i][0],10);	
 	for(j=0;j<4;j++)
		if(conver_id[i][j]!=0)
			buf[cnt++]=conver_id[i][j];
		else
			break;

  buf[cnt++]=' '; 

  buf[cnt++]='P';
	my_itoa(leg[sel].sys.PWM_OUT[i],&conver[i][0],10);	
	for(j=0;j<4;j++)
		if(conver[i][j]!=0)
			buf[cnt++]=conver[i][j];
		else
			break;
  buf[cnt++]=' '; 
	buf[cnt++]='T';
  buf[cnt++]='1';
  buf[cnt++]=0x0d;
  buf[cnt++]=0x0a;	
  buf[cnt++]=0x00;
  buf[cnt++]=0x00;
 }
	UsartSend_LEG_BUF(buf,cnt,sel);
}
}
//#1PULK 
void LEG_POWER_OFF(u8 sel)
{u8 i=0,j,cnt=0;
 u8 buf[40]={0};

 
  for(i=0;i<3;i++){
  buf[cnt++]='#';
  buf[cnt++]=i+1+48;
  buf[cnt++]='P';
	buf[cnt++]='U';
  buf[cnt++]='L';
	buf[cnt++]='K';	
  buf[cnt++]=0x0d;
  buf[cnt++]=0x0a;	
  }
	
	UsartSend_LEG_BUF(buf,cnt,sel);
}

void Send_LEG(u8 sel)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	#if !USE_BUS_DJ
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=sel;//功能字
	data_to_send[_cnt++]=0;//数据量
  _temp=leg[sel].pos_tar[0].x*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	 _temp=leg[sel].pos_tar[0].x*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	 _temp=leg[sel].pos_tar[0].x*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	 _temp=leg[sel].pos_tar[1].x*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	 _temp=leg[sel].pos_tar[1].y*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	 _temp=leg[sel].pos_tar[1].z*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	 _temp=leg[sel].pos_tar[2].x*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	 _temp=leg[sel].pos_tar[2].y*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	 _temp=leg[sel].pos_tar[2].z*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=leg[sel].sys.sita_tar[0]*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=leg[sel].sys.sita_tar[1]*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=leg[sel].sys.sita_tar[2]*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	if(brain.control_mode)
	_temp=1;	
	else
	_temp=leg[sel].control_mode;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=brain.sys.leg_use_ground;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=0;//brain.control_mode;
	data_to_send[_cnt++]=BYTE0(_temp);
	if(brain.power_all)
	_temp=1;	
	else
	_temp=leg[sel].leg_power;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=leg[sel].deng[0]*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=leg[sel].deng[1]*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp=brain.sys.center_off.x*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=brain.sys.center_off.y*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
		_temp=brain.sys.leg_t*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
		_temp=brain.sys.leg_h*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=brain.sys.off_leg[sel].x*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=brain.sys.off_leg[sel].y*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=brain.sys.off_leg[sel].z*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
  data_to_send[3] = _cnt-4;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	UsartSend_LEG_BUF(data_to_send, _cnt,sel);
	#else
	//if(leg[sel].leg_power)
	//UsartSend_LEG_BUF_BUF(sel);
	UsartSend_LEG_BUF_BUF_2(sel);
	#endif
}

//-----------------------------------------------CMD Topic
void Data_LEG_CMD(u8 *data_buf,u8 num,u8 sel)
{ static u8 cnt[4];
	u8 id;
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)>=1&&*(data_buf+2)<=4)//FLOW_MINE_frame
  { id=*(data_buf+2);
	  brain.leg_connect=1;
		brain.sys.leg_loss_cnt=0;
	  leg[id].pos_tar[0].x=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10;
		leg[id].pos_tar[0].y=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10;
	  leg[id].pos_tar[0].z=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10;
		leg[id].pos_tar[1].x=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/10;
		leg[id].pos_tar[1].y=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/10;
		leg[id].pos_tar[1].z=(float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/10;
		leg[id].pos_tar[2].x=(float)((int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/10;
		leg[id].pos_tar[2].y=(float)((int16_t)(*(data_buf+18)<<8)|*(data_buf+19)/10);
		leg[id].pos_tar[2].z=(float)((int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/10;
		leg[id].sys.sita_tar[0]		 =(float)((int16_t)(*(data_buf+22)<<8)|*(data_buf+23))/10;
		leg[id].sys.sita_tar[1]		 =(float)((int16_t)(*(data_buf+24)<<8)|*(data_buf+25))/10;
		leg[id].sys.sita_tar[2]		 =(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27))/10;
	  leg[id].control_mode		 							 =*(data_buf+28);
		leg[id].deng[0]		 =(float)((int16_t)(*(data_buf+29)<<8)|*(data_buf+30))/1;
		leg[id].deng[1]		 =(float)((int16_t)(*(data_buf+31)<<8)|*(data_buf+32))/1;
    
	
	}		
}

void Data_Receive_Anl1_RC(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头

  if(*(data_buf+2)==0x66)//RC_GET1
  {
		for(i=0;i<32;i++)
		NRF24L01_RXDATA[i]=*(data_buf+i+4);
		
	  NRF_DataAnl();
  
	}
}

u8 TxBuffer1[256];
u8 TxCounter1=0;
u8 count1=0; 

u8 Rx_Buf1[256];	//串口接收缓存
u8 RxBuffer1[50];
u8 RxState1 = 0;
u8 RxBufferNum1 = 0;
u8 RxBufferCnt1 = 0;
u8 RxLen1 = 0;
static u8 _data_len1 = 0,_data_cnt1 = 0;
void USART1_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(USART1->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART1->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断标志

		com_data = USART1->DR;
		if(RxState1==0&&com_data==0xAA)
		{
			RxState1=1;
			RxBuffer1[0]=com_data;
		}
		else if(RxState1==1&&com_data==0xAF)
		{
			RxState1=2;
			RxBuffer1[1]=com_data;
		}
		else if(RxState1==2&&com_data>0&&com_data<0XF1)
		{
			RxState1=3;
			RxBuffer1[2]=com_data;
		}
		else if(RxState1==3&&com_data<50)
		{
			RxState1= 4;
			RxBuffer1[3]=com_data;
			_data_len1 = com_data;
			_data_cnt1 = 0;
		}
		else if(RxState1==4&&_data_len1>0)
		{
			_data_len1--;
			RxBuffer1[4+_data_cnt1++]=com_data;
			if(_data_len1==0)
				RxState1= 5;
		}
		else if(RxState1==5)
		{
			RxState1 = 0;
			RxBuffer1[4+_data_cnt1]=com_data;
			Data_LEG_CMD(RxBuffer1,_data_cnt1+5,1);
			Data_Receive_Anl1_RC(RxBuffer1,_data_cnt1+5);
		}
		else
			RxState1 = 0;
	}

	//发送（进入移位）中断
	if( USART_GetITStatus(USART1,USART_IT_TXE ) )
	{
				
		USART1->DR = TxBuffer1[TxCounter1++]; //写DR清除中断标志          
		if(TxCounter1 == count1)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}

   OSIntExit(); 

}

//leg1
u8 set1=0;
 void Data_LEG(u8 *data_buf,u8 num,u8 sel)
{ static u8 cnt[4];
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)==sel)//FLOW_MINE_frame
  {
	if(cnt[sel]++>100){cnt[sel]=0;	LEDRGB_STATE(sel);}
	  leg[sel].leg_connect=1;
		leg[sel].sys.leg_loss_cnt=0;
	  leg[sel].pos_now[0].x=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10;
		leg[sel].pos_now[0].y=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10;
	  leg[sel].pos_now[0].z=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10;
		leg[sel].pos_now[1].x=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/10;
		leg[sel].pos_now[1].y=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/10;
		leg[sel].pos_now[1].z=(float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/10;
		leg[sel].pos_now[2].x=(float)((int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/10;
		leg[sel].pos_now[2].y=(float)((int16_t)(*(data_buf+18)<<8)|*(data_buf+19))/10;
		leg[sel].pos_now[2].z=(float)((int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/10;
		leg[sel].sita[0]		 =(float)((int16_t)(*(data_buf+22)<<8)|*(data_buf+23))/10;
		leg[sel].sita[1]		 =(float)((int16_t)(*(data_buf+24)<<8)|*(data_buf+25))/10;
		leg[sel].sita[2]		 =(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27))/10;
	  if(*(data_buf+28))
			 leg[sel].sys.leg_ground_cnt++;
		else
		{leg[sel].leg_ground= leg[sel].sys.leg_ground_cnt=0;}
		if(leg[sel].sys.leg_ground_cnt>set1)
	  leg[sel].leg_ground=1;		 							   
	
	
	  leg[sel].err		 					 						 =*(data_buf+29);
	  leg[sel].leg_end_force[2]=(float)((int16_t)(*(data_buf+30)<<8)|*(data_buf+31))/1000.;
	  leg[sel].pos_tar_trig[2].x=(float)((int16_t)(*(data_buf+32)<<8)|*(data_buf+33))/10;
	  leg[sel].pos_tar_trig[2].y=(float)((int16_t)(*(data_buf+34)<<8)|*(data_buf+35))/10;
	  leg[sel].pos_tar_trig[2].z=(float)((int16_t)(*(data_buf+36)<<8)|*(data_buf+37))/10;
	}		
}

u8 TxBuffer2[256];
u8 TxCounter2=0;
u8 count2=0; 

u8 Rx_Buf2[256];	//串口接收缓存
u8 RxBuffer2[50];
u8 RxState2 = 0;
u8 RxBufferNum2 = 0;
u8 RxBufferCnt2 = 0;
u8 RxLen2 = 0;
static u8 _data_len2 = 0,_data_cnt2 = 0;
void USART2_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(USART2->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART2->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志

		com_data = USART2->DR;
		if(RxState2==0&&com_data==0xAA)
		{
			RxState2=1;
			RxBuffer2[0]=com_data;
		}
		else if(RxState2==1&&com_data==0xAF)
		{
			RxState2=2;
			RxBuffer2[1]=com_data;
		}
		else if(RxState2==2&&com_data>0&&com_data<0XF1)
		{
			RxState2=3;
			RxBuffer2[2]=com_data;
		}
		else if(RxState2==3&&com_data<50)
		{
			RxState2 = 4;
			RxBuffer2[3]=com_data;
			_data_len2 = com_data;
			_data_cnt2 = 0;
		}
		else if(RxState2==4&&_data_len2>0)
		{
			_data_len2--;
			RxBuffer2[4+_data_cnt2++]=com_data;
			if(_data_len2==0)
				RxState2= 5;
		}
		else if(RxState2==5)
		{
			RxState2 = 0;
			RxBuffer2[4+_data_cnt2]=com_data;
			Data_LEG(RxBuffer2,_data_cnt2+5,1);
		}
		else
			RxState2 = 0;
	}

	//发送（进入移位）中断
	if( USART_GetITStatus(USART2,USART_IT_TXE ) )
	{
				
		USART2->DR = TxBuffer2[TxCounter2++]; //写DR清除中断标志          
		if(TxCounter2 == count2)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}

   OSIntExit(); 

}


//leg2
u8 TxBuffer3[256];
u8 TxCounter3=0;
u8 count3=0; 
u8 Rx_Buf3[256];	//串口接收缓存
u8 RxBuffer3[50];
u8 RxState3 = 0;
u8 RxBufferNum3 = 0;
u8 RxBufferCnt3 = 0;
u8 RxLen3 = 0;
static u8 _data_len3 = 0,_data_cnt3 = 0;
void USART3_IRQHandler(void)
{  OSIntEnter();  
	u8 com_data;
	
	if(USART3->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART3->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志

		com_data = USART3->DR;
		
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
			Data_LEG(RxBuffer3,_data_cnt3+5,2);
		}
		else
			RxState3 = 0;
	
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{
				
		USART3->DR = TxBuffer2[TxCounter3++]; //写DR清除中断标志          
		if(TxCounter3 == count3)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}

 OSIntExit();        
}


//leg3

u8 TxBuffer4[256];
u8 TxCounter4=0;
u8 count4=0; 

u8 Rx_Buf4[256];	//串口接收缓存
u8 RxBuffer4[50];
u8 RxState4 = 0;
u8 RxBufferNum4 = 0;
u8 RxBufferCnt4 = 0;
u8 RxLen4= 0;
static u8 _data_len4 = 0,_data_cnt4 = 0;
void UART4_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(UART4->SR & USART_SR_ORE)//ORE中断
	{
		com_data = UART4->DR;
	}

  //接收中断
	if( USART_GetITStatus(UART4,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);//清除中断标志

		com_data = UART4->DR;
		if(RxState4==0&&com_data==0xAA)
		{
			RxState4=1;
			RxBuffer4[0]=com_data;
		}
		else if(RxState4==1&&com_data==0xAF)
		{
			RxState4=2;
			RxBuffer4[1]=com_data;
		}
		else if(RxState4==2&&com_data>0&&com_data<0XF1)
		{
			RxState4=3;
			RxBuffer4[2]=com_data;
		}
		else if(RxState4==3&&com_data<50)
		{
			RxState4 = 4;
			RxBuffer4[3]=com_data;
			_data_len4 = com_data;
			_data_cnt4 = 0;
		}
		else if(RxState4==4&&_data_len4>0)
		{
			_data_len4--;
			RxBuffer4[4+_data_cnt4++]=com_data;
			if(_data_len4==0)
				RxState4 =5;
		}
		else if(RxState4==5)
		{
			RxState4 = 0;
			RxBuffer4[4+_data_cnt4]=com_data;
			Data_LEG(RxBuffer4,_data_cnt4+5,3);
		}
		else
			RxState4 = 0;
	}

	//发送（进入移位）中断
	if( USART_GetITStatus(UART4,USART_IT_TXE ) )
	{
				
		UART4->DR = TxBuffer4[TxCounter4++]; //写DR清除中断标志          
		if(TxCounter4 == count4)
		{
			UART4->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}
   OSIntExit(); 

}



//leg4

u8 TxBuffer5[256];
u8 TxCounter5=0;
u8 count5=0; 

u8 Rx_Buf5[256];	//串口接收缓存
u8 RxBuffer5[50];
u8 RxState5 = 0;
u8 RxBufferNum5 = 0;
u8 RxBufferCnt5 = 0;
u8 RxLen5= 0;
static u8 _data_len5 = 0,_data_cnt5 = 0;
void UART5_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(UART5->SR & USART_SR_ORE)//ORE中断
	{
		com_data = UART5->DR;
	}

  //接收中断
	if( USART_GetITStatus(UART5,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);//清除中断标志

		com_data = UART5->DR;
		if(RxState5==0&&com_data==0xAA)
		{
			RxState5=1;
			RxBuffer5[0]=com_data;
		}
		else if(RxState5==1&&com_data==0xAF)
		{
			RxState5=2;
			RxBuffer5[1]=com_data;
		}
		else if(RxState5==2&&com_data>0&&com_data<0XF1)
		{
			RxState5=3;
			RxBuffer5[2]=com_data;
		}
		else if(RxState5==3&&com_data<50)
		{
			RxState5 = 4;
			RxBuffer5[3]=com_data;
			_data_len5 = com_data;
			_data_cnt5 = 0;
		}
		else if(RxState5==4&&_data_len5>0)
		{
			_data_len5--;
			RxBuffer5[4+_data_cnt5++]=com_data;
			if(_data_len5==0)
				RxState5= 5;
		}
		else if(RxState5==5)
		{
			RxState5 = 0;
			RxBuffer5[4+_data_cnt5]=com_data;
			Data_LEG(RxBuffer5,_data_cnt5+5,4);
		}
		else
			RxState5 = 0;
	}

	//发送（进入移位）中断
	if( USART_GetITStatus(UART5,USART_IT_TXE ) )
	{
				
		UART5->DR = TxBuffer2[TxCounter5++]; //写DR清除中断标志          
		if(TxCounter5 == count5)
		{
			UART5->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}
   OSIntExit(); 

}


u8 SendBuff1[SEND_BUF_SIZE1];
u8 SendBuff3[SEND_BUF_SIZE3];



u16 leg_uart_cnt;
void data_per_uart1(u8 sel)
{
	u8 i;	u8 sum = 0;
	u16 _cnt=0,cnt_reg;
	vs16 _temp;
  vs32 _temp32;

	cnt_reg=leg_uart_cnt;
  SendBuff1[leg_uart_cnt++]=0xAA;
	SendBuff1[leg_uart_cnt++]=0xAF;
	if(leg[sel].leg_connect)
	SendBuff1[leg_uart_cnt++]=sel;
	else
	SendBuff1[leg_uart_cnt++]=9;	
	SendBuff1[leg_uart_cnt++]=0;//数据量
  
	_temp = leg[sel].pos_now[0].x*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	_temp = leg[sel].pos_now[0].y*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	_temp = leg[sel].pos_now[0].z*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	
	_temp = leg[sel].pos_now[1].x*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	_temp = leg[sel].pos_now[1].y*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	_temp = leg[sel].pos_now[1].z*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	
	_temp = leg[sel].pos_now[2].x*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	_temp = leg[sel].pos_now[2].y*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	_temp = leg[sel].pos_now[2].z*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);

  
	_temp = leg[sel].sita[0]*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	_temp = leg[sel].sita[1]*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	_temp = leg[sel].sita[2]*10;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
  
	_temp = leg[sel].leg_ground;
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
  _temp = leg[sel].err;
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	
	
  _temp = leg[sel].leg_end_force[0]*1000;
	SendBuff1[leg_uart_cnt++]=BYTE1(_temp);
	SendBuff1[leg_uart_cnt++]=BYTE0(_temp);
	

	SendBuff1[cnt_reg+3] = leg_uart_cnt-cnt_reg-4;
	for( i=cnt_reg;i< leg_uart_cnt;i++)
		sum += SendBuff1[i];
	SendBuff1[leg_uart_cnt++] = sum;
}

void GOL_LINK_TASK_DMA(void)//5ms
{
static u8 cnt[10];
static u8 flag[10];
u8 i;
//传感器值

data_per_uart1(1);
data_per_uart1(2);
data_per_uart1(3);
data_per_uart1(4);
end_gol_link1:;
}

void clear_leg_uart(void)
{u16 i;
leg_uart_cnt=0;
for(i=0;i<SEND_BUF_SIZE1;i++)
SendBuff1[i]=0;

}
