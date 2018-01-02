/* MAIN.C file
 * 
 * stm32-project
 */
/***************************************************************
/ 深圳EU电子出品-版权所有-翻版必究
/ EU-热爱嵌入式开发
/ http://euse.taobao.com
***************************************************************/
//类型转换
typedef unsigned char       bool;
typedef unsigned char       u8;
typedef unsigned short      u16;
#define  True  1
#define  False 0

//SET BIT.    Example: a |= SETBIT0
enum
{
	SETBIT0 = 0x0001,  SETBIT1 = 0x0002,	SETBIT2 = 0x0004,	 SETBIT3 = 0x0008,
	SETBIT4 = 0x0010,	 SETBIT5 = 0x0020,	SETBIT6 = 0x0040,	 SETBIT7 = 0x0080,
	SETBIT8 = 0x0100,	 SETBIT9 = 0x0200,	SETBIT10 = 0x0400, SETBIT11 = 0x0800,
	SETBIT12 = 0x1000, SETBIT13 = 0x2000,	SETBIT14 = 0x4000, SETBIT15 = 0x8000		
};
//CLR BIT.    Example: a &= CLRBIT0
enum
{
	CLRBIT0 = 0xFFFE,  CLRBIT1 = 0xFFFD,	CLRBIT2 = 0xFFFB,	 CLRBIT3 = 0xFFF7,	
	CLRBIT4 = 0xFFEF,	 CLRBIT5 = 0xFFDF,	CLRBIT6 = 0xFFBF,	 CLRBIT7 = 0xFF7F,
	CLRBIT8 = 0xFEFF,	 CLRBIT9 = 0xFDFF,	CLRBIT10 = 0xFBFF, CLRBIT11 = 0xF7FF,
	CLRBIT12 = 0xEFFF, CLRBIT13 = 0xDFFF,	CLRBIT14 = 0xBFFF, CLRBIT15 = 0x7FFF
};
//CHOSE BIT.  Example: a = b&CHSBIT0
enum
{
	CHSBIT0 = 0x0001,  CHSBIT1 = 0x0002,	CHSBIT2 = 0x0004,	 CHSBIT3 = 0x0008,
	CHSBIT4 = 0x0010,	 CHSBIT5 = 0x0020,	CHSBIT6 = 0x0040,	 CHSBIT7 = 0x0080,
	CHSBIT8 = 0x0100,	 CHSBIT9 = 0x0200,	CHSBIT10 = 0x0400, CHSBIT11 = 0x0800,
	CHSBIT12 = 0x1000, CHSBIT13 = 0x2000,	CHSBIT14 = 0x4000, CHSBIT15 = 0x8000		
};

/* INCLUDES */
//MCU
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "delay.h"
//PWM接口开启 并执行初始化

void PWM_Set(const uint16_t pwm1, const uint16_t pwm2)
{
	TIM_SetCompare1(TIM1, pwm1);
	TIM_SetCompare2(TIM1, pwm2);
}
#define LED PBout(5)	
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define  MAX_PWM 20000  //50hz
void time_init(){
 GPIO_InitTypeDef GPIO_InitStructure2;         
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;        
 TIM_OCInitTypeDef TIM_OCInitStructure;        
 TIM_BDTRInitTypeDef TIM_BDTRInitStructure;               
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_TIM1,ENABLE);        
 GPIO_InitStructure2.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9;         
 GPIO_InitStructure2.GPIO_Speed=GPIO_Speed_50MHz;         
 GPIO_InitStructure2.GPIO_Mode=GPIO_Mode_AF_PP;                        
 GPIO_Init(GPIOA,&GPIO_InitStructure2);        
 TIM_TimeBaseStructure.TIM_Period=20000;                       
 TIM_TimeBaseStructure.TIM_Prescaler=72-1;                         
 TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;         
 TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;         
 TIM_TimeBaseStructure.TIM_RepetitionCounter=0;                         
 TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);      
 TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;                           
 TIM_OCInitStructure.TIM_Pulse=500;                                         
 TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;                    
 TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;                 
 TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;                
 TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;      
 TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;             
 TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;     
 TIM_OC1Init(TIM1,&TIM_OCInitStructure);                            
 TIM_OC1Init(TIM1, &TIM_OCInitStructure);  					
 TIM_OC2Init(TIM1, &TIM_OCInitStructure);  					
 TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);      
 TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);     
 TIM_ARRPreloadConfig(TIM1, ENABLE);                        
 TIM_Cmd(TIM1,ENABLE);                                     
 TIM_CtrlPWMOutputs(TIM1, ENABLE);                                   
 TIM_SetCompare1(TIM1,1500);
 TIM_SetCompare2(TIM1,1500);
}



void uart_init(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能USART1，GPIOA时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART1，GPIOA时钟
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART3, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART3, ENABLE);                    //使能串口1 

}



int dj[2]={1500,1500};
void Data_Receive_Anl3(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;	
  if(*(data_buf+2)==0x66)
  {
	 LED=!LED;	
   dj[0]=((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
   dj[1]=((int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
	}				
}

u8 Rx_Buf3[256];	
u8 RxBuffer3[50];
u8 RxState3 = 0;
u8 RxBufferNum3 = 0;
u8 RxBufferCnt3 = 0;
u8 RxLen3 = 0;
static u8 _data_len3 = 0,_data_cnt3 = 0;
void USART3_IRQHandler(void)
{   
	u8 com_data;
	
	if(USART3->SR & USART_SR_ORE)//ORE??
	{
		com_data = USART3->DR;
	}
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//??????

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
			Data_Receive_Anl3(RxBuffer3,_data_cnt3+5);
		}
		else
			RxState3 = 0;
	
	}
	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{
				
		USART_ClearITPendingBit(USART3,USART_IT_TXE);
	}        
}

void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PA,PD端口时钟
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //LED0-->PA.8 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.8
 GPIO_SetBits(GPIOB,GPIO_Pin_5);						 //PA.8 输出高
 GPIO_SetBits(GPIOB,GPIO_Pin_5); 						 //PD.2 输出高 
}
 

/*-------------------------------------------------------------------------------------------------------
*  程序从这里执行				
-------------------------------------------------------------------------------------------------------*/
int test_dj[2]={1500,1500};
int main(void)
{	
	  delay_init();	    	 //延时函数初始化	  
	  time_init();
	  uart_init(576000L);
	  LED_Init();
   	while(1)
	{  
 		delay_ms(10);	   					 
		PWM_Set(LIMIT(dj[0],500+100,2500-100),LIMIT(dj[1],500+100,2500-100));
	} 
}

