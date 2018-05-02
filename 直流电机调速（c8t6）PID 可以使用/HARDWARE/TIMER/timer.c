#include "timer.h"
#include "usart.h"
#include "sys.h"


//PA0,PA1,PA2,PA3  ������A��ķ������ڣ���������ٶ�
//PA6,PA7���Ƶ��1,PB0,PB1���Ƶ��2
//PB6,PB7���Ƶ��2,PB8,PB9���Ƶ��4



//�����8·PWM������ͨ��ʱ�������������
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6|GPIO_Pin_7; //TIM_CH1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_6|GPIO_Pin_7);					
	
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1; //TIM_CH1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_1);			
	
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler =psc;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  
	
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
  TIM_OCInitStructure.TIM_Pulse = 0;  
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  
	
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);  
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  
	
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);  
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  
	
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);  
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  
	
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);  
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  
	
  TIM_CtrlPWMOutputs(TIM3,ENABLE);  
  TIM_ARRPreloadConfig(TIM3, ENABLE); 
  TIM_Cmd(TIM3, ENABLE);  
	
}

void TIM4_PWM_Init(u16 arr,u16 psc)
{  
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; //TIM_CH1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);					 
	
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler =psc;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);  
	
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
  TIM_OCInitStructure.TIM_Pulse = 0;  
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  
	
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);  
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  
	
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);  
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  
	
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);  
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  
	
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);  
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  
	
  TIM_CtrlPWMOutputs(TIM4,ENABLE);  
  TIM_ARRPreloadConfig(TIM4, ENABLE); 
  TIM_Cmd(TIM4, ENABLE);  
	
}


//��ͨ��ʱ��2������ÿ��������ٶ�


TIM_ICInitTypeDef  TIM2_ICInitStructure;

void TIM2_Cap_Init(u16 arr,u16 psc)
{	 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//ʹ��TIM2ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIOAʱ��
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;  //PA0 ���֮ǰ����  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0 ����  
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);						 //PA0 ����
	
	//��ʼ����ʱ��2 TIM2	 
	TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 	//Ԥ��Ƶ��   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
  
	//��ʼ��TIM2���벶�����
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
  TIM2_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
  TIM2_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	
	//�жϷ����ʼ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 
	
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);//��������ж� ,����CC1IE�����ж�	
	
  TIM_Cmd(TIM2,ENABLE ); 	//ʹ�ܶ�ʱ��2
 
}


u8  TIM2CH1_CAPTURE_STA=0;	//���벶��״̬		    				
u16	TIM2CH1_CAPTURE_VAL;	//���벶��ֵ
u32 time_sum=0;
u32 time1=0;
u32 time2=0;
u32 time3=0;
u32 time4=0;
u32 time11=0;
u32 time12=0;
u32 time21=0;
u32 time22=0;
u32 time31=0;
u32 time32=0;
u32 time41=0;
u32 time42=0;
double step=43*3.1415926/7/210;
float v1=0;
float v2=0;
float v3=0;
float v4=0;
void TIM2_IRQHandler(void)
{ 
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)//����1���������¼�
	{
	    TIM2CH1_CAPTURE_VAL=TIM_GetCapture1(TIM2);
      time11=time12;
		  time12=TIM2CH1_CAPTURE_STA*65536+TIM2CH1_CAPTURE_VAL;
		  time1=time12-time11;
		  v1=(float)step*1000000/time1;
	}
  else if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)	
	{
			TIM2CH1_CAPTURE_VAL=TIM_GetCapture2(TIM2);
      time21=time22;
		  time22=TIM2CH1_CAPTURE_STA*65536+TIM2CH1_CAPTURE_VAL;
		  time2=time22-time21;
		  v2=(float)step*1000000/time2;
	}
  else if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)	
	{
			TIM2CH1_CAPTURE_VAL=TIM_GetCapture3(TIM2);
      time31=time32;
		  time32=TIM2CH1_CAPTURE_STA*65536+TIM2CH1_CAPTURE_VAL;
		  time3=time32-time31;
	  	v3=(float)step*1000000/time3;
	}
	 else if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)	
	{
			TIM2CH1_CAPTURE_VAL=TIM_GetCapture4(TIM2);
      time41=time42;
		  time42=TIM2CH1_CAPTURE_STA*65536+TIM2CH1_CAPTURE_VAL;
		  time4=time42-time41;
		  v4=(float)step*1000000/time4;
	}
	else
	{
	  TIM2CH1_CAPTURE_STA+=1;
	}
  TIM_ClearITPendingBit(TIM2, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update); //����жϱ�־λ
}











double SumError1=0;
double LastError1=0;
double revError1=0;
double PrevError1;
void motor1_PID(u8 direction,int v_set,int v,double kp,double ki,double kd)
{
			double dError,Error;
      Error = v_set-v; //  
      SumError1 += Error; // 
      dError = LastError1 - revError1; // 
      PrevError1 = LastError1;
      LastError1 = Error;			
			 if(direction==0x00)     //�������
			{
		    if(v_set>(int)v)
				{
				 TIM_SetCompare1(TIM3,(kp * Error + ki *SumError1 +kd * dError));
				}
				else
				{
				 TIM_SetCompare1(TIM3,(kp * Error + ki *SumError1 +kd * dError));         
				}
				TIM_SetCompare2(TIM3,0);
			}else
			{
			  if(v_set>(int)v)
				{
				 TIM_SetCompare2(TIM3,(kp * Error + ki *SumError1 +kd * dError));
				}
				else
				{
				 TIM_SetCompare2(TIM3,(kp * Error + ki *SumError1 +kd * dError));         
				}
				TIM_SetCompare1(TIM3,0);
			}		
}

double SumError2=0;
double LastError2=0;
double revError2=0;
double PrevError2;
void motor2_PID(u8 direction,int v_set,int v,double kp,double ki,double kd)
{
			double dError,Error;
      Error = v_set-v; //  
      SumError2 += Error; // 
      dError = LastError2 - revError2; // 
      PrevError2 = LastError2;
      LastError2 = Error;			
		  if(direction==0x00)     //�������
			{
		    if(v_set>(int)v)
				{					
				 TIM_SetCompare3(TIM3,(kp * Error + ki *SumError2 +kd * dError));
				}
				else
				{
				 TIM_SetCompare3(TIM3,(kp * Error + ki *SumError2 +kd * dError));         
				}
				TIM_SetCompare4(TIM3,0);
			}else
			{
			  if(v_set>(int)v)
				{
				 TIM_SetCompare4(TIM3,(kp * Error + ki *SumError2 +kd * dError));
				}
				else
				{
				 TIM_SetCompare4(TIM3,(kp * Error + ki *SumError2 +kd * dError));         
				}
				TIM_SetCompare3(TIM3,0);	
			}	
}

double SumError3=0;
double LastError3=0;
double revError3=0;
double PrevError3;
void motor3_PID(u8 direction,int v_set,int v,double kp,double ki,double kd)
{
			double dError,Error;
      Error = v_set-v; //  
      SumError3 += Error; // 
      dError = LastError3 - revError3; // 
      PrevError3 = LastError3;
      LastError3 = Error;			
      if(direction==0x00)     //�������
			{
		    if(v_set>(int)v)
				{
				 TIM_SetCompare1(TIM4,(kp * Error + ki *SumError3 +kd * dError));
				}
				else
				{
				 TIM_SetCompare1(TIM4,(kp * Error + ki *SumError3 +kd * dError));         
				}
				TIM_SetCompare2(TIM4,0);			
			}else
			{
			  if(v_set>(int)v)
				{
				 TIM_SetCompare2(TIM4,(kp * Error + ki *SumError3 +kd * dError));
				}
				else
				{
				 TIM_SetCompare2(TIM4,(kp * Error + ki *SumError3 +kd * dError));         
				}
				TIM_SetCompare1(TIM4,0);
			}		
}

double SumError4=0;
double LastError4=0;
double revError4=0;
double PrevError4;
void motor4_PID(u8 direction,int v_set,int v,double kp,double ki,double kd)
{
			double dError,Error;
      Error = v_set-v; //  
      SumError4 += Error; // 
      dError = LastError4- revError4; // 
      PrevError4 = LastError4;
      LastError4 = Error;			
		  if(direction==0x00)     //�������
			{
		    if(v_set>(int)v)
				{
				 TIM_SetCompare3(TIM4,(kp * Error + ki *SumError4 +kd * dError));
				}
				else
				{
				 TIM_SetCompare3(TIM4,(kp * Error + ki *SumError4 +kd * dError));         
				}
				TIM_SetCompare4(TIM4,0);
			}else
			{
			  if(v_set>(int)v)
				{
				 TIM_SetCompare4(TIM4,(kp * Error + ki *SumError4 +kd * dError));
				}
				else
				{
				 TIM_SetCompare4(TIM4,(kp * Error + ki *SumError4 +kd * dError));         
				}
				TIM_SetCompare3(TIM4,0);		
			}	
}


