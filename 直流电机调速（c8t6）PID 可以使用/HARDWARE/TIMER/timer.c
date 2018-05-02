#include "timer.h"
#include "usart.h"
#include "sys.h"


//PA0,PA1,PA2,PA3  捕获电机A相的方波周期，计算输出速度
//PA6,PA7控制电机1,PB0,PB1控制电机2
//PB6,PB7控制电机2,PB8,PB9控制电机4



//电机的8路PWM　，开通定时器３，４输出。
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


//开通定时器2，计算每个电机的速度


TIM_ICInitTypeDef  TIM2_ICInitStructure;

void TIM2_Cap_Init(u16 arr,u16 psc)
{	 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//使能TIM2时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOA时钟
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;  //PA0 清除之前设置  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0 输入  
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);						 //PA0 下拉
	
	//初始化定时器2 TIM2	 
	TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 	//预分频器   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
  
	//初始化TIM2输入捕获参数
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
  TIM2_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
  TIM2_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	
	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 
	
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	
  TIM_Cmd(TIM2,ENABLE ); 	//使能定时器2
 
}


u8  TIM2CH1_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM2CH1_CAPTURE_VAL;	//输入捕获值
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
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
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
  TIM_ClearITPendingBit(TIM2, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update); //清除中断标志位
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
			 if(direction==0x00)     //电机正向
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
		  if(direction==0x00)     //电机正向
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
      if(direction==0x00)     //电机正向
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
		  if(direction==0x00)     //电机正向
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


