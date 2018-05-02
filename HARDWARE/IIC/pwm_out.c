
#include "pwm_in.h"
#include "pwm_out.h"
#include "include.h"
#include "my_math.h"
#include "usart_fc.h"

//21��Ƶ�� 84000000/21 = 4M   0.25us
u32 Rc_Pwm_Inr_mine[8];
u32 Rc_Pwm_In_mine[8],Rc_Pwm_Out_mine[8]={1500,1500,1500,1500,1500,1500,1500,1500};
PWMIN pwmin;
#define INIT_DUTY 4000 //u16(1000/0.25)
#define ACCURACY 10000 //u16(2500/0.25) //accuracy
#define PWM_RADIO 4//(8000 - 4000)/1000.0

u8 PWM_Out_Init(uint16_t hz)//400hz
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	uint16_t PrescalerValue = 0;
	u32 hz_set = ACCURACY*hz;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
	
	hz_set = LIMIT (hz_set,1,84000000);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

/////////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5|GPIO_Pin_6 | GPIO_Pin_7|GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);
	
	
	/* Compute the prescaler value */
  PrescalerValue = (uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE);//--------------------------��̨ ���
/////////////////////////////////////////////////////////////////////////////
	/* Compute the prescaler value */
  PrescalerValue = (uint16_t) ( ( SystemCoreClock/2 ) / hz_set ) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  //TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  //TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);	

  Set_DJ_PWM();
	if( hz_set > 84000000 )
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
/*                        head
 3                                                    1


   

4        3d 3x 3t | 4d 4x 4t || 2t  2x  2d | 1t 1x 1d   2   
*/
int dj_out[12]={1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500};
int dj_pan[2]={1500,1500};
void Set_DJ_PWM(void)
{ static u8 cnt=2;
	dj_out[11]=leg[1].sys.PWM_OUT[D_LEG];
	dj_out[10]=leg[1].sys.PWM_OUT[X_LEG];
	dj_out[9] =leg[1].sys.PWM_OUT[T_LEG];
	dj_out[8] =leg[2].sys.PWM_OUT[D_LEG];
	dj_out[7] =leg[2].sys.PWM_OUT[X_LEG];
	dj_out[6] =leg[2].sys.PWM_OUT[T_LEG];
	dj_out[0] =leg[3].sys.PWM_OUT[D_LEG];
	dj_out[1] =leg[3].sys.PWM_OUT[X_LEG];
	dj_out[2] =leg[3].sys.PWM_OUT[T_LEG];
	dj_out[3] =leg[4].sys.PWM_OUT[D_LEG];
	dj_out[4] =leg[4].sys.PWM_OUT[X_LEG];
	dj_out[5] =leg[4].sys.PWM_OUT[T_LEG];
	
  TIM3->CCR1 = ( LIMIT(dj_out[0],600,2400)/2 ) ;				//1	
	TIM3->CCR2 = ( LIMIT(dj_out[1],600,2400)/2 ) ;				//2
	TIM3->CCR3 = ( LIMIT(dj_out[2],600,2400)/2 ) ;				//3	
	TIM3->CCR4 = ( LIMIT(dj_out[3],600,2400)/2 ) ;				//4
	TIM4->CCR1 = ( LIMIT(dj_out[4],600,2400)/2 ) ;				//1	
	TIM4->CCR2 = ( LIMIT(dj_out[5],600,2400)/2 ) ;				//2
	TIM4->CCR3 = ( LIMIT(dj_out[6],600,2400)/2 ) ;				//3	
	TIM4->CCR4 = ( LIMIT(dj_out[7],600,2400)/2 ) ;				//4
	TIM1->CCR1 = ( LIMIT(dj_out[8],600,2400)/2 ) ;				//4
	TIM1->CCR4 = ( LIMIT(dj_out[9],600,2400)/2 ) ;				//4
	//if(cnt++>=2){cnt=0;
	Send_DJ11_12();
//}
}	

u8 PWM_AUX_Out_Init(uint16_t hz)//50Hz
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_OCInitTypeDef  TIM_OCInitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		uint16_t PrescalerValue = 0;
		u32 hz_set = ACCURACY*hz*2;

		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_OCStructInit(&TIM_OCInitStructure);

		hz_set = LIMIT (hz_set,1,84000000);

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	
		//////////////////////////////////////TIM8///////////////////////////////////////////
	  hz_set = ACCURACY*hz;
		hz_set = LIMIT (hz_set,1,84000000)/2;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_DOWN;//GPIO_PuPd_UP ;
		GPIO_Init(GPIOA, &GPIO_InitStructure); 

		GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);
		//------------------
		/* Compute the prescaler value */
		PrescalerValue = (uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		/* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC1Init(TIM1, &TIM_OCInitStructure);
		TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
		TIM_OC4Init(TIM1, &TIM_OCInitStructure);


		TIM_CtrlPWMOutputs(TIM1, ENABLE);
		TIM_ARRPreloadConfig(TIM1, ENABLE);
		TIM_Cmd(TIM1, ENABLE);	
}
