

#include "led_fc.h"
#include "include.h"
#include "mpu6050.h"
#include "hml5833l.h"
#include "beep.h"
void LEDRGB_COLOR(u8 color);
void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(ANO_RCC_LED,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
	GPIO_Init(ANO_GPIO_LED, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	 //SEL
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   =GPIO_Pin_15 ;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	LEDRGB_COLOR(BLUE);
	Delay_ms(500);
	LEDRGB_COLOR(RED);
	Delay_ms(500);
	LEDRGB_COLOR(GREEN);
  Delay_ms(500);

}


void LEDRGB(u8 sel,u8 on)
{
switch(sel)
{
case BLUE:
if(!on)
GPIO_ResetBits(GPIOC,GPIO_Pin_7);
else
GPIO_SetBits(GPIOC,GPIO_Pin_7);
break;
case RED:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_5);
else
GPIO_SetBits(GPIOA,GPIO_Pin_5);
break;
case GREEN:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_6);
else
GPIO_SetBits(GPIOA,GPIO_Pin_6);
break;
}
}
u8 LED[3]={0};
void LEDRGB_COLOR(u8 color)
{
switch (color)
{
case RED:
LED[0]=1;
LED[1]=0;
LED[2]=0;
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case BLUE:
LED[0]=0;
LED[1]=1;
LED[2]=0;	
LEDRGB(RED,0);
LEDRGB(BLUE,1);
LEDRGB(GREEN,0);
break;
case GREEN:
LED[0]=0;
LED[1]=0;
LED[2]=1;	
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
case WHITE:
LED[0]=1;
LED[1]=1;
LED[2]=1;
LEDRGB(RED,1);
LEDRGB(BLUE,1);
LEDRGB(GREEN,1);
break;
case BLACK:
LED[0]=0;
LED[1]=0;
LED[2]=0;	
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case YELLOW:
LED[0]=1;
LED[1]=0;
LED[2]=1;	
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
}
}
#define IDLE 0
#define CAL_MPU 1
#define CAL_M  2
#include "circle.h"
void LEDRGB_STATE(float dt)
{
static u8 main_state;
static u8 mpu_state,m_state,idle_state;
static u16 cnt,cnt_idle;
static u8 beep_sel;
u8 mode_control;
	
	
switch(main_state)
{ 
	
	case IDLE:
	if(mpu6050.Gyro_CALIBRATE)
	{idle_state=0;main_state=CAL_MPU;

	}
	else if(Mag_CALIBRATED)
	{idle_state=0;main_state=CAL_M;

	}
	break;
	case CAL_MPU:
  break;
	case CAL_M:
  break;
}
//   | | | |    | | | |   | | | |   | | | |   | | | |
//    ARM          GPS1     GPS2      GPS3      MODE  
#define RGB_DELAY 3
static u8 cnt_gps;
static u8 flag_cnt_gps;
if(cnt_gps++>1){cnt_gps=0;
	flag_cnt_gps=!flag_cnt_gps;
}
idle_state=0;
{
main_state=0;
	switch(idle_state)
	{//ARM
		case 0:
			if(main_state==IDLE)
				{idle_state=1;cnt_idle=0;}
		break;
		case 1:
			 {LEDRGB_COLOR(RED);fc_state_beep[0]=BEEP_STATE1;} 
		if(cnt_idle++>0.1/0.05)
		{idle_state=2;cnt_idle=0;}
		break;
		case 2:
			LEDRGB_COLOR(BLACK);	
		if(cnt_idle++>0.1/0.05)
		{idle_state=3;cnt_idle=0;}
		break;
		
		case 3:
		{LEDRGB_COLOR(RED);fc_state_beep[1]=BEEP_STATE1;} 
		if(cnt_idle++>0.1/0.05)
		{idle_state=4;cnt_idle=0;}
		break;
		case 4:
			LEDRGB_COLOR(BLACK);	
		if(cnt_idle++>0.1/0.05)
		{idle_state=5;cnt_idle=0;}
		break;
		
		case 5:
			 {LEDRGB_COLOR(RED);fc_state_beep[2]=BEEP_STATE1; }
		if(cnt_idle++>0.1/0.05)
		{idle_state=6;cnt_idle=0;}
		break;
		case 6:
			LEDRGB_COLOR(BLACK);	
		if(cnt_idle++>1.2/0.05)
		{idle_state=0;cnt_idle=0;}
		break;
	}
  Play_Music_Task(BEEP_STATE,dt);
}
}


