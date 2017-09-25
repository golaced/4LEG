

#include "led_fc.h"
#include "include.h"

void LEDRGB_COLOR(u8 color);
void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4| GPIO_Pin_5| GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}



/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/


void LEDRGB(u8 sel,u8 on)
{
switch(sel)
{
case 2:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_4);
else
GPIO_SetBits(GPIOA,GPIO_Pin_4);
break;
case 1:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_5);
else
GPIO_SetBits(GPIOA,GPIO_Pin_5);
break;
case 3:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_6);
else
GPIO_SetBits(GPIOA,GPIO_Pin_6);
break;
case 4:
if(!on)
GPIO_ResetBits(GPIOA,GPIO_Pin_7);
else
GPIO_SetBits(GPIOA,GPIO_Pin_7);
break;
}
}

#define IDLE 0
#define CAL_MPU 1
#define CAL_M  2
#include "circle.h"
void LEDRGB_STATE(u8 sel)
{
static u8 flag[4];
if(flag[sel]){flag[sel]=0;
	
	LEDRGB(sel,1);

}else{flag[sel]=1;

LEDRGB(sel,0);
	
}

}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

