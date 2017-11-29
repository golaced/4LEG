#ifndef _PWM_OUT_H_
#define _PWM_OUT_H_

#include "stm32f4xx.h"
#define MAXMOTORS 		(4)		//�������
u8 PWM_Out_Init(uint16_t hz);
void  Set_DJ(float ang1,float ang2,float ang3,float ang4);
void SetPwm(u32 pwm[8],int off[8],u32 min,u32 max);
void LEG_POWER(u8 sel);
void SEL_Init();
u8 PWM_Out_Init_FOR_CAL(uint16_t hz,uint16_t min,uint16_t max);
void SHOOT_Init(void);
void EN_SHOOT(u8 on);
void Set_DJ_PWM(void);
extern int dj_out[12];
void SetPwm_AUX(float pit,float rol);

typedef struct 
{ u16 pwm_tem[2];
	int flag[2];
	u16 init[2];
	u16 max[2];
	u16 min[2];
	float att[2],att_ctrl[2],att_off[2];
	float pwm_per_dig[2];
	float ero[2],ero_reg[2];
}AUX_S;

extern AUX_S aux;
u8 PWM_AUX_Out_Init(uint16_t hz);//50Hz
#endif

