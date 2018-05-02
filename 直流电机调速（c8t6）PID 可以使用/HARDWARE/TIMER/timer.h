#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

void TIM2_Cap_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM4_PWM_Init(u16 arr,u16 psc);
void motor1_PID(u8 direction,int v_set,int v,double kp,double ki,double kd);
void motor2_PID(u8 direction,int v_set,int v,double kp,double ki,double kd);
void motor3_PID(u8 direction,int v_set,int v,double kp,double ki,double kd);
void motor4_PID(u8 direction,int v_set,int v,double kp,double ki,double kd);
#endif
