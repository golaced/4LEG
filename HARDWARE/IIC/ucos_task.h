#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"


extern OS_TMR   * tmr1;			//软件定时器1
extern OS_TMR   * tmr2;			//软件定时器2
extern OS_TMR   * tmr3;			//软件定时器3
void tmr1_callback(OS_TMR *ptmr,void *p_arg); 		  	   
void tmr2_callback(OS_TMR *ptmr,void *p_arg); 	  	   
void tmr3_callback(OS_TMR *ptmr,void *p_arg); 
	
//设置任务优先级 0->Highest     
//10->开始任务的优先级设置为最低


#define LEG1_TASK_PRIO       			  1 //MEMS
#define LEG2_TASK_PRIO       			  2 //MEMS
#define LEG3_TASK_PRIO       			  3 //MEMS
#define LEG4_TASK_PRIO       			  4 //MEMS

#define BRAIN_TASK_PRIO       			6 //MEMS

#define UART_TASK_PRIO       			  9 //MEMS



//-----------------------LEG解算线程
//设置任务堆栈大小
#define LEG_STK_SIZE  					64*4
//任务堆栈	
extern OS_STK LEG1_TASK_STK[LEG_STK_SIZE];
//任务函数
void leg1_task(void *pdata);


//任务堆栈	
extern OS_STK LEG2_TASK_STK[LEG_STK_SIZE];
//任务函数
void leg2_task(void *pdata);

//任务堆栈	
extern OS_STK LEG3_TASK_STK[LEG_STK_SIZE];
//任务函数
void leg3_task(void *pdata);

//任务堆栈	
extern OS_STK LEG4_TASK_STK[LEG_STK_SIZE];
//任务函数
void leg4_task(void *pdata);

//-----------------------LEG解算线程
//设置任务堆栈大小
#define BRAIN_STK_SIZE  					64*16
//任务堆栈	
extern OS_STK BRAIN_TASK_STK[BRAIN_STK_SIZE];
//任务函数
void brain_task(void *pdata);

//------------------------UART线程
//设置任务堆栈大小
#define UART_STK_SIZE  					64*4
//任务堆栈	
extern OS_STK UART_TASK_STK[UART_STK_SIZE];
//任务函数
void uart_task(void *pdata);
//


#endif

