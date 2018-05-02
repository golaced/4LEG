#include "delay.h"
#include "sys.h"
#include "timer.h"
#include "usart.h"
#include "led.h"
//测量速度
extern float v1;  
extern float v2;
extern float v3;
extern float v4;
extern u8  USART_RX_BUF[USART_REC_LEN]; 
u8  buffer[USART_REC_LEN];
extern u8  direction1;
extern u8  direction2;
extern u8  direction3;
extern u8  direction4;
//给定速度
extern int v1_set;
extern int v2_set;
extern int v3_set;
extern int v4_set;

//PID
double kp=0.5; //  
double ki=0.5; //  
double kd=0.5; // 
 int main(void)
 {	
 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	delay_init();	    	 //延时函数初始化	
	uart_init(115200);	 			//115200	 
 	TIM3_PWM_Init(999,72-1); 			//不分频。PWM频率=72000/72/(999+1)=1Khz
  TIM4_PWM_Init(999,72-1);
  TIM2_Cap_Init(0XFFFF,72-1);		//以1Mhz的频率计数 
	
  while(1)
		
	
	{
		delay_ms(10);
    //2+1+1+2+2+2+2+1=13位
   //接收格式 8888+F1+数据长度(0x0D) +电机1正反转(0x00/0x01)
		//+速度(0x00)+电机2正反转(0x00/0x01)+速度(0x00)+电机3正反转(0x00/0x01)+速度(0x00)	+电机4正反转(0x00/0x01)+速度(0x00)+校验位		
		
  //    `Data_Receive_Anl(USART_RX_BUF,12);

		  //电机1
      motor1_PID(direction1, v1_set, v1, kp, ki, kd);			
	    //电机2
		  motor2_PID(direction2, v2_set, v2, kp, ki, kd);	
	    //电机3
	  	motor3_PID(direction3, v3_set, v3, kp, ki, kd);		
		  //电机4
	  	motor4_PID(direction4, v4_set, v4, kp, ki, kd);	
			
			if((v1>200)||(v2>200)||(v3>200)||(v4>200))
			{
			  v1=0;
				v2=0;
				v3=0;
				v4=0;
			}
      send_out_D1(direction1,v1,direction2,v2,direction3,v3,direction4,v4);
			// printf("电机1 %i  %f mm/s 电机2 %i  %f mm/s 电机3 %i  %f mm/s 电机4 %i  %f mm/s \r\n",(int)direction1,(double)v1,(int)direction2,(double)v2,(int)direction3,(double)v3,(int)direction4,(double)v4);	
			 // send_out_D1(v1,v2,v3,v4);	
	}
	
} 
