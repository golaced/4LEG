#include "delay.h"
#include "sys.h"
#include "timer.h"
#include "usart.h"
#include "led.h"
//�����ٶ�
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
//�����ٶ�
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
 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	delay_init();	    	 //��ʱ������ʼ��	
	uart_init(115200);	 			//115200	 
 	TIM3_PWM_Init(999,72-1); 			//����Ƶ��PWMƵ��=72000/72/(999+1)=1Khz
  TIM4_PWM_Init(999,72-1);
  TIM2_Cap_Init(0XFFFF,72-1);		//��1Mhz��Ƶ�ʼ��� 
	
  while(1)
		
	
	{
		delay_ms(10);
    //2+1+1+2+2+2+2+1=13λ
   //���ո�ʽ 8888+F1+���ݳ���(0x0D) +���1����ת(0x00/0x01)
		//+�ٶ�(0x00)+���2����ת(0x00/0x01)+�ٶ�(0x00)+���3����ת(0x00/0x01)+�ٶ�(0x00)	+���4����ת(0x00/0x01)+�ٶ�(0x00)+У��λ		
		
  //    `Data_Receive_Anl(USART_RX_BUF,12);

		  //���1
      motor1_PID(direction1, v1_set, v1, kp, ki, kd);			
	    //���2
		  motor2_PID(direction2, v2_set, v2, kp, ki, kd);	
	    //���3
	  	motor3_PID(direction3, v3_set, v3, kp, ki, kd);		
		  //���4
	  	motor4_PID(direction4, v4_set, v4, kp, ki, kd);	
			
			if((v1>200)||(v2>200)||(v3>200)||(v4>200))
			{
			  v1=0;
				v2=0;
				v3=0;
				v4=0;
			}
      send_out_D1(direction1,v1,direction2,v2,direction3,v3,direction4,v4);
			// printf("���1 %i  %f mm/s ���2 %i  %f mm/s ���3 %i  %f mm/s ���4 %i  %f mm/s \r\n",(int)direction1,(double)v1,(int)direction2,(double)v2,(int)direction3,(double)v3,(int)direction4,(double)v4);	
			 // send_out_D1(v1,v2,v3,v4);	
	}
	
} 
