#include "include.h" 
#include "iic_soft.h"
#include "hml_sample.h"
#include "ms5611.h"
#include "ms5611_2.h"
#include "hml5833l.h"
#include "alt_kf.h"
#include "flash.h"
#include "led_fc.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "ucos_task.h"
#include "pwm_in.h"
#include "flow.h"
#include "usart_fc.h"
#include "sbus.h"
#include "gps.h"
#include "filter.h"

float leg_dt[5];
OS_STK LEG1_TASK_STK[LEG_STK_SIZE];
void leg1_task(void *pdata)
{
 u8 i;
 static u8 init;	
 static u16 cnt_init;
	leg_init(&leg[1],1);
 	while(1)
	{
	leg_dt[0] = Get_Cycle_T(GET_T_LEG1); 						//获取内环准确的执行周期
	leg_dt[0]=0.005;	
//  leg_drive(&leg[1],0.01);//leg_dt[0]);
//  //Send_LEG(1);
//  //UsartSend_LEG_BUF_BUF(1);
	MPU6050_Read(); 															//??mpu6????
  ANO_AK8975_Read_Mag_Data();
	MPU6050_Data_Prepare( leg_dt[0] );			//mpu6????????

	if(cnt_init++>1/0.005){cnt_init=65530;
 
 	IMUupdate(0.5f *leg_dt[0],mpu6050_fc.Gyro_deg.x, mpu6050_fc.Gyro_deg.y, mpu6050_fc.Gyro_deg.z, mpu6050_fc.Acc.x, mpu6050_fc.Acc.y, mpu6050_fc.Acc.z
	,&Roll,&Pitch,&Yaw);
	
	
  float a_br[3],acc_temp[3];
	static float acc_flt[3];
	a_br[0] =(float) mpu6050_fc.Acc.x/4096.;//16438.;
	a_br[1] =(float) mpu6050_fc.Acc.y/4096.;//16438.;
	a_br[2] =(float) mpu6050_fc.Acc.z/4096.;//16438.;
	acc_temp[0] = a_br[1]*reference_vr[2]  - a_br[2]*reference_vr[1] ;
	acc_temp[1] = a_br[2]*reference_vr[0]  - a_br[0]*reference_vr[2] ;
	acc_temp[2] = reference_vr[2] *a_br[2] + reference_vr[0] *a_br[0]+ reference_vr[1] *a_br[1] - 1 ;
	
	
	brain.now_acc[0]=acc_flt[0] = -firstOrderFilter(acc_temp[0] ,&firstOrderFilters[ACC_LOWPASS_X],leg_dt[0])*9.8;
	brain.now_acc[1]=acc_flt[1] = firstOrderFilter(acc_temp[1] ,&firstOrderFilters[ACC_LOWPASS_Y],leg_dt[0])*9.8;
  brain.now_acc[2]=acc_flt[2] = firstOrderFilter(acc_temp[2] ,&firstOrderFilters[ACC_LOWPASS_Z],leg_dt[0])*9.8;
  }

	delay_ms(5);
	}
}		

OS_STK LEG2_TASK_STK[LEG_STK_SIZE];
void leg2_task(void *pdata)
{
 u8 i;
 static u8 init;	
	leg_init(&leg[2],2);
 	while(1)
	{
	leg_dt[1] = Get_Cycle_T(GET_T_LEG2); 						//获取内环准确的执行周期
  leg_drive(&leg[2],0.02);//leg_dt[1]);
 // Send_LEG(2);

	delay_ms(6);
	}
}		
OS_STK LEG3_TASK_STK[LEG_STK_SIZE];
void leg3_task(void *pdata)
{
 u8 i;
 static u8 init;	
	leg_init(&leg[3],3);
 	while(1)
	{
	leg_dt[2] = Get_Cycle_T(GET_T_LEG3); 						//获取内环准确的执行周期
  leg_drive(&leg[3],0.02);//leg_dt[2]);
  //Send_LEG(3);

	delay_ms(6);
	}
}		
OS_STK LEG4_TASK_STK[LEG_STK_SIZE];
void leg4_task(void *pdata)
{
 u8 i;
 static u8 init;	
	leg_init(&leg[4],4);
 	while(1)
	{
	leg_dt[3] = Get_Cycle_T(GET_T_LEG4); 						//获取内环准确的执行周期
  leg_drive(&leg[4],0.02);//leg_dt[3]);
 // Send_LEG(4);

	delay_ms(6);
	}
}		

//========================外环  任务函数============================路径规划
float k_rc_spd=0.005;
float k_z_c= 0.16;
OS_STK BRAIN_TASK_STK[BRAIN_STK_SIZE];
float test[5]={1,1,4};
float k_rc[2]={0.015,0.015};
float Yaw_set;
float kp_yaw=0.68;
void brain_task(void *pdata)
{	static u8 cnt,cnt1,cnt2,init,rc_update;	
	float T;float spd,spdy,spdx,yaw=0,w_rad;
	u16 temps;
 	while(1)
	{	
	leg_dt[4] = Get_Cycle_T(GET_T_BRAIN);								//获取外环准确的执行周期
  T=0.02;
	if(Rc_Get_SBUS.connect&&Rc_Get_SBUS.update){
	temps=((channels[0])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
	if(temps>900&&temps<2100)
	Rc_Get_SBUS.ROLL=		 temps;
	temps=((channels[1])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
	if(temps>900&&temps<2100)
	Rc_Get_SBUS.PITCH=		 temps;
	temps=((channels[2])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
	if(temps>900&&temps<2100)
	Rc_Get_SBUS.THROTTLE=		 temps;
	temps=((channels[3])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
	if(temps>900&&temps<2100)
	Rc_Get_SBUS.YAW=		 temps;
	temps=((channels[4])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
	if(temps>900&&temps<2100)
	Rc_Get_SBUS.AUX1=		 temps;
	temps=((channels[5])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
	if(temps>900&&temps<2100)
	Rc_Get_SBUS.AUX2=		 temps;
	temps=((channels[6])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
	if(temps>900&&temps<2100)
	Rc_Get_SBUS.AUX3=		 temps;
	temps=((channels[7])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
	if(temps>900&&temps<2100)
	Rc_Get_SBUS.AUX4=		 temps;
	Rc_Get_PWM.THROTTLE=Rc_Get_PWM.ROLL=Rc_Get_PWM.PITCH=Rc_Get_PWM.YAW=1500;
	Rc_Get_PWM.THROTTLE=LIMIT(Rc_Get_SBUS.THROTTLE,1000,2000)	;
	Rc_Get_PWM.ROLL=my_deathzoom_rc(Rc_Get_SBUS.ROLL,2)	;
	Rc_Get_PWM.PITCH=my_deathzoom_rc(Rc_Get_SBUS.PITCH,2)	;
	Rc_Get_PWM.YAW=my_deathzoom_rc(Rc_Get_SBUS.YAW,2)	;
	Rc_Get_PWM.AUX1=Rc_Get_SBUS.AUX1;
	Rc_Get_PWM.AUX2=Rc_Get_SBUS.AUX2;
	Rc_Get_PWM.AUX3=Rc_Get_SBUS.AUX3;
	Rc_Get_PWM.AUX4=Rc_Get_SBUS.AUX4;
	Rc_Get_PWM.connect=Rc_Get_SBUS.connect;
	Rc_Get_PWM.update=Rc_Get_SBUS.update;
	Rc_Get_PWM.POS_MODE=Rc_Get_SBUS.AUX3;
	Rc_Get_PWM.HEIGHT_MODE=Rc_Get_SBUS.AUX4;
	
	static u8 flag;
	if(Rc_Get_PWM.AUX1>1500)
	{brain.power_all=brain.control_mode=1;flag=1;}
	else
	 brain.power_all=brain.control_mode=0;
	
	static u16 cnt_soft_rst;
	brain.sys.desire_time=LIMIT(0.5+(Rc_Get_PWM.AUX4-1500)/1000.,0.2,2);
	if(brain.trot_gait)
	brain.sys.desire_time=LIMIT(brain.sys.desire_time,0.3,0.6);
	
	if(flag&&Rc_Get_PWM.AUX1<1500)
	{flag=0;brain.rst_all_soft=1;cnt_soft_rst=0;}
	
	if(flag&&Rc_Get_PWM.AUX3>1500)
	  brain.trot_gait=1;
	else
		brain.trot_gait=0;
	
	
//	if(brain.rst_all_soft>0)
//		brain.sys.desire_time=0.33;
	
	if(brain.rst_all_soft>0)
		cnt_soft_rst++;
	else
		cnt_soft_rst=0;
	if(cnt_soft_rst>10/0.02)
	{brain.rst_all_soft=0;cnt_soft_rst=0;}
	
	
	 spdy=my_deathzoom((Rc_Get_PWM.PITCH-1500)*k_rc_spd,0.1);//cm
	 spdx=my_deathzoom((Rc_Get_PWM.ROLL-1500)*k_rc_spd,0.1);//cm
	 w_rad=my_deathzoom((Rc_Get_PWM.YAW-1500)*0.001*20,1);//rad.cm
	 spd=LIMIT(sqrt(pow(spdx,2)+pow(spdy,2)),0,2)*brain.sys.desire_time_init/brain.sys.desire_time;
	 
	 if(spd>0){
	 yaw=fast_atan2(spdx,spdy)*57.3;
	  if(brain.rst_all_soft>0)
			brain.rst_all_soft=0;
	 }
//	#if HORIZON_USE_FORWARD_CENTER
//  if((yaw<30&&yaw>=0)||(yaw>-30&&yaw<0))
//	{brain.spd_yaw=0;}
//	else if((yaw>180-30&&yaw>=0)||(yaw<-180+30&&yaw<0))//b
//	{brain.spd_yaw=180;}
//	else
//	 brain.spd_yaw=yaw;	
//  #else	 
	 if(yaw>60&&yaw<90+60)//r
	{brain.spd_yaw=90;}
	else if(yaw<-60&&yaw>-90-60)//l
	{brain.spd_yaw=-90;}
	else if((yaw<60&&yaw>=0)||(yaw>-60&&yaw<0))//f
	{brain.spd_yaw=0;}
	else//b
	{brain.spd_yaw=180;}
	//#endif
	 brain.tar_w_set=w_rad;
	 if(w_rad!=0&&spd==0)
		 brain.spd=0.1;
	 else{
	 if(Rc_Get_PWM.AUX2>1500)
	 brain.spd=0.1;	 
	 else
	 brain.spd=spd;
   }
	 
	 	brain.tar_h=LIMIT(leg[1].sys.init_end_pos.z-(Rc_Get_PWM.THROTTLE-1000)/1000.*(leg[1].sys.init_end_pos.z-leg[1].sys.limit_min.z)    
	,leg[1].sys.limit_min.z,leg[1].sys.init_end_pos.z);
	 
  }else
	{
	Rc_Get_PWM.THROTTLE=1500;
	Rc_Get_PWM.ROLL=1500;
	Rc_Get_PWM.PITCH=1500;
	Rc_Get_PWM.YAW=1500;
	}
	
	if(w_rad!=0||(fabs(Yaw_set-Yaw)>25))
	Yaw_set=Yaw;
	else if(brain.spd!=0)
	brain.tar_w_set=my_deathzoom(Yaw_set-Yaw,1)*kp_yaw;
  else 
	brain.tar_w_set=0;
	
	if(((brain.tar_w_set!=0&&w_rad!=0)||brain.spd>0)&&(brain.way==1||brain.way==3))
	brain.tar_w=LIMIT(my_deathzoom(brain.tar_w_set-mpu6050_fc.Gyro_deg.z,0.1)*k_z_c,-10,10)*brain.global.value[2];
	else
	brain.tar_w=0;
	brain.tar_w=brain.tar_w_set*k_z_c*brain.global.value[2];
	static u8 state_spd_rst;
	switch(state_spd_rst){
		case 0:
	   if(brain.spd>0)
		 {state_spd_rst=1;brain.sys.no_control_cnt=0;}
		 break;
	  case 1:
			if(brain.spd==0)
			  brain.sys.no_control_cnt++;
			if(brain.sys.no_control_cnt>3/0.02)
			{state_spd_rst=2;brain.rst_all_soft=1;}
			break;
		case 2:
			if(brain.rst_all_soft==0)
				state_spd_rst=0;
			break;
		}
	if(Rc_Get_SBUS.update==0&&rc_update==1)
	brain.rst_all_soft=1;
	rc_update=Rc_Get_SBUS.update;
		
	//test
	
	leg_task1(T);
	leg_drive(&leg[1],T);
	leg_drive(&leg[2],T);
	leg_drive(&leg[3],T);
	leg_drive(&leg[4],T);
	Set_DJ_PWM();
	delay_ms(20);
	}
}		


//=======================串口 任务函数===========================
OS_STK  UART_TASK_STK[UART_STK_SIZE];
u8 UART_UP_LOAD_SEL=4;//<------------------------------UART UPLOAD DATA SEL
u8 state_v_test=0;
u8 num_need_to_check;
void uart_task(void *pdata)
{	static u8 cnt[4];	
  static u8 sd_sel;	
 	while(1)
	{			
		Send_LEG(1);
		Send_LEG(2);
		Send_LEG(3);
		Send_LEG(4);
		GOL_LINK_TASK();	
    ReportMotion(brain.global.steady_value*1000,brain.now_acc[Xr]*1000,brain.now_acc[Yr]*1000,
		brain.global.center_stable_weight*1000,brain.global.end_pos_global[0].z*100,0,
		center_control_out[Xr],center_control_out[Yr],0);		
		delay_ms(20);  
	}
}	


//------------------------------软件定时器--------------------------------//
OS_TMR   * tmr1;			//软件定时器1
OS_TMR   * tmr2;			//软件定时器2
OS_TMR   * tmr3;			//软件定时器3

//软件定时器1的回调函数	
//每100ms执行一次,用于显示CPU使用率和内存使用率		
 u16 cpuusage=0;
void tmr1_callback(OS_TMR *ptmr,void *p_arg) 
{
	static u8 tcnt=0;	    

	if(tcnt==5)
	{
 		
		cpuusage=0;
		tcnt=0; 
	}
	cpuusage+=OSCPUUsage;
	tcnt++;				    
}
 #include "circle.h"
//软件定时器2的回调函数				  50ms	 
void tmr2_callback(OS_TMR *ptmr,void *p_arg) 
{	
u8 i;	
static u16 cnt_1,cnt_2;	
static u8 cnt;
  for(i=0;i<5;i++)
	 if(leg[i].sys.leg_loss_cnt++>2/0.05)leg[i].leg_connect=0;
	 if(brain.sys.leg_loss_cnt++>2/0.05)brain.sys.leg_connect=0;
	 if(Rc_Get_SBUS.lose_cnt++>2/0.05)Rc_Get_SBUS.connect=0;
	
}
//软件定时器3的回调函数				  	   
void tmr3_callback(OS_TMR *ptmr,void *p_arg) 
{	
 
} 


//