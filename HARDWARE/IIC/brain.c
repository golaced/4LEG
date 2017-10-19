#include "include.h" 
u8 trig_list_trot[5]={0,14,23,14,23};
//------------------------------------------------------------brain---------------------------------------------------------------------
//       
//<-----W-------->
//			  y
//	3----------1          /\
// 	     |                |
//			 O                L
//			 |                |
//	4----------2   x			\/
u8 stop_leg=4;
void barin_init(BRAIN_STRUCT *in)
{
float W=5.7  *2;//cm	
float L=21.0;
in->sys.yaw_trig=fast_atan2(W/2,L/2)*57.3;	
	
in->sys.leg_local[1].x=W/2;
in->sys.leg_local[1].y=L/2;
in->sys.leg_local[1].z=0;	
	
in->sys.leg_local[2].x=W/2;
in->sys.leg_local[2].y=-L/2;
in->sys.leg_local[2].z=0;	
	
in->sys.leg_local[3].x=-W/2;
in->sys.leg_local[3].y=L/2;
in->sys.leg_local[3].z=0;	
	
in->sys.leg_local[4].x=-W/2;
in->sys.leg_local[4].y=-L/2;
in->sys.leg_local[4].z=0;	

in->global.area_of_leg[1]=in->area_of_leg[1]=W*L;
	
in->sys.min_range=0.707*leg[1].sys.l3*0.3;//7.8 4.742556
in->sys.max_range=0.707*leg[1].sys.l2*0.78;//6.3 3.830526
in->sys.leg_move_range[Yr]=MAX(in->sys.max_range, in->sys.min_range);//cm		
in->sys.leg_move_range[Xr]=W/4*0.88;//cm	
//for leg 1
in->sys.leg_move_range1[1]=0.707*leg[1].sys.l2;
in->sys.leg_move_range1[2]=sin(22/57.3)*(leg[1].sys.l2+leg[1].sys.l3);
in->sys.leg_move_range1[3]=0.707*leg[1].sys.l3*0.3/1.618;
in->sys.leg_move_range1[4]=MIN(sin(35/57.3)*(leg[1].sys.l2+leg[1].sys.l3)*0.618,W/2*0.268)/1.618;

in->sys.kp_center[0]=0.8;
in->sys.kp_center[1]=0.6;
in->sys.k_center_fp=0.0;

in->sys.move_range_k=0.66;

in->min_st[0]=0.69;//cm
in->min_st[1]=in->min_st[0];
in->sys.k_center_c[0]=100;
in->sys.k_center_c[1]=100;

in->sys.att_off[0]=4;
in->sys.att_off[1]=4;

in->sys.center_off.x=0;
in->sys.center_off.y=0;
in->sys.center_off1.x=0;
in->sys.center_off1.y=0;

in->sys.center_off_when_move[Xr]=0;//-0.2;
in->sys.center_off_when_move[Yr]=0;//-0.88;
in->sys.leg_t=0.5;
in->sys.leg_h[1]=4.68;
in->sys.leg_h[2]=4.68;
in->sys.leg_h[3]=4.68;
in->sys.leg_h[4]=4.68;

#if USE_LEG_TRIG_DELAY // BEST TIM 0.4
in->sys.desire_time=0.6;//0.7;//0.76;
#else
in->sys.desire_time=0.8;//0.7;//0.76;
#endif
in->sys.desire_time_init=in->sys.desire_time;
in->sys.leg_move_min_dt=in->sys.desire_time*0.68;//35 ;

in->sys.k_spd_to_range=2;

in->sys.yaw_dead=10;

in->sys.down_h=0;//0.88;
in->sys.down_t=0;//0.2;
in->sys.in_rst_check=1.8;

leg[1].sys.id=1;
leg[2].sys.id=2;
leg[3].sys.id=3;
leg[4].sys.id=4;

brain.tar_h=leg[1].sys.limit_min.z*1.618;
#if TEST_MODE1
in->sys.k_center_c[0]=4;
in->sys.k_center_c[1]=4;
in->sys.desire_time=3.3;//0.76;
in->sys.leg_move_min_dt=0;
in->sys.down_t=in->sys.down_h=0;
#endif
}	


//-------------------------------------------------Fail Reset------------------------------------------------------
float reset_deng=45;
float reset_sita[3]={10,125,0};//15};
float reset_sita1[3]={45,85,-8};
float reset_sita2[3]={45,85,-8};
void fall_reset(float dt)
{ u8 i;
  static u8 state,no_move;
	static u16 cnt[3];
	int flag;
	switch(state)
	{
	  case 0:	 
			 if(brain.fall!=0)
			 {brain.sys.control_angle=1;
			  cnt[0]=cnt[1]=cnt[2]=0;			 
				leg[1].sita_force[0]=reset_sita[0];leg[1].sita_force[1]=reset_sita[1];leg[1].sita_force[2]=reset_sita[2];leg[1].sita_force[3]=0;//reset_sita[2];
				leg[2].sita_force[0]=reset_sita[0];leg[2].sita_force[1]=reset_sita[1];leg[2].sita_force[2]=-reset_sita[2];leg[2].sita_force[3]=0;//reset_sita[2];
				leg[3].sita_force[0]=reset_sita[0];leg[3].sita_force[1]=reset_sita[1];leg[3].sita_force[2]=-reset_sita[2];leg[3].sita_force[3]=0;//reset_sita[2];
				leg[4].sita_force[0]=reset_sita[0];leg[4].sita_force[1]=reset_sita[1];leg[4].sita_force[2]=reset_sita[2];leg[4].sita_force[3]=0;//reset_sita[2];
				state=1;	
			 }
		break;
	  case 1:
			 if(cnt[0]++>2/dt)
			 {state=2;cnt[0]=cnt[1]=0;no_move=0;}
		break;
	  case 2:
			 if((brain.att[1]>10)&&no_move)
				 state=3;
			 else if((brain.att[1]<-10)&&no_move)
		     state=13;
			 else if(ABS(brain.att[1])<10)
				 state=23;

				if(fabs(mpu6050_fc.Gyro_deg.x)<44&&fabs(mpu6050_fc.Gyro_deg.y)<44)
				 cnt[1]++;
				else
				 cnt[1]=0;	
				if(cnt[1]>3/dt)
					no_move=1;
		break;
		case 3://left1
		   leg[3].sita_force[3]-=(leg[3].sita_force[3]-reset_deng)*dt*1.618;
		   leg[4].sita_force[3]-=(leg[3].sita_force[3]-reset_deng)*dt*1.618;
		   if(fabs(fabs(leg[3].sita_force[3])-fabs(reset_deng))<1)
			 {state=33;cnt[0]=0;}	 			 
			 if(ABS(brain.att[1])<10)
				 state=33;
		break;
		case 4://left2
		   leg[3].pos_tar[2].z-=(leg[3].pos_tar[2].z-leg[1].sys.limit_min.z*1.618)*dt*1.618;
		   leg[4].pos_tar[2].z-=(leg[4].pos_tar[2].z-leg[1].sys.limit_min.z*1.618)*dt*1.618;
		   if(fabs(fabs(leg[3].pos_tar[2].z)-fabs(leg[1].sys.limit_min.z*1.618))<0.61)
			 {state=33;cnt[0]=0;}
			 
			 if(ABS(brain.att[1])<10)
				 state=33;
		break;	 		 
	  case 13://right
			 leg[1].sita_force[3]-=(leg[1].sita_force[3]-reset_deng)*dt*1.618;
		   leg[2].sita_force[3]-=(leg[2].sita_force[3]-reset_deng)*dt*1.618;
		   if(fabs(fabs(leg[1].sita_force[3])-fabs(reset_deng))<1)
			 {state=33;cnt[0]=0;}	 			 
			 if(ABS(brain.att[1])<10)
				 state=33;
		break;
			 
		case 33://回到蜷缩状态	 
				//leg[1].sita_force[0]=reset_sita[0];leg[1].sita_force[1]=reset_sita[1];
				leg[1].sita_force[2]=reset_sita[2];
				//leg[2].sita_force[0]=reset_sita[0];leg[2].sita_force[1]=reset_sita[1];
				leg[2].sita_force[2]=-reset_sita[2];
				//leg[3].sita_force[0]=reset_sita[0];leg[3].sita_force[1]=reset_sita[1];
				leg[3].sita_force[2]=-reset_sita[2];
				//leg[4].sita_force[0]=reset_sita[0];leg[4].sita_force[1]=reset_sita[1];
				leg[4].sita_force[2]=reset_sita[2];	
        if(cnt[0]++>0.2/dt)
			 {state=34;cnt[0]=0;}		
		break;
	  case 34:
			 if(ABS(brain.att[1])<10)
			 { state=23;
				 
			  }		
			 else 
				 state=0;
		break;
		case 23:
		   brain.sys.control_angle=0;
		    for(i=1;i<5;i++)
		   {
				if(i==1||i==2)
					flag=1;
				else 
					flag=1;
				leg[i].pos_tar[2].x=leg[i].sys.init_end_pos.x*flag+(float)RNG_Get_RandomRange(-1000,1000)/1000000.;//+off_x*cos(tar_yaw/ 57.3f);
				leg[i].pos_tar[2].y=leg[i].sys.init_end_pos.y+(float)RNG_Get_RandomRange(-1000,1000)/1000000.;//+off_y*sin(tar_yaw/ 57.3f);
				leg[i].pos_tar[2].z=leg[i].sys.limit_min.z*1.234;
			 }
		   
			 if(cnt[0]++>2/dt)
			 {state=0;
		    brain.tar_h=leg[1].sys.init_end_pos.z;
        brain.fall=0;				
			 }	
		break;
	}
}

u8 per_stop=10;
void leg_task1(float dt)//$%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
{
static u16 cnt_init;	
static u8 init;
static u8 cnt_stop;	
static u16 cnt[10];	
static u8 leg_trig_flag_reg;
u8 i;
if(!init&&cnt_init++>3/0.02){init=1;
barin_init(&brain);}

	if(init){
				if(cnt_stop++>per_stop&&brain.leg_move_state>0)
				cnt_stop=0;
			 
				for(i=1;i<5;i++) 	  
				conver_legpos_to_barin(&brain,&leg[i],i);      
		
				estimate_center(&brain,brain.att,brain.now_spd,brain.now_acc,brain.sys.tar_spd,brain.tar_w);//估计机体重心
    #if TEST_MODE1
    brain.fall=0;
    #endif				
		if(brain.fall!=0)
			  fall_reset(dt);
		else{
			  state_clear();	
			
			  cal_pos_global(dt);
			
			  cal_center_of_leg_ground(&brain);//估计着地多边形
			  //brain.trot_gait=1;
			if(brain.trot_gait){
				check_leg_need_move_global_tro(&brain,brain.now_spd,brain.sys.tar_spd,brain.tar_w,dt);//判断需要跨脚  规划跨腿		
        
			  center_control_global_tro(dt);//在着地区域内移动重心

        att_control(dt);					
			}else{
				check_leg_need_move_global(&brain,brain.now_spd,brain.sys.tar_spd,brain.tar_w,dt);//判断需要跨脚  规划跨腿		
        
			  center_control_global(dt);//在着地区域内移动重心
			
				att_control(dt);	
			}
		 }
	}//end_init
}
//------------------------------------------------START HERE-----------------------------------------------------------------

///////////////////////////////////////////////////OLD LOCAL///////////////////////////////////////////////


float k_acc=0;
//估计机器人中心
void estimate_center(BRAIN_STRUCT *in,float att[3],float spd_body[3],float acc_body[3],float spd_tar[3],float w_tar)//估计机体重心
{ u8 i,j;
	float high_robot,temp,k_off_z;
	for(i=1;i<5;i++)
		if(leg[i].leg_ground)
		{temp+=leg[i].pos_now[2].z;j++;}
		if(j>0){
		high_robot=temp/j;
		k_off_z=LIMIT(high_robot/15.5*2.68,0.2,2.2);
		}else
		k_off_z=1;

		in->center.x=0+brain.now_acc[0]*k_acc;
		in->center.y=0+brain.now_acc[1]*k_acc;
	  in->center.z=high_robot;
		
		
		brain.att[0]=Pitch;
		brain.att[1]=Roll;
		
		if(brain.att[1]>66)
			brain.fall=1;
		else if(brain.att[1]<-66)
      brain.fall=-1;			
}


//计算着地脚重心
void cal_center_of_leg_ground(BRAIN_STRUCT *in)
{
u8 i=0,j=0;	
double x[5]={-5,0,4,0},y[5]={0,5,0,0};
float temp[2][3];
in->ground_leg_num=0;
  for(i=1;i<5;i++){
		if(leg[i].leg_ground){
		x[j]=leg[i].pos_now_brain[2].x;
		y[j]=leg[i].pos_now_brain[2].y;
		j++;
		}
  }
in->ground_leg_num=j;
  
switch(in->ground_leg_num) 
 {	 
 case 3:
   in->leg_ground_center[Xr]=(float)(x[0]+x[1]+x[2])/3.;
   in->leg_ground_center[Yr]=(float)(y[0]+y[1]+y[2])/3.;
   in->area_of_leg[0]=cal_area_trig( x[0],y[0], x[1],y[1], x[2],y[2]);
   in->steady_value=cal_steady_s(in->leg_ground_center[Xr],in->leg_ground_center[Yr],x[0],y[0], x[1],y[1], x[2],y[2]);
 break;
 case 4:
    temp[0][Xr]=(float)(x[0]+x[1]+x[2])/3.;
    temp[0][Yr]=(float)(y[0]+y[1]+y[2])/3.;
    temp[1][Xr]=(float)(x[1]+x[2]+x[3])/3.;
    temp[1][Yr]=(float)(y[1]+y[2]+y[3])/3.;
    in->leg_ground_center[Xr]=(float)(temp[0][Xr]+temp[1][Xr])/2.;
    in->leg_ground_center[Yr]=(float)(temp[0][Yr]+temp[1][Yr])/2.;
    in->area_of_leg[0]=cal_area_trig( x[0],y[0], x[1],y[1], x[2],y[2])/2+cal_area_trig( x[3],y[3], x[1],y[1], x[2],y[2])/2;
 break;
 default:
	  in->area_of_leg[0]=0;
 break;
 }	 
}

float att_control_out[5];
#if TEST_MODE1
float kp_att=0.0;
float ki_att=0.0;
float kd_att=0;
float flt=0.223;
#else
float kp_att=0.0;
float ki_att=0.0;
float kd_att=0;
float flt=0.223;
#endif
void att_control(float dt)
{ static float int_ero[2];
  static float ero[2],ero_r[2];
  float control[2],ero_d[2];	
  ero[0]+=( 1 / ( 1 + 1 / ( flt *3.14f *dt ) ) ) *(my_deathzoom(LIMIT(brain.tar_att[0]-brain.att[0],-33,33),0.235)- ero[0]);
	ero[1]+=( 1 / ( 1 + 1 / ( flt *3.14f *dt ) ) ) *(my_deathzoom(LIMIT(brain.tar_att[1]-brain.att[1],-33,33),0.235)- ero[1]);
	
	if(ki_att==0)
	int_ero[1]=int_ero[0]=0;	
	else{
	int_ero[0]+=my_deathzoom(LIMIT(brain.tar_att[0]-brain.att[0],-33,33),0.235)*ki_att;
	int_ero[1]+=my_deathzoom(LIMIT(brain.tar_att[1]-brain.att[1],-33,33),0.235)*ki_att;
	}
	int_ero[0]=LIMIT(int_ero[0],-3,3);
	int_ero[1]=LIMIT(int_ero[1],-3,3);
	
	ero_d[0] = 0.02f/dt *kd_att * (ero[0]-ero_r[0]) *dt;
  ero_d[1] = 0.02f/dt *kd_att * (ero[1]-ero_r[1]) *dt;
	
	control[0]=LIMIT(ero[0]*kp_att+int_ero[0]+ero_d[0],-5,5)*brain.global.center_stable_weight;
	control[1]=LIMIT(ero[1]*kp_att+int_ero[1]+ero_d[1],-5,5)*brain.global.center_stable_weight;
  
	ero_r[0]=ero[0];
	ero_r[1]=ero[1];
	
	att_control_out[1] = LIMIT(-control[0]+control[1],-6,8) ;
	att_control_out[3] = LIMIT(-control[0]-control[1],-6,8) ;
	att_control_out[2] = LIMIT(control[0]+control[1] ,-6,8) ;
	att_control_out[4] = LIMIT(control[0]-control[1] ,-6,8) ;
	att_control_out[0] =  fabs(control[1])+fabs(control[0]);
}	

//--------------------------------------------Trot -----------------Gait 
float k_size_trot=0.0068;
float size_k_trot;
float limit_deng_tro=17.0;//12;
float k_acc_control_trot[2]={-0.036,-0.036};
float k_spd_control_trot[2]={0.25,0.15};
void center_control_global_tro(float dt)
{
  static u8 reg_flag;
  u8 i;
	u8 id_f[3]={0};
	float ero[2];
	float center_tar_x,center_tar_y;
	float spd_use;
	float k_c=0,b_c=0;
	float k_c1=0,b_c1=0;
	float gx=brain.global.leg_ground_center[Xr];
	float gy=brain.global.leg_ground_center[Yr];
	float k_spd=1;
	float tar_x,tar_y;
	u8 brain_ground_leg_num=brain.ground_leg_num;
	u8 leg_ground[5];
	float leg_end_use[5][2];
	
	for(i=0;i<5;i++)
	leg_ground[i]=leg[i].leg_ground;
	
	spd_use=brain.spd*k_spd*brain.global.out_value;

	//resize
	float gx_use,gy_use;
	float low_k=brain.sys.leg_local[1].x/brain.sys.leg_local[1].y*k_size_trot;
	float temp1=(1-low_k)/2;
	float temp2=cos(brain.spd_yaw*2*ANGLE_TO_RADIAN);
	size_k_trot= LIMIT(temp1*temp2+low_k+temp1,0,1);	
	size_k_trot=1;
	float re_x=0,re_y=0;
	for(i=1;i<5;i++)
	resize_point_with_arrow(brain.global.end_pos_global[i].x,brain.global.end_pos_global[i].y,re_x,re_y,brain.spd_yaw,size_k_trot,&leg_end_use[i][Xr],&leg_end_use[i][Yr]);
	
	resize_point_with_arrow(gx,gy,re_x,re_y,brain.spd_yaw,size_k_trot,&gx_use,&gy_use);
	
	//normal //计算重心方向
	line_function_from_arrow(gx_use,gy_use,brain.spd_yaw,&k_c,&b_c);
	
	//4leg in trot
	check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
	brain.global.end_pos_global[0].y+cos(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
	brain.spd_yaw,k_c,b_c,&tar_x,&tar_y);
	
	center_tar_x=tar_x;
	center_tar_y=tar_y;

	brain.global.tar_center[Xr]=brain.tar_center[Xr]=center_tar_x;
	brain.global.tar_center[Yr]=brain.tar_center[Yr]=center_tar_y;

	ero[Xr]=my_deathzoom((brain.tar_center[Xr]-brain.global.end_pos_global[0].x
	+k_spd_control_trot[Xr]*my_deathzoom(LIMIT(brain.now_spd[Xr],-0.66,0.66),0.01)
	+k_acc_control_trot[Xr]*brain.global.end_pos_global[0].z/9.87*my_deathzoom(LIMIT(brain.now_acc[Xr],-2,2),0.3)),0.001);
  ero[Yr]=my_deathzoom((brain.tar_center[Yr]-brain.global.end_pos_global[0].y
	+k_spd_control_trot[Yr]*my_deathzoom(LIMIT(brain.now_spd[Yr],-0.66,0.66),0.01)
	+k_acc_control_trot[Yr]*brain.global.end_pos_global[0].z/9.87*my_deathzoom(LIMIT(brain.now_acc[Yr],-2,2),0.3)),0.001);
		
	float ero1[2];
	ero1[Xr]=my_deathzoom((brain.tar_center[Xr]-(brain.global.end_pos_global[0].x+sin(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt)),0.001);
  ero1[Yr]=my_deathzoom((brain.tar_center[Yr]-(brain.global.end_pos_global[0].y+cos(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt)),0.001);
	
//	if((brain.leg_move_state==S_BODY_MOVE||brain.leg_move_state==S_IDLE)&&(brain_ground_leg_num==2||brain_ground_leg_num==4))
//	{
	center_control_out[Xr]=ero[Xr]*brain.sys.k_center_c[Xr];
	center_control_out[Yr]=ero[Yr]*brain.sys.k_center_c[Yr];
//  }
	float dis_ero=sqrt(pow(ero1[Xr],2)+pow(ero1[Yr],2));
	brain.global.center_stable_weight=LIMIT((brain.sys.leg_local[1].x/2-LIMIT(dis_ero,0,brain.sys.leg_local[1].x/2))/(brain.sys.leg_local[1].x/2),0,1);
	
	if(dis_ero<0.256)
	brain.center_stable=1; 
	else
	brain.center_stable=0;
	
	float fall_con[4];
	fall_treat_tro(dt,&fall_con[0]);
  //output
	for(i=1;i<5;i++){
	if((leg[i].control_mode||brain.control_mode)&&leg[i].leg_ground&&!leg[i].sys.leg_ground_force){	
	leg[i].deng[Xr]=LIMIT(center_control_out[Xr],-limit_deng_tro*brain.global.area_value,limit_deng_tro*brain.global.area_value)+fall_con[i-1];
	leg[i].deng[Yr]=LIMIT(center_control_out[Yr],-limit_deng_tro*brain.global.area_value,limit_deng_tro*brain.global.area_value);	}
	}	
}



u8 last_move_id_tro,last_last_move_id_tro;
u16 out_range_move_tro[5];
float Ts[3];
float set_max_tro=2.618;//2
float min_spd_tro=0.321;
static u8 stop_leg_tro;
float set_trig_value_tro=0.9;
float trot_delay=0.1;//步态着地时间
//判断机器人需要跨腿的ID // GLOBAL
void check_leg_need_move_global_tro(BRAIN_STRUCT *in,float spd_body[3],float spd_tar[3],float w_tar,float dt)
{ u8 i,j;
	u8 need_move_leg=0;

	float spd=brain.spd;	
	static u8 lose_center_flag;
 
	if(brain.ground_leg_num<4)//may cause hard error when use if else
	brain.sys.dt_leg_min_trig_cnt=0;
	if(brain.sys.dt_leg_min_trig_cnt++>brain.sys.leg_move_min_dt*brain.sys.desire_time/dt)
	brain.can_move_leg=1;
		
 //超出移动范围需要跨腿
	float range_in[5]={1},range_out[5]={1};
	float jiaodiao[2][2]={0};	
 for(i=1;i<5;i++){
	//check out range
	 if(leg[i].leg_ground){
	 float length,tar_x,tar_y;
   tar_x=brain.global.end_pos_global[i].x;
	 tar_y=brain.global.end_pos_global[i].y;
	 
	 float circle_center[2]={0};
	 conver_body_to_global(
	 brain.sys.leg_local[i].x+leg[i].sys.init_end_pos.x,
	 brain.sys.leg_local[i].y+leg[i].sys.init_end_pos.y,
	 &circle_center[0],&circle_center[1]);//转换初始点到全局坐标系
	 if(i==stop_leg)//debug
		 j=0;
	 cal_jiao_of_range_and_line_tangle(i,circle_center[0],circle_center[1],in->sys.min_range,in->sys.max_range,in->spd_yaw,&jiaodiao[0][Xr],&jiaodiao[0][Yr],&jiaodiao[1][Xr],&jiaodiao[1][Yr]);
	 //计算当前点与交点距离  0->速度指向交点 
	 float dis[2];
	 dis[0]=cal_dis_of_points(tar_x,tar_y,jiaodiao[0][Xr],jiaodiao[0][Yr]);
	 dis[1]=cal_dis_of_points(tar_x,tar_y,jiaodiao[1][Xr],jiaodiao[1][Yr]);

	 range_out[i]=check_in_move_range_tangle(i,tar_x,tar_y,circle_center[0],circle_center[1],in->sys.min_range,in->sys.max_range);
	 if(range_out[i]==0)
	 brain.global.dis_leg_out[i]=dis[1];
	 else
	 brain.global.dis_leg_out[i]=0;
	 
	 if(brain.rst_all_soft)
	  brain.tabu=need_move_leg=1;  	 
 
	 if(dis[1]<dis[0]&&range_out[i]==0)//&&fabs(spd)>0)
	 { 
	 leg[i].need_move=3;
	 out_range_move_tro[i]++; 
	 }
	 
	 brain.leg_move[i]=leg[i].need_move;
  }
 } 
//计算超限衰减值
 float max_out_range=brain.global.dis_leg_out[1];

 for(i=1;i<5;i++)
 if(brain.global.dis_leg_out[i]>max_out_range) 
 {max_out_range=brain.global.dis_leg_out[i];}
  brain.global.out_value=LIMIT(1-LIMIT(max_out_range,0,set_max_tro)/set_max_tro,min_spd_tro,1);
 
//............................................................................................................
 //----------.......................跨腿触发条件:重心越过 两交点  .......................................
 if(brain.ground_leg_num>3&&brain.center_stable&&brain.leg_move_state==S_IDLE){
	 float k1,b1,k2,b2;
	 float cro_x,cro_y;
	 cro_x=(brain.global.end_pos_global[1].x+brain.global.end_pos_global[2].x+brain.global.end_pos_global[3].x+brain.global.end_pos_global[4].x)/4;
   cro_y=(brain.global.end_pos_global[1].y+brain.global.end_pos_global[2].y+brain.global.end_pos_global[3].y+brain.global.end_pos_global[4].y)/4;
   if(check_leg_near_init(brain.sys.in_rst_check)==0||brain.spd>0)	
	 {
		   need_move_leg=1;
	 }
 }
//---------------------------------------规划腿---状态机------------------------------------	
 	
  static float center_now[2];
  static u8 leg_flag=1;
  static u16 cnt;
  u8 temp; 

	switch(brain.leg_move_state)
	{
	 case S_IDLE:			
		 if(need_move_leg&&brain.can_move_leg&&brain.center_stable&&brain.ground_leg_num>3)
		 {
		 u8 id_need_to_move=0;
		 float yaw_in=To_180_degrees(in->spd_yaw);
		 float yaw_temp=in->sys.yaw_trig;
 
				 id_need_to_move=trig_list_trot[leg_flag];
				 temp=id_need_to_move;
	       leg_flag++;
				 if(leg_flag>4)
         leg_flag=1;
				 
				 in->move_id=temp;
				 last_last_move_id_tro=last_move_id_tro;
				 last_move_id_tro=in->move_id;
							
				 if(in->move_id>0){
				 if(brain.leg_move_state==S_IDLE)	
				 brain.leg_move_state=S_BODY_MOVE;					 
				 brain.can_move_leg=0;
				 cnt=0;			 
				 }
				 
			 }// end plan 
		break;
	  case S_BODY_MOVE:
		 if(cnt++>trot_delay/dt){cnt=0;
		 brain.leg_move_state=S_LEG_TRIG;
		 }
		break;
    case S_LEG_TRIG:
			if(brain.global.center_stable_weight>0.9&&brain.ground_leg_num>3)
			{ Ts[0]=(float)Get_Cycle_T(GET_T_TRIG);
			brain.leg_move_state=S_LEG_TRIG_LEAVE_GROUND_CHECK;		 
				if(brain.move_id==14){
				leg_tar_est_global_tro(&brain,&leg[1],0,0,0,1,dt);
				out_range_move_tro[1]=0; leg[1].sys.leg_move_pass_cnt=0;	
				leg_tar_est_global_tro(&brain,&leg[4],0,0,0,1,dt);
				out_range_move_tro[4]=0; leg[4].sys.leg_move_pass_cnt=0;	
				}
				else{
				leg_tar_est_global_tro(&brain,&leg[2],0,0,0,1,dt);
				out_range_move_tro[2]=0; leg[1].sys.leg_move_pass_cnt=0;	
				leg_tar_est_global_tro(&brain,&leg[3],0,0,0,1,dt);
				out_range_move_tro[3]=0; leg[4].sys.leg_move_pass_cnt=0;	
				}
			}			
		break;
	  case S_LEG_TRIG_LEAVE_GROUND_CHECK:	
		if(cnt++>trot_delay/dt){cnt=0;	
		brain.leg_move_state=S_LEG_TRIG_ING;}
		if(brain.ground_leg_num<3)
		{brain.leg_move_state=S_LEG_TRIG_ING;}
		break;
		case S_LEG_TRIG_ING:
		if(brain.ground_leg_num>3&&brain.center_stable)
	  {brain.leg_move_state=S_IDLE;}
		break;
	}

}
 

u8 en_off_trig_tro=0;
float h_k2_tro=1.0;
float k_off_tro=0.2;
float k_trig1_tro=1;//2;
float k_rad_tro=1.6;
float k_trig_tro=1;
u8 en_att_trig=0;
void leg_tar_est_global_tro(BRAIN_STRUCT *in,LEG_STRUCT *leg,float spd_body[3],float spd_tar[3],float w_tar,u8 need_move,float dt)
{
u8 id=leg->sys.id;
float band_x,band_y,range_limit;
float tar_yaw=in->spd_yaw;
float spd=in->spd;
float tar_x,tar_y,tar_z;	
float off_x,off_y;
float yaw_in=To_180_degrees(brain.spd_yaw);
float yaw_temp=brain.sys.yaw_trig;
u8 way_sel;
u8 fp_point_id[4];//L  R
 if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)//r
 {fp_point_id[0]=1;fp_point_id[1]=2;   fp_point_id[2]=3;fp_point_id[3]=4;way_sel=2;}
 else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)//l
 {fp_point_id[0]=4;fp_point_id[1]=3;   fp_point_id[2]=2;fp_point_id[3]=1;way_sel=4;}
 else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))//f
 {fp_point_id[0]=3;fp_point_id[1]=1;   fp_point_id[2]=4;fp_point_id[3]=2;way_sel=1;}
 else//b
 {fp_point_id[0]=2;fp_point_id[1]=4;   fp_point_id[2]=1;fp_point_id[3]=3;way_sel=3;}
 float off_k=1;
 if(id==fp_point_id[2]||id==fp_point_id[3])
	 off_k=1-k_off_tro;
 else
   off_k=1+k_off_tro;
 
if(leg->sys.id==1||leg->sys.id==2)
off_x=LIMIT(leg->pos_now[2].x-leg->sys.init_end_pos.x,0,leg->sys.off_local[0])*fabs(cos(tar_yaw*ANGLE_TO_RADIAN));
else
off_x=LIMIT(leg->pos_now[2].x-leg->sys.init_end_pos.x,-leg->sys.off_local[0],0)*fabs(cos(tar_yaw*ANGLE_TO_RADIAN));	
if(leg->sys.id==1||leg->sys.id==3)
off_y=LIMIT(leg->pos_now[2].y-leg->sys.init_end_pos.y,0,leg->sys.off_local[1])*fabs(sin(tar_yaw*ANGLE_TO_RADIAN));
else
off_y=LIMIT(leg->pos_now[2].y-leg->sys.init_end_pos.y,-leg->sys.off_local[1],0)*fabs(sin(tar_yaw*ANGLE_TO_RADIAN));	

float spd_wx,spd_wy;
float tar_w;
float min=1.45;
if(brain.tar_w<min&&brain.tar_w>0)
	tar_w=min;
else if(brain.tar_w>-min&&brain.tar_w<0)
	tar_w=-min;
else
	tar_w=brain.tar_w;
float x_temp=fabs(sin((90-brain.sys.yaw_trig)*ANGLE_TO_RADIAN))*tar_w*k_rad_tro;
float y_temp=fabs(cos((90-brain.sys.yaw_trig)*ANGLE_TO_RADIAN))*tar_w*k_rad_tro;
	switch(id)
	{
	case 1:
	spd_wx=x_temp;
	spd_wy=-y_temp;
	break;
	case 2:
	spd_wx=-x_temp;
	spd_wy=-y_temp;
	break;
	case 3:
	spd_wx=x_temp;
	spd_wy=y_temp;
	break;
	case 4:
	spd_wx=-x_temp;
	spd_wy=y_temp;
	break;
	}
	
	
float tar_x1,tar_y1;
float T=LIMIT(Ts[0],brain.sys.desire_time,1.618*(brain.sys.desire_time+trot_delay));
float spdx;
float spdy;
spdx=sin(tar_yaw*ANGLE_TO_RADIAN)*(	in->spd)+spd_wx;
spdy=cos(tar_yaw*ANGLE_TO_RADIAN)*(	in->spd)+spd_wy;	
	
tar_x1=LIMIT(spdx*T/2*k_trig1_tro,-2*brain.sys.leg_move_range1[1],2*brain.sys.leg_move_range1[1]);
tar_y1=LIMIT(spdy*T/2*k_trig1_tro,-2*brain.sys.leg_move_range1[1],2*brain.sys.leg_move_range1[1]);

float dx,dy;
dx=tan(brain.att[1]*ANGLE_TO_RADIAN)*brain.global.end_pos_global[0].z;	
dy=tan(brain.att[0]*ANGLE_TO_RADIAN)*brain.global.end_pos_global[0].z;	
tar_x=spd_wx+sin(tar_yaw*ANGLE_TO_RADIAN)*LIMIT((spd)*off_k*in->sys.k_spd_to_range,-2*brain.sys.leg_move_range1[1],2*brain.sys.leg_move_range1[1]);//,-in->sys.leg_move_range[Xr],in->sys.leg_move_range[Xr]);
tar_y=spd_wy+cos(tar_yaw*ANGLE_TO_RADIAN)*LIMIT((spd)*off_k*in->sys.k_spd_to_range,-2*brain.sys.leg_move_range1[1],2*brain.sys.leg_move_range1[1]);//,-in->sys.leg_move_range[Yr],in->sys.leg_move_range[Yr]);

leg->pos_tar_trig[2].x=dx*en_att_trig+leg->sys.init_end_pos.x*k_trig1_tro+tar_x1+RANDOM;
leg->pos_tar_trig[2].y=dy*en_att_trig+leg->sys.init_end_pos.y*k_trig1_tro+tar_y1+RANDOM;
leg->pos_tar_trig[2].z=brain.tar_h*h_k2_tro;

//LIMIT
 limit_move_range_tangle( id
,leg->sys.init_end_pos.x,leg->sys.init_end_pos.y,
 leg->pos_tar_trig[2].x,leg->pos_tar_trig[2].y
 ,in->sys.min_range,in->sys.max_range
 ,&leg->pos_tar_trig[2].x,&leg->pos_tar_trig[2].y);

leg->pos_tar_trig[2].z+=RANDOM;

if(brain.rst_all_soft>0)
{
brain.rst_all_soft++;
leg->pos_tar_trig[2].x=leg->sys.init_end_pos.x*k_trig1_tro+RANDOM;
leg->pos_tar_trig[2].y=leg->sys.init_end_pos.y*k_trig1_tro+RANDOM;
leg->pos_tar_trig[2].z=brain.tar_h;
}
if(brain.rst_all_soft>4)
brain.rst_all_soft=0;
}



//侧翻简单控制
float dead_1_tro[2]={0.1,1};
float k_fall_tro[3]={0.6,0.1};
float flt_fall_tro;
void fall_treat_tro(float dt,float *out)
{ static u8 state;
	float fall_control[4];
	if(fabs(brain.att[1])>dead_1_tro[0]&&fabs(mpu6050_fc.Gyro_deg.x)>dead_1_tro[1]&&brain.fall==0){
	
	  out[0]=my_deathzoom(LIMIT(brain.att[1],-20,20),1)*k_fall_tro[0]+(LIMIT(mpu6050_fc.Gyro_deg.x,-33,33)*k_fall_tro[1]);	
		out[1]=my_deathzoom(LIMIT(brain.att[1],-20,20),1)*k_fall_tro[0]+(LIMIT(mpu6050_fc.Gyro_deg.x,-33,33)*k_fall_tro[1]);	
		out[2]=(my_deathzoom(LIMIT(brain.att[1],-20,20),1)*k_fall_tro[0]+(LIMIT(mpu6050_fc.Gyro_deg.x,-33,33)*k_fall_tro[1]));		
		out[3]=(my_deathzoom(LIMIT(brain.att[1],-20,20),1)*k_fall_tro[0]+(LIMIT(mpu6050_fc.Gyro_deg.x,-33,33)*k_fall_tro[1]));	
//   if(brain.att[1]<0)//r
//		{
//		out[0]=-fabs(out[0]);	
//		out[1]=-fabs(out[1]);		
//		out[2]=0;	
//		out[3]=0;	
//		}	
//		else//l
//		{
//		out[2]=fabs(out[2]);	
//		out[3]=fabs(out[3]);		
//		out[0]=0;	
//		out[1]=0;		
//		}		
	}
	else 
	{
		out[0]=out[1]=out[2]=out[3]=0;
	}
	
}