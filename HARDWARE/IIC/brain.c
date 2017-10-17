#include "include.h" 

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
in->sys.leg_move_range1[1]=0.707*leg[1].sys.l2*0.9;
in->sys.leg_move_range1[2]=sin(22/57.3)*(leg[1].sys.l2+leg[1].sys.l3);
in->sys.leg_move_range1[3]=0.707*leg[1].sys.l3*0.3/1.618;
in->sys.leg_move_range1[4]=MIN(0.707*(leg[1].sys.l2+leg[1].sys.l3)*0.618,W/2*0.268)/1.618;

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
in->sys.leg_h[2]=3.68;
in->sys.leg_h[3]=3.68;
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

				check_leg_need_move_global(&brain,brain.now_spd,brain.sys.tar_spd,brain.tar_w,dt);//判断需要跨脚  规划跨腿		
        
			  center_control_global(dt);//在着地区域内移动重心
			
				att_control(dt);	
       
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
//--------------------------------------------Gait 

void trot_need_move(float dt)
{
   static u8 flag;


   if(flag)
	 {
	  brain.move_id=14;
	  flag=0;
	 }else{
		brain.move_id=23;
	  flag=0;
	 }

}	

float k_size_trot=0.0068;
float size_k_trot;
float limit_deng_trot=7.0;//12;
float k_acc_control_trot[2]={-0.036,-0.036};
float k_spd_control_trot[2]={0.25,0.15};
void trot_center_control(float dt)
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
	
	spd_use=brain.spd*k_spd*brain.global.center_stable_weight*brain.global.out_value;

	//resize
	float gx_use,gy_use;
	float low_k=brain.sys.leg_local[1].x/brain.sys.leg_local[1].y*k_size_trot;
	float temp1=(1-low_k)/2;
	float temp2=cos(brain.spd_yaw*2*ANGLE_TO_RADIAN);
	size_k_trot= LIMIT(temp1*temp2+low_k+temp1,0,1);	
	//size_k=1;
	float re_x=0,re_y=0;
	for(i=1;i<5;i++)
	resize_point_with_arrow(brain.global.end_pos_global[i].x,brain.global.end_pos_global[i].y,re_x,re_y,brain.spd_yaw,size_k_trot,&leg_end_use[i][Xr],&leg_end_use[i][Yr]);
	
	resize_point_with_arrow(gx,gy,re_x,re_y,brain.spd_yaw,size_k_trot,&gx_use,&gy_use);
	
	//normal
	line_function_from_arrow(gx_use,gy_use,brain.spd_yaw,&k_c,&b_c);
	
	
	switch(brain_ground_leg_num)
	{
		case 3://三条腿着地
			//Debug
		  if(leg[3].leg_ground==0) 
		   i=0;
			//	
		  u8 front_leg_num=0;
		  float f_leg_x,f_leg_y;
		  for(i=1;i<5;i++)
		  if(leg_ground[i])
		      if(check_point_front_arrow(leg_end_use[i][Xr],leg_end_use[i][Yr],gx_use,gy_use,brain.spd_yaw)){
						 id_f[front_leg_num++]=i;
					}		
      brain.sys.front_leg_num=front_leg_num;					
		  f_leg_x=leg_end_use[id_f[0]][Xr];
			f_leg_y=leg_end_use[id_f[0]][Yr];
	 if(front_leg_num==1){	//前方一条腿
			float cross_point_x,cross_point_y;						
			float k_f,b_f;
			for(i=1;i<5;i++)
		  if(leg_ground[i]&&i!=id_f[0])		 
      {
       float k_temp,b_temp;
			 line_function_from_two_point(f_leg_x,f_leg_y,leg_end_use[i][Xr],leg_end_use[i][Yr],&k_temp,&b_temp);					
				if(check_cross_arrow_line(gx_use,gy_use,brain.spd_yaw,k_temp,b_temp,&cross_point_x,&cross_point_y)){	
         id_f[1]=i; 
				 k_f=k_temp;
				 b_f=b_temp;
				 break;}   					
			}
		  //添加余量
			cross_point_x-=brain.min_st[0]*sin((brain.spd_yaw)*ANGLE_TO_RADIAN)*size_k_trot;
			cross_point_y-=brain.min_st[0]*cos((brain.spd_yaw)*ANGLE_TO_RADIAN)*size_k_trot;
			
			int traj_flag;
			//分段轨迹
			traj_flag=check_point_front_arrow(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,cross_point_x,cross_point_y,brain.spd_yaw);
			if(traj_flag){//斜线
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.spd_yaw,k_f,b_f,&tar_x,&tar_y);
			}	
			else//直线
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.spd_yaw,k_c,b_c,&tar_x,&tar_y);
			
		
			#if USE_SIMPLE_CENTER
			 //			//new
      float k_cc,b_cc;//中心和顶点直线		
			line_function_from_two_point(gx,gy,f_leg_x,f_leg_y,&k_cc,&b_cc);
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.spd_yaw,k_cc,b_cc,&tar_x,&tar_y);
			#endif
  		if(cal_dis_of_points(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
		  f_leg_x,f_leg_y)<brain.min_st[0]*size_k_trot)
	    {tar_x=f_leg_x-brain.min_st[0]*size_k_trot*sin((brain.spd_yaw)*ANGLE_TO_RADIAN);tar_y=f_leg_y-brain.min_st[0]*size_k_trot*cos((brain.spd_yaw)*ANGLE_TO_RADIAN);brain.force_stop=1;}
		}
	  else//前方两个腿
		{
		  float f_leg_x1,f_leg_y1;
		  f_leg_x1=leg_end_use[id_f[1]][Xr];
			f_leg_y1=leg_end_use[id_f[1]][Yr];
		
			u8 id3;				 
			for(i=1;i<5;i++)
		  if(leg_ground[i]&&i!=id_f[0]&&i!=id_f[1])		 
      {
			id3=i;
      break;				
			}
			float jiao[2][2];
			float coner[3][2];
			if(leg[4].leg_ground==0)
				i=0;
			coner[0][Xr]=f_leg_x;coner[0][Yr]=f_leg_y;
			coner[1][Xr]=f_leg_x1;coner[1][Yr]=f_leg_y1;
			coner[2][Xr]=leg_end_use[id3][Xr];coner[2][Yr]=leg_end_use[id3][Yr];
			check_point_to_trig(gx_use,gy_use,brain.spd_yaw,f_leg_x,f_leg_y,f_leg_x1,f_leg_y1,leg_end_use[id3][Xr],leg_end_use[id3][Yr],
	    &jiao[0][Xr],&jiao[0][Yr],&jiao[1][Xr],&jiao[1][Yr]);
			float dis[2][3];
			dis[0][0]=cal_dis_of_points(jiao[0][Xr],jiao[0][Yr],f_leg_x,f_leg_y);
	    dis[0][1]=cal_dis_of_points(jiao[0][Xr],jiao[0][Yr],f_leg_x1,f_leg_y1);
	    dis[0][2]=cal_dis_of_points(jiao[0][Xr],jiao[0][Yr],leg_end_use[id3][Xr],leg_end_use[id3][Yr]);
			dis[1][0]=cal_dis_of_points(jiao[1][Xr],jiao[1][Yr],f_leg_x,f_leg_y);
	    dis[1][1]=cal_dis_of_points(jiao[1][Xr],jiao[1][Yr],f_leg_x1,f_leg_y1);
	    dis[1][2]=cal_dis_of_points(jiao[1][Xr],jiao[1][Yr],leg_end_use[id3][Xr],leg_end_use[id3][Yr]);
			float dis_big[2]={dis[0][0],dis[1][0]};
			u8 id_closet[2]={1,1};
			for(i=1;i<3;i++)
			  if(dis[0][i]<dis_big[0])
					 id_closet[0]=i;
			for(i=1;i<3;i++)
			  if(dis[1][i]<dis_big[1])
					 id_closet[1]=i;	
		
			float cross_point[2][2];
		  float coner_point[2][2];		
			coner_point[0][Xr]=coner[id_closet[0]][Xr];	
			coner_point[0][Yr]=coner[id_closet[0]][Yr];	
			coner_point[1][Xr]=coner[id_closet[1]][Xr]+brain.min_st[1]*size_k_trot*sin((brain.spd_yaw)*ANGLE_TO_RADIAN);	
			coner_point[1][Yr]=coner[id_closet[1]][Yr]+brain.min_st[1]*size_k_trot*cos((brain.spd_yaw)*ANGLE_TO_RADIAN);	

			cross_point[0][Xr]=jiao[0][Xr]-brain.min_st[0]*size_k_trot*sin((brain.spd_yaw)*ANGLE_TO_RADIAN);
			cross_point[0][Yr]=jiao[0][Yr]-brain.min_st[0]*size_k_trot*cos((brain.spd_yaw)*ANGLE_TO_RADIAN);
			cross_point[1][Xr]=jiao[1][Xr]+brain.min_st[1]*size_k_trot*sin((brain.spd_yaw)*ANGLE_TO_RADIAN);
			cross_point[1][Yr]=jiao[1][Yr]+brain.min_st[1]*size_k_trot*cos((brain.spd_yaw)*ANGLE_TO_RADIAN);
			//
			float k_tr[3],b_tr[3];
			line_function_from_two_point(coner_point[0][Xr],coner_point[0][Yr],cross_point[0][Xr],cross_point[0][Yr],&k_tr[0],&b_tr[0]);				
 	    line_function_from_two_point(cross_point[0][Xr],cross_point[0][Yr],cross_point[1][Xr],cross_point[1][Yr],&k_tr[1],&b_tr[1]);
		  line_function_from_two_point(cross_point[1][Xr],cross_point[1][Yr],coner_point[1][Xr],coner_point[1][Yr],&k_tr[2],&b_tr[2]);
			
			int traj_flag[3];
			//
			traj_flag[0]=check_point_front_arrow(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,cross_point[0][Xr],cross_point[0][Yr],brain.spd_yaw);
			traj_flag[1]=check_point_front_arrow(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,cross_point[1][Xr],cross_point[1][Yr],brain.spd_yaw);
			
			if(traj_flag[0]==1&&traj_flag[1]==1)
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.spd_yaw,k_tr[0],b_tr[0],&tar_x,&tar_y);
			else if(traj_flag[0]==0&&traj_flag[1]==0)
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.spd_yaw,k_tr[2],b_tr[2],&tar_x,&tar_y);	
			//{tar_x=cross_point[1][Xr];tar_y=cross_point[1][Yr];}
			else
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.spd_yaw,k_tr[1],b_tr[1],&tar_x,&tar_y);	
			#if USE_SIMPLE_CENTER
						//----new
			float k_cc1,b_cc1;
			line_function_from_two_point(gx,gy,coner_point[1][Xr],coner_point[1][Yr],&k_cc1,&b_cc1);
			traj_flag[2]=check_point_front_arrow(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,gx,gy,brain.spd_yaw);
			if(traj_flag[2])//line
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.spd_yaw,k_tr[1],b_tr[1],&tar_x,&tar_y);	
			else //coner
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.spd_yaw,k_cc1,b_cc1,&tar_x,&tar_y);	
			#endif
			
			if(cal_dis_of_points(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
		  coner_point[0][Xr],coner_point[0][Yr])<brain.min_st[0]*size_k_trot||
			cal_dis_of_points(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
		  coner_point[1][Xr],coner_point[1][Yr])<brain.min_st[1]*size_k_trot)
	    brain.force_stop=1;
		}//end----
					
		  center_tar_x=tar_x;
			center_tar_y=tar_y;
		break;
		case 4:	
		  check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw*ANGLE_TO_RADIAN)*spd_use*dt,
			brain.spd_yaw,k_c,b_c,&tar_x,&tar_y);
			center_tar_x=tar_x;
			center_tar_y=tar_y;
		break;	
	}

	
	
	//resize_point_with_arrow(tar_x,tar_y,brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,brain.spd_yaw,size_k,&center_tar_x,&center_tar_y);
	
			
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
	
	
	center_control_out[Xr]=ero[Xr]*brain.sys.k_center_c[Xr];
	center_control_out[Yr]=ero[Yr]*brain.sys.k_center_c[Yr];
  
	float dis_ero=sqrt(pow(ero1[Xr],2)+pow(ero1[Yr],2));
	brain.global.center_stable_weight=LIMIT((brain.sys.leg_local[1].x/2-LIMIT(dis_ero,0,brain.sys.leg_local[1].x/2))/(brain.sys.leg_local[1].x/2),0,1);
	
	if(dis_ero<0.256)
	brain.center_stable=1; 
	else
	brain.center_stable=0;
  //output
	for(i=1;i<5;i++){
	if((leg[i].control_mode||brain.control_mode)&&leg[i].leg_ground&&!leg[i].sys.leg_ground_force){	
	leg[i].deng[Xr]=LIMIT(center_control_out[Xr],-limit_deng_trot,limit_deng_trot);
	leg[i].deng[Yr]=LIMIT(center_control_out[Yr],-limit_deng_trot,limit_deng_trot);	}
	}	
	
  //att set ?
	 reg_flag=brain.leg_move_state;
}