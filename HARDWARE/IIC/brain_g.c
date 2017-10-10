#include "include.h"
u8 trig_list_f[5]=		 {0,1,4,3,2};
u8 trig_list_r[5]=		 {0,2,3,1,4};
u8 trig_list_b[5]=		 {0,4,1,2,3};
u8 trig_list_l[5]=		 {0,3,2,4,1};
	
void state_clear(void)
{
 u8 i;
   brain.force_stop=0;
	
		for(i=1;i<5;i++)
	{
		if(leg[i].leg_ground){
      leg[i].need_move=0;	
      brain.leg_out_range[i]=0;			
		}
	}
	
	 if(brain.global.area_of_leg[0]<brain.global.area_of_leg[1]*0.45)
		 brain.rst_all_soft=1;
	 if(check_leg_near_init(brain.sys.in_rst_check))
		 brain.rst_all_soft=0;
}	
int flag_acc[2]={-1,-1};
float k_acc1=1;
void cal_pos_global(float dt)
{ u8 i=0,j=0;	
	float z_zero=leg[1].sys.init_end_pos.z;
	double x[5]={-5,0,4,0},y[5]={0,5,0,0};
	float temp[2][3];
	float off_cor[2];
	u8 ground_leg_num=0;
	brain.sys.off_cor[Xr]=-(brain.sys.leg_local[1].x-leg[4].pos_now[2].x);
	brain.sys.off_cor[Yr]=-(brain.sys.leg_local[1].y-leg[4].pos_now[2].y);
	//brain.sys.off_cor[Xr]=brain.sys.off_cor[Yr]=0;
	off_cor[Xr]=brain.sys.off_cor[Xr];
	off_cor[Yr]=brain.sys.off_cor[Yr];
	for(i=1;i<4;i++){
	brain.global.end_pos_global[i].x=leg[i].pos_now_brain[2].x+brain.sys.leg_local[1].x-leg[4].pos_now[2].x+off_cor[Xr];
	brain.global.end_pos_global[i].y=leg[i].pos_now_brain[2].y+brain.sys.leg_local[1].y-leg[4].pos_now[2].y+off_cor[Yr];
	brain.global.end_pos_global[i].z=leg[i].pos_now_brain[2].z-z_zero;
	}
	
  brain.global.end_pos_global[4].x=off_cor[Xr];
  brain.global.end_pos_global[4].y=off_cor[Yr];
	brain.global.end_pos_global[4].z=leg[4].pos_now_brain[2].z-z_zero;
  //机体中心
	float k_acc1=10.0/brain.sys.k_center_c[0];
	float temp1=leg[1].pos_now_brain[2].z+leg[2].pos_now_brain[2].z+leg[3].pos_now_brain[2].z+leg[4].pos_now_brain[2].z;
	brain.global.end_pos_global[0].z=temp1/4+0.5*brain.now_acc[2]*dt*dt;
  brain.global.end_pos_global[0].x=brain.sys.leg_local[1].x-leg[4].pos_now[2].x
	+flag_acc[0]*my_deathzoom(brain.global.end_pos_global[0].z/9.87*brain.now_acc[Xr],0.06)*k_acc1+off_cor[Xr];
	brain.global.end_pos_global[0].y=brain.sys.leg_local[1].y-leg[4].pos_now[2].y
	+flag_acc[1]*my_deathzoom(brain.global.end_pos_global[0].z/9.87*brain.now_acc[Yr],0.06)*k_acc1+off_cor[Yr];

	
	for(i=1;i<5;i++){
	brain.global.body_coner[i].x=brain.global.end_pos_global[0].x+brain.sys.leg_local[i].x;
	brain.global.body_coner[i].y=brain.global.end_pos_global[0].y+brain.sys.leg_local[i].y;
	brain.global.body_coner[i].z=brain.global.end_pos_global[0].z;	
	}

//--------------------------计算中心和静态稳定余量 
for(i=1;i<5;i++){
		if(leg[i].leg_ground){
		x[j]=brain.global.end_pos_global[i].x;
		y[j]=brain.global.end_pos_global[i].y;
		j++;
		}
  }
ground_leg_num=j;
switch(ground_leg_num) 
 {	 
 case 3:
   brain.global.leg_ground_center[Xr]=(float)(x[0]+x[1]+x[2])/3.;
   brain.global.leg_ground_center[Yr]=(float)(y[0]+y[1]+y[2])/3.;
   brain.global.area_of_leg[0]=cal_area_trig( x[0],y[0], x[1],y[1], x[2],y[2]);
   brain.global.steady_value=cal_steady_s( brain.global.end_pos_global[0].x, brain.global.end_pos_global[0].y,x[0],y[0], x[1],y[1], x[2],y[2]);
 break;
 case 4:
    temp[0][Xr]=(float)(x[0]+x[1]+x[2])/3.;
    temp[0][Yr]=(float)(y[0]+y[1]+y[2])/3.;
    temp[1][Xr]=(float)(x[1]+x[2]+x[3])/3.;
    temp[1][Yr]=(float)(y[1]+y[2]+y[3])/3.;
    brain.global.leg_ground_center[Xr]=(float)(temp[0][Xr]+temp[1][Xr])/2.;
    brain.global.leg_ground_center[Yr]=(float)(temp[0][Yr]+temp[1][Yr])/2.;
    brain.global.area_of_leg[0]=cal_area_trig( x[0],y[0], x[1],y[1], x[2],y[2])/2+cal_area_trig( x[3],y[3], x[1],y[1], x[2],y[2])/2;
    brain.global.steady_value=cal_steady_s4( brain.global.end_pos_global[0].x, brain.global.end_pos_global[0].y,x[0],y[0], x[1],y[1], x[2],y[2],x[3],y[3]);	
 break;
 default:
	  brain.global.area_of_leg[0]=0;
 break;
 }	 
 
 
//--------------------------计算中心和静态稳定余量FAKE 
if(brain.leg_move_state==S_BODY_MOVE||brain.leg_move_state==S_LEG_TRIG){ 
j=0;
for(i=1;i<5;i++){
		if(i!=brain.move_id){
		x[j]=brain.global.end_pos_global[i].x;
		y[j]=brain.global.end_pos_global[i].y;
		j++;
		}
  }
   brain.global.leg_ground_center_trig[Xr]=(float)(x[0]+x[1]+x[2])/3.;
   brain.global.leg_ground_center_trig[Yr]=(float)(y[0]+y[1]+y[2])/3.;
} 
}	



//重心控制
float center_control_out[2];
void center_control_global(float dt)//????PID  GLOBAL
{ static u8 reg_flag;
  u8 i;
	u8 id_f[3]={0};
	float ero[2];
	float center_tar_x,center_tar_y;
	float spd_use;
	float k_c=0,b_c=0;
	float k_c1=0,b_c1=0;
	//???????
	float gx=brain.global.leg_ground_center[Xr];
	float gy=brain.global.leg_ground_center[Yr];
	float k_spd=1;
	float tar_x,tar_y;
	u8 brain_ground_leg_num=brain.ground_leg_num;
	u8 leg_ground[5];
	
	for(i=0;i<5;i++)
	leg_ground[i]=leg[i].leg_ground;
	
	if(brain.leg_move_state==S_BODY_MOVE||brain.leg_move_state==S_LEG_TRIG){
	gx=brain.global.leg_ground_center_trig[Xr];
  gy=brain.global.leg_ground_center_trig[Yr];
	brain_ground_leg_num=3;	
	leg_ground[brain.move_id]=0;	
	}
	#if TEST_MODE1
	k_spd=10;
	if(brain.force_stop)
	spd_use=k_spd*brain.global.center_stable_weight/4;
	else
	spd_use=brain.spd*k_spd*brain.global.center_stable_weight;
	#else
	if(brain.force_stop)
		spd_use=brain.spd*k_spd*brain.global.center_stable_weight*brain.global.out_value;
	else
		spd_use=brain.spd*k_spd*brain.global.center_stable_weight*brain.global.out_value;
	#endif
	switch(brain_ground_leg_num)
	{
		case 3:
			//?????????????
		  if(leg[3].leg_ground==0) 
		   i=0;
			
			line_function_from_arrow(gx,gy,brain.spd_yaw,&k_c,&b_c);
		
			//??????????????????
		  u8 front_leg_num=0;
		  float f_leg_x,f_leg_y;//???????
		  for(i=1;i<5;i++)
		  if(leg_ground[i])
		      if(check_point_front_arrow(brain.global.end_pos_global[i].x,brain.global.end_pos_global[i].y,gx,gy,brain.spd_yaw)){
					   id_f[front_leg_num++]=i;
           // break;		
					}		
      brain.sys.front_leg_num=front_leg_num;					
		  f_leg_x=brain.global.end_pos_global[id_f[0]].x;
			f_leg_y=brain.global.end_pos_global[id_f[0]].y;
	 if(front_leg_num<2){	//?????		
			//???????????????
			float cross_point_x,cross_point_y;						
			float k_f,b_f;
			for(i=1;i<5;i++)
		  if(leg_ground[i]&&i!=id_f[0])		 
      {
       float k_temp,b_temp;
			 //????????	
			 line_function_from_two_point(f_leg_x,f_leg_y,brain.global.end_pos_global[i].x,brain.global.end_pos_global[i].y,&k_temp,&b_temp);				
       //????????			
				if(check_cross_arrow_line(gx,gy,brain.spd_yaw,k_temp,b_temp,&cross_point_x,&cross_point_y)){	
         //id_f[1]=i; 
				 k_f=k_temp;
				 b_f=b_temp;
				 break;}   					
			}
		  //????????
			cross_point_x-=brain.min_st[0]*sin((brain.spd_yaw)/57.3);
			cross_point_y-=brain.min_st[0]*cos((brain.spd_yaw)/57.3);
			//????
			int traj_flag;
			//????????
			traj_flag=check_point_front_arrow(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,cross_point_x,cross_point_y,brain.spd_yaw);
			if(traj_flag){
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_f,b_f,&tar_x,&tar_y);
				if(cross_point_x<f_leg_x)
				tar_x=LIMIT(tar_x,cross_point_x,f_leg_x);
				else
				tar_x=LIMIT(tar_x,f_leg_x,cross_point_x);	
				if(cross_point_y<f_leg_y)
				tar_y=LIMIT(tar_y,cross_point_y,f_leg_y);	
				else
				tar_y=LIMIT(tar_y,f_leg_y,cross_point_y);		
			}	
			else
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_c,b_c,&tar_x,&tar_y);
			
			if(cal_dis_of_points(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
		  f_leg_x,f_leg_y)<brain.min_st[0])
	    brain.force_stop=1;
			#if USE_SIMPLE_CENTER
			 //			//new
      float k_cc,b_cc;//中心和顶点直线		
			line_function_from_two_point(gx,gy,f_leg_x,f_leg_y,&k_cc,&b_cc);
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_cc,b_cc,&tar_x,&tar_y);
			#endif
//			
			if(cal_dis_of_points(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
		  f_leg_x,f_leg_y)<brain.min_st[0])
	    brain.force_stop=1;
		}
	  else//?????
		{
		  float f_leg_x1,f_leg_y1;
		  f_leg_x1=brain.global.end_pos_global[id_f[1]].x;
			f_leg_y1=brain.global.end_pos_global[id_f[1]].y;
		
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
			if(leg[2].leg_ground==0)
     		i=0;
			coner[0][Xr]=f_leg_x;coner[0][Yr]=f_leg_y;
			coner[1][Xr]=f_leg_x1;coner[1][Yr]=f_leg_y1;
			coner[2][Xr]=brain.global.end_pos_global[id3].x;coner[2][Yr]=brain.global.end_pos_global[id3].y;
			check_point_to_trig(gx,gy,brain.spd_yaw,f_leg_x,f_leg_y,f_leg_x1,f_leg_y1,brain.global.end_pos_global[id3].x,brain.global.end_pos_global[id3].y,
	    &jiao[0][Xr],&jiao[0][Yr],&jiao[1][Xr],&jiao[1][Yr]);
			float dis[2][3];
			dis[0][0]=cal_dis_of_points(jiao[0][Xr],jiao[0][Yr],f_leg_x,f_leg_y);
	    dis[0][1]=cal_dis_of_points(jiao[0][Xr],jiao[0][Yr],f_leg_x1,f_leg_y1);
	    dis[0][2]=cal_dis_of_points(jiao[0][Xr],jiao[0][Yr],brain.global.end_pos_global[id3].x,brain.global.end_pos_global[id3].y);
			dis[1][0]=cal_dis_of_points(jiao[1][Xr],jiao[1][Yr],f_leg_x,f_leg_y);
	    dis[1][1]=cal_dis_of_points(jiao[1][Xr],jiao[1][Yr],f_leg_x1,f_leg_y1);
	    dis[1][2]=cal_dis_of_points(jiao[1][Xr],jiao[1][Yr],brain.global.end_pos_global[id3].x,brain.global.end_pos_global[id3].y);
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
			coner_point[0][Xr]=coner[id_closet[0]][Xr];	coner_point[0][Yr]=coner[id_closet[0]][Yr];	
			coner_point[1][Xr]=coner[id_closet[1]][Xr]+brain.min_st[1]*sin((brain.spd_yaw)/57.3);	
			coner_point[1][Yr]=coner[id_closet[1]][Yr]+brain.min_st[1]*cos((brain.spd_yaw)/57.3);	
			//????????
			cross_point[0][Xr]=jiao[0][Xr]-brain.min_st[0]*sin((brain.spd_yaw)/57.3);
			cross_point[0][Yr]=jiao[0][Yr]-brain.min_st[0]*cos((brain.spd_yaw)/57.3);
			cross_point[1][Xr]=jiao[1][Xr]+brain.min_st[1]*sin((brain.spd_yaw)/57.3);
			cross_point[1][Yr]=jiao[1][Yr]+brain.min_st[1]*cos((brain.spd_yaw)/57.3);
			//
			float k_tr[3],b_tr[3];
			line_function_from_two_point(coner_point[0][Xr],coner_point[0][Yr],cross_point[0][Xr],cross_point[0][Yr],&k_tr[0],&b_tr[0]);				
 	    line_function_from_two_point(cross_point[0][Xr],cross_point[0][Yr],cross_point[1][Xr],cross_point[1][Yr],&k_tr[1],&b_tr[1]);
		  line_function_from_two_point(cross_point[1][Xr],cross_point[1][Yr],coner_point[1][Xr],coner_point[1][Yr],&k_tr[2],&b_tr[3]);
			
			int traj_flag[3];
			//????????
			traj_flag[0]=check_point_front_arrow(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,cross_point[0][Xr],cross_point[0][Yr],brain.spd_yaw);
			traj_flag[1]=check_point_front_arrow(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,cross_point[1][Xr],cross_point[1][Yr],brain.spd_yaw);
			
			if(traj_flag[0]==1&&traj_flag[1]==1)
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_tr[0],b_tr[0],&tar_x,&tar_y);
			else if(traj_flag[0]==0&&traj_flag[1]==0)
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_tr[2],b_tr[2],&tar_x,&tar_y);	
			//{tar_x=cross_point[1][Xr];tar_y=cross_point[1][Yr];}
			else
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_tr[1],b_tr[1],&tar_x,&tar_y);	
			#if USE_SIMPLE_CENTER
						//----new
			float k_cc1,b_cc1;
			line_function_from_two_point(gx,gy,coner_point[1][Xr],coner_point[1][Yr],&k_cc1,&b_cc1);
			traj_flag[2]=check_point_front_arrow(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,gx,gy,brain.spd_yaw);
			if(traj_flag[2])//line
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_tr[1],b_tr[1],&tar_x,&tar_y);	
			else //coner
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_cc1,b_cc1,&tar_x,&tar_y);	
			#endif
			
			if(cal_dis_of_points(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
		  coner_point[0][Xr],coner_point[0][Yr])<brain.min_st[0]||
			cal_dis_of_points(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
		  coner_point[1][Xr],coner_point[1][Yr])<brain.min_st[1])
	    brain.force_stop=1;
		}//end----
					
		  center_tar_x=tar_x;
			center_tar_y=tar_y;
		break;
		case 4:
			line_function_from_arrow(gx,gy,brain.spd_yaw,&k_c1,&b_c1);		
		  check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_c1,b_c1,&tar_x,&tar_y);
			center_tar_x=tar_x;
			center_tar_y=tar_y;
		break;	
	}

	brain.global.tar_center[Xr]=brain.tar_center[Xr]=center_tar_x;
	brain.global.tar_center[Yr]=brain.tar_center[Yr]=center_tar_y;

	ero[Xr]=my_deathzoom((brain.tar_center[Xr]-brain.global.end_pos_global[0].x),0.001);
  ero[Yr]=my_deathzoom((brain.tar_center[Yr]-brain.global.end_pos_global[0].y),0.001);
	float ero1[2];
	ero1[Xr]=my_deathzoom((brain.tar_center[Xr]-(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt)),0.001);
  ero1[Yr]=my_deathzoom((brain.tar_center[Yr]-(brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt)),0.001);
	
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
	leg[i].deng[Xr]=LIMIT(center_control_out[Xr],-8.8,8.8);
	leg[i].deng[Yr]=LIMIT(center_control_out[Yr],-8.8,8.8);	}
	}	
	
  //att set ?


	
	 reg_flag=brain.leg_move_state;
}



void center_control_global1(float dt)//中心控制PID  GLOBAL
{ static u8 reg_flag;
  u8 i;
	u8 id_f[3]={0};
	float ero[2];
	float center_tar_x,center_tar_y;
	float spd_use;
	float k_c=0,b_c=0;
	float k_c1=0,b_c1=0;
	//当前着地脚中心
	float gx=brain.global.leg_ground_center[Xr];
	float gy=brain.global.leg_ground_center[Yr];
	float k_spd=1;
	float tar_x,tar_y;
	u8 brain_ground_leg_num=brain.ground_leg_num;
	u8 leg_ground[5];
	
	for(i=0;i<5;i++)
	leg_ground[i]=leg[i].leg_ground;
	
	if(brain.leg_move_state==S_BODY_MOVE||brain.leg_move_state==S_LEG_TRIG){
	gx=brain.global.leg_ground_center_trig[Xr];
  gy=brain.global.leg_ground_center_trig[Yr];
	brain_ground_leg_num=3;	
	leg_ground[brain.move_id]=0;	
	}
	
	#if TEST_MODE1
	k_spd=1;
	if(brain.force_stop)
	spd_use=LIMIT(brain.spd,0,1)*k_spd*brain.global.center_stable_weight/4;
	else
	spd_use=brain.spd*k_spd*brain.global.center_stable_weight;
	#else
	if(brain.force_stop)
		spd_use=brain.spd*k_spd*brain.global.center_stable_weight/2;
	else
		spd_use=brain.spd*k_spd*brain.global.center_stable_weight;
	#endif
	
	
	      //resize
			float end_pos_global[5][2];
			float gx_use,gy_use;
      float size_k;
			size_k=LIMIT(1-(1-brain.sys.leg_local[1].x/brain.sys.leg_local[1].y)*fabs(sin(brain.spd_yaw/57.3)),0,1);		
			for(i=1;i<5;i++){
			resize_point_with_arrow(brain.global.end_pos_global[i].x,brain.global.end_pos_global[i].y,
			brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,brain.spd_yaw,size_k,&end_pos_global[i][Xr],&end_pos_global[i][Yr]);
			}
	

			resize_point_with_arrow(gx,gy,
			brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,brain.spd_yaw,size_k,&gx_use,&gy_use);
			
			line_function_from_arrow(gx_use,gy_use,brain.spd_yaw,&k_c,&b_c);
	
	
	switch(brain_ground_leg_num)
	{
		case 3:
			//计算着地脚中心沿矢量的方程
		  if(leg[3].leg_ground==0) 
		   i=0;
		
			//找到着地中心在移动方向上最近的着地腿
		  u8 front_leg_num=0;
		  float f_leg_x,f_leg_y;//前方着地腿坐标
			resize_point_with_arrow(brain.global.end_pos_global[id_f[0]].x,brain.global.end_pos_global[id_f[0]].y,
			brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,brain.spd_yaw,size_k,&f_leg_x,&f_leg_y);
		  for(i=1;i<5;i++)
		  if(leg_ground[i])
		      if(check_point_front_arrow(brain.global.end_pos_global[i].x,brain.global.end_pos_global[i].y,gx,gy,brain.spd_yaw)){
					   id_f[front_leg_num++]=i;	
					}		
									
      brain.sys.front_leg_num=front_leg_num;				
			
	 if(front_leg_num==1){//前方一条腿		  前腿抬起
			//计算着地中心前方直线方程和交点
			float cross_point_x,cross_point_y;		
      float coner_x,coner_y	;		
			float k_f,b_f;
			for(i=1;i<5;i++)
		  if(leg_ground[i]&&i!=id_f[0])		 
      {
       float k_temp,b_temp;
			 //遍历建立直线方程	
			 line_function_from_two_point(f_leg_x,f_leg_y,end_pos_global[i][Xr],end_pos_global[i][Yr],&k_temp,&b_temp);				
       //点矢量与直线交点			
				if(check_cross_arrow_line(gx_use,gy_use,brain.spd_yaw,k_temp,b_temp,&cross_point_x,&cross_point_y)){	
         id_f[2]=i; 
				 k_f=k_temp;
				 b_f=b_temp;
				 break;}   					
			}
			
		  //添加最小稳态余量
			cross_point_x-=brain.min_st[0]*sin((brain.spd_yaw)/57.3);
			cross_point_y-=brain.min_st[0]*cos((brain.spd_yaw)/57.3);
			//前方点坐标
      coner_x=f_leg_x-brain.min_st[0]*sin((brain.spd_yaw)/57.3);
		  coner_y=f_leg_y-brain.min_st[0]*cos((brain.spd_yaw)/57.3);
			line_function_from_two_point(cross_point_x,cross_point_y,coner_x,coner_y,&k_f,&b_f);		
			//规划目标
			int traj_flag;
			//判断ZMP点是否超过前方交点
			traj_flag=check_point_front_arrow(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
			cross_point_x,cross_point_y,brain.spd_yaw);
			if(traj_flag){//超过
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_f,b_f,&tar_x,&tar_y);
			//tar_x=cross_point_x;tar_y=cross_point_y;
			}	
			else
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_c,b_c,&tar_x,&tar_y);
		#if USE_SIMPLE_CENTER	
			//new
      float k_cc,b_cc;//中心和顶点直线		
			line_function_from_two_point(gx,gy,f_leg_x,f_leg_y,&k_cc,&b_cc);
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_cc,b_cc,&tar_x,&tar_y);
		#endif	
			if(cal_dis_of_points(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
		  f_leg_x,f_leg_y)<brain.min_st[0])
	    brain.force_stop=1;
			 
		}
	  else//----------前方两条腿  后退抬起--------------
		{
		  float f_leg_x1,f_leg_y1;//前方二号点
		  resize_point_with_arrow(brain.global.end_pos_global[id_f[1]].x,brain.global.end_pos_global[id_f[1]].y,
			brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,brain.spd_yaw,size_k,&f_leg_x1,&f_leg_y1);
			
			u8 id3=1;				 
			for(i=1;i<5;i++)
		  if(leg_ground[i]&&i!=id_f[0]&&i!=id_f[1])		 
      {
			id3=i;
      break;				
			}//三号点ID
			
			float jiao[2][2];
			float coner[3][2];
			if(leg[4].leg_ground==0)
				i=0;
			if(leg[2].leg_ground==0)
     		i=0;
			coner[0][Xr]=f_leg_x;coner[0][Yr]=f_leg_y;
			coner[1][Xr]=f_leg_x1;coner[1][Yr]=f_leg_y1;
			resize_point_with_arrow(brain.global.end_pos_global[id3].x,brain.global.end_pos_global[id3].y,
			brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,brain.spd_yaw,size_k,&coner[2][Xr],&coner[2][Yr]);
			//找前后两交点
			check_point_to_trig(gx_use,gy_use,brain.spd_yaw,f_leg_x,f_leg_y,f_leg_x1,f_leg_y1,brain.global.end_pos_global[id3].x,brain.global.end_pos_global[id3].y,
	    &jiao[0][Xr],&jiao[0][Yr],&jiao[1][Xr],&jiao[1][Yr]);
			//找离前后交点最近的两点
			float dis[2][3];
			dis[0][0]=cal_dis_of_points(jiao[0][Xr],jiao[0][Yr],f_leg_x,f_leg_y);
	    dis[0][1]=cal_dis_of_points(jiao[0][Xr],jiao[0][Yr],f_leg_x1,f_leg_y1);
	    dis[0][2]=cal_dis_of_points(jiao[0][Xr],jiao[0][Yr],coner[2][Xr],coner[2][Yr]);
			dis[1][0]=cal_dis_of_points(jiao[1][Xr],jiao[1][Yr],f_leg_x,f_leg_y);
	    dis[1][1]=cal_dis_of_points(jiao[1][Xr],jiao[1][Yr],f_leg_x1,f_leg_y1);
	    dis[1][2]=cal_dis_of_points(jiao[1][Xr],jiao[1][Yr],coner[2][Xr],coner[2][Yr]);
			float dis_big[2]={dis[0][0],dis[1][0]};
			u8 id_closet[2]={1,1};
			for(i=1;i<3;i++)
			  if(dis[0][i]<dis_big[0])
				{id_closet[0]=i;dis_big[0]=dis[0][i];}
			for(i=1;i<3;i++)
			  if(dis[1][i]<dis_big[1])
				{id_closet[1]=i;dis_big[1]=dis[1][i];}	
		
			float cross_point[2][2];
		  float coner_point[2][2];		
			coner_point[0][Xr]=coner[id_closet[0]][Xr];	
			coner_point[0][Yr]=coner[id_closet[0]][Yr];	
			coner_point[1][Xr]=coner[id_closet[1]][Xr]+brain.min_st[1]*sin((brain.spd_yaw)/57.3);	
			coner_point[1][Yr]=coner[id_closet[1]][Yr]+brain.min_st[1]*cos((brain.spd_yaw)/57.3);	
			//添加最小稳态余量
			cross_point[0][Xr]=jiao[0][Xr]-brain.min_st[0]*sin((brain.spd_yaw)/57.3);
			cross_point[0][Yr]=jiao[0][Yr]-brain.min_st[0]*cos((brain.spd_yaw)/57.3);
			cross_point[1][Xr]=jiao[1][Xr]+brain.min_st[1]*sin((brain.spd_yaw)/57.3);
			cross_point[1][Yr]=jiao[1][Yr]+brain.min_st[1]*cos((brain.spd_yaw)/57.3);
			//
			float k_tr[3],b_tr[3];
			line_function_from_two_point(coner_point[0][Xr],coner_point[0][Yr],cross_point[0][Xr],cross_point[0][Yr],&k_tr[0],&b_tr[0]);				
 	    line_function_from_two_point(cross_point[0][Xr],cross_point[0][Yr],cross_point[1][Xr],cross_point[1][Yr],&k_tr[1],&b_tr[1]);
		  line_function_from_two_point(cross_point[1][Xr],cross_point[1][Yr],coner_point[1][Xr],coner_point[1][Yr],&k_tr[2],&b_tr[3]);
			
			int traj_flag[3];
			//判断是否超过交点
			traj_flag[0]=check_point_front_arrow(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,cross_point[0][Xr],cross_point[0][Yr],brain.spd_yaw);
			traj_flag[1]=check_point_front_arrow(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,cross_point[1][Xr],cross_point[1][Yr],brain.spd_yaw);
			
			if(traj_flag[0]==1&&traj_flag[1]==1)
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_tr[0],b_tr[0],&tar_x,&tar_y);
			else if(traj_flag[0]==0&&traj_flag[1]==0)//后
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_tr[2],b_tr[2],&tar_x,&tar_y);	
			//{tar_x=cross_point[1][Xr];tar_y=cross_point[1][Yr];}
			else//line
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_tr[1],b_tr[1],&tar_x,&tar_y);	
			#if USE_SIMPLE_CENTER
			//----new
			float k_cc1,b_cc1;
			line_function_from_two_point(gx_use,gy_use,coner_point[1][Xr],coner_point[1][Yr],&k_cc1,&b_cc1);
			traj_flag[2]=check_point_front_arrow(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,gx,gy,brain.spd_yaw);
			if(traj_flag[2])//line
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_tr[1],b_tr[1],&tar_x,&tar_y);	
			else //coner
			check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_cc1,b_cc1,&tar_x,&tar_y);	
			#endif
			//--------------------------------------------------------
			if(cal_dis_of_points(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
		  coner_point[0][Xr],coner_point[0][Yr])<brain.min_st[0]||
			cal_dis_of_points(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
		  coner_point[1][Xr],coner_point[1][Yr])<brain.min_st[1])
	    brain.force_stop=1;
		}//end----
					
		  center_tar_x=tar_x;
			center_tar_y=tar_y;
		break;
		case 4://四脚着地
			line_function_from_arrow(gx_use,gy_use,brain.spd_yaw,&k_c1,&b_c1);		
		  check_cross_arrow90_line(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt,
			brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt,
			brain.spd_yaw,k_c1,b_c1,&tar_x,&tar_y);
			center_tar_x=tar_x;
			center_tar_y=tar_y;
		break;	
	}//end switch



	brain.global.tar_center[Xr]=brain.tar_center[Xr]=center_tar_x;
	brain.global.tar_center[Yr]=brain.tar_center[Yr]=center_tar_y;
  static float ero_r[2];
	float pid_d[2];
	ero[Xr]=my_deathzoom((brain.tar_center[Xr]-brain.global.end_pos_global[0].x),0.005);
  ero[Yr]=my_deathzoom((brain.tar_center[Yr]-brain.global.end_pos_global[0].y),0.005);
	pid_d[Xr]=(ero[Xr]-ero_r[Xr]);
	pid_d[Yr]=(ero[Yr]-ero_r[Yr]);
	
	center_control_out[Xr]=ero[Xr]*brain.sys.k_center_c[Xr];
	center_control_out[Yr]=ero[Yr]*brain.sys.k_center_c[Yr];
	
	ero_r[Xr]=ero[Xr];
	ero_r[Yr]=ero[Yr];
	
  float ero1[2];
	ero1[Xr]=my_deathzoom((brain.tar_center[Xr]-(brain.global.end_pos_global[0].x+sin(brain.spd_yaw/57.3)*spd_use*dt)),0.005); 
	ero1[Yr]=my_deathzoom((brain.tar_center[Yr]-(brain.global.end_pos_global[0].y+cos(brain.spd_yaw/57.3)*spd_use*dt)),0.005);
	float dis_ero=sqrt(pow(ero1[Xr],2)+pow(ero1[Yr],2));
	float max=brain.sys.leg_local[1].x/3;
	brain.global.center_stable_weight=LIMIT((max-LIMIT(dis_ero,0,max))/(max),0,1);
	
	if(dis_ero<0.68)
	brain.center_stable=1; 
	else
	brain.center_stable=0;
  //output
	for(i=1;i<5;i++){
	if((leg[i].control_mode||brain.control_mode)&&leg[i].leg_ground&&!leg[i].sys.leg_ground_force){	
	leg[i].deng[Xr]=LIMIT(center_control_out[Xr],-8.8,8.8);//cm/s
	leg[i].deng[Yr]=LIMIT(center_control_out[Yr],-8.8,8.8);//cm/s
		}
	}	
	
  //att set 缺


	 reg_flag=brain.leg_move_state;
}

static u8 stop_leg;
//判断机器人需要跨腿的ID // GLOBAL
void check_leg_need_move_global(BRAIN_STRUCT *in,float spd_body[3],float spd_tar[3],float w_tar,float dt)
{ u8 i,j;
	u8 need_move_leg=0;
	static u8 out_range_move[5];
	float spd=brain.spd;	
	static u8 last_move_id,last_last_move_id;
	static u8 lose_center_flag;
 
	if(brain.ground_leg_num<4)//may cause hard error when use if else
	brain.sys.dt_leg_min_trig_cnt=0;
	if(brain.sys.dt_leg_min_trig_cnt++>brain.sys.leg_move_min_dt*brain.sys.desire_time/dt)
	brain.can_move_leg=1;
	
	float yaw_in=To_180_degrees(brain.spd_yaw);
	float yaw_temp=brain.sys.yaw_trig;
	u8 way;//侧身方向
	u8 fp_point_id[4];//L  R
	if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)//r
	{fp_point_id[0]=1;fp_point_id[1]=2;   fp_point_id[2]=3;fp_point_id[3]=4;brain.way=way=2;}
	else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)//l
	{fp_point_id[0]=4;fp_point_id[1]=3;   fp_point_id[2]=2;fp_point_id[3]=1;brain.way=way=4;}
	else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))//f
	{fp_point_id[0]=3;fp_point_id[1]=1;   fp_point_id[2]=4;fp_point_id[3]=2;brain.way=way=1;}
	else//b
	{fp_point_id[0]=2;fp_point_id[1]=4;   fp_point_id[2]=1;fp_point_id[3]=3;brain.way=way=3;}
		
 //超出移动范围需要跨腿
	float range_in[5]={1},range_out[5]={1};
	float jiaodiao[2][2]={0};	
 for(i=1;i<5;i++){
	//check out range
	 #if TEST_MODE1
	 if(leg[i].leg_ground||1){
	 #else
	 if(leg[i].leg_ground){
	 #endif
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
	 //cal_jiao_of_tuo_and_line(circle_center[0],circle_center[1],in->sys.leg_move_range[Xr],in->sys.leg_move_range[Yr],in->spd_yaw,&jiaodiao[0][Xr],&jiaodiao[0][Yr],&jiaodiao[1][Xr],&jiaodiao[1][Yr]); 
	 //cal_jiao_of_range_and_line(i,circle_center[0],circle_center[1],in->sys.min_range,in->sys.max_range,in->spd_yaw,&jiaodiao[0][Xr],&jiaodiao[0][Yr],&jiaodiao[1][Xr],&jiaodiao[1][Yr]);
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
	 out_range_move[i]++;
   brain.force_stop=1; 			 
	 }
	 
	 brain.leg_move[i]=leg[i].need_move;
  }
 } 
 float max_out_range=brain.global.dis_leg_out[1];
 for(i=1;i<5;i++)
 if(brain.global.dis_leg_out[i]>max_out_range) 
 {max_out_range=brain.global.dis_leg_out[i];}
 
 brain.global.out_value=LIMIT(1-LIMIT(max_out_range,0,2.618)/2.618,0.12,1);
 
//............................................................................................................
 //----------.......................跨腿触发条件:重心越过 两交点  .......................................
 if(brain.ground_leg_num>3&&brain.center_stable)
 {
	 float k1,b1,k2,b2;
	 float cro_x,cro_y;
	 cro_x=(brain.global.end_pos_global[1].x+brain.global.end_pos_global[2].x+brain.global.end_pos_global[3].x+brain.global.end_pos_global[4].x)/4;
   cro_y=(brain.global.end_pos_global[1].y+brain.global.end_pos_global[2].y+brain.global.end_pos_global[3].y+brain.global.end_pos_global[4].y)/4;
   #if TEST_MODE1
//   if(check_point_front_arrow(brain.global.end_pos_global[0].x
//		 -sin(brain.spd_yaw/57.3)*brain.min_st[1],
//	 brain.global.end_pos_global[0].y-cos(brain.spd_yaw/57.3)*brain.min_st[1],cro_x,cro_y,brain.spd_yaw)==1)
//	 {
//	
//	   if((cal_dis_of_points(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,cro_x,cro_y)>brain.min_st[1]*1)||
//			 brain.sys.front_leg_num==1)

//		   need_move_leg=1;
//	 }
	 
	 if(check_leg_near_init(brain.sys.in_rst_check)==0||brain.spd>0)	
	 {
	
//	   if((cal_dis_of_points(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,cro_x,cro_y)>brain.min_st[1]*1)||
//			 brain.sys.front_leg_num==1)

		   need_move_leg=1;
	 }
	 #else
//	     if(check_point_front_arrow(brain.global.end_pos_global[0].x
//		 -sin(brain.spd_yaw/57.3)*brain.min_st[1],
//	 brain.global.end_pos_global[0].y-cos(brain.spd_yaw/57.3)*brain.min_st[1],cro_x,cro_y,brain.spd_yaw)==1)
   if(check_leg_near_init(brain.sys.in_rst_check)==0||brain.spd>0)	
	 {
	
//	   if((cal_dis_of_points(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,cro_x,cro_y)>brain.min_st[1]*1)||
//			 brain.sys.front_leg_num==1)

		   need_move_leg=1;
	 }
	 #endif
 }
 ///////////////////////////////////

//---------------------------------------规划腿---状态机------------------------------------	
  static float center_now[2];
  if(brain.leg_move_state==S_LEG_TRIG&&brain.global.steady_value>0.8&&brain.ground_leg_num>3)
	{
	
	if(cal_dis_of_points(center_now[Xr],center_now[Yr],brain.global.leg_ground_center[Xr],brain.global.leg_ground_center[Yr])>1){	
	brain.leg_move_state=S_LEG_TRIG_LEAVE_GROUND_CHECK;		 
	leg_tar_est_global(&brain,&leg[brain.move_id],0,0,0,1,dt);	}
	i=0;
	}

  if(brain.leg_move_state==S_LEG_TRIG_LEAVE_GROUND_CHECK&&brain.ground_leg_num<4)
	{brain.leg_move_state=S_LEG_TRIG_ING;}
 
	if(brain.leg_move_state==S_LEG_TRIG_ING&&brain.ground_leg_num>3)
	{brain.leg_move_state=S_IDLE;}
 
	static u16 mask;
  if(brain.leg_move_state==S_BODY_MOVE)
	{
		mask++;
		center_now[Xr]=brain.global.leg_ground_center[Xr];
		center_now[Yr]=brain.global.leg_ground_center[Yr];
		//if(mask>6){mask=0;
		brain.leg_move_state=S_LEG_TRIG;
	//}		 
	}
	
  #if !USE_LEG_TRIG_DELAY
   brain.leg_move_state=S_IDLE;
	#endif
	
 static u8 leg_flag; 
 if(need_move_leg&&brain.can_move_leg&&brain.ground_leg_num>3&&brain.leg_move_state==S_IDLE)
 {
 u8 id_need_to_move=1;
 float yaw_in=To_180_degrees(in->spd_yaw);
 float yaw_temp=in->sys.yaw_trig;
 
 if(brain.rst_all_soft){	 
	 leg_flag=brain.rst_all_soft;
	 if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)
	 id_need_to_move=trig_list_r[leg_flag];
	 else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)
	 id_need_to_move=trig_list_l[leg_flag];
	 else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))
	 id_need_to_move=trig_list_f[leg_flag];
	 else
	 id_need_to_move=trig_list_b[leg_flag];
	 
	 goto endl;
	 }
	else{
   //高优先 --超出移动范围
	 float most_out=out_range_move[1];
   for(i=2;i<5;i++){
     if(out_range_move[i]>most_out)	 
		 {id_need_to_move=i;most_out=out_range_move[i];} 
	 }
	 
	 id_need_to_move=LIMIT(id_need_to_move,1,4);
	 if(out_range_move[id_need_to_move]>0)
	 { out_range_move[id_need_to_move]=0;
	   goto endl;//输出
	 }
	 
	 if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)
	 id_need_to_move=trig_list_r[leg_flag];
	 else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)
	 id_need_to_move=trig_list_l[leg_flag];
	 else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))
	 id_need_to_move=trig_list_f[leg_flag];
	 else
	 id_need_to_move=trig_list_b[leg_flag];
	 
	 //低优先 --面积贪婪规划器
   //id_need_to_move=planner_leg(last_move_leg,last_last_move_id);///////////////////////////mine new/////////////////////////
  }
 
		leg_flag++;
		if(leg_flag>4)
		leg_flag=1;

	 //	姿态倾斜  缺
	 endl:;
	 last_last_move_id=last_move_leg;
	 last_move_leg=in->move_id=leg_repeat_protect(id_need_to_move, last_move_id, last_last_move_id,brain.spd_yaw,brain.sys.yaw_trig);
	 #if !DENG_LIMIT_TEST	
	 if(brain.leg_move_state==S_IDLE)	
	 brain.leg_move_state=S_BODY_MOVE;	

	 #if !USE_LEG_TRIG_DELAY
	 leg_tar_est_global(&brain,&leg[brain.move_id],0,0,0,1,dt);
	 out_range_move[brain.move_id]=0;
		//	姿态倾斜 
		if(brain.move_id==1||brain.move_id==3)	 
		brain.tar_att[0]=-brain.sys.att_off[0]; 
		else
		brain.tar_att[0]=brain.sys.att_off[0]; 	 

		if(brain.move_id==1||brain.move_id==2)	 
		brain.tar_att[1]=brain.sys.att_off[1]; 
		else
		brain.tar_att[1]=-brain.sys.att_off[1]; 
	 
	 
	 #endif	
	 #endif
	 brain.can_move_leg=0;
	 leg[brain.move_id].sys.leg_move_pass_cnt=0;	 
 }// end plan 
 


 //--姿态测试
  if(brain.tar_att_force[0]!=0)
	 brain.tar_att[0]=brain.tar_att_force[0];
	if(brain.tar_att_force[1]!=0)
	 brain.tar_att[1]=brain.tar_att_force[1];

}

//-----------------------GLOBAL
u8 planner_leg(u8 last_move_id,u8 last_last_move_id)
{ 
	u8 i;

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

   float k_lr,b_lr;
	 float k_l3,b_l3;
	 float k_r3,b_r3;
   line_function_from_two_point(brain.global.end_pos_global[fp_point_id[0]].x,brain.global.end_pos_global[fp_point_id[0]].y,
	 brain.global.end_pos_global[fp_point_id[1]].x,brain.global.end_pos_global[fp_point_id[1]].x,
	 &k_lr,&b_lr);
	 //L
	 line_function_from_two_point(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
	 brain.global.end_pos_global[fp_point_id[2]].x,brain.global.end_pos_global[fp_point_id[2]].x,
	 &k_l3,&b_l3);
	 //R
	 line_function_from_two_point(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
	 brain.global.end_pos_global[fp_point_id[3]].x,brain.global.end_pos_global[fp_point_id[3]].x,
	 &k_r3,&b_r3);
	 //计算两方向左右两侧面积
	 float coner_l[4][2];
	 check_cross_arrow_line(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
	 brain.spd_yaw,k_lr,b_lr,&coner_l[0][Xr],&coner_l[0][Yr]);
	 check_cross_arrow90_line(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
	 brain.spd_yaw,k_l3,b_l3,&coner_l[1][Xr],&coner_l[1][Yr]);
	 coner_l[2][Xr]=brain.global.end_pos_global[fp_point_id[0]].x;coner_l[2][Yr]=brain.global.end_pos_global[fp_point_id[0]].y;
	 coner_l[3][Xr]=brain.global.end_pos_global[0].x;coner_l[3][Yr]=brain.global.end_pos_global[0].y;
	 float coner_r[4][2];
	 check_cross_arrow_line(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
	 brain.spd_yaw,k_lr,b_lr,&coner_r[0][Xr],&coner_r[0][Yr]);
	 check_cross_arrow90_line(brain.global.end_pos_global[0].x,brain.global.end_pos_global[0].y,
	 brain.spd_yaw,k_r3,b_r3,&coner_r[1][Xr],&coner_r[1][Yr]);
	 coner_r[2][Xr]=brain.global.end_pos_global[fp_point_id[1]].x;coner_r[2][Yr]=brain.global.end_pos_global[fp_point_id[1]].y;
	 coner_r[3][Xr]=brain.global.end_pos_global[0].x;coner_r[3][Yr]=brain.global.end_pos_global[0].y; 
	 
	 //cal_area_size
	 float size[2];
	 //L
	 size[0]=cal_area_trig(coner_l[0][Xr],coner_l[0][Yr],coner_l[2][Xr],coner_l[2][Yr],coner_l[3][Xr],coner_l[3][Yr])+
	 cal_area_trig(coner_l[1][Xr],coner_l[1][Yr],coner_l[2][Xr],coner_l[2][Yr],coner_l[3][Xr],coner_l[3][Yr]);
	 //R
	 size[1]=cal_area_trig(coner_r[0][Xr],coner_r[0][Yr],coner_r[2][Xr],coner_r[2][Yr],coner_r[3][Xr],coner_r[3][Yr])+
	 cal_area_trig(coner_r[1][Xr],coner_r[1][Yr],coner_r[2][Xr],coner_r[2][Yr],coner_r[3][Xr],coner_r[3][Yr]);
	 
	 //plan way
	 u8 way;
	 if(size[0]>size[1])
		 way=0;
	 else
		 way=1;
	 
	 //plan leg
	 u8 can_leg_id[2];
	 
	 switch(way_sel)
	 {
		//f
		case 1:
		if(way==0)
		{can_leg_id[0]=3;can_leg_id[1]=4;}
		else
		{can_leg_id[0]=1;can_leg_id[1]=2;}	
		break;
		//r
		case 2:
		if(way==0)
		{can_leg_id[0]=1;can_leg_id[1]=3;}
		else
		{can_leg_id[0]=2;can_leg_id[1]=4;}	
		break;
		//b
		case 3:
		if(way==0)
		{can_leg_id[0]=2;can_leg_id[1]=1;}
		else
		{can_leg_id[0]=4;can_leg_id[1]=3;}	
		break;
		//l
		case 4:
		if(way==0)
		{can_leg_id[0]=4;can_leg_id[1]=2;}
		else
		{can_leg_id[0]=3;can_leg_id[1]=1;}	
		break;
	 }	 
	 
	 //decide final leg				 
		static u8 flag1=0;//0  扩大重心  1 跟进
    //判断下一次重心是否被扩充 缺
		if(flag1==0)
    {	flag1=1;return can_leg_id[0];}		
		else
		{	flag1=0;return can_leg_id[1];}			
}	

u8 en_off_trig=1;
float h_k2=1.0;
float k_off=0.2;
float k_trig1=2;
void leg_tar_est_global(BRAIN_STRUCT *in,LEG_STRUCT *leg,float spd_body[3],float spd_tar[3],float w_tar,u8 need_move,float dt)
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
	 off_k=1-k_off;
 else
   off_k=1+k_off;
 
if(leg->sys.id==1||leg->sys.id==2)
off_x=LIMIT(leg->pos_now[2].x-leg->sys.init_end_pos.x,0,leg->sys.off_local[0])*fabs(cos(tar_yaw/57.3));
else
off_x=LIMIT(leg->pos_now[2].x-leg->sys.init_end_pos.x,-leg->sys.off_local[0],0)*fabs(cos(tar_yaw/57.3));	
if(leg->sys.id==1||leg->sys.id==3)
off_y=LIMIT(leg->pos_now[2].y-leg->sys.init_end_pos.y,0,leg->sys.off_local[1])*fabs(sin(tar_yaw/57.3));
else
off_y=LIMIT(leg->pos_now[2].y-leg->sys.init_end_pos.y,-leg->sys.off_local[1],0)*fabs(sin(tar_yaw/57.3));	

if(brain.spd>0.68)
spd=LIMIT(spd,brain.spd*2,10);
tar_x=LIMIT(sin(tar_yaw/ 57.3f)*LIMIT(spd*off_k*in->sys.k_spd_to_range,-8,8),-in->sys.leg_move_range[Xr],in->sys.leg_move_range[Xr]);
tar_y=LIMIT(cos(tar_yaw/ 57.3f)*LIMIT(spd*off_k*in->sys.k_spd_to_range,-8,8),-in->sys.leg_move_range[Yr],in->sys.leg_move_range[Yr]);
leg->pos_tar_trig[2].x=leg->sys.init_end_pos.x*k_trig1+tar_x+RANDOM+off_x*cos(tar_yaw/ 57.3f)*en_off_trig;
leg->pos_tar_trig[2].y=leg->sys.init_end_pos.y*k_trig1+tar_y+RANDOM+off_y*sin(tar_yaw/ 57.3f)*en_off_trig;
leg->pos_tar_trig[2].z=leg->sys.init_end_pos.z*h_k2;


//LIMIT
 limit_move_range_tangle( id
,leg->sys.init_end_pos.x,leg->sys.init_end_pos.y,
 leg->pos_tar_trig[2].x,leg->pos_tar_trig[2].y
 ,in->sys.min_range,in->sys.max_range
 ,&leg->pos_tar_trig[2].x,&leg->pos_tar_trig[2].y);

if(brain.rst_all_soft>0)
{
brain.rst_all_soft++;
leg->pos_tar_trig[2].x=leg->sys.init_end_pos.x*k_trig1+RANDOM;//+off_x*cos(tar_yaw/ 57.3f);
leg->pos_tar_trig[2].y=leg->sys.init_end_pos.y*k_trig1+RANDOM;//+off_y*sin(tar_yaw/ 57.3f);
leg->pos_tar_trig[2].z=leg->sys.init_end_pos.z*1;
}
if(brain.rst_all_soft>4)
brain.rst_all_soft=0;
}