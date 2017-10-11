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

in->area_of_leg[1]=(leg[1].sys.init_end_pos.x+W/2)*(leg[1].sys.init_end_pos.y+L/2)*4;
	
in->sys.min_range=0.707*leg[1].sys.l3*0.3;//7.8 4.742556
in->sys.max_range=0.707*leg[1].sys.l2*0.78;//6.3 3.830526
in->sys.leg_move_range[Yr]=MAX(in->sys.max_range, in->sys.min_range);//cm		
in->sys.leg_move_range[Xr]=W/4*0.88;//cm	
//for leg 1
in->sys.leg_move_range1[1]=in->sys.max_range;
in->sys.leg_move_range1[2]=0.707*leg[1].sys.l2;
in->sys.leg_move_range1[3]=in->sys.min_range;
in->sys.leg_move_range1[4]=0.707*leg[1].sys.l3*0.1;

in->sys.kp_center[0]=0.8;
in->sys.kp_center[1]=0.6;
in->sys.k_center_fp=0.0;

in->sys.move_range_k=0.66;

in->min_st[0]=1;//cm
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
in->sys.leg_h=3.68;
#if USE_LEG_TRIG_DELAY
in->sys.desire_time=0.468;//0.7;//0.76;
#else
in->sys.desire_time=0.8;//0.7;//0.76;
#endif
in->sys.leg_move_min_dt=in->sys.desire_time*0.68;//35 ;

in->sys.k_spd_to_range=2;

in->sys.yaw_dead=10;

in->sys.down_h=0;//0.88;
in->sys.down_t=0;//0.2;
in->sys.in_rst_check=1.618;

leg[1].sys.id=1;
leg[2].sys.id=2;
leg[3].sys.id=3;
leg[4].sys.id=4;


#if TEST_MODE1
in->sys.k_center_c[0]=4;
in->sys.k_center_c[1]=4;
in->sys.desire_time=3.3;//0.76;
in->sys.leg_move_min_dt=0;
in->sys.down_t=in->sys.down_h=0;
#endif
}	


//Ñ¡ÔñÐèÒª¿çÍÈ 
float test1[3];
void find_leg_need_move(float spd_tar[3],float str[4],float end[4]) {
float yaw_spd,yaw_line,yaw_ero;
	if(fabs(spd_tar[Xr])>0||fabs(spd_tar[Yr])>0)
	{
			yaw_spd=fast_atan2(spd_tar[Xr],spd_tar[Yr])*57.3+180;
			float x_point_s,y_point_s,x_point_e,y_point_e; 
			x_point_s=str[Xr];  
		  y_point_s=str[Yr];
			x_point_e=end[Xr];  
		  y_point_e=end[Yr];
			
				float y_se= y_point_e-y_point_s;
				float x_se= x_point_e-x_point_s;
				if (x_se==0 && y_se>0)
						yaw_line = 360;
				if (x_se==0 && y_se<0)
						yaw_line = 180;
				if (y_se==0 && x_se>0)
						yaw_line = 90;
				if (y_se==0 && x_se<0)
						yaw_line = 270;
				float temp=fast_atan2(x_se,y_se)*57.3;
				if (x_se>0 && y_se>0)
					 yaw_line = temp;
				else if( x_se<0 && y_se>0)
					 yaw_line = 360 + temp;
				else if (x_se<0 && y_se<0)
					 yaw_line = 360 +temp;
				else if (x_se>0 && y_se<0)
					 yaw_line = temp;
			
			
			yaw_ero=fabs(To_180_degrees(yaw_line-yaw_spd));
			if(yaw_ero==90)//´¹Ö±
			{
			float random=RNG_Get_RandomRange(0,9);//»ñÈ¡[0,9]Çø¼äµÄËæ»úÊý
			  if(random>4.5)
				{
				leg[(u8)end[3]].need_move+=2;
		  	//leg[(u8)end[3]].leg_move_pass_cnt=0;
        //leg[(u8)str[3]].leg_move_pass_cnt++;	
				}else{
				leg[(u8)str[3]].need_move+=2;
			  //leg[(u8)str[3]].leg_move_pass_cnt=0;
        //leg[(u8)end[3]].leg_move_pass_cnt++;		
					
				}
				
			}	
			else if(yaw_ero>90)//¶Û½Ç
			{
			leg[(u8)end[3]].need_move+=2;
      //leg[(u8)end[3]].leg_move_pass_cnt++;				
			}	
			else//Èñ½Ç
			{
			leg[(u8)str[3]].need_move+=2;
      //leg[(u8)str[3]].leg_move_pass_cnt++;	
			}	
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
		
				estimate_center(&brain,brain.att,brain.now_spd,brain.now_acc,brain.sys.tar_spd,brain.tar_w);//¹À¼Æ»úÌåÖØÐÄ
    #if TEST_MODE1
    brain.fall=0;
    #endif				
		if(brain.fall!=0)
			  fall_reset(dt);
		else{
			  state_clear();	
			
			  cal_pos_global(dt);
			
			  cal_center_of_leg_ground(&brain);//¹À¼Æ×ÅµØ¶à±ßÐÎ
	      #if BODY_CONTROL_USE_GLOBAL
				check_leg_need_move_global(&brain,brain.now_spd,brain.sys.tar_spd,brain.tar_w,dt);//ÅÐ¶ÏÐèÒª¿ç½Å  ¹æ»®¿çÍÈ		
        
			  center_control_global(dt);//ÔÚ×ÅµØÇøÓòÄÚÒÆ¶¯ÖØÐÄ
			
				att_control(dt);	
        #else			
 				check_leg_need_move(&brain,brain.now_spd,brain.sys.tar_spd,brain.tar_w,dt);//ÅÐ¶ÏÐèÒª¿çµÄ½Å

				cal_target_of_center_move(&brain);
				
				center_control(dt);	
			
				att_control(dt);
			
        cal_deng_from_spd(&brain);	
 			  
				if(brain.leg_move_state==S_BODY_MOVE&&brain.center_stable)
				brain.leg_move_state=S_LEG_TRIG;
	 			 
				if(brain.leg_move_state==S_LEG_TRIG){			
					leg_tar_est(&brain,&leg[brain.move_id],0,0,0,1,dt);
					brain.can_move_leg=0;
					leg[brain.move_id].sys.leg_move_pass_cnt=0;	 
					brain.leg_move_state=S_LEG_TRIG_LEAVE_GROUND_CHECK;
				}	
				
				if(brain.leg_move_state==S_LEG_TRIG_LEAVE_GROUND_CHECK&&brain.ground_leg_num<4)
					brain.leg_move_state=S_LEG_TRIG_ING;
				
				if(brain.leg_move_state==S_LEG_TRIG_ING&&brain.ground_leg_num>3)
				{brain.leg_move_state=S_BODY_RETURN;brain.tar_att[0]=brain.tar_att[1]=0;}
				
			  if(brain.leg_move_state==S_BODY_RETURN&&brain.center_stable)
				{brain.force_stop=brain.leg_move_state=S_IDLE;	
					
					if(brain.rst_all_soft>0)
						brain.rst_all_soft++;
					if(brain.rst_all_soft>4)
						 brain.rst_all_soft=0;
				}		
				leg_trig_flag_reg=brain.leg_move_state;
				#endif	
		 }
	}//end_init
}
//------------------------------------------------START HERE-----------------------------------------------------------------

///////////////////////////////////////////////////OLD LOCAL///////////////////////////////////////////////


float k_acc=0;
//¹À¼Æ»úÆ÷ÈËÖÐÐÄ
void estimate_center(BRAIN_STRUCT *in,float att[3],float spd_body[3],float acc_body[3],float spd_tar[3],float w_tar)//¹À¼Æ»úÌåÖØÐÄ
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
		
//	in->center.x=0-leg[1].leg_ground*in->sys.center_off_when_move[Xr]-leg[2].leg_ground*in->sys.center_off_when_move[Xr]
//	+leg[3].leg_ground*in->sys.center_off_when_move[Xr]+leg[4].leg_ground*in->sys.center_off_when_move[Xr];
//	
//	in->center.y=0-leg[1].leg_ground*in->sys.center_off_when_move[Yr]-leg[3].leg_ground*in->sys.center_off_when_move[Yr]
//	+leg[2].leg_ground*in->sys.center_off_when_move[Yr]+leg[4].leg_ground*in->sys.center_off_when_move[Yr];
//	in->center.x*=k_off_z;	
//	in->center.y*=k_off_z;	
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



//¼ÆËã×ÅµØ½ÅÖØÐÄ
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


//ÅÐ¶Ï»úÆ÷ÈËÐèÒª¿çÍÈµÄID
void check_leg_need_move(BRAIN_STRUCT *in,float spd_body[3],float spd_tar[3],float w_tar,float dt)
{ u8 i;
	u8 cnt;
	u8 min_id[2];
	static u8 lose_center_flag;
	float end_pos[4][4];
	float temp_x,temp_y;
  brain.force_stop=0;
	for(i=1;i<5;i++)//´æ´¢ÁÙÊ±±äÁ
	{
		if(leg[i].leg_ground){
      leg[i].need_move=0;	
      brain.leg_out_range[i]=0;			
		}
	}
		
 //³¬³öÒÆ¶¯·¶Î§ÐèÒª¿çÍÈ
	float range_in[5],range_out[5];
  u8 out_cnt=0;
	float spd=my_sqrt(pow(spd_tar[Xr],2)+pow(spd_tar[Yr],2));	
	float tyaw=90-in->spd_yaw+0.0001;
	float jiaodiao[2][2];	
 for(i=1;i<5;i++){
	//check out range
	 float length,tar_x,tar_y;//kick the bias of local 
	 if(i==1||i==2)
	 tar_x=leg[i].pos_now[2].x-leg[i].sys.off_local[Xr]+dt*LIMIT(spd_tar[Xr],-0.5,0.5)*0;
	  else
	 tar_x=leg[i].pos_now[2].x+leg[i].sys.off_local[Xr]+dt*LIMIT(spd_tar[Xr],-0.5,0.5)*0;
		
   if(i==1||i==3) 
	 tar_y=leg[i].pos_now[2].y-leg[i].sys.off_local[Yr]+dt*LIMIT(spd_tar[Yr],-0.5,0.5)*0;	
	 else
	 tar_y=leg[i].pos_now[2].y+leg[i].sys.off_local[Yr]+dt*LIMIT(spd_tar[Yr],-0.5,0.5)*0;
	 
	 //¼ÆËãËÙ¶ÈÖ±ÏßÓëÍÖÔ²½»µã
	 float temp=sqrt(pow(in->sys.leg_move_range[Xr],2)/(1+pow(in->sys.leg_move_range[Xr]*tan(tyaw/57.3)/in->sys.leg_move_range[Yr],2)));
	 //ÅÐ¶ÏËÙ¶È·½Ïò½»µã·ûºÅ
	 if(in->spd_yaw+0.0001>0&&in->spd_yaw+0.0001<180)
	 {jiaodiao[0][Xr]=temp;jiaodiao[1][Xr]=-temp;}
	 else 
	 {jiaodiao[0][Xr]=-temp;jiaodiao[1][Xr]=temp;}
	 jiaodiao[0][Yr]=tan(tyaw/57.3)*jiaodiao[0][Xr];
	 jiaodiao[1][Yr]=tan(tyaw/57.3)*jiaodiao[1][Xr];
	 jiaodiao[0][Xr]+=leg[i].sys.off_local[Xr];jiaodiao[1][Xr]+=leg[i].sys.off_local[Xr];
	 jiaodiao[0][Yr]+=leg[i].sys.off_local[Yr];jiaodiao[1][Yr]+=leg[i].sys.off_local[Yr];
	 //¼ÆËãµ±Ç°µãÓë½»µã¾àÀë  0->ËÙ¶ÈÖ¸Ïò½»µã 
	 float dis[2];
	 dis[0]=pow(leg[i].pos_now[2].x-jiaodiao[0][Xr],2)+pow(leg[i].pos_now[2].y-jiaodiao[0][Yr],2);
	 dis[1]=pow(leg[i].pos_now[2].x-jiaodiao[1][Xr],2)+pow(leg[i].pos_now[2].y-jiaodiao[1][Yr],2);
	 
	 range_in[i]=pow(tar_x/(in->sys.leg_move_range[Xr]*brain.sys.move_range_k),2)+pow(tar_y/(in->sys.leg_move_range[Yr]*brain.sys.move_range_k),2);
	
	 
	 if(spd>0||brain.rst_all_soft)
	  brain.tabu=1;
	 else
		brain.tabu=0;  
	 
	 if((dis[1]<dis[0]&&range_in[i]>1&&spd>0)||brain.tabu)
	 {
	 leg[i].need_move=1;	 
	 }
	 
	 range_out[i]=pow(tar_x/in->sys.leg_move_range[Xr],2)+pow(tar_y/in->sys.leg_move_range[Yr],2);
	 if(dis[1]<dis[0]&&range_out[i]>1&&spd>0)
	 { 
	 leg[i].need_move=3;	 
	 brain.force_stop=1; 	 
	 }
	 
	 brain.leg_move[i]=leg[i].need_move;
 } 
  
   get_leg_tar_trig(in,brain.now_spd,brain.sys.tar_spd,brain.tar_w, dt);
}



//ÅÐ¶Ï¿ç½Å
u8 last_move_leg;	
void get_leg_tar_trig(BRAIN_STRUCT *in,float spd_body[3],float spd_tar[3],float w_tar,float dt)//mmmm
{
static float dt_leg_min_trig_cnt;
u8 i,cnt,id,id_star,leg_move_pass_cnt;
u8 leg_out_cnt[3]={0};//flag of state robot	
u8 out_range_id[4];
u8 loss_center_id[4];	
static u8 leg_flag;
	for(i=1;i<5;i++){//cnt_leg_situation
	if(leg[i].need_move==1)//out range leg
	{
	 	leg[i].sys.leg_move_pass_cnt++;
	 leg_out_cnt[0]++;	
	}
	else if(leg[i].need_move==2)//loss center
	{leg[i].sys.leg_move_pass_cnt++;
	 leg_out_cnt[1]++;
	}
	else if(leg[i].need_move==3)//out range force
	{leg[i].sys.leg_move_pass_cnt+=3;
	 leg_out_cnt[2]++;
	}
 }
	
 if(brain.ground_leg_num<4)//may cause hard error when use if else
	 brain.sys.dt_leg_min_trig_cnt=0;
 if(brain.sys.dt_leg_min_trig_cnt++>brain.sys.leg_move_min_dt*brain.sys.desire_time/dt)
	 brain.can_move_leg=1;
 
 //ÅÐ¶ÏË­È¨ÏÞ¸ß
 u8 id_need_to_move=RNG_Get_RandomRange(1,4);
 
 float yaw_in=To_180_degrees(in->spd_yaw);
 float yaw_temp=in->sys.yaw_trig;
 
 if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)
 id_need_to_move=trig_list_r[leg_flag];
 else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)
 id_need_to_move=trig_list_l[leg_flag];
 else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))
 id_need_to_move=trig_list_f[leg_flag];
 else
 id_need_to_move=trig_list_f[leg_flag];
	 
 if(brain.can_move_leg&&brain.leg_move_state==S_IDLE&&(leg_out_cnt[0]+leg_out_cnt[1]+leg_out_cnt[2]>0)){
	 
 last_move_leg=in->move_id = id_need_to_move;
 //	×ËÌ¬ÇãÐ± 
	if(id_need_to_move==1||id_need_to_move==3)	 
	brain.tar_att[0]=-brain.sys.att_off[0]; 
	else
	brain.tar_att[0]=brain.sys.att_off[0]; 	 

	if(id_need_to_move==1||id_need_to_move==2)	 
	brain.tar_att[1]=brain.sys.att_off[1]; 
	else
	brain.tar_att[1]=-brain.sys.att_off[1]; 
 leg_flag++;
  if(leg_flag>4)
   leg_flag=1;	

  if(brain.rst_all_soft>0)
		{
			if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)
			last_move_leg=in->move_id =trig_list_r[brain.rst_all_soft];
			else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)
			last_move_leg=in->move_id =trig_list_l[brain.rst_all_soft];
			else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))
			last_move_leg=in->move_id =trig_list_f[brain.rst_all_soft];
			else
			last_move_leg=in->move_id =trig_list_b[brain.rst_all_soft];
		}
 }
 
  if(brain.tar_att_force[0]!=0)
	 brain.tar_att[0]=brain.tar_att_force[0];
	if(brain.tar_att_force[1]!=0)
	 brain.tar_att[1]=brain.tar_att_force[1];
}

u8 pass_id[5]={0,3,4,1,2};
u8 pass_id1[5]={0,1,2,3,4};
void cal_target_of_center_move(BRAIN_STRUCT *in)
{
u8 i=0,j=0,j1=0;
static u8 leg_move_id;	
double x[5],y[5];
double x1[5],y1[5];
float temp[2][3];

  for(i=1;i<5;i++){
		if(i!=pass_id1[in->move_id]){
		x[j]=leg[i].sys.init_end_pos.x+in->sys.leg_local[i].x;y[j]=leg[i].sys.init_end_pos.y+in->sys.leg_local[i].y;
		j=j+1;
		}
		if(i!=pass_id1[in->move_id]){
    x1[j1]=leg[i].pos_now_brain[2].x;y1[j1]=leg[i].pos_now_brain[2].y;
		j1=j1+1;
		}
  }
	
   if(in->move_id!=leg_move_id&&in->ground_leg_num>3){
	 in->leg_move_state=S_BODY_MOVE; 
   in->leg_ground_center_trig_init[Xr]=-(float)(x[0]+x[1]+x[2])/3.;
   in->leg_ground_center_trig_init[Yr]=-(float)(y[0]+y[1]+y[2])/3.;
	 in->leg_ground_center_trig[Xr]=-(float)(x1[0]+x1[1]+x1[2])/3.;
   in->leg_ground_center_trig[Yr]=-(float)(y1[0]+y1[1]+y1[2])/3.;	 
   }
   leg_move_id=in->move_id;	
}



#define  USE_INIT_AS_LINE 1  
//ÖØÐÄ¿ØÖÆ
void center_control(float dt)//ÖÐÐÄ¿ØÖÆPID
{
  
	float ero[2];
	float yaw;
	yaw=90-brain.spd_yaw+0.000001;
	float center_tar_x,center_tar_y;
	#if USE_INIT_AS_LINE
	if(brain.ground_leg_num<4||brain.leg_move_state==S_BODY_MOVE){
	center_tar_y=brain.leg_ground_center_trig_init[Yr];
	center_tar_x=brain.leg_ground_center_trig_init[Xr];	
	}else
	center_tar_y=center_tar_x=0;
	#else
	center_tar_y=brain.leg_ground_center_trig[Yr];
	center_tar_x=brain.leg_ground_center_trig[Xr];
	#endif
	
	double b = (center_tar_y*brain.sys.kp_center[1])-tan(yaw/ 57.3f) * (center_tar_x*brain.sys.kp_center[0]);//Ä¿±ê
	double b2 = brain.leg_ground_center[Yr]+ brain.leg_ground_center[Xr]/tan(yaw/ 57.3f) ;//µ±Ç°
	//¹ýÄ¿±êÆ½ÐÐÏßÉÏ  µ±Ç°µã´¹Ö±Ïß½»µã
	float tar_x_9 = (b2 - b) / (tan(yaw/ 57.3f) + 1 / tan(yaw/ 57.3f));
	float tar_y_9 = tan(yaw/ 57.3f) * tar_x_9 + b;
	
	float spdx=sin(brain.spd_yaw/ 57.3f)*brain.spd+0.0001;
	float spdy=cos(brain.spd_yaw/ 57.3f)*brain.spd+0.0001;
	
	static u8 reg_flag;
	int flag[2];
	if(brain.move_id==1||brain.move_id==3)
	 flag[1]=1;
	else
	 flag[1]=-1;	
	
	if(brain.move_id==3||brain.move_id==4)
	 flag[0]=-1;
	else
	 flag[0]=1;	
	
	brain.tar_center[Xr]=tar_x_9;//+(spdx/fabs(spdx))*brain.sys.k_center_fp*flag[0]*0;
	brain.tar_center[Yr]=tar_y_9;//+(spdy/fabs(spdy))*brain.sys.k_center_fp*flag[1]*0;

	if(brain.leg_move_state==S_BODY_MOVE||brain.leg_move_state==S_BODY_RETURN
		#if BODY_MOVE_WHEN_LEG_UP
		||brain.leg_move_state==S_LEG_TRIG_LEAVE_GROUND_CHECK||brain.leg_move_state==S_LEG_TRIG_ING
	  #endif
	){ 
	ero[Xr]=-my_deathzoom((brain.tar_center[Xr]-brain.leg_ground_center[Xr])-brain.center.x*1,0.005);
  ero[Yr]=-my_deathzoom((brain.tar_center[Yr]-brain.leg_ground_center[Yr])-brain.center.y*1,0.005);
	}else
	ero[Xr]=ero[Yr]=0;
	
	center_control_out[Xr]=ero[Xr]*brain.sys.k_center_c[Xr];
	center_control_out[Yr]=ero[Yr]*brain.sys.k_center_c[Yr];
  
	float dis_ero=sqrt(pow(ero[Xr],2)+pow(ero[Yr],2));
	
   if(dis_ero<0.01&&(brain.leg_move_state==S_BODY_MOVE||brain.leg_move_state==S_BODY_RETURN
		#if BODY_MOVE_WHEN_LEG_UP
		||brain.leg_move_state==S_LEG_TRIG_LEAVE_GROUND_CHECK||brain.leg_move_state==S_LEG_TRIG_ING 
	  #endif
	 ))
	 brain.center_stable=1; 
	 else
	 brain.center_stable=0;
	
	 reg_flag=brain.leg_move_state;
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

//ÓÉ»úÌåËÙ¶È»»ËãµÅÍÈËÙ¶È
void cal_deng_from_spd(BRAIN_STRUCT *in)
{
u8 i;	
float spdx,spdy,spdz;	
float spd=in->spd;
float yaw=in->spd_yaw;
	

	float k_move=1;
  in->sys.tar_spd[Xr]=spdx=sin(yaw/ 57.3f)*spd*k_move;
	in->sys.tar_spd[Yr]=spdy=cos(yaw/ 57.3f)*spd*k_move;
	if((in->force_stop&&1)||brain.rst_all_soft>0)
  spdx=spdy=0;
	
		for(i=1;i<5;i++){
		if((leg[i].control_mode||in->control_mode)&&leg[i].leg_ground&&!leg[i].sys.leg_ground_force){	
		leg[i].deng[Xr]=LIMIT(spdx+1*center_control_out[Xr],-8.8,8.8);
		leg[i].deng[Yr]=LIMIT(spdy+1*center_control_out[Yr],-8.8,8.8);	}
		}	
}


#define EN_KUAI_ADD_MOVE_OFF 1
//ÐèÒª¿çÍÈµÄÄ¿±êµ÷ÓÃ¸Ãº¯Êý
float h_k=1;
float k_band=0.8;
void leg_tar_est(BRAIN_STRUCT *in,LEG_STRUCT *leg,float spd_body[3],float spd_tar[3],float w_tar,u8 need_move,float dt)
{
u8 id=leg->sys.id;
float band_x,band_y,range_limit;
float tar_yaw,spd;
		tar_yaw=in->spd_yaw;
	  spd=in->spd;
float tar_x,tar_y,tar_z;	
float off_x,off_y;
	
#if EN_KUAI_ADD_MOVE_OFF
if(leg->sys.id==1||leg->sys.id==2)
off_x=LIMIT(leg->pos_now[2].x-leg->sys.init_end_pos.x,0,leg->sys.off_local[0])*fabs(cos(tar_yaw/57.3));
else
off_x=LIMIT(leg->pos_now[2].x-leg->sys.init_end_pos.x,-leg->sys.off_local[0],0)*fabs(cos(tar_yaw/57.3));	
if(leg->sys.id==1||leg->sys.id==3)
off_y=LIMIT(leg->pos_now[2].y-leg->sys.init_end_pos.y,0,leg->sys.off_local[1])*fabs(sin(tar_yaw/57.3));
else
off_y=LIMIT(leg->pos_now[2].y-leg->sys.init_end_pos.y,-leg->sys.off_local[1],0)*fabs(sin(tar_yaw/57.3));	
#endif	
	float k1=1;
  #if BODY_CONTROL_USE_GLOBAL
  k1=2;
  #endif 
  
	band_x=in->sys.leg_local[id].x*k_band ;
  band_y=in->sys.leg_local[id].y*k_band;
  range_limit=0;
switch(need_move){//³¬³ö·¶Î§
	case 1 :
		tar_x=LIMIT(sin(tar_yaw/ 57.3f)*LIMIT(spd*in->sys.k_spd_to_range,-20,20),-in->sys.leg_move_range[Xr]*in->sys.move_range_k,in->sys.leg_move_range[Xr]*in->sys.move_range_k);
	  tar_y=LIMIT(cos(tar_yaw/ 57.3f)*LIMIT(spd*in->sys.k_spd_to_range,-20,20),-in->sys.leg_move_range[Yr]*in->sys.move_range_k,in->sys.leg_move_range[Yr]*in->sys.move_range_k);
	  leg->pos_tar_trig[2].x=leg->sys.init_end_pos.x*k1+tar_x+(float)RNG_Get_RandomRange(-1000,1000)/1000000.+off_x*cos(tar_yaw/ 57.3f);
	  leg->pos_tar_trig[2].y=leg->sys.init_end_pos.y*k1+tar_y+(float)RNG_Get_RandomRange(-1000,1000)/1000000.+off_y*sin(tar_yaw/ 57.3f);
	  leg->pos_tar_trig[2].z=leg->sys.init_end_pos.z*h_k;
	
	

//	tar_x=sin(tar_yaw/ 57.3f)*LIMIT(spd*in->sys.k_spd_to_range,-in->sys.leg_move_range[Xr]*in->sys.move_range_k,in->sys.leg_move_range[Xr]*in->sys.move_range_k)+leg->sys.init_end_pos.x;//leg->pos_now[2].x;
//	tar_y=cos(tar_yaw/ 57.3f)*LIMIT(spd*in->sys.k_spd_to_range,-in->sys.leg_move_range[Yr]*in->sys.move_range_k,in->sys.leg_move_range[Yr]*in->sys.move_range_k)+leg->sys.init_end_pos.y;//leg->pos_now[2].y;
//	leg->pos_tar_trig[2].x=tar_x;
//	leg->pos_tar_trig[2].y=tar_y;
//	leg->pos_tar_trig[2].x=LIMIT(leg->pos_tar_trig[2].x,-in->sys.leg_move_range[Xr]+leg->sys.init_end_pos.x,in->sys.leg_move_range[Xr]+leg->sys.init_end_pos.x)+(float)RNG_Get_RandomRange(-1000,1000)/1000000.;
//	leg->pos_tar_trig[2].y=LIMIT(leg->pos_tar_trig[2].y,-in->sys.leg_move_range[Yr]+leg->sys.init_end_pos.y,in->sys.leg_move_range[Yr]+leg->sys.init_end_pos.y)+(float)RNG_Get_RandomRange(-1000,1000)/1000000.;
//	leg->pos_tar_trig[2].z=leg->sys.init_end_pos.z*h_k;
	
//	if(leg->pos_tar_trig[2].x<band_x&&leg->pos_tar_trig[2].y<band_y&&id==1)
//	{leg->pos_tar_trig[2].x=band_x;leg->pos_tar_trig[2].y=band_y;}
//	if(leg->pos_tar_trig[2].x<band_x&&leg->pos_tar_trig[2].y>band_y&&id==2)
//	{leg->pos_tar_trig[2].x=band_x;leg->pos_tar_trig[2].y=band_y;}
//	if(leg->pos_tar_trig[2].x>band_x&&leg->pos_tar_trig[2].y<band_y&&id==3)
//	{leg->pos_tar_trig[2].x=band_x;leg->pos_tar_trig[2].y=band_y;}
//	if(leg->pos_tar_trig[2].x>band_x&&leg->pos_tar_trig[2].y>band_y&&id==4)
//	{leg->pos_tar_trig[2].x=band_x;leg->pos_tar_trig[2].y=band_y;}
	break;
	case 2 ://ÖØÐÄ¶ªÊ§
		tar_x=LIMIT(sin(tar_yaw*0.0173)*LIMIT(spd*in->sys.k_spd_to_range,-20,20),-in->sys.leg_move_range[Xr]*0.98,in->sys.leg_move_range[Xr]*0.98);
	  tar_y=LIMIT(cos(tar_yaw*0.0173)*LIMIT(spd*in->sys.k_spd_to_range,-20,20),-in->sys.leg_move_range[Yr]*0.98,in->sys.leg_move_range[Yr]*0.98);
	  leg->pos_tar[2].x=tar_x+(float)RNG_Get_RandomRange(-1000,1000 )/100000.;
		leg->pos_tar[2].y=tar_y+(float)RNG_Get_RandomRange(-1000,1000)/100000.;	
	break;
}

		if(brain.rst_all_soft>0)
		{
		leg->pos_tar_trig[2].x=leg->sys.init_end_pos.x*2+(float)RNG_Get_RandomRange(-1000,1000)/1000000.;//+off_x*cos(tar_yaw/ 57.3f);
	  leg->pos_tar_trig[2].y=leg->sys.init_end_pos.y+(float)RNG_Get_RandomRange(-1000,1000)/1000000.;//+off_y*sin(tar_yaw/ 57.3f);
	  leg->pos_tar_trig[2].z=leg->sys.init_end_pos.z*h_k;
		}
}
//-------------------------------------------------Fail Reset------------------------------------------------------
float reset_deng=30;
float reset_sita[3]={10,125,15};
float reset_sita1[3]={45,85,-8};
float reset_sita2[3]={45,85,-8};
void fall_reset(float dt)
{ u8 i;
  static u8 state;
	static u16 cnt[3];
	int flag;
	switch(state)
	{
	  case 0:	 
			 if(brain.fall!=0)
			 {brain.sys.control_angle=1;
			  cnt[0]=cnt[1]=cnt[2]=0;			 
				leg[1].sita_force[0]=reset_sita[0];leg[1].sita_force[1]=reset_sita[1];leg[1].sita_force[2]=reset_sita[2];
				leg[2].sita_force[0]=reset_sita[0];leg[2].sita_force[1]=reset_sita[1];leg[2].sita_force[2]=-reset_sita[2];
				leg[3].sita_force[0]=reset_sita[0];leg[3].sita_force[1]=reset_sita[1];leg[3].sita_force[2]=-reset_sita[2];
				leg[4].sita_force[0]=reset_sita[0];leg[4].sita_force[1]=reset_sita[1];leg[4].sita_force[2]=reset_sita[2];
				state=1;	
			 }
		break;
	  case 1:
			 if(cnt[0]++>2/dt)
			 {state=2;cnt[0]=0;}
		break;
	  case 2:
			 if((brain.att[1]>10))
				 state=3;
			 else if((brain.att[1]<-10))
		     state=13;
			 else if(ABS(brain.att[1])<10)
				 state=23;
		break;
		case 3://left
			 leg[3].sita_force[2]=reset_deng;
		   leg[4].sita_force[2]=-reset_deng;
		     if(cnt[0]++>0.5/dt)
			 {state=33;cnt[0]=0;}  
		break;
	  case 13://right
			  leg[1].sita_force[2]=-reset_deng;
		    leg[2].sita_force[2]=reset_deng;		  
		     if(cnt[0]++>0.5/dt)
			 {state=33;cnt[0]=0;}
		break;
			 
		case 33:	 
	  	  leg[1].sita_force[0]=reset_sita[0];leg[1].sita_force[1]=reset_sita[1];leg[1].sita_force[2]=reset_sita[2];
				leg[2].sita_force[0]=reset_sita[0];leg[2].sita_force[1]=reset_sita[1];leg[2].sita_force[2]=-reset_sita[2];
				leg[3].sita_force[0]=reset_sita[0];leg[3].sita_force[1]=reset_sita[1];leg[3].sita_force[2]=-reset_sita[2];
				leg[4].sita_force[0]=reset_sita[0];leg[4].sita_force[1]=reset_sita[1];leg[4].sita_force[2]=reset_sita[2];	
        if(cnt[0]++>2/dt)
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
//			 	leg[1].sita_force[0]=reset_sita1[0];leg[1].sita_force[1]=reset_sita1[1];leg[1].sita_force[2]=reset_sita1[2];
//				leg[2].sita_force[0]=reset_sita1[0];leg[2].sita_force[1]=reset_sita1[1];leg[2].sita_force[2]=-reset_sita1[2];
//				leg[3].sita_force[0]=reset_sita1[0];leg[3].sita_force[1]=reset_sita1[1];leg[3].sita_force[2]=-reset_sita1[2];
//				leg[4].sita_force[0]=reset_sita1[0];leg[4].sita_force[1]=reset_sita1[1];leg[4].sita_force[2]=reset_sita1[2];
		   brain.sys.control_angle=0;
		    for(i=1;i<4;i++)
		   {
				if(i==1||i==2)
					flag=1;
				else 
					flag=1;
				leg[i].pos_tar[2].x=leg[i].sys.init_end_pos.x*flag+(float)RNG_Get_RandomRange(-1000,1000)/1000000.;//+off_x*cos(tar_yaw/ 57.3f);
				leg[i].pos_tar[2].y=leg[i].sys.init_end_pos.y+(float)RNG_Get_RandomRange(-1000,1000)/1000000.;//+off_y*sin(tar_yaw/ 57.3f);
				leg[i].pos_tar[2].z=leg[i].sys.init_end_pos.z*h_k;
			 }
			 if(cnt[0]++>2/dt)
			 {state=0;
		    
        brain.fall=0;				
			 }	
		break;
	}
}