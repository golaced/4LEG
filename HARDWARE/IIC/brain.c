#include "include.h" 

void get_leg_tar_trig(BRAIN_STRUCT *in,float spd_body[3],float spd_tar[3],float w_tar,float dt);


float cal_area_trig(float x1,float y1,float x2,float y2,float x3,float y3)
{
float S;	
S=(1/2)*(x1*y2+x2*y3+x3*y1-x1*y3-x2*y1-x3*y2);
return S;
}	

//Ò»¸öµãÔÚÈı½ÇĞÎÄÚ²¿
u8 inTrig(float x, float y,float x1,float y1,float x2,float y2,float x3,float y3) {
  POS a,b,c,p;
  p.x=x;p.y=y;
	a.x=x1;a.y=y1;
	b.x=x2;b.y=y2;
	c.x=x3;c.y=y3;
	
  float signOfTrig = (b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x);
  float signOfAB = (b.x - a.x)*(p.y - a.y) - (b.y - a.y)*(p.x - a.x);
  float signOfCA = (a.x - c.x)*(p.y - c.y) - (a.y - c.y)*(p.x - c.x);
  float signOfBC = (c.x - b.x)*(p.y - b.y) - (c.y - b.y)*(p.x - b.x);

  u8 d1 = (signOfAB * signOfTrig > 0);
  u8 d2 = (signOfCA * signOfTrig > 0);
  u8 d3 = (signOfBC * signOfTrig > 0);

  return d1 && d2 && d3;
}
//Ò»¸öµãÔÚËÄ±ßĞÎÄÚ²¿
//			  y
//	d----------b          /\
// 	     |                |
//			 O                L
//			 |                |
//	a----------c   x			\/
u8 segmentsIntr(POS b,POS c,POS d,POS a,float *x,float *y){  
  
/** 1 ½âÏßĞÔ·½³Ì×é, ÇóÏß¶Î½»µã. **/  
// Èç¹û·ÖÄ¸Îª0 ÔòÆ½ĞĞ»ò¹²Ïß, ²»Ïà½»  
    float denominator = (b.y - a.y)*(d.x - c.x) - (a.x - b.x)*(c.y - d.y);  
    if (denominator==0) {  
        return 0;  
    }  
   
// Ïß¶ÎËùÔÚÖ±ÏßµÄ½»µã×ø±ê (x , y)      
     *x = ( (b.x - a.x) * (d.x - c.x) * (c.y - a.y)   
                + (b.y - a.y) * (d.x - c.x) * a.x   
                - (d.y - c.y) * (b.x - a.x) * c.x ) / denominator ;  
     *y = -( (b.y - a.y) * (d.y - c.y) * (c.x - a.x)   
                + (b.x - a.x) * (d.y - c.y) * a.y   
                - (d.x - c.x) * (b.y - a.y) * c.y ) / denominator;  
  
/** 2 ÅĞ¶Ï½»µãÊÇ·ñÔÚÁ½ÌõÏß¶ÎÉÏ **/  
    if (  
        // ½»µãÔÚÏß¶Î1ÉÏ  
        (*x - a.x) * (*x - b.x) <= 0 && (*y - a.y) * (*y - b.y) <= 0  
        // ÇÒ½»µãÒ²ÔÚÏß¶Î2ÉÏ  
         && (*x - c.x) * (*x - d.x) <= 0 && (*y - c.y) * (*y - d.y) <= 0  
        ){  
  
        // ·µ»Ø½»µãp  
        return 1;
    }  
    //·ñÔò²»Ïà½»  
    return 0;  
  
}  
//Ò»¸öµãÔÚËÄ±ßĞÎÄÚ²¿
u8 inTrig2(float x, float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4) {
	u8 in_tri1=0,in_tri2=0,in_line_t12=0,in_tri3=0,in_tri4=0;
	in_tri1=inTrig(x,y, x1, y1, x2, y2, x3, y3);
  in_tri2=inTrig(x,y, x2, y2, x3, y3, x4, y4);
  in_tri3=inTrig(x,y, x1, y1, x3, y3, x4, y4);
  in_tri4=inTrig(x,y, x2, y2, x1, y1, x4, y4);
	POS b, c, d, a;
	b.x=x1;b.y=y1;
	c.x=x2;c.y=y2;
	d.x=x3;d.y=y3;
	a.x=x4;a.y=y4;
	float x_o,y_o;
	u8 cross;
	cross=segmentsIntr( b, c, d, a, &x_o, &y_o); 
	if(x_o==x&&y_o==y&&cross)
	in_line_t12=1;
  return in_tri1||in_tri2||in_tri3||in_tri4||in_line_t12;
}

static void swap(float *a, float *b)  
{  
    int     c;  
     c = *a;  
    *a = *b;  
    *b =  c;  
}  
//ÕÒµ½Àëxy×î½üµÄµã
void find_closet_point(u8*min_id,float x, float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4,u8 num) {
	u8 i,j;
	float dis[4],dis_id[4]={0,1,2,3};
	  dis[0]=sqrt(pow(x-x1,2)+pow(y-y1,2));
	  dis[1]=sqrt(pow(x-x2,2)+pow(y-y2,2));
	  dis[2]=sqrt(pow(x-x3,2)+pow(y-y3,2));
	  dis[3]=sqrt(pow(x-x4,2)+pow(y-y4,2));
    for (i = 0; i < num; i++)  
    {  
        //Ã¿Ò»´ÎÓÉµ×ÖÁÉÏµØÉÏÉı  
        for (j = num-1; j > i; j--)  
        {  
            if (dis[j] < dis[j-1])  
            {  
                swap(&dis[i], &dis[j]); 
								swap(&dis_id[i], &dis_id[j]);  
            }  
        }  
    }  
   if(num==3)
	 {
	  min_id[0]=dis_id[0];
		min_id[1]=dis_id[1]; 
	 }
	 else 
	 {
	  min_id[0]=dis_id[0];
		min_id[1]=dis_id[1]; 
	 } 
}


//------------------------------------------------------------brain---------------------------------------------------------------------
//       
//<-----W-------->
//			  y
//	3----------1          /\
// 	     |                |
//			 O                L
//			 |                |
//	4----------2   x			\/
 
void barin_init(BRAIN_STRUCT *in)
{
float W=5.7*2;//cm	
float L=18.5;
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

in->area_of_leg[1]=
cal_area_trig(in->sys.leg_local[1].x,in->sys.leg_local[1].y,
in->sys.leg_local[2].x,in->sys.leg_local[2].y,
in->sys.leg_local[3].x,in->sys.leg_local[3].y)/2+
cal_area_trig(in->sys.leg_local[4].x,in->sys.leg_local[4].y,
in->sys.leg_local[2].x,in->sys.leg_local[2].y,
in->sys.leg_local[3].x,in->sys.leg_local[3].y)/2;
	

in->leg_move_range[Yr]=leg[1].sys.limit.y*0.98;//cm		
in->leg_move_range[Xr]=in->leg_move_range[Yr]*0.78;//cm	

in->sys.k_spd_to_range=2.0;//0.88;
in->sys.kp_center=0.765;
in->sys.k_center_fp=0.0;

in->sys.move_range_k=0.66;//0.8;
in->sys.k_center_c[0]=66;
in->sys.k_center_c[1]=66;

in->sys.center_off.x=0;
in->sys.center_off.y=0;
in->sys.center_off1.x=0;
in->sys.center_off1.y=0;

in->center_off_when_move[Xr]=0;//-0.2;
in->center_off_when_move[Yr]=0;//-0.88;
in->sys.leg_t=0.5;
in->sys.leg_h=4.68;
in->sys.desire_time=0.3;//0.76;
in->leg_move_min_dt=0;//35 ;

leg[1].sys.id=1;
leg[2].sys.id=2;
leg[3].sys.id=3;
leg[4].sys.id=4;
}	

//×ª»»ÍÈ¾Ö²¿×ø±êÏµµ½È«¾Ö»úÌå×ø±êÏµ
void conver_legpos_to_barin(BRAIN_STRUCT *in,LEG_STRUCT * inl,u8 id)
{ u8 i;
	for(i=0;i<3;i++){
	inl->pos_now_brain[i].x=inl->pos_now[i].x+in->sys.leg_local[id].x;
  inl->pos_now_brain[i].y=inl->pos_now[i].y+in->sys.leg_local[id].y;
	inl->pos_now_brain[i].z=inl->pos_now[i].z+in->sys.leg_local[id].z;
	}
}


//Ñ¡ÔñĞèÒª¿çÍÈ 
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
			float random=RNG_Get_RandomRange(0,9);//»ñÈ¡[0,9]Çø¼äµÄËæ»úÊı
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
		
				estimate_center(&brain,brain.att,brain.now_spd,brain.now_acc,brain.sys.tar_spd,brain.tar_w);//¹À¼Æ»úÌåÖØĞÄ

				cal_center_of_leg_ground(&brain);//¹À¼Æ×ÅµØ¶à±ßĞÎ
		
				check_leg_need_move(&brain,brain.now_spd,brain.sys.tar_spd,brain.tar_w,dt);//ÅĞ¶ÏĞèÒª¿çµÄ½Å
			
				cal_target_of_center_move(&brain);
				
				center_control(dt);	
					
        cal_deng_from_spd(&brain);	
 				
				if(brain.leg_move_state==S_LEG_TRIG){			
					leg_tar_est(&brain,&leg[brain.move_id],0,0,0,1,dt);
					brain.can_move_leg=0;
					leg[brain.move_id].sys.leg_move_pass_cnt=0;	 
					brain.leg_move_state=S_LEG_TRIG_ING;
				}	
				if(brain.leg_move_state==S_LEG_TRIG_ING&&brain.ground_leg_num<4)
					brain.leg_move_state=S_LEG_TRIG_ING1;
				if(brain.leg_move_state==S_LEG_TRIG_ING1&&brain.ground_leg_num>3)
				  brain.leg_move_state=S_BODY_RETURN;
			  if(brain.center_stable&&brain.leg_move_state==S_BODY_RETURN)
					brain.force_stop=brain.leg_move_state=S_IDLE;	
				
				
				
				leg_trig_flag_reg=brain.leg_move_state;
	}
}



//¹À¼Æ»úÆ÷ÈËÖĞĞÄ
void estimate_center(BRAIN_STRUCT *in,float att[3],float spd_body[3],float acc_body[3],float spd_tar[3],float w_tar)//¹À¼Æ»úÌåÖØĞÄ
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
		
	in->center.x=0-leg[1].leg_ground*in->center_off_when_move[Xr]-leg[2].leg_ground*in->center_off_when_move[Xr]
	+leg[3].leg_ground*in->center_off_when_move[Xr]+leg[4].leg_ground*in->center_off_when_move[Xr];
	
	in->center.y=0-leg[1].leg_ground*in->center_off_when_move[Yr]-leg[3].leg_ground*in->center_off_when_move[Yr]
	+leg[2].leg_ground*in->center_off_when_move[Yr]+leg[4].leg_ground*in->center_off_when_move[Yr];
	in->center.x*=k_off_z;	
	in->center.y*=k_off_z;	
	in->center.z=0;
}


//¼ÆËã×ÅµØ½ÅÖØĞÄ
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


//ÅĞ¶Ï»úÆ÷ÈËĞèÒª¿çÍÈµÄID
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
		
 //³¬³öÒÆ¶¯·¶Î§ĞèÒª¿çÍÈ
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
	 float temp=sqrt(pow(in->leg_move_range[Xr],2)/(1+pow(in->leg_move_range[Xr]*tan(tyaw/57.3)/in->leg_move_range[Yr],2)));
	 //ÅĞ¶ÏËÙ¶È·½Ïò½»µã·ûºÅ
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
	 
	 range_in[i]=pow(tar_x/(in->leg_move_range[Xr]*brain.sys.move_range_k),2)+pow(tar_y/(in->leg_move_range[Yr]*brain.sys.move_range_k),2);
	 range_out[i]=pow(tar_x/in->leg_move_range[Xr],2)+pow(tar_y/in->leg_move_range[Yr],2);
	 
	 if(spd>0)
	  brain.tabu=1;
	 else
		brain.tabu=0;  
	 
	 if((dis[1]<dis[0]&&range_in[i]>1&&spd>0)||brain.tabu)
	 {
	 leg[i].need_move=1;	 
	 }
	 
	 if(range_out[i]>1)
	 { 
	 leg[i].need_move=3;	 
	 brain.force_stop=1; 	 
	 }
	 
	 brain.leg_move[i]=leg[i].need_move;
 } 
  
//  if(range_out[1]<1.06&&range_out[2]<1.06&&range_out[3]<1.06&&range_out[4]<1.06) 
//   brain.force_stop=0;
   get_leg_tar_trig(in,brain.now_spd,brain.sys.tar_spd,brain.tar_w, dt);
}



//ÅĞ¶Ï¿ç½Å
u8 trig_list[5]=		 {0,3,2,4,1};
u8 trig_list_ero[5]= {0,2,3,4,1};
u8 trig_list_ero2[5]={0,0,0,0,0};
u8 test112=15;
float move_off[2]={0};
u16 set11=30;
 u8 last_move_leg;	
void get_leg_tar_trig(BRAIN_STRUCT *in,float spd_body[3],float spd_tar[3],float w_tar,float dt)//mmmm
{
static float dt_leg_min_trig_cnt;
u8 i,cnt,id,id_star,leg_move_pass_cnt;
u8 leg_out_cnt[3]={0};//flag of state robot	
u8 out_range_id[4];
u8 loss_center_id[4];	
static u8 last_move_leg,leg_flag;
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
 if(brain.sys.dt_leg_min_trig_cnt++>brain.leg_move_min_dt*brain.sys.desire_time/dt)
	 brain.can_move_leg=1;
 
 //ÅĞ¶ÏË­È¨ÏŞ¸ß
 u8 id_need_to_move=RNG_Get_RandomRange(1,4);
// if(leg_out_cnt[2]==0)//normal no outter
// {
   id_need_to_move=trig_list[leg_flag];
// }
// else{
// for(i=1;i<5;i++)
//   if(leg[i].sys.leg_move_pass_cnt>leg[id_need_to_move].sys.leg_move_pass_cnt)
//		  id_need_to_move=i;
//	 
//	 id_need_to_move=trig_list[id_need_to_move];
// }	 
 
 if(brain.can_move_leg&&brain.leg_move_state==S_IDLE&&(leg_out_cnt[0]+leg_out_cnt[1]+leg_out_cnt[2]>0)){
 last_move_leg=in->move_id = id_need_to_move;
 leg_flag++;
  if(leg_flag>4)
   leg_flag=1;		
 }
}

u8 pass_id[5]={0,3,4,1,2};
void cal_target_of_center_move(BRAIN_STRUCT *in)
{
u8 i=0,j=0,j1=0;
static u8 leg_move_id;	
double x[5],y[5];
double x1[5],y1[5];
float temp[2][3];

  for(i=1;i<5;i++){
		if(i!=pass_id[in->move_id]){
		x[j]=leg[i].sys.init_end_pos.x+in->sys.leg_local[i].x;y[j]=leg[i].sys.init_end_pos.y+in->sys.leg_local[i].y;
		j=j+1;
		}
		if(i!=pass_id[in->move_id]){
    x1[j1]=leg[i].pos_now_brain[2].x;y1[j1]=leg[i].pos_now_brain[2].y;
		j1=j1+1;
		}
  }
	
   if(in->move_id!=leg_move_id&&in->ground_leg_num>3){
	 in->leg_move_state=S_BODY_MOVE; 
   in->leg_ground_center_trig_init[Xr]=(float)(x[0]+x[1]+x[2])/3.;
   in->leg_ground_center_trig_init[Yr]=(float)(y[0]+y[1]+y[2])/3.;
	 in->leg_ground_center_trig[Xr]=(float)(x1[0]+x1[1]+x1[2])/3.;
   in->leg_ground_center_trig[Yr]=(float)(y1[0]+y1[1]+y1[2])/3.;	 
   }
   leg_move_id=in->move_id;	
}

#define  USE_INIT_AS_LINE 1  
//ÖØĞÄ¿ØÖÆ
float center_control_out[2];
void center_control(float dt)//ÖĞĞÄ¿ØÖÆPID
{
  
	float ero[2];
	float yaw;
	yaw=90-brain.spd_yaw+0.001;
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
	
	double b = center_tar_y-tan(yaw/ 57.3f) * center_tar_x*brain.sys.kp_center;
	double b2 = brain.leg_ground_center[Yr]+ brain.leg_ground_center[Xr]/tan(yaw/ 57.3f) ;
	float tar_x_9 = (b2 - b) / (tan(yaw/ 57.3f) + 1 / tan(yaw/ 57.3f));
	float tar_y_9 = tan(yaw/ 57.3f) * tar_x_9 + b;
	float spdx=sin(brain.spd_yaw*0.0173)*brain.spd+0.0001;
	float spdy=cos(brain.spd_yaw*0.0173)*brain.spd+0.0001;
	
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
	
	brain.tar_center[Xr]=tar_x_9+(spdx/fabs(spdx))*brain.sys.k_center_fp*flag[0]*0;
	brain.tar_center[Yr]=tar_y_9+(spdy/fabs(spdy))*brain.sys.k_center_fp*flag[1]*0;

	if(brain.leg_move_state==S_BODY_MOVE||brain.leg_move_state==S_BODY_RETURN){ 
	ero[Xr]=-my_deathzoom(brain.tar_center[Xr]-brain.leg_ground_center[Xr],0.001);
  ero[Yr]=-my_deathzoom(brain.tar_center[Yr]-brain.leg_ground_center[Yr],0.001);
	}else
	ero[Xr]=ero[Yr]=0;
	
	center_control_out[Xr]=ero[Xr]*brain.sys.k_center_c[Xr];
	center_control_out[Yr]=ero[Yr]*brain.sys.k_center_c[Yr];

   if(sqrt(pow(ero[Xr],2)+pow(ero[Yr],2))<0.005&&(brain.leg_move_state==S_BODY_MOVE||brain.leg_move_state==S_BODY_RETURN))
	 brain.center_stable=1; 
	 else
	 brain.center_stable=0;
	
	 if(brain.center_stable&&brain.leg_move_state==S_BODY_MOVE)
		 brain.leg_move_state=S_LEG_TRIG;
	 
	 	reg_flag=brain.leg_move_state;
}



//ÓÉ»úÌåËÙ¶È»»ËãµÅÍÈËÙ¶È
void cal_deng_from_spd(BRAIN_STRUCT *in)
{
u8 i;	
float spdx,spdy,spdz;	
float spd=in->spd;
float yaw=in->spd_yaw;
	

	float k_move=1;
  in->sys.tar_spd[Xr]=spdx=sin(yaw*0.0173)*spd*k_move;
	in->sys.tar_spd[Yr]=spdy=cos(yaw*0.0173)*spd*k_move;
	if(in->force_stop)
  spdx=spdy=0;
	
		for(i=1;i<5;i++){
		if((leg[i].control_mode||in->control_mode)&&leg[i].leg_ground&&!leg[i].sys.leg_ground_force){	
		leg[i].deng[Xr]=LIMIT(spdx+1*center_control_out[Xr],-8.8,8.8);
		leg[i].deng[Yr]=LIMIT(spdy+1*center_control_out[Yr],-8.8,8.8);	}
		}
		
}

#define EN_KUAI_ADD_MOVE_OFF 1
//ĞèÒª¿çÍÈµÄÄ¿±êµ÷ÓÃ¸Ãº¯Êı
float h_k=0.98;
void leg_tar_est(BRAIN_STRUCT *in,LEG_STRUCT *leg,float spd_body[3],float spd_tar[3],float w_tar,u8 need_move,float dt)
{
u8 id=leg->sys.id;
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
	
switch(need_move){//³¬³ö·¶Î§
	case 1 :
		tar_x=LIMIT(sin(tar_yaw*0.0173)*LIMIT(spd*in->sys.k_spd_to_range,-20,20),-in->leg_move_range[Xr]*in->sys.move_range_k,in->leg_move_range[Xr]*in->sys.move_range_k);
	  tar_y=LIMIT(cos(tar_yaw*0.0173)*LIMIT(spd*in->sys.k_spd_to_range,-20,20),-in->leg_move_range[Yr]*in->sys.move_range_k,in->leg_move_range[Yr]*in->sys.move_range_k);
	  leg->pos_tar_trig[2].x=leg->sys.init_end_pos.x*1+tar_x+(float)RNG_Get_RandomRange(-1000,1000)/100000.+off_x;
	  leg->pos_tar_trig[2].y=leg->sys.init_end_pos.y*1+tar_y+(float)RNG_Get_RandomRange(-1000,1000)/100000.+off_y;
	  leg->pos_tar_trig[2].z=leg->sys.init_end_pos.z*h_k;
	break;
	case 2 ://ÖØĞÄ¶ªÊ§
		tar_x=LIMIT(sin(tar_yaw*0.0173)*LIMIT(spd*in->sys.k_spd_to_range,-20,20),-in->leg_move_range[Xr]*0.98,in->leg_move_range[Xr]*0.98);
	  tar_y=LIMIT(cos(tar_yaw*0.0173)*LIMIT(spd*in->sys.k_spd_to_range,-20,20),-in->leg_move_range[Yr]*0.98,in->leg_move_range[Yr]*0.98);
	  leg->pos_tar[2].x=tar_x+(float)RNG_Get_RandomRange(-1000,1000)/100000.;
		leg->pos_tar[2].y=tar_y+(float)RNG_Get_RandomRange(-1000,1000)/100000.;	
	break;
}
}
