#include "include.h" 
LEG_STRUCT leg[5];
BRAIN_STRUCT brain;
//% leg ending positon calculate'
//%         _                side view            ----  ------>x  back view
//%         |  l1                               3  ||   |
//%      __ O                o---> y               OO   |
//%     1  //                |                     ||   \/ Z 
//%       //     l2          |                     ||
//%      O                 Z \/                    OO
//%    /  \\                                       ||
//%   / 2  \\    l3                                ||
//%          O                                     OO
//PWM gain of Pan
#define DJ_MG995 9.34
#define DJ_MG956 9.34
#define DJ_MG955 9.34
#define DJ_MG945 5.7
#define DJ_MG355 11.34
#define DJ_6221MG 12.64
#define DJ_DSERVO 11.34

float off_local[2]={2.68,0.680};
float k_z=0.968;
#if TIRG_CURVE_USE_BAI
float flt_leg=0;
#else
float flt_leg=0.8;	
#endif
u16 SET_PWM3_OFF=0;

void READ_LEG_ID(LEG_STRUCT *in)
{
in->sys.id=0;
}

void leg_init( LEG_STRUCT *in,u8 id)
{
static u8 init;
if(!init)
	init=1;
	
in->sys.id=id;
in->leg_ground=1;
in->sys.init_mode=0;	
in->sys.l1=3.9;
in->sys.l2=6.4;
in->sys.l3=6.8;	
in->sys.off_local[0]=off_local[0];
in->sys.off_local[1]=off_local[1];
switch(in->sys.id){
case 1:	
in->sys.leg_set_invert=1;
in->sys.PWM_OFF[0]=1730;	
in->sys.PWM_OFF[1]=870;	
in->sys.PWM_OFF[2]=1500+SET_PWM3_OFF;		
in->sys.PWM_OFF[3]=1380;
in->sys.sita_flag[0]=-1;
in->sys.sita_flag[1]=1;	
in->sys.sita_flag[2]=1;
in->sys.sita_flag[3]=1;
in->sys.pwm_id[0]=9;
in->sys.pwm_id[1]=8;
in->sys.pwm_id[2]=10;
in->sys.pwm_id[3]=11;
break;
case 2:	
in->sys.leg_set_invert=0;
in->sys.PWM_OFF[0]=586;	
in->sys.PWM_OFF[1]=1980;	
in->sys.PWM_OFF[2]=1510+SET_PWM3_OFF;	
in->sys.PWM_OFF[3]=1380;
in->sys.sita_flag[0]=1;
in->sys.sita_flag[1]=-1;	
in->sys.sita_flag[2]=1;	
in->sys.sita_flag[3]=-1;
in->sys.pwm_id[0]=15;
in->sys.pwm_id[1]=14;
in->sys.pwm_id[2]=13;
in->sys.pwm_id[3]=12;
break;
case 3:	
in->sys.leg_set_invert=1;	
in->sys.PWM_OFF[0]=1060;	
in->sys.PWM_OFF[1]=1980;	
in->sys.PWM_OFF[2]=1510-SET_PWM3_OFF;	
in->sys.PWM_OFF[3]=1416;
in->sys.sita_flag[0]=1;
in->sys.sita_flag[1]=-1;	
in->sys.sita_flag[2]=1;
in->sys.sita_flag[3]=-1;
in->sys.pwm_id[0]=6;
in->sys.pwm_id[1]=7;
in->sys.pwm_id[2]=5;
in->sys.pwm_id[3]=4;
break;
case 4:	
in->sys.leg_set_invert=0;	
in->sys.PWM_OFF[0]=1835;	
in->sys.PWM_OFF[1]=720;	
in->sys.PWM_OFF[2]=1390-SET_PWM3_OFF;	
in->sys.PWM_OFF[3]=1360;
in->sys.sita_flag[0]=-1;
in->sys.sita_flag[1]=1;	
in->sys.sita_flag[2]=1;
in->sys.sita_flag[3]=1;
in->sys.pwm_id[0]=1;
in->sys.pwm_id[1]=0;
in->sys.pwm_id[2]=2;
in->sys.pwm_id[3]=3;
break;
}
switch(in->sys.id){
case 1:
in->sys.PWM_PER_DEGREE[0]=DJ_DSERVO;
in->sys.PWM_PER_DEGREE[1]=DJ_6221MG;
in->sys.PWM_PER_DEGREE[2]=DJ_MG355;
in->sys.PWM_PER_DEGREE[3]=DJ_MG955;
break;
case 2:
in->sys.PWM_PER_DEGREE[0]=DJ_DSERVO;	
in->sys.PWM_PER_DEGREE[1]=DJ_DSERVO;
in->sys.PWM_PER_DEGREE[2]=DJ_MG355;
in->sys.PWM_PER_DEGREE[3]=DJ_MG955;
break;
case 3:
in->sys.PWM_PER_DEGREE[0]=DJ_DSERVO;//da
in->sys.PWM_PER_DEGREE[1]=DJ_DSERVO;//xiao
in->sys.PWM_PER_DEGREE[2]=DJ_MG355;//zhuan
in->sys.PWM_PER_DEGREE[3]=DJ_MG955;
break;
case 4:
in->sys.PWM_PER_DEGREE[0]=DJ_DSERVO;	
in->sys.PWM_PER_DEGREE[1]=DJ_6221MG;
in->sys.PWM_PER_DEGREE[2]=DJ_MG355;
in->sys.PWM_PER_DEGREE[3]=DJ_MG955;
break;
default:
in->sys.PWM_PER_DEGREE[0]=9.34;
in->sys.PWM_PER_DEGREE[1]=9.34;
in->sys.PWM_PER_DEGREE[2]=9.34;	
in->sys.PWM_PER_DEGREE[3]=9.34;
break;
}

int flag[2]={1,1};
if(in->sys.id==3||in->sys.id==4)
	flag[0]=-1;
if(in->sys.id==2||in->sys.id==4)
	flag[1]=-1;
in->sys.init_end_pos.x=in->pos_tar[2].x=in->sys.pos_tar_trig_test[2].x=in->sys.off_local[0]*flag[0];	
in->sys.init_end_pos.y=in->pos_tar[2].y=in->sys.pos_tar_trig_test[2].y=in->sys.off_local[1]*flag[1];		
in->sys.init_end_pos.z=in->pos_tar[2].z=in->sys.pos_tar_trig_test[2].z=(in->sys.l1+in->sys.l2+in->sys.l3)*0.88*k_z;
	
in->pos_tar_trig[2].x=in->sys.init_end_pos.x;
in->pos_tar_trig[2].y=in->sys.init_end_pos.y;
in->pos_tar_trig[2].z=in->sys.init_end_pos.z;

in->sys.limit.x=(in->sys.l1+in->sys.l2+in->sys.l3)*sin(25*ANGLE_TO_RADIAN);	
in->sys.limit.y=(in->sys.l1+in->sys.l2+in->sys.l3)*sin(25*ANGLE_TO_RADIAN);//0.25;	
in->sys.limit.z=(in->sys.l1+in->sys.l2*cos(30*ANGLE_TO_RADIAN)+in->sys.l3*cos(30*ANGLE_TO_RADIAN));	
	
in->sys.limit_min.z=(in->sys.l3-(in->sys.l2-in->sys.l1))*1.15;

in->sys.desire_time=0.4;

int DL=88;
in->sys.PWM_MIN[0]=500+DL;	
in->sys.PWM_MIN[1]=500+DL;
in->sys.PWM_MIN[2]=500+DL;
in->sys.PWM_MIN[3]=500+DL;
in->sys.PWM_MAX[0]=2500-DL;
in->sys.PWM_MAX[1]=2500-DL;	
in->sys.PWM_MAX[2]=2500-DL;	
in->sys.PWM_MAX[3]=2500-DL;	
in->sys.en_pwm_out=1;

in->sys.leg_up_high=3;

in->leg_ground=1;

}	
//位置超出可移动范围检测
u8 pos_range_check(LEG_STRUCT * in,float x,float y,float z)
{
 float r=(in->sys.l1+in->sys.l2+in->sys.l3)*1;
 float r_pos=sqrt(x*x+y*y+z*z);
 if(r_pos>r||r_pos<(in->sys.l3-(in->sys.l2-in->sys.l1))*1)
	 return 0;
 else
	 return 1;
}


//
u8 cal_sita_form_pos_tri_leg(float r1,float r2,float x,float y,float z,float *sita1,float *sita2,float *sita3)
{
	u8 i;
	float sita_out[3];

	sita_out[2]=atan2(x,sqrt(y*y+z*z))*57.3;	
	
	
	float y_t=y,z_t=-z/cos(sita_out[2]*ANGLE_TO_RADIAN);
	
	float r=sqrt(z_t*z_t+y_t*y_t);//%??? 
	if (r1+r2<=r || ABS(r1-r2)>=r) 	
		return 0;
	float seta;
	seta=acos((r1*r1+r*r-r2*r2)/2/r/r1); // %??????????????? 
	float r_seta;
	r_seta=atan2(z_t,y_t);		//	%???????(?x???) 
	float alpha[2];
  alpha[0]=r_seta-seta;
  alpha[1]=r_seta+seta;  	//%?????x???? 
	float crossy[2],crossz[2];
	for (i=0;i<2;i++){
	crossy[i]=r1*cos(alpha[i]); 
	crossz[i]=r1*sin(alpha[i]);
  }
	
	float s1,s2;
	if (crossy[0]<0)
	s1=-atan(crossz[0]/crossy[0])*57.3+90+180;   
	else    
	s1=-atan(crossz[0]/crossy[0])*57.3+90;
	
	if (crossy[1]<0)
	s2=-atan(crossz[1]/crossy[1])*57.3+90+180;  
	else   
	s2=-atan(crossz[1]/crossy[1])*57.3+90;
	
	if(s1>s2)
	{sita_out[1]=s1;sita_out[0]=s2;}
	else
  {sita_out[1]=s2;sita_out[0]=s1;}
	*sita1=sita_out[0];
	*sita2=sita_out[1];
	*sita3=sita_out[2];
	return 1;
}

void cal_PWM_for_tri_leg(float sita1,float sita2,float sita3)
{
  static u8 init;
	float s1,s2;
	u8 i;
	if(!init)
	{
	init=1;
	aux.init[0]=1310;
	aux.init[1]=1210;
	aux.min[0]=1100;
	aux.min[1]=750;
  aux.max[0]=1750;
	aux.max[1]=1900;

	aux.flag[0]=1;	
  aux.flag[1]=1;		
	aux.pwm_per_dig[0]=9.4;
	aux.pwm_per_dig[1]=9.4;
	}	
	if(aux.att[0]!=0||aux.att[1]!=0){
	s1=aux.att[0];
	s2=aux.att[1];	
	}
	else{
	s1=(sita1-90);
	s2=90-(sita2-180);
	}
	aux.pwm_tem[0]=aux.init[0]+aux.pwm_per_dig[0]*s1*aux.flag[0];
	aux.pwm_tem[1]=aux.init[1]+aux.pwm_per_dig[1]*s2*aux.flag[1];
	for(i=0;i<2;i++)
	{
			aux.pwm_tem[i] = LIMIT(aux.pwm_tem[i],aux.min[i],aux.max[i]);
	}
	
	TIM8->CCR2 = (aux.pwm_tem[0] )/2 ;				//1	
	TIM8->CCR1 = (aux.pwm_tem[1] )/2 ;				//2
}


//从位置结算关节角度 
void cal_sita_from_pos(LEG_STRUCT * in,float x_i,float y_i,float z_i,u8 out)
{ 
u8 id=in->sys.id;	
u8 ero;	
	if(id==1&&in->curve_trig)
	id=1;
	
static float sita_reg[5][3],sita5_reg[5];
float x=LIMIT(x_i,-in->sys.limit.x,in->sys.limit.x);	
float y=LIMIT(y_i,-in->sys.limit.y,in->sys.limit.y);
float z=LIMIT(z_i,-in->sys.limit.z,in->sys.limit.z);	
float x1,y1,z1;	
if(in->curve_trig&&flt_leg>0){
arrow_check_to_bow(x_i,y_i,z_i,in->sys.limit.x,in->sys.limit.y,in->sys.limit.z,&x1,&y1,&z1);	

x=(x_i*(1-flt_leg)+x1*flt_leg);
y=(y_i*(1-flt_leg)+y1*flt_leg);
z=(z_i*(1-flt_leg)+z1*flt_leg);	
}	


if(id==1&&in->curve_trig)
	id=1;

	if(pos_range_check(in,x,y,z))	
	{
	in->err=0;
	}		
	else
	in->err=1;

		
	in->sita[2]=fast_atan2(x,z)*RtA;
	float l1=cos(in->sita[2]*AtR)*in->sys.l1;
	float l2=cos(in->sita[2]*AtR)*in->sys.l2;
	float l3=cos(in->sita[2]*AtR)*in->sys.l3;
	float temp=z-l1;
	float l4=sqrt(y*y+temp*temp);
	temp=(l2*l2+l3*l3-l4*l4)/(2*l2*l3);
	float sita5=acos(LIMIT(temp,-1,1))*RtA;
	if(isnan(sita5))
		ero=1;
	in->sita[1]=LIMIT(180-sita5,0.5,179.5);  
	temp=(l2*l2+l4*l4-l3*l3)/(2*l2*l4);
	in->sita[0]=180-acos(y/l4)*RtA-acos(LIMIT(temp,-1,1))*RtA;//*cos(in->sita[2]*AtR);
  if(id==1)
	id=1;
	//protect	
	if(isnan(in->sita[0]))
	in->sita[0]=sita_reg[id][0];
	if(isnan(in->sita[1]))
	in->sita[1]=sita_reg[id][1];
	if(isnan(in->sita[2]))
	in->sita[2]=sita_reg[id][2];

	if(!isnan(in->sita[0]))
	sita_reg[id][0]=in->sita[0];
	if(!isnan(in->sita[1]))
	sita_reg[id][1]=in->sita[1];
	if(!isnan(in->sita[2]))
	sita_reg[id][2]=in->sita[2];
	//recal_pos 
	if(out){//作为输出才重新计算当前角度对应的逆运算末端位置
	l1=cos(in->sita[2]*AtR)*in->sys.l1;
	l2=cos(in->sita[2]*AtR)*in->sys.l2;
	l3=cos(in->sita[2]*AtR)*in->sys.l3;
	in->pos_now[0].x=l1*sin(in->sita[2]*AtR);in->pos_now[0].y=0;in->pos_now[0].z=cos(in->sita[2]*AtR)*l1;
	float h1=sin(in->sita[0]*AtR)*l2;
	float h2=sin((180-in->sita[0]-in->sita[1])*AtR)*l3;
	float d1=cos(in->sita[2]*AtR)*l2;
	float d2=cos(in->sita[2]*AtR)*l3;
	in->pos_now[1].x=(l1+h1)*sin(in->sita[2]*AtR);in->pos_now[1].y=-cos(in->sita[0]*AtR)*d1;in->pos_now[1].z=cos(in->sita[2]*AtR)*(l1+h1);
	in->pos_now[2].x=(l1+h1+h2)*sin(in->sita[2]*AtR);in->pos_now[2].y=-cos(in->sita[0]*AtR)*d1+cos((180-in->sita[0]-in->sita[1])*AtR)*d2;in->pos_now[2].z=cos(in->sita[2]*AtR)*(l1+h1+h2);	
  //in->pos_now[2].x=x;in->pos_now[2].y=y;in->pos_now[2].z=z;
	if(in->sys.leg_set_invert)
		for(u8 i=0;i<3;i++)
	   {
		 in->pos_now[i].x*=-1;
		 in->pos_now[i].y*=-1;
		 }
  if(isnan(in->sita[1])||isnan(in->sita[0]))
		ero=1;
  }
}	

//从关节角度计算位置
void cal_pos_from_sita(LEG_STRUCT * in,float sita1,float sita2,float sita3)
{                                                                                                                                                                                                            

float l1=cos(sita3*AtR)*in->sys.l1;
float l2=cos(sita3*AtR)*in->sys.l2;
float l3=cos(sita3*AtR)*in->sys.l3;
//recal_pos 
in->pos_now[0].x=l1*sin(sita3*AtR);in->pos_now[0].y=0;in->pos_now[0].z=cos(sita3*AtR)*l1;
float h1=sin(sita1*AtR)*l2;
float h2=sin((180-sita1-sita2)*AtR)*l3;
float d1=cos(sita3*AtR)*l2;
float d2=cos(sita3*AtR)*l3;
in->pos_now[1].x=(l1+h1)*sin(sita3*AtR);in->pos_now[1].y=-cos(sita1*AtR)*d1;in->pos_now[1].z=cos(sita3*AtR)*(l1+h1);
in->pos_now[2].x=(l1+h1+h2)*sin(sita3*AtR);in->pos_now[2].y=-cos(sita1*AtR)*d1+cos((180-sita1-sita2)*AtR)*d2;in->pos_now[2].z=cos(sita3*AtR)*(l1+h1+h2);	

}	

//从关节角度计算舵机PWM
void cal_pwm_from_sita(LEG_STRUCT * in)
{ u8 i=0;
	in->sys.PWM_OUT[i]=LIMIT(in->sys.PWM_OFF[i]+0*in->sys.sita_flag[i]*in->sys.PWM_PER_DEGREE[i]
	+in->sys.sita_flag[i]*in->sita[i]*in->sys.PWM_PER_DEGREE[i],in->sys.PWM_MIN[i],in->sys.PWM_MAX[i]);
	for(i=1;i<4;i++)
	in->sys.PWM_OUT[i]=LIMIT(in->sys.PWM_OFF[i]+in->sys.sita_flag[i]*in->sita[i]*in->sys.PWM_PER_DEGREE[i],in->sys.PWM_MIN[i],in->sys.PWM_MAX[i]);
}	

//计算采样点曲线三维坐标
float curve_cal(float c0,float c3,float c4,float c5,float c6,float t)
{
float temp;
temp=c0+c3*pow(t,3)+c4*pow(t,4)+c5*pow(t,5)+c6*pow(t,6);
return temp;
}

//计算二次曲线系数
float c0[5][3]; //x y z
float c3[5][3];
float c4[5][3];
float c5[5][3];
float c6[5][3];
void cal_curve_from_pos(LEG_STRUCT * in,float desire_time)
{
u8 id=in->sys.id;	
float pos_now[3];
float pos_tar[3];
pos_now[Xs]=in->pos_now[2].x;
pos_now[Ys]=in->pos_now[2].y;
pos_now[Zs]=in->pos_now[2].z;
pos_tar[Xs]=in->pos_tar_trig[2].x;
pos_tar[Ys]=in->pos_tar_trig[2].y;
pos_tar[Zs]=in->pos_tar_trig[2].z;
	
		float t1=0;t1=desire_time/2; //middle 0.5s
    float t2=0;t2=desire_time; //end 1s
    float p0[3];
    float p1[3];
    float p2[3];
//----------start
    p0[0]=pos_now[Xs];
   	p0[1]=pos_now[Ys];
   	p0[2]=pos_now[Zs];
//--------------end
	  p2[0]=pos_tar[Xs];//x
   	p2[1]=pos_tar[Ys];//y
   	p2[2]=pos_tar[Zs];//z
//-------------middle
   	p1[0]=(p0[0]+p2[0])/2;
   	p1[1]=(p0[1]+p2[1])/2;
   	p1[2]=LIMIT((p0[2]+p2[2])/2-in->sys.leg_up_high,in->sys.limit_min.z,30);//wait
		
  float p1_p0[3];
	float p0_p2[3];
	int i;
	for(i=0;i<3;i++)
	{
		c0[id][i]=p0[i];
		p1_p0[i]=p1[i]-p0[i];
		p0_p2[i]=p0[i]-p2[i];
	}

	float t1_3=pow(t1,3);
	float t1_4=pow(t1,4);
	float t1_5=pow(t1,5);
	float t1_6=pow(t1,6);

	float t2_2=pow(t2,2);
	float t2_3=pow(t2,3);
	float t2_5=pow(t2,5);
	float t2_6=pow(t2,6);

	float temp1=0;temp1=1/(t1_3*pow((t1-t2),3)*t2_3);
	for(i=0;i<3;i++)
	{	c3[id][i]=-1*temp1*(t2_6*(p1_p0[i])+5*t1_4*t2_2*3*(p0_p2[i])
		+2*t1_6*5*(p0_p2[i])-3*t1_5*t2*8*(p0_p2[i]));

    c4[id][i]=temp1/t2*(3*t2_6*(p1_p0[i])+15*t1_3*t2_3*(p0_p2[i])
    		-27*t1_5*t2*(p0_p2[i])+t1_6*15*(p0_p2[i]));

	c5[id][i]=-temp1/t2_2*3*(
			t2_6*(p1_p0[i])
			+2*t1_6*(p0_p2[i])
			+t1_3*t2_3*8*(p0_p2[i])
			-t1_4*t2_2*9*(p0_p2[i]));

	c6[id][i]=temp1/t2_2*
	(t2_5*(p1_p0[i])
			+6*t1_5*(p0_p2[i])
			+10*t1_3*t2_2*(p0_p2[i])
			-t1_4*t2*15*(p0_p2[i]));}
}

//读出规划曲线各采样三维位置
void  cal_pos_tar_from_curve(LEG_STRUCT * in,float time_now,float dt)
{
u8 id=in->sys.id;	
float cal_curve[3];	
u8 i;
for(i=0;i<3;i++)
cal_curve[i]=curve_cal(c0[id][i],c3[id][i],c4[id][i],c5[id][i],c6[id][i],time_now);
	
in->pos_tar[2].x=cal_curve[Xs];
in->pos_tar[2].y=cal_curve[Ys];
in->pos_tar[2].z=cal_curve[Zs];	
}	

void leg_curve_bai(LEG_STRUCT * in,float sx,float sy,float sz,float tx,float ty,float tz,float h,float time_now,float dt,float T)
{
float sita=(2*3.1415926/(T/dt))*(time_now/dt);
in->pos_tar[2].x=(tx-sx)*(sita-sin(sita))/(2*3.1415926)+sx;
in->pos_tar[2].y=(ty-sy)*(sita-sin(sita))/(2*3.1415926)+sy;
float off_h=0;
//if(tz>sz)
off_h=(tz-sz)	;
//off_h=0;
in->pos_tar[2].z=-(h+off_h)*(1-cos(sita))/2+sz;
}	


float rate_delay_kuai=0.066;
float delay_time_kuai=0.66;
//跨腿
void leg_follow_curve(LEG_STRUCT * in,float desire_time,u8 *en,float dt,float down_h,float down_t)
{
u8 id=in->sys.id;
static u16 ground_mask[5];	
static u8 state[5];
static float time[5],delay[5];	
static float temp_h;
static float pos_str[5][3],pos_tar[5][3];
//判断是否重合等
	
switch(state[id])
{
case 0:
if(*en){//由着地点规划当前轨迹
//cal_curve_from_pos(in,desire_time);
state[id]=1;	
ground_mask[id]=time[id]=delay[id]=0;	
in->leg_ground=0;
}
break;
case 1://有时间和轨迹计算每一时间的曲线坐标
if(*en){
in->pos_tar[2].z=in->pos_now[2].z+down_h;
temp_h=in->pos_now[2].z;
{state[id]=2;}
}
break;
case 2:
if(*en){
time[id]+=dt;
if(time[id]>down_t)	
{
state[id]=3;
time[id]=0;		
in->pos_tar[2].z=temp_h-1.61*down_h;
}
}
break;///////////////
case 3:
if(*en){
in->pos_tar[2].z=temp_h-1.61*down_h;	
#if !TIRG_CURVE_USE_BAI
cal_curve_from_pos(in,desire_time);	
#else	
pos_str[id][Xs]=in->pos_now[2].x;
pos_str[id][Ys]=in->pos_now[2].y;
pos_str[id][Zs]=in->pos_now[2].z;
pos_tar[id][Xs]=in->pos_tar_trig[2].x;
pos_tar[id][Ys]=in->pos_tar_trig[2].y;
pos_tar[id][Zs]=in->pos_tar_trig[2].z;	
#endif
time[id]+=dt;
state[id]=4;
}
break;
case 4:
if(*en){
#if !TIRG_CURVE_USE_BAI
cal_pos_tar_from_curve(in,time[id],dt);
#else
leg_curve_bai(in,pos_str[id][Xr],pos_str[id][Yr],pos_str[id][Zr],pos_tar[id][Xr],pos_tar[id][Yr],pos_tar[id][Zr]
	,in->sys.leg_up_high,time[id],dt,desire_time-dt);
#endif
#if TWO_LEG_TEST
if(time[id]<desire_time/2)	
time[id]+=dt;
#else	
time[id]+=dt;
if(time[id]>desire_time)	
{state[id]=5;}
#endif
}
break;
case 5:
	in->leg_ground=1;
  state[id]=0;
  if(in->rst_leg)
		in->rst_leg=0;
  *en=0;
break;
}	
}

//蹬腿
void  cal_pos_tar_for_deng(LEG_STRUCT * in,float spdx,float spdy,float dt)
{
u8 id=in->sys.id;
static u8 state;
static float h[5];
static float time;
static float reg[5][3];	
	float spd_wx,spd_wy;
	float x_temp=fabs(sin((90-brain.sys.yaw_trig)/57.3))*brain.tar_w;
	float y_temp=fabs(cos((90-brain.sys.yaw_trig)/57.3))*brain.tar_w;
	if(brain.ground_leg_num<4)
		x_temp=y_temp=0;
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
	
//判断是否重合等
	if(!in->err&&in->leg_ground){
	 if(in->sys.leg_set_invert){	
	in->pos_tar[2].x+=-(spdx+spd_wx)*dt;
	in->pos_tar[2].y+=-(spdy+spd_wy)*dt;
	 }else{
	in->pos_tar[2].x+=-(spdx+spd_wx)*dt;
	in->pos_tar[2].y+=-(spdy+spd_wy)*dt;
	 }
	 
	//protect	
	if(isnan(in->pos_tar[2].x))
	in->pos_tar[2].x=reg[id][0];
	if(isnan(in->pos_tar[2].y))
	in->pos_tar[2].y=reg[id][1];
	if(isnan(in->pos_tar[2].z))
	in->pos_tar[2].z=reg[id][2];

	if(!isnan(in->pos_tar[2].x))
	reg[id][0]=in->pos_tar[2].x;
	if(!isnan(in->pos_tar[2].y))
	reg[id][1]=in->pos_tar[2].y;
	if(!isnan(in->pos_tar[2].z))
	reg[id][2]=in->pos_tar[2].z; 

	 
	in->pos_tar[2].x=LIMIT(in->pos_tar[2].x,-in->sys.limit.x,in->sys.limit.x);
	in->pos_tar[2].y=LIMIT(in->pos_tar[2].y,-in->sys.limit.y,in->sys.limit.y);	 
	limit_range_leg(in->pos_tar[2].x,in->pos_tar[2].y,in->sys.limit.x,in->sys.limit.y,&in->pos_tar[2].x,&in->pos_tar[2].y);
	  
	//in->pos_tar[2].z=LIMIT(in->pos_now[2].z+att_control_out[id]-(brain.tar_h-brain.global.end_pos_global[0].z)*dt*1.618,in->sys.limit_min.z,in->sys.limit.z);	 
	in->pos_tar[2].z=LIMIT(att_control_out[id]+brain.tar_h,in->sys.limit_min.z,in->sys.limit.z);	 
	in->pos_tar[2].z=LIMIT(in->pos_tar[2].z,in->sys.limit_min.z,in->sys.limit.z);
	}
}	

//着地检测
void  leg_ground_check(LEG_STRUCT * in)
{
u8 id=in->sys.id;
static u8 state;
static float time;	
   if(in->sys.leg_ground_force==2)
   in->leg_ground=1;
	 else if(in->sys.leg_ground_force==1)
	 in->leg_ground=0;	 
}	



float sita_test[4]={90,0,0};
u8 line_test[4];
u8 force_test_mode;
void leg_publish(LEG_STRUCT * in)
{
u8 id=in->sys.id;
float x_temp,y_temp,z_temp;
static u16 cnt[5];	
	if(in->sys.id==1||in->sys.id==3){
	in->sys.off_all.x=brain.sys.center_off.x+center_control_out[Xr]*0;
	in->sys.off_all.y=brain.sys.center_off.y+center_control_out[Yr]*0;
	in->sys.off_all.z=brain.sys.center_off.z;
	}
	else
	{
	in->sys.off_all.x=brain.sys.center_off1.x+center_control_out[Xr]*0;
	in->sys.off_all.y=brain.sys.center_off1.y+center_control_out[Yr]*0;
	in->sys.off_all.z=brain.sys.center_off1.z;
	}
	in->sys.leg_up_high=brain.sys.leg_h[id];
	in->sys.desire_time=brain.sys.desire_time;
	if(brain.power_all)
		in->leg_power=1;
	if(brain.control_mode)
		in->control_mode=1;
	
	static u8 rst_all;
	static u8 rst_flag=1;
	switch(rst_all){
		case 0:
			if(brain.rst_all){
			brain.spd=0;
			leg[1].rst_leg=1;
		  rst_all=1;
			}
		break;
		case 1:
			if(leg[rst_flag].rst_leg==0)
			{rst_flag++;leg[rst_flag].rst_leg=1;}
		if(rst_flag==5)
		{rst_flag=1;brain.rst_all=0;rst_all=0;}
		break;	
	}

	#if USE_BUS_DJ&&!USE_DJ_CONTROL_BOARD
	if(in->leg_power==0){	
		if(cnt[in->sys.id]++>1/0.02){cnt[in->sys.id]=0;
		LEG_POWER_OFF(in->sys.id);}
	}
	#endif
  if(in->control_mode||force_test_mode)
	{	
	
  if(in->sys.leg_set_invert)//<---------------------use normal mode 
	{	
	x_temp=-(in->pos_tar[2].x);
	y_temp=-(in->pos_tar[2].y);
	z_temp=in->pos_tar[2].z;
	}else{
	x_temp=in->pos_tar[2].x;
	y_temp=in->pos_tar[2].y;
	z_temp=in->pos_tar[2].z;
	}		
	//由坐标计算角度
	cal_sita_from_pos(in,x_temp+in->sys.off_all.x,y_temp+in->sys.off_all.y,
	z_temp+in->sys.off_all.z,1);
	in->sita[3]=0;
	 if(line_test[3]){//强制角度测试
	 in->sita[0]=sita_test[0];in->sita[1]=sita_test[1];in->sita[2]=sita_test[2];in->sita[3]=sita_test[3];}
	 else if(brain.sys.control_angle)//强制角度控制
	 { in->sita[0]=in->sita_force[0];
		 in->sita[1]=in->sita_force[1];
		 in->sita[2]=in->sita_force[2];
	   in->sita[3]=in->sita_force[3];}
  cal_pwm_from_sita(in);//计算PWM由角度	
	if(brain.sys.control_angle)
  cal_sita_from_pos(in,x_temp+in->sys.off_all.x,y_temp+in->sys.off_all.y,
	z_temp+in->sys.off_all.z,1);
  else	
	cal_sita_from_pos(in,x_temp,y_temp,z_temp,0);//从角度反推位置	
	}
	
	else{//----------------------not on control mode 强制测试摸个位置
		in->pos_tar[2].x=in->sys.init_end_pos.x;in->pos_tar[2].y=in->sys.init_end_pos.y;in->pos_tar[2].z=in->sys.init_end_pos.z;
		 if(in->sys.leg_set_invert)
	{	
	x_temp=-(in->sys.init_end_pos.x-in->sys.off_all.x);
	y_temp=-(in->sys.init_end_pos.y-in->sys.off_all.y);
	z_temp=in->sys.init_end_pos.z+in->sys.off_all.z;
	}else{
	x_temp=in->sys.init_end_pos.x+in->sys.off_all.x;
	y_temp=in->sys.init_end_pos.y+in->sys.off_all.y;
	z_temp=in->sys.init_end_pos.z+in->sys.off_all.z;
	}		
	//由坐标计算角度
	cal_sita_from_pos(in,x_temp+in->sys.off_all.x,y_temp+in->sys.off_all.y,
	z_temp+in->sys.off_all.z,1);
	in->sita[3]=0;
	 if(line_test[3]){//强制角度测试
	in->sita[0]=sita_test[0];in->sita[1]=sita_test[1];in->sita[2]=sita_test[2];in->sita[3]=sita_test[3];}
  cal_pwm_from_sita(in);//计算PWM由角度	
	cal_sita_from_pos(in,x_temp,y_temp,z_temp,0);//从角度反推位置	
	}	
}
float k_add_test22=0.01;
float test22[9]={8,14,0,0,13,0,0,0};
void leg_drive(LEG_STRUCT * in,float dt)
{  
 u8 id=in->sys.id; 	
 static u8 init[5];	
	if(!init[id])
	{
	init[id]=1;	
	in->sys.pos_tar_reg[0]=in->pos_tar_trig[2].x;
	in->sys.pos_tar_reg[1]=in->pos_tar_trig[2].y;
	in->sys.pos_tar_reg[2]=in->pos_tar_trig[2].z;	
	} 
	  if(line_test[0]){
			static u8 flag;
			if(flag)
			test22[4]+=k_add_test22;
			else
			test22[4]-=k_add_test22;	
			
			if(test22[4]>(test22[1]+test22[0])*0.7)
				flag=0;
			else if(test22[4]<test22[0])
				flag=1;
		}
		if(line_test[1]){
			static u8 flag;
			if(flag)
			test22[3]+=k_add_test22;
			else
			test22[3]-=k_add_test22;	
			
			if(test22[3]>(test22[1])*0.7)
				flag=0;
			else if(test22[3]<-test22[0]*0.7)
				flag=1;
		}
    test22[8]=cal_sita_form_pos_tri_leg(test22[0],test22[1],test22[2],test22[3],test22[4],&test22[5],&test22[6],&test22[7]);
	
	
	  cal_PWM_for_tri_leg(test22[5],test22[6],test22[7]);
	  if(in->rst_leg)
		{
		in->pos_tar_trig[2].x=in->sys.init_end_pos.x;
		in->pos_tar_trig[2].y=in->sys.init_end_pos.y;
		in->pos_tar_trig[2].z=in->sys.init_end_pos.z+(float)RNG_Get_RandomRange(-1000,1000)/100000.;
		}
	
		if((in->pos_tar_trig[2].x!=in->sys.pos_tar_reg[0]||
			 in->pos_tar_trig[2].y!=in->sys.pos_tar_reg[1]||
			 in->pos_tar_trig[2].z!=in->sys.pos_tar_reg[2])&&!in->curve_trig)	
			{in->curve_trig=1;}//trig 使能
    //跨
		if(in->curve_trig&&in->sys.desire_time>0)
		leg_follow_curve(in,in->sys.desire_time,& in->curve_trig,dt,brain.sys.down_h,brain.sys.down_t);
		//着地
		leg_ground_check(in);
		//蹬
		if(!in->curve_trig)
		cal_pos_tar_for_deng(in,in->deng[0],in->deng[1],dt);	
		//输出	
	  leg_publish(in);
		
		in->sys.pos_tar_reg[0]=in->pos_tar_trig[2].x;
		in->sys.pos_tar_reg[1]=in->pos_tar_trig[2].y;
		in->sys.pos_tar_reg[2]=in->pos_tar_trig[2].z;
		//
}
//--------------------------------------------------VMC control-----------------------------














