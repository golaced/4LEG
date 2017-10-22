#include "include.h" 
//--------------------------LEG_LIB---------------------------
void cpuidGetId(void);
//ת���Ⱦֲ�����ϵ��ȫ�ֻ�������ϵ
void conver_legpos_to_barin(BRAIN_STRUCT *in,LEG_STRUCT * inl,u8 id)
{ u8 i;
	static u8 init;
	if(!init){init=1;
	cpuidGetId();
	}
	for(i=0;i<3;i++){
	inl->pos_now_brain[i].x=inl->pos_now[i].x+in->sys.leg_local[id].x;
  inl->pos_now_brain[i].y=inl->pos_now[i].y+in->sys.leg_local[id].y;
	inl->pos_now_brain[i].z=inl->pos_now[i].z+in->sys.leg_local[id].z;
	}
}
//�жϽŵ����ʼ����λ��
u8 check_leg_near_init(float ero)
{
 u8 i;
 float dis[5];	
	for(i=1;i<5;i++)
	 {
	   dis[i]=cal_dis_of_points(leg[i].pos_now[2].x,leg[i].pos_now[2].y,
		 leg[i].sys.init_end_pos.x,leg[i].sys.init_end_pos.y);
	 }
  if(dis[1]<ero&&dis[2]<ero&&dis[3]<ero&&dis[4]<ero)
	  return 1;
	else 
		return 0;
}	
//�����ظ�������
u8 leg_repeat_protect(u8 id,u8 last_move_id,u8 last_last_move_id,float yaw,float yaw_trig){
u8 i;	
u8 trig_list_f[5]=		 {0,1,4,3,2};
u8 trig_list_r[5]=		 {0,2,3,1,4};
u8 trig_list_b[5]=		 {0,4,1,2,3};
u8 trig_list_l[5]=		 {0,3,2,4,1};	
float yaw_in=To_180_degrees(yaw);
float yaw_temp=yaw_trig;
u8 out;
u8 temp_id;
				if(id==last_move_id)
				{
				  for(i=1;i<5;i++){
					if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)
					temp_id=trig_list_r[i];
					else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)
					temp_id=trig_list_l[i];
					else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))
					temp_id=trig_list_f[i];
					else
					temp_id=trig_list_b[i];
					if(temp_id==id)
						break;
				  }
					if(i>=4)
						if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)
						out= trig_list_r[1];
						else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)
						out= trig_list_l[1];
						else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))
						out= trig_list_f[1];
						else
						out= trig_list_b[1];
					else
						if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)
						out= trig_list_r[i+1];
						else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)
						out= trig_list_l[i+1];
						else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))
						out= trig_list_f[i+1];
						else
						out= trig_list_b[i+1];
				}else	
        out=id;
				
			if(out==last_move_id)
			for(i=1;i<5;i++)
			{
			if(i!=last_move_id)
			{out=i;break;}
			}
				
			return out;	
}

//�����ظ�������
u8 leg_repeat_protect1(u8 id,u8 last_move_id,u8 last_last_move_id,float yaw,float yaw_trig){
u8 i;	
u8 trig_list_f[5]=		 {0,1,4,3,2};
u8 trig_list_r[5]=		 {0,2,3,1,4};
u8 trig_list_b[5]=		 {0,4,1,2,3};
u8 trig_list_l[5]=		 {0,3,2,4,1};	
float yaw_in=To_180_degrees(yaw);
float yaw_temp=yaw_trig;
u8 temp_id;
				if(id==last_move_id||id==last_last_move_id)
				{
				  for(i=1;i<5;i++)
					 {
					   if(i!=last_move_id&&i!=last_last_move_id)
						 return id=i;
					 }
				}else	
        return id;
}

//����ʸ���Գ�����
void resize_point_with_arrow(float x,float y,float cx,float cy,float yaw,float k,float *nx,float *ny)
{
float kc,bc;	
float kp_90,bp_90;	
float cro_x,cro_y;
if(k==1)
{
*nx=x;
*ny=y;	
}else{	
line_function_from_arrow( cx, cy, yaw, &kc,&bc);
line_function90_from_arrow(x, y, yaw, &kp_90,&bp_90);
cross_point_of_lines(kc,bc,kp_90,bp_90,&cro_x,&cro_y);
float dis=cal_dis_of_points(x,y,cro_x,cro_y)*(1-k);	
u8 flag=check_point_front_arrow( cx, cy, x, y, yaw+90);
float yaw1;
if(flag)
yaw1=yaw+90;
else
yaw1=yaw-90;

*nx=x+sin(yaw1*ANGLE_TO_RADIAN)*dis;
*ny=y+cos(yaw1*ANGLE_TO_RADIAN)*dis;	
}	
}
//��ŷ�Χ����
void limit_trig_pos_of_leg(float cx,float cy,float cz,float rx,float ry,float rz,float *x,float *y,float *z)
{
 u8 flag[3];
 float k[3],b[3];
 float dis[2];
 float jiaodiaoxy[3][3]={0};
 float jiaodiaoxz[3][3]={0};
 float jiaodiaoyz[3][3]={0};	
 
	flag[0]=in_circle(0,0,rx,ry,cx,cy);
	flag[1]=in_circle(0,0,rz,ry,cz,cy);
	flag[2]=in_circle(0,0,rx,rz,cx,cz);
 if(flag[0]&&flag[1]&&flag[2])
 {
  *x=cx;
	*y=cy;
  *z=cz;
 }else
 {
 //xy
   k[0]=cy/(cx+0.00001);
	 //�����ٶ�ֱ������Բ����
	 float temp=sqrt(pow(rx,2)/(1+pow(rx*k[0]/ry,2)));
	 //�ж��ٶȷ��򽻵����
	 jiaodiaoxy[0][Xr]=temp; 
	 jiaodiaoxy[1][Xr]=-temp; 
	 jiaodiaoxy[0][Yr]=k[0]*jiaodiaoxy[0][Xr];
	 jiaodiaoxy[1][Yr]=k[0]*jiaodiaoxy[1][Xr];
	 dis[0]=cal_dis_of_points(cx,cy,jiaodiaoxy[0][Xr],jiaodiaoxy[0][Yr]);
   dis[1]=cal_dis_of_points(cx,cy,jiaodiaoxy[1][Xr],jiaodiaoxy[1][Yr]);
	 if(dis[0]<dis[1])
	 {jiaodiaoxy[2][Xr]=jiaodiaoxy[0][Xr];jiaodiaoxy[2][Yr]=jiaodiaoxy[0][Yr];}
	 else
	 {jiaodiaoxy[2][Xr]=jiaodiaoxy[1][Xr];jiaodiaoxy[2][Yr]=jiaodiaoxy[1][Yr];}
	//xz 
   cal_jiao_of_tuo_and_line(cx,cz,rx,rz,90,&jiaodiaoxz[0][Xr],&jiaodiaoxz[0][Zr],&jiaodiaoxz[1][Xr],&jiaodiaoxz[1][Zr]);
	 dis[0]=cal_dis_of_points(cx,cz,jiaodiaoxz[0][Xr],jiaodiaoxz[0][Zr]);
   dis[1]=cal_dis_of_points(cx,cz,jiaodiaoxz[1][Xr],jiaodiaoxz[1][Zr]);
	 if(dis[0]<dis[1])
	 {jiaodiaoxz[2][Xr]=jiaodiaoxz[0][Xr];jiaodiaoxz[2][Zr]=jiaodiaoxz[0][Zr];}
	 else
	 {jiaodiaoxz[2][Xr]=jiaodiaoxz[1][Xr];jiaodiaoxz[2][Zr]=jiaodiaoxz[1][Zr];}
	//yz 
	 cal_jiao_of_tuo_and_line(cy,cz,ry,rz,90,&jiaodiaoyz[0][Yr],&jiaodiaoyz[0][Zr],&jiaodiaoyz[1][Yr],&jiaodiaoyz[1][Zr]);
	 dis[0]=cal_dis_of_points(cy,cz,jiaodiaoyz[0][Yr],jiaodiaoyz[0][Zr]);
   dis[1]=cal_dis_of_points(cy,cz,jiaodiaoyz[1][Yr],jiaodiaoyz[1][Zr]);
	 if(dis[0]<dis[1])
	 {jiaodiaoyz[2][Yr]=jiaodiaoyz[0][Yr];jiaodiaoyz[2][Zr]=jiaodiaoyz[0][Zr];}
	 else
	 {jiaodiaoyz[2][Yr]=jiaodiaoyz[1][Yr];jiaodiaoyz[2][Zr]=jiaodiaoyz[1][Zr];}
	
  if(fabs(jiaodiaoxz[2][Xr])<fabs(jiaodiaoxy[2][Xr]))	 
	*x=jiaodiaoxz[2][Xr];
	else
	*x=jiaodiaoxy[2][Xr];	
	
	if(fabs(jiaodiaoyz[2][Yr])<fabs(jiaodiaoxy[2][Yr]))	 
	*y=jiaodiaoyz[2][Yr];
	else
	*y=jiaodiaoxy[2][Yr];	
	
	*z=(jiaodiaoxz[2][Zr]+jiaodiaoyz[2][Zr])/2;	
 }
}
//��������Ľ���
u8 arrow_check_to_bow(float cx,float cy,float cz,float rx,float ry,float rz,float *x,float *y,float *z)
{
  float yaw[3];
  u8 flag[3];
	float k[3],b[3];
	float jiao1[3][3],jiao2[3][3];
	float jiaon[3][3]={0};
	float dis[2];
	flag[0]=in_circle(0,0,rx,ry,cx,cy);
	flag[1]=in_circle(0,0,rz,ry,cz,cy);
	flag[2]=in_circle(0,0,rx,rz,cx,cz);
	
	if(flag[0]&&flag[1]&&flag[2]){
  *x=cx;
	*y=cy;
  *z=cz;
  return 1; 		
	}
  else
  {
	//xy
  limit_range_leg( cx, cy,rx,ry,&jiaon[0][Xr],&jiaon[0][Yr]);
	//xz
	limit_range_leg( cx, cz,rx,rz,&jiaon[1][Xr],&jiaon[1][Zr]);
  //yz		
  limit_range_leg( cy, cz,ry,rz,&jiaon[2][Yr],&jiaon[2][Zr]);
		
	*x=(jiaon[0][Xr]+jiaon[1][Xr])/2;
	*y=(jiaon[0][Yr]+jiaon[2][Yr])/2;	
	*z=(jiaon[2][Zr]+jiaon[1][Zr])/2;	
//	*x=jiaon[0][Xr];
//	*y=jiaon[0][Yr];
//	*z=jiaon[2][Zr];
	 return 0;	
	}		
}


//������ֱ�߷���
void line_function_from_two_point(float x1,float y1,float x2,float y2,float *k,float *b)
{ 
	float k_temp=0;
  *k=k_temp=(y1-y2)/(x1-x2+0.000001);
  *b=y1-k_temp*x1;
}	

//ʸ����ֱ�߷���
void line_function_from_arrow(float x,float y,float yaw,float *k,float *b)
{ 
	float tyaw=90-yaw+0.000011;
	float k_temp=0;
  *k=k_temp=tan(tyaw*ANGLE_TO_RADIAN);
  *b=y-k_temp*x;
}	

//ʸ�����߷���
void line_function90_from_arrow(float x,float y,float yaw,float *k,float *b)
{ 
	float tyaw=90-yaw+0.000011;
	float k_temp=0;
  *k=k_temp=-1/tan(tyaw*ANGLE_TO_RADIAN);
  *b=y-k_temp*x;
}	

//����ֱ����
u8 check_point_on_line(float x,float y,float k,float b,float err)
{ 
	float temp=k*x+b-y;
	if(ABS(temp)<err)
		return 1;
	else
		return 0;
}	

//��ֱ�߽���
u8 cross_point_of_lines(float k1,float b1,float k2,float b2,float *x,float *y)
{ 
	if(ABS(k1-k2)<0.001){
		*x=*y=0;
		return 0;}
	float x_temp;
	*x=x_temp=(b1-b2)/(k2-k1+0.00001);
//	if(fabs(k1)>10000&&fabs(b1)>10000)
//  *y=0;
//  else	
	*y=k1*x_temp+b1;
	
	return 1;
}	

//���ڵ�ʸ������ǰ
u8 check_point_front_arrow(float x,float y,float cx,float cy,float yaw)
{ 
  float tyaw=90-yaw+0.000011;
	float kc_90=-1/tan(tyaw*ANGLE_TO_RADIAN);
	float bc_90=cy-kc_90*cx;
	float cx_t=cx+sin(yaw*ANGLE_TO_RADIAN)*1,cy_t=cy+cos(yaw*ANGLE_TO_RADIAN)*1;
	float flag[2];
	flag[0]=kc_90*cx_t+bc_90-cy_t;
	flag[1]=kc_90*x+bc_90-y;
	if((flag[0]>0&&flag[1]>0)||(flag[0]<0&&flag[1]<0))
	return 1;
	else 
  return 0;
}	

//�ж���������ͬһ��
u8 check_points_same_side(float x1,float y1,float x2,float y2,float k,float b)
{ 
	float flag[2];
	flag[0]=k*x1+b-y1;
	flag[1]=k*x2+b-y2;
	if(flag[0]*flag[1]>0)
	return 1;
	else 
  return 0;
}	

//��ʸ����ֱ�߽���
u8 check_cross_arrow_line(float cx,float cy,float yaw,float k,float b,float *x,float *y)
{ 
  float tyaw=90-yaw+0.000011;
	float kc=tan(tyaw*ANGLE_TO_RADIAN);
	float bc=cy-kc*cx;
	float cro_x,cro_y;
	u8 flag;	
	flag=cross_point_of_lines(k,b,kc,bc,&cro_x,&cro_y);
	*x=cro_x;
	*y=cro_y;
	
	if(flag==0||fabs(cro_y)>100||fabs(cro_x)>100)
		return 0;
	//�н�������ǰ��
	if(check_point_front_arrow(cro_x,cro_y, cx, cy, yaw))	
	return 1;
	else{
	*x=*y=0;	
	return 0;
	}
}	

//��ʸ��������ֱ�߽���
u8 check_cross_arrow90_line(float cx,float cy,float yaw,float k,float b,float *x,float *y)
{ 
  float tyaw=90-yaw+0.000011;
	float kc=tan(tyaw*ANGLE_TO_RADIAN);
	float kc_90=-1/(kc+0.00001);
	float bc_90=cy-kc_90*cx;
	float cro_x,cro_y;
	
	u8 flag;	
	flag=cross_point_of_lines(k,b,kc_90,bc_90,&cro_x,&cro_y);
	*x=cro_x;
	*y=cro_y;
	
	if(flag==0){
	*x=*y=0;	
  return 0;}
  else
	return 1;
}	
//������������������
void cal_center_of_trig(float x1,float y1,float x2,float y2,float x3,float x4,float *cx,float *cy)
{

}	

//�����������
float cal_dis_of_points(float x1,float y1,float x2,float y2)
{
return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}	

//�ж�һ��������Բ�ڲ�
u8 in_circle(float cx,float cy,float d_short,float d_long,float x,float y)
{
   float temp=pow(x-cx,2)/pow(d_short,2)+pow(y-cy,2)/pow(d_long,2);
   if(temp>1)//����
		 return 0;
	 else 
		 return 1;
}

//�㵽ֱ�߾���
float dis_point_to_line(float x,float y,float k,float b)
{ 
  float k_90=-1/(k+0.000011);
	float b_90=y-k_90*x;
	float cx,cy;
	cross_point_of_lines(k,b,k_90,b_90,&cx,&cy);
	
	return cal_dis_of_points(x,y,cx,cy);
}	

//����������ת����ȫ������
void conver_body_to_global(float bx,float by,float *gx,float *gy)
{
 *gx=bx+brain.sys.leg_local[1].x-leg[4].pos_now[2].x+brain.sys.off_cor[Xr];
 *gy=by+brain.sys.leg_local[1].y-leg[4].pos_now[2].y+brain.sys.off_cor[Yr];
}

//�����ʸ������ԳƵ�
void two_point_between_arror(float cx,float cy,float yaw,float *x1,float *y1,float *x2,float *y2,float dis)
{
  float k_90,b_90;
  float jiao[2][2];
	line_function90_from_arrow(cx,cy,yaw,&k_90,&b_90);
	cal_jiao_of_tuo_and_line(cx,cy,yaw+90,dis,dis,&jiao[0][Xr],&jiao[0][Yr],&jiao[1][Xr],&jiao[1][Yr]); 
	*x1=jiao[0][Xr];
  *y1=jiao[0][Yr];
  *x2=jiao[1][Xr];
  *y2=jiao[1][Yr];
}	


void limit_range_leg(float x,float y,float min,float max,float *xout,float *yout)
{
  u8 flag;
	float yaw;
	float jiaodiao[2][2]={0};
	float temp;
	float k;
	float dis[2];
   flag=in_circle(0,0,min,max,x,y);
   if(flag==1){
		 *xout=x;
	   *yout=y;
	 }else 
	 {
	   k=y/(x+0.00001);
	
	 //�����ٶ�ֱ������Բ����
	 float temp=sqrt(pow(min,2)/(1+pow(min*k/max,2)));
	 //�ж��ٶȷ��򽻵����
	 jiaodiao[0][Xr]=temp; 
	 jiaodiao[1][Xr]=-temp; 
	 jiaodiao[0][Yr]=k*jiaodiao[0][Xr];
	 jiaodiao[1][Yr]=k*jiaodiao[1][Xr];
	 dis[0]=cal_dis_of_points(x,y,jiaodiao[0][Xr],jiaodiao[0][Yr]);
   dis[1]=cal_dis_of_points(x,y,jiaodiao[1][Xr],jiaodiao[1][Yr]);
	 if(dis[0]<dis[1])
	 {*xout=jiaodiao[0][Xr];*yout=jiaodiao[0][Yr];}
	 else
	 {*xout=jiaodiao[1][Xr];*yout=jiaodiao[1][Yr];}
	 }
}	


//�ƶ������޷��� ����
void limit_move_range_tangle(u8 id,float cx,float cy,float x,float y,float min,float max,float *outx,float *outy)
{
  float tangle[4][2];
	float length[5];
	
	
	switch(id){
	case 1:
	length[1]=brain.sys.leg_move_range1[1];
	length[2]=brain.sys.leg_move_range1[2];
	length[3]=brain.sys.leg_move_range1[3];
	length[4]=brain.sys.leg_move_range1[4];
	break;
	case 2:
	length[1]=brain.sys.leg_move_range1[3];
	length[2]=brain.sys.leg_move_range1[2];
	length[3]=brain.sys.leg_move_range1[1];
	length[4]=brain.sys.leg_move_range1[4];
	break;
	case 3:
	length[1]=brain.sys.leg_move_range1[1];
	length[2]=brain.sys.leg_move_range1[4];
	length[3]=brain.sys.leg_move_range1[3];
	length[4]=brain.sys.leg_move_range1[2];
	break;
	case 4:
	length[1]=brain.sys.leg_move_range1[3];
	length[2]=brain.sys.leg_move_range1[4];
	length[3]=brain.sys.leg_move_range1[1];
	length[4]=brain.sys.leg_move_range1[2];
	break;
  }
	tangle[0][Xr]=-length[4];tangle[0][Yr]=length[1];
	tangle[1][Xr]=length[2];tangle[1][Yr]=length[1];
	tangle[2][Xr]=length[2];tangle[2][Yr]=-length[3];
	tangle[3][Xr]=-length[4];tangle[3][Yr]=-length[3];
	
	*outx=LIMIT(x,tangle[0][Xr]+cx,tangle[1][Xr]+cx);
	*outy=LIMIT(y,tangle[2][Yr]+cy,tangle[1][Yr]+cy);
}	

//�жϵ����ƶ������ڲ�tangle
u8 check_in_move_range_tangle(u8 id,float x,float y,float cx,float cy,float min,float max,float *min_dis)
{
		float tangle[4][2];
		u8 flag[3];
		float length[5];
		switch(id){
		case 1:
		length[1]=brain.sys.leg_move_range1[1];
		length[2]=brain.sys.leg_move_range1[2];
		length[3]=brain.sys.leg_move_range1[3];
		length[4]=brain.sys.leg_move_range1[4];
		break;
		case 2:
		length[1]=brain.sys.leg_move_range1[3];
		length[2]=brain.sys.leg_move_range1[2];
		length[3]=brain.sys.leg_move_range1[1];
		length[4]=brain.sys.leg_move_range1[4];
		break;
		case 3:
		length[1]=brain.sys.leg_move_range1[1];
		length[2]=brain.sys.leg_move_range1[4];
		length[3]=brain.sys.leg_move_range1[3];
		length[4]=brain.sys.leg_move_range1[2];
		break;
		case 4:
		length[1]=brain.sys.leg_move_range1[3];
		length[2]=brain.sys.leg_move_range1[4];
		length[3]=brain.sys.leg_move_range1[1];
		length[4]=brain.sys.leg_move_range1[2];
		break;
		}
		tangle[0][Xr]=-length[4]+cx;tangle[0][Yr]=length[1]+cy;
		tangle[1][Xr]=length[2]+cx;tangle[1][Yr]=length[1]+cy;
		tangle[2][Xr]=length[2]+cx;tangle[2][Yr]=-length[3]+cy;
		tangle[3][Xr]=-length[4]+cx;tangle[3][Yr]=-length[3]+cy;

	 flag[0]=inTrig2(x,y,tangle[0][Xr],tangle[0][Yr],tangle[1][Xr],tangle[1][Yr]
													 ,tangle[2][Xr],tangle[2][Yr],tangle[3][Xr],tangle[3][Yr]); 
	 
	 *min_dis=get_min_dis_arrow_to_tangle(x,y,tangle[0][Xr],tangle[0][Yr],tangle[1][Xr],tangle[1][Yr]
											 ,tangle[2][Xr],tangle[2][Yr],tangle[3][Xr],tangle[3][Yr]);
		
	 if(flag[0])
	  return 1;
	 else 
		return 0;
}

//�жϵ����ƶ������ڲ�
u8 check_in_move_range(u8 id,float x,float y,float cx,float cy,float min,float max)
{
		float tangle[4][2];
		float tar_x,tar_y;
		u8 flag[3];
	
	  float cx1=x-cx,cy1=y-cy;
	 switch (id)
	 {
		case 1:
		tar_x=cx1;
		tar_y=cy1;
		break;
		case 2:
		tar_x=cx1;
		tar_y=-cy1;
		break;
		case 4:
		tar_x=-cx1;
		tar_y=-cy1;
		break;
		case 3:
		tar_x=-cx1;
		tar_y=cy1;
		break;
	 }
	 
	 tangle[0][Xr]=-min;tangle[0][Yr]=max;
	 tangle[1][Xr]=max;tangle[1][Yr]=max;
	 tangle[2][Xr]=max;tangle[2][Yr]=-min;
	 tangle[3][Xr]=-min;tangle[3][Yr]=-min;
	 flag[0]=inTrig2(tar_x,tar_y,tangle[0][Xr],tangle[0][Yr],tangle[1][Xr],tangle[1][Yr]
													 ,tangle[2][Xr],tangle[2][Yr],tangle[3][Xr],tangle[3][Yr]); 
	 flag[1]=in_circle(0,0,max,max,tar_x,tar_y);
	 flag[2]=in_circle(0,0,min,min,tar_x,tar_y);
	 
	 if(flag[0]&&flag[1]&&flag[2]==1)
	  return 1;
	 else 
		return 0;
}


//�����ƶ�������ʸ������������
void cal_jiao_of_range_and_line(u8 id,float cx,float cy,float min,float max,float yaw,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y)
{

	 float jiaodiao[2][2]={0};
   float yaw_use;	
	 float tangle[4][2];
	 float tar_x=0,tar_y=0;
	 u8 flag[3];
	 switch (id)
	 {
		case 1:
		yaw_use=yaw;
		break;
		case 2:
		yaw_use=180-yaw;
		break;
		case 3:
		yaw_use=-yaw;
		break;
		case 4:
		yaw_use=yaw+180;
		break;
	 }
	 
	 tangle[0][Xr]=-min;tangle[0][Yr]=max;
	 tangle[1][Xr]=max;tangle[1][Yr]=max;
	 tangle[2][Xr]=max;tangle[2][Yr]=-min;
	 tangle[3][Xr]=-min;tangle[3][Yr]=-min;

	  u8 sel=0;
		float yaw_t=To_180_degrees(yaw_use+0.01);
		 if(yaw_t>0&&yaw_t<90)
			 sel=1;
		 else if(yaw_t>90&&yaw_t<=180)
		   sel=2;
		 else if(yaw_t<0&&yaw_t>-90)
		   sel=4;
		 else 
			 sel=3;
		 float jiao1[2][2];
		 float jiao2[2][2];
		 switch(sel){
			 case 1:
			 cal_jiao_of_tuo_and_line(0,0,max,max,yaw_t,&jiao1[0][Xr],&jiao1[0][Yr],&jiao1[1][Xr],&jiao1[1][Yr]);
       cal_jiao_of_tuo_and_line(0,0,min,min,yaw_t,&jiao2[0][Xr],&jiao2[0][Yr],&jiao2[1][Xr],&jiao2[1][Yr]);
			 jiaodiao[0][Xr]=jiao1[0][Xr];
			 jiaodiao[0][Yr]=jiao1[0][Yr];
			 jiaodiao[1][Xr]=jiao2[1][Xr];
			 jiaodiao[1][Yr]=jiao2[1][Yr];
			 break; 
			 case 3:
			 cal_jiao_of_tuo_and_line(0,0,max,max,yaw_t,&jiao1[0][Xr],&jiao1[0][Yr],&jiao1[1][Xr],&jiao1[1][Yr]);
       cal_jiao_of_tuo_and_line(0,0,min,min,yaw_t,&jiao2[0][Xr],&jiao2[0][Yr],&jiao2[1][Xr],&jiao2[1][Yr]);
			 jiaodiao[0][Xr]=jiao2[0][Xr];
			 jiaodiao[0][Yr]=jiao2[0][Yr];
			 jiaodiao[1][Xr]=jiao1[1][Xr];
			 jiaodiao[1][Yr]=jiao1[1][Yr]; 
			 break;
			 default:
			 check_point_to_tangle(0,0,yaw_t,tangle[0][Xr],tangle[0][Yr],tangle[1][Xr],tangle[1][Yr]
													 ,tangle[2][Xr],tangle[2][Yr],tangle[3][Xr],tangle[3][Yr],
														&jiao1[0][Xr],&jiao1[0][Yr],&jiao1[1][Xr],&jiao1[1][Yr]);	 	
			 jiaodiao[0][Xr]=jiao1[0][Xr];
			 jiaodiao[0][Yr]=jiao1[0][Yr];
			 jiaodiao[1][Xr]=jiao2[1][Xr];
			 jiaodiao[1][Yr]=jiao2[1][Xr];
			 break;
			 
		 }
	  int flag3[2];
		switch (id)
		{
		case 1:
    flag3[0]=1;
		flag3[1]=1;
		break;
		case 2:
    flag3[0]=1;
		flag3[1]=-1;
		break;
		case 4:
    flag3[0]=-1;
		flag3[1]=-1;
		break;
		case 3:
    flag3[0]=-1;
		flag3[1]=1;
		break;
		}
	 jiaodiao[0][Xr]*=flag3[0];
	 jiaodiao[0][Yr]*=flag3[1];
	 jiaodiao[1][Xr]*=flag3[0];
	 jiaodiao[1][Yr]*=flag3[1];
		
	 jiaodiao[0][Xr]+=cx;jiaodiao[1][Xr]+=cx;
	 jiaodiao[0][Yr]+=cy;jiaodiao[1][Yr]+=cy;

   *jiao1_x=jiaodiao[0][Xr];
	 *jiao1_y=jiaodiao[0][Yr];
	 *jiao2_x=jiaodiao[1][Xr];
	 *jiao2_y=jiaodiao[1][Yr];
	 
}


//�����ƶ�������ʸ������������ ����
void cal_jiao_of_range_and_line_tangle(u8 id,float cx,float cy,float min,float max,float yaw,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y)
{

	 float jiaodiao[2][2]={0};
   float yaw_use;	
	 float tangle[4][2];
	 float tar_x=0,tar_y=0;
	 u8 flag[3];
     float length[5];

	switch(id){
	case 1:
	length[1]=brain.sys.leg_move_range1[1];
	length[2]=brain.sys.leg_move_range1[2];
	length[3]=brain.sys.leg_move_range1[3];
	length[4]=brain.sys.leg_move_range1[4];
	break;
	case 2:
	length[1]=brain.sys.leg_move_range1[3];
	length[2]=brain.sys.leg_move_range1[2];
	length[3]=brain.sys.leg_move_range1[1];
	length[4]=brain.sys.leg_move_range1[4];
	break;
	case 3:
	length[1]=brain.sys.leg_move_range1[1];
	length[2]=brain.sys.leg_move_range1[4];
	length[3]=brain.sys.leg_move_range1[3];
	length[4]=brain.sys.leg_move_range1[2];
	break;
	case 4:
	length[1]=brain.sys.leg_move_range1[3];
	length[2]=brain.sys.leg_move_range1[4];
	length[3]=brain.sys.leg_move_range1[1];
	length[4]=brain.sys.leg_move_range1[2];
	break;
  }
	tangle[0][Xr]=-length[4];tangle[0][Yr]=length[1];
	tangle[1][Xr]=length[2];tangle[1][Yr]=length[1];
	tangle[2][Xr]=length[2];tangle[2][Yr]=-length[3];
	tangle[3][Xr]=-length[4];tangle[3][Yr]=-length[3];
	 
	 check_point_to_tangle(0,0,yaw,tangle[0][Xr],tangle[0][Yr],tangle[1][Xr],tangle[1][Yr]
											 ,tangle[2][Xr],tangle[2][Yr],tangle[3][Xr],tangle[3][Yr],
												&jiaodiao[0][Xr],&jiaodiao[0][Yr],&jiaodiao[1][Xr],&jiaodiao[1][Yr]);	 
	
			
	 jiaodiao[0][Xr]+=cx;jiaodiao[1][Xr]+=cx;
	 jiaodiao[0][Yr]+=cy;jiaodiao[1][Yr]+=cy;

   *jiao1_x=jiaodiao[0][Xr];
	 *jiao1_y=jiaodiao[0][Yr];
	 *jiao2_x=jiaodiao[1][Xr];
	 *jiao2_y=jiaodiao[1][Yr]; 
}
//��ʸ�뷽����̾���
float get_min_dis_arrow_to_tangle(float x,float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4)
{
	u8 i;
	float k[4],b[4];
	line_function_from_two_point(x1,y1,x2,y2,&k[0],&b[0]);
	line_function_from_two_point(x2,y2,x3,y3,&k[1],&b[1]);
	line_function_from_two_point(x3,y3,x4,y4,&k[2],&b[2]);
	line_function_from_two_point(x4,y4,x1,y1,&k[3],&b[3]);
  float dis[4];
  dis[0]=dis_point_to_line(x,y,k[0],b[0]);
  dis[1]=dis_point_to_line(x,y,k[1],b[1]);
	dis[2]=dis_point_to_line(x,y,k[2],b[2]);
	dis[3]=dis_point_to_line(x,y,k[3],b[3]);
	float min_dis=dis[0];
  for(i=0;i<4;i++)	
	  if(dis[i]<min_dis)
	     min_dis=dis[i];
		
	return min_dis;
}

//��ʸ�����ı��ν���
u8 check_point_to_tangle(float x,float y,float yaw,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4
	,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y)
{
	u8 i,j=0;
  float k[4],b[4];
  float kc,bc;
	line_function_from_two_point(x1,y1,x2,y2,&k[0],&b[0]);
	line_function_from_two_point(x2,y2,x3,y3,&k[1],&b[1]);
	line_function_from_two_point(x3,y3,x4,y4,&k[2],&b[2]);
	line_function_from_two_point(x4,y4,x1,y1,&k[3],&b[3]);
  line_function_from_arrow(x,y,yaw,&kc,&bc);
	u8 flag[4];
	float cro_x[4],cro_y[4];
  flag[0]=cross_point_of_lines(kc,bc,k[0],b[0],&cro_x[0],&cro_y[0]);
	flag[1]=cross_point_of_lines(kc,bc,k[1],b[1],&cro_x[1],&cro_y[1]);
	flag[2]=cross_point_of_lines(kc,bc,k[2],b[2],&cro_x[2],&cro_y[2]);
	flag[3]=cross_point_of_lines(kc,bc,k[3],b[3],&cro_x[3],&cro_y[3]);
  
	
	if(flag[0]||flag[1]||flag[2]||flag[3]==1)
	{	
	float jiaodiao1[2][2];
	for(i=0;i<4;i++)
		{
		  if(flag[i]&&(fabs(cro_x[i])<fabs(x2)+fabs(x1))&&(fabs(cro_y[i])<fabs(y2)+fabs(y3)))
			{ jiaodiao1[j][Xr]=cro_x[i];jiaodiao1[j][Yr]=cro_y[i];
        j++;
				if(j>2)
					break;
      }				
		}
	
	if(j<1)
		return 0;
	
  u8 flag2;	
	flag2=check_point_front_arrow(jiaodiao1[0][Xr],jiaodiao1[0][Yr],x,y,yaw);
		
	 if(flag2){
	 *jiao1_x=jiaodiao1[0][Xr];
	 *jiao1_y=jiaodiao1[0][Yr];
	 *jiao2_x=jiaodiao1[1][Xr];
	 *jiao2_y=jiaodiao1[1][Yr];}
	 else
		{
	 *jiao1_x=jiaodiao1[1][Xr];
	 *jiao1_y=jiaodiao1[1][Yr];
	 *jiao2_x=jiaodiao1[0][Xr];
	 *jiao2_y=jiaodiao1[0][Yr];} 
	return 1;
	}else 
	return 0;
}
	


//��ʸ���������ν���
u8 check_point_to_trig(float x,float y,float yaw,float x1,float y1,float x2,float y2,float x3,float y3
	,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y)
{
	u8 i,j=0;
  float k[3],b[3];
  float kc,bc;
	line_function_from_two_point(x1,y1,x2,y2,&k[0],&b[0]);
	line_function_from_two_point(x2,y2,x3,y3,&k[1],&b[1]);
	line_function_from_two_point(x3,y3,x1,y1,&k[2],&b[2]);
  line_function_from_arrow(x,y,yaw,&kc,&bc);
	u8 flag[3];
	float cro_x[3],cro_y[3];
  flag[0]=cross_point_of_lines(kc,bc,k[0],b[0],&cro_x[0],&cro_y[0]);
	flag[1]=cross_point_of_lines(kc,bc,k[1],b[1],&cro_x[1],&cro_y[1]);
	flag[2]=cross_point_of_lines(kc,bc,k[2],b[2],&cro_x[2],&cro_y[2]);
  
	float dis[3];
	dis[0]=cal_dis_of_points(x1,y1,x,y);
	dis[1]=cal_dis_of_points(x2,y2,x,y);
	dis[2]=cal_dis_of_points(x3,y3,x,y);
	
	
	if(flag[0]||flag[1]||flag[2])
	{	
	float jiaodiao1[2][2],temp;
	for(i=0;i<3;i++)
		{
			temp=cal_dis_of_points(cro_x[i],cro_y[i],x,y);
		  if(flag[i]&&(temp<dis[0]||temp<dis[1]||temp<dis[2]))
			{ jiaodiao1[j][Xr]=cro_x[i];jiaodiao1[j][Yr]=cro_y[i];
        j++;
				if(j>2)
					break;
      }				
		}
	
	if(j<1)
		return 0;
	
  u8 flag2;	
	flag2=check_point_front_arrow(jiaodiao1[0][Xr],jiaodiao1[0][Yr],x,y,yaw);
		
	 if(flag2){
	 *jiao1_x=jiaodiao1[0][Xr];
	 *jiao1_y=jiaodiao1[0][Yr];
	 *jiao2_x=jiaodiao1[1][Xr];
	 *jiao2_y=jiaodiao1[1][Yr];}
	 else
		{
	 *jiao1_x=jiaodiao1[1][Xr];
	 *jiao1_y=jiaodiao1[1][Yr];
	 *jiao2_x=jiaodiao1[0][Xr];
	 *jiao2_y=jiaodiao1[0][Yr];} 
	return 1;
	}else 
	return 0;
}

//������Բ��ʸ������������
void cal_jiao_of_tuo_and_line(float cx,float cy,float d_short,float d_long,float yaw,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y)
{
   float tyaw=90-yaw+0.000011;
	 float tan_yaw=tan(tyaw*ANGLE_TO_RADIAN);
	 float jiaodiao[2][2]={0};
	 //�����ٶ�ֱ������Բ����
	 float temp=sqrt(pow(d_short,2)/(1+pow(d_short*tan_yaw/d_long,2)));
	 //�ж��ٶȷ��򽻵����
	 if(yaw+0.000011>0&&yaw+0.000011<180)
	 {jiaodiao[0][Xr]=temp; jiaodiao[1][Xr]=-temp;}
	 else 
	 {jiaodiao[0][Xr]=-temp;jiaodiao[1][Xr]=temp;}
	 
	 jiaodiao[0][Yr]=tan_yaw*jiaodiao[0][Xr];
	 jiaodiao[1][Yr]=tan_yaw*jiaodiao[1][Xr];
	 
	 jiaodiao[0][Xr]+=cx;jiaodiao[1][Xr]+=cx;
	 jiaodiao[0][Yr]+=cy;jiaodiao[1][Yr]+=cy;
	 
   *jiao1_x=jiaodiao[0][Xr];
	 *jiao1_y=jiaodiao[0][Yr];
	 *jiao2_x=jiaodiao[1][Xr];
	 *jiao2_y=jiaodiao[1][Yr];
}

static void swap(float *a, float *b)  
{  
    int     c;  
     c = *a;  
    *a = *b;  
    *b =  c;  
}  

//�������������
float cal_area_trig(float ax,float ay,float bx,float by,float cx,float cy)
{
float mx=cx-ax,my=cy-ay,nx=bx-ax,ny=by-ay;
float Lm= sqrt(mx*mx+my*my),Ln= sqrt(nx*nx+ny*ny),cosA=(mx*nx+my*ny)/Lm/Ln;
float sinA=sqrt(1-cosA*cosA);
float S_tri=0.5*Lm*Ln*sinA;
if(S_tri>=0) 
	return S_tri;
else  
	return (-1*S_tri);
}	

//һ�������������ڲ�
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
//���㵱ǰ��̬����
float cal_steady_s(float cx,float cy,float x1,float y1,float x2,float y2,float x3, float y3 )
{
 float ST;
 float k[3],b[3];
 float D[3],temp;
 u8 i,intrig;
	intrig=inTrig(cx,cy,x1,y1,x2,y2,x3,y3);
	if(intrig){
	if(x2==x1)
		x2+=0.001;
	if(x3==x1)
		x3+=0.001;
	
	line_function_from_two_point(x1,y1,x2,y2,&k[0],&b[0]);
	line_function_from_two_point(x3,y3,x2,y2,&k[1],&b[1]);
	line_function_from_two_point(x1,y1,x3,y3,&k[2],&b[2]);
	
  for(i=0;i<3;i++)
	D[i]=dis_point_to_line(cx,cy,k[i],b[i]);
	
	ST=D[0];
	for(i=1;i<3;i++)
	  if(D[i]<ST)
			 ST=D[i];
		
  if(ST>0)
   return ST;
  else 
   return 0;	
  }else
  return 0;	
}	

//���㵱ǰ��̬����4leg
float cal_steady_s4(float cx,float cy,float x1,float y1,float x2,float y2,float x3, float y3 ,float x4,float y4 )
{
 float ST;
 float k[4],b[4];
 float D[4],temp;
 u8 i,intrig;
	
	line_function_from_two_point(x1,y1,x2,y2,&k[0],&b[0]);
	line_function_from_two_point(x2,y2,x3,y3,&k[1],&b[1]);
	line_function_from_two_point(x3,y3,x4,y4,&k[2],&b[2]);
	line_function_from_two_point(x4,y4,x1,y1,&k[3],&b[3]);
  for(i=0;i<4;i++)
	D[i]=dis_point_to_line(cx,cy,k[i],b[i]);
	
	ST=D[0];
	for(i=1;i<4;i++)
	  if(D[i]<ST)
			 ST=D[i];
		
  if(ST>0)
   return ST;
  else 
   return 0;	
}	
//�ж�һ�������ı����ڲ�
//			  y
//	d----------b          /\
// 	     |                |
//			 O                L
//			 |                |
//	a----------c   x			\/
u8 segmentsIntr(POS b,POS c,POS d,POS a,float *x,float *y){  
  
/** 1 �����Է�����, ���߶ν���. **/  
// �����ĸΪ0 ��ƽ�л���, ���ཻ  
    float denominator = (b.y - a.y)*(d.x - c.x) - (a.x - b.x)*(c.y - d.y);  
    if (denominator==0) {  
        return 0;  
    }  
   
// �߶�����ֱ�ߵĽ������� (x , y)      
     *x = ( (b.x - a.x) * (d.x - c.x) * (c.y - a.y)   
                + (b.y - a.y) * (d.x - c.x) * a.x   
                - (d.y - c.y) * (b.x - a.x) * c.x ) / denominator ;  
     *y = -( (b.y - a.y) * (d.y - c.y) * (c.x - a.x)   
                + (b.x - a.x) * (d.y - c.y) * a.y   
                - (d.x - c.x) * (b.y - a.y) * c.y ) / denominator;  
  
/** 2 �жϽ����Ƿ��������߶��� **/  
    if (  
        // �������߶�1��  
        (*x - a.x) * (*x - b.x) <= 0 && (*y - a.y) * (*y - b.y) <= 0  
        // �ҽ���Ҳ���߶�2��  
         && (*x - c.x) * (*x - d.x) <= 0 && (*y - c.y) * (*y - d.y) <= 0  
        ){  
  
        // ���ؽ���p  
        return 1;
    }  
    //�����ཻ  
    return 0;  
  
}  
//һ�������ı����ڲ�
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

//�ҵ���xy����ĵ�
void find_closet_point(u8*min_id,float x, float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4,u8 num) {
	u8 i,j;
	float dis[4],dis_id[4]={0,1,2,3};
	  dis[0]=sqrt(pow(x-x1,2)+pow(y-y1,2));
	  dis[1]=sqrt(pow(x-x2,2)+pow(y-y2,2));
	  dis[2]=sqrt(pow(x-x3,2)+pow(y-y3,2));
	  dis[3]=sqrt(pow(x-x4,2)+pow(y-y4,2));
    for (i = 0; i < num; i++)  
    {  
        //ÿһ���ɵ����ϵ�����  
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


//----- DSE
/*-------------------------------------------------------
      Data Encryption Standard  56λ��Կ����64λ���� 
                  2011.10
--------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
/*�����ͱ���*/
//--------------------------------------------------------------
u8 license_check(char *data,char *key_lock,char * key_unlock,unsigned int num,u8 en);

LIS license;
char KEY_LOCK[9]={'1','2','3','4','5','6','7','8'}; 
char KEY_UNLOCK[9]={'1','2','3','4','5','6','7','8'}; 
char RX_BUF[125];
void des_test(char *data,char *key_lock,char * key_unlock,unsigned int num);
void cpuidGetId(void)
{  
    license.id[0]= *(__IO u32*)(0x1FFF7A10);
    license.id[1]= *(__IO u32*)(0x1FFF7A14);
    license.id[2]= *(__IO u32*)(0x1FFF7A18);
		RX_BUF[0]=license.id[0];
		RX_BUF[1]=license.id[1];
		RX_BUF[2]=license.id[2];
	
	  des_test(RX_BUF,KEY_LOCK,KEY_UNLOCK,125);
	  license.state=license_check(RX_BUF,KEY_LOCK,KEY_UNLOCK,125,1);
}


typedef enum
  {
    false = 0,
    true  = 1
  } bool;
// ������ִ��IP�û��õ�L0,R0 ��L��32λ,R��32λ��               [���Ĳ���]
static const char IP_Table[64]={             
	58,50,42,34,26,18,10, 2,60,52,44,36,28,20,12, 4,
	62,54,46,38,30,22,14, 6,64,56,48,40,32,24,16, 8,
	57,49,41,33,25,17, 9, 1,59,51,43,35,27,19,11, 3,
	61,53,45,37,29,21,13, 5,63,55,47,39,31,23,15, 7 
};

// �Ե������L16,R16ִ��IP���û�,�������
static const char IPR_Table[64]={              
	40, 8,48,16,56,24,64,32,39, 7,47,15,55,23,63,31,
	38, 6,46,14,54,22,62,30,37, 5,45,13,53,21,61,29,
	36, 4,44,12,52,20,60,28,35, 3,43,11,51,19,59,27,
	34, 2,42,10,50,18,58,26,33, 1,41, 9,49,17,57,25	
};

/*--------------------------- �������� ----------------------------*/ 

// F����,32λ��R0����E�任,��Ϊ48λ��� (R1~R16)        [����A]  [���Ĳ���] 
static char E_Table[48]={
	32, 1, 2, 3, 4, 5, 4, 5, 6, 7, 8, 9,
	 8, 9,10,11,12,13,12,13,14,15,16,17,
    16,17,18,19,20,21,20,21,22,23,24,25,
    24,25,26,27,28,29,28,29,30,31,32, 1
};

// ����ԿK(i)�Ļ�ȡ ��ԿΪK ������6,16,24,32,40,48,64λ          [��Կ����] 
// ��PC1ѡλ ��Ϊ ǰ28λC0,��28λD0 ������  
static char PC1_Table[56]={
	57,49,41,33,25,17, 9, 1,58,50,42,34,26,18,
	10, 2,59,51,43,35,27,19,11, 3,60,52,44,36,
	63,55,47,39,31,23,15, 7,62,54,46,38,30,22,
	14, 6,61,53,45,37,29,21,13, 5,28,20,12, 4
};

// ��C0,D0�ֱ��������,��16��,����λ���������Ӧ                 [��Կ����]
static char Move_Table[16]={
	 1, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1
};

// C1,D1Ϊ��һ�����ƺ�õ�,����PC2ѡλ,�õ�48λ���K1   [����B]   [��Կ����]     
static char PC2_Table[48]={
	14,17,11,24, 1, 5, 3,28,15, 6,21,10,
	23,19,12, 4,26, 8,16, 7,27,20,13, 2,
	41,52,31,37,47,55,30,40,51,34,33,48,
	44,49,39,56,34,53,46,42,50,36,29,32	
};

/*------------- F���� ����A�ͱ���B ��� �õ�48λ��� ---------------*/ 

// ����Ľ��48λ��Ϊ8��,ÿ��6λ,��Ϊ8��S�е�����             [��ϲ���] 
// S����6λ��Ϊ����(8��),4λ��Ϊ���(4*(8��)=32λ)
// S����ԭ�� ��������ΪA=abcdef ,��bcde�����������0-15֮���
// һ������Ϊ X=bcde ,af�������0-3֮���һ����,��Ϊ Y=af 
// ��S1��X��,Y���ҵ�һ����Value,����0-15֮��,�����ö����Ʊ�ʾ
// ����Ϊ4bit (��32λ)  
static char S_Box[8][4][16]={
	//S1
	14, 4,13, 1, 2,15,11, 8, 3,10, 6,12, 5, 9, 0, 7,
	 0,15, 7, 4,14, 2,13, 1,10, 6,12,11, 9, 5, 3, 8,
	 4, 1,14, 8,13, 6, 2,11,15,12, 9, 7, 3,10, 5, 0,
	15,12, 8, 2, 4, 9, 1, 7, 5,11, 3,14,10, 0, 6,13,
	//S2
	15, 1, 8,14, 6,11, 3, 4, 9, 7, 2,13,12, 0, 5,10,
	 3,13, 4, 7,15, 2, 8,14,12, 0, 1,10, 6, 9,11, 5,
	 0,14, 7,11,10, 4,13, 1, 5, 8,12, 6, 9, 3, 2,15,
	13, 8,10, 1, 3,15, 4, 2,11, 6, 7,12, 0, 5,14, 9,
	//S3
	10, 0, 9,14, 6, 3,15, 5, 1,13,12, 7,11, 4, 2, 8,
	13, 7, 0, 9, 3, 4, 6,10, 2, 8, 5,14,12,11,15, 1,
	13, 6, 4, 9, 8,15, 3, 0,11, 1, 2,12, 5,10,14, 7,
	 1,10,13, 0, 6, 9, 8, 7, 4,15,14, 3,11, 5, 2,12,
	//S4
	 7,13,14, 3, 0, 6, 9,10, 1, 2, 8, 5,11,12, 4,15,
	13, 8,11, 5, 6,15, 0, 3, 4, 7, 2,12, 1,10,14, 9,
	10, 6, 9, 0,12,11, 7,13,15, 1, 3,14, 5, 2, 8, 4,
	 3,15, 0, 6,10, 1,13, 8, 9, 4, 5,11,12, 7, 2,14,
	//S5
	 2,12, 4, 1, 7,10,11, 6, 8, 5, 3,15,13, 0,14, 9,
	14,11, 2,12, 4, 7,13, 1, 5, 0,15,10, 3, 9, 8, 6,
	 4, 2, 1,11,10,13, 7, 8,15, 9,12, 5, 6, 3, 0,14,
	11, 8,12, 7, 1,14, 2,13, 6,15, 0, 9,10, 4, 5, 3,
	//S6
	12, 1,10,15, 9, 2, 6, 8, 0,13, 3, 4,14, 7, 5,11,
	10,15, 4, 2, 7,12, 0, 5, 6, 1,13,14, 0,11, 3, 8,
	 9,14,15, 5, 2, 8,12, 3, 7, 0, 4,10, 1,13,11, 6,
   	 4, 3, 2,12, 9, 5,15,10,11,14, 1, 7, 6, 0, 8,13,
	//S7
	 4,11, 2,14,15, 0, 8,13, 3,12, 9, 7, 5,10, 6, 1,
	13, 0,11, 7, 4, 0, 1,10,14, 3, 5,12, 2,15, 8, 6,
	 1, 4,11,13,12, 3, 7,14,10,15, 6, 8, 0, 5, 9, 2,
	 6,11,13, 8, 1, 4,10, 7, 9, 5, 0,15,14, 2, 3,12,
	//S8
	13, 2, 8, 4, 6,15,11, 1,10, 9, 3,14, 5, 0,12, 7,
	 1,15,13, 8,10, 3, 7, 4,12, 5, 6,11, 0,14, 9, 2,
	 7,11, 4, 1, 9,12,14, 2, 0, 6,10,13,15, 3, 5, 8,
	 2, 1,14, 7, 4,10, 8,13,15,12, 9, 0, 3, 5, 6,11
};

// F���� ���ڶ���,��S�������32����P�û�                     [��ϲ���]
// �����ֵ����һ�ε���:
// L(i)=R(i-1)
// R(i)=L(i-1)^f(R(i-1),K(i)) ��� 
static char P_Table[32]={
	16, 7,20,21,29,12,28,17, 1,15,23,26, 5,18,31,10,
	 2, 8,24,14,32,27, 3, 9,19,13,30, 6,22,11, 4,25
};

// 16������ԿK(1~16) 
static bool SubKey[16][48]={0};                              
void SetKey(char KeyIn[8]);                         // ������Կ
void PlayDes(char MesOut[8],char MesIn[8]);       // ִ��DES����
void KickDes(char MesOut[8],char MesIn[8]);             // ִ��DES���� 
char MesHex_OUT[128]={0};         // 16���ַ��������ڴ�� 64λ16���Ƶ�����
char MyMessage_Kick[125]={0};
void des_test(char *data,char *key_lock,char * key_unlock,unsigned int num)
{
    u8 i=0; 
    char MesHex[16]={0};         // 16���ַ��������ڴ�� 64λ16���Ƶ�����
		char MesHexr[16]={0};
    char MyKey[8]={0};           // ��ʼ��Կ 8�ֽ�*8
    char YourKey[8]={0};         // ����Ľ�����Կ 8�ֽ�*8
    char MyMessage[8]={0};       // ��ʼ���� 
    char MyMessage_OUT[8]={0};       // ��ʼ���� 
/*-----------------------------------------------*/
    u16 group,group_ero;
		u16 j=0,k=0,k1=0,l=0,l1=0;		
		group_ero=num%8;
		group=num/8;
		if(group_ero!=0)
    group++;
		
		for (i=0;i<8;i++){MyKey[i]=key_lock[i];YourKey[i]=key_unlock[i];}//copy key
		
		SetKey(MyKey);               // set key master

		for(j=0;j<group;j++)//lock
		{
		for (i=0;i<8;i++)
			MesHex[i]=0;//clear hex_buf
			
	  if(group_ero!=0&&j==group-2)
		{for (i=0;i<group_ero;i++)
			MyMessage[i]=data[l++];//copy group	
		 for (i=group_ero;i<8;i++)
			MyMessage[i]=0x00;//copy group	
		}
		else
		for (i=0;i<8;i++)
		MyMessage[i]=data[l++];//copy group
			
		PlayDes(MesHex,MyMessage);   // DSE
			
		for (i=0;i<16;i++)
		MesHex_OUT[k++]=MesHex[i];//out hex	
			
    }
    SetKey(YourKey);             // set key slave
  	for(j=0;j<group;j++)//unlock
		{
			for (i=0;i<16;i++)
			MesHexr[i]=MesHex_OUT[k1++];//out hex	
       KickDes(MyMessage_OUT,MesHexr);                     // ���������MyMessage   
      for (i=0;i<8;i++)
			MyMessage_Kick[l1++]=MyMessage_OUT[i];//copy group
		}
}


u8 license_check(char *data,char *key_lock,char * key_unlock,unsigned int num,u8 en)
{
     u8 i=0; 
    char MesHex[16]={0};         // 16���ַ��������ڴ�� 64λ16���Ƶ�����
		char MesHexr[16]={0};
    char MyKey[8]={0};           // ��ʼ��Կ 8�ֽ�*8
    char YourKey[8]={0};         // ����Ľ�����Կ 8�ֽ�*8
    char MyMessage[8]={0};       // ��ʼ���� 
    char MyMessage_OUT[8]={0};       // ��ʼ���� 
/*-----------------------------------------------*/
    u16 group,group_ero;
		u16 j=0,k=0,k1=0,l=0,l1=0;		
		group_ero=num%8;
		group=num/8;
		if(group_ero!=0)
    group++;
		
		for (i=0;i<8;i++){MyKey[i]=key_lock[i];YourKey[i]=key_unlock[i];}//copy key
		
		SetKey(MyKey);               // set key master

		for(j=0;j<group;j++)//lock
		{
		for (i=0;i<8;i++)
			MesHex[i]=0;//clear hex_buf
			
	  if(group_ero!=0&&j==group-2)
		{for (i=0;i<group_ero;i++)
			MyMessage[i]=data[l++];//copy group	
		 for (i=group_ero;i<8;i++)
			MyMessage[i]=0x00;//copy group	
		}
		else
		for (i=0;i<8;i++)
		MyMessage[i]=data[l++];//copy group
			
		PlayDes(MesHex,MyMessage);   // DSE
			
		for (i=0;i<16;i++)
		MesHex_OUT[k++]=MesHex[i];//out hex	
			
    }
    SetKey(YourKey);             // set key slave
  	for(j=0;j<group;j++)//unlock
		{
			for (i=0;i<16;i++)
			MesHexr[i]=MesHex_OUT[k1++];//out hex	
       KickDes(MyMessage_OUT,MesHexr);                     // ���������MyMessage   
      for (i=0;i<8;i++)
			MyMessage_Kick[l1++]=MyMessage_OUT[i];//copy group
		}
		
		for(i=0;i<num;i++)
		  if(MyMessage_Kick[i]!=MesHex_OUT[i])
				 return 0;
			
		return 1;	
}
/*-------------------------------
 ��DatIn��ʼ�ĳ���λLenλ�Ķ�����
 ���Ƶ�DatOut��
--------------------------------*/
void BitsCopy(bool *DatOut,bool *DatIn,int Len)     // ���鸴�� OK 
{
    int i=0;
    for(i=0;i<Len;i++)
    {
        DatOut[i]=DatIn[i];
    }
}

/*-------------------------------
 �ֽ�ת����λ���� 
 ÿ8�λ�һ���ֽ� ÿ��������һλ
 ��1��ȡ���һλ ��64λ 
--------------------------------*/
void ByteToBit(bool *DatOut,char *DatIn,int Num)       // OK
{
    int i=0;
    for(i=0;i<Num;i++)
    {
        DatOut[i]=(DatIn[i/8]>>(i%8))&0x01;   
    }                                       
}

/*-------------------------------
 λת�����ֽں���
 �ֽ�����ÿ8����һλ
 λÿ�������� ����һ�λ�   
---------------------------------*/
void BitToByte(char *DatOut,bool *DatIn,int Num)        // OK
{
    int i=0;
    for(i=0;i<(Num/8);i++)
    {
        DatOut[i]=0;
    } 
    for(i=0;i<Num;i++)
    {
        DatOut[i/8]|=DatIn[i]<<(i%8);    
    }        
}


/*----------------------------------
 ����������ת��Ϊʮ������
 ��Ҫ16���ַ���ʾ
-----------------------------------*/
void BitToHex(char *DatOut,bool *DatIn,int Num)
{
    int i=0;
    for(i=0;i<Num/4;i++)
    {
        DatOut[i]=0;
    }
    for(i=0;i<Num/4;i++)
    {
        DatOut[i] = DatIn[i*4]+(DatIn[i*4+1]<<1)
                    +(DatIn[i*4+2]<<2)+(DatIn[i*4+3]<<3);
        if((DatOut[i]%16)>9)
        {
            DatOut[i]=DatOut[i]%16+'7';       //  ��������9ʱ���� 10-15 to A-F
        }                                     //  ����ַ� 
        else
        {
            DatOut[i]=DatOut[i]%16+'0';       //  ����ַ�       
        }
    }
    
}

/*---------------------------------------------
 ʮ�������ַ�ת������
----------------------------------------------*/
void HexToBit(bool *DatOut,char *DatIn,int Num)
{
    int i=0;                        // �ַ������� 
    for(i=0;i<Num;i++)
    {
        if((DatIn[i/4])>'9')         //  ����9 
        {
            DatOut[i]=((DatIn[i/4]-'7')>>(i%4))&0x01;               
        }
        else
        {
            DatOut[i]=((DatIn[i/4]-'0')>>(i%4))&0x01;     
        } 
    }    
}

// ���û�����  OK
void TablePermute(bool *DatOut,bool *DatIn,const char *Table,int Num)  
{
    int i=0;
    static bool Temp[256]={0};
    for(i=0;i<Num;i++)                // NumΪ�û��ĳ��� 
    {
        Temp[i]=DatIn[Table[i]-1];  // ԭ�������ݰ���Ӧ�ı��ϵ�λ������ 
    }
    BitsCopy(DatOut,Temp,Num);       // �ѻ���Temp��ֵ��� 
} 

// ����Կ����λ
void LoopMove(bool *DatIn,int Len,int Num) // ѭ������ Len���ݳ��� Num�ƶ�λ��
{
    static bool Temp[256]={0};    // ����   OK
    BitsCopy(Temp,DatIn,Num);       // ����������ߵ�Numλ(���Ƴ�ȥ��)����Temp 
    BitsCopy(DatIn,DatIn+Num,Len-Num); // ��������߿�ʼ�ĵ�Num����ԭ���Ŀռ�
    BitsCopy(DatIn+Len-Num,Temp,Num);  // ���������Ƴ�ȥ�����ݼӵ����ұ� 
} 

// ��λ���
void Xor(bool *DatA,bool *DatB,int Num)           // �����
{
    int i=0;
    for(i=0;i<Num;i++)
    {
        DatA[i]=DatA[i]^DatB[i];                  // ��� 
    }
} 

// ����48λ ���32λ ��Ri���
void S_Change(bool DatOut[32],bool DatIn[48])     // S�б任
{
    int i,X,Y;                                    // iΪ8��S�� 
    for(i=0,Y=0,X=0;i<8;i++,DatIn+=6,DatOut+=4)   // ÿִ��һ��,��������ƫ��6λ 
    {                                              // ÿִ��һ��,�������ƫ��4λ
        Y=(DatIn[0]<<1)+DatIn[5];                          // af����ڼ���
        X=(DatIn[1]<<3)+(DatIn[2]<<2)+(DatIn[3]<<1)+DatIn[4]; // bcde����ڼ���
        ByteToBit(DatOut,&S_Box[i][Y][X],4);      // ���ҵ��ĵ����ݻ�Ϊ������    
    }
}

// F����
void F_Change(bool DatIn[32],bool DatKi[48])       // F����
{
    static bool MiR[48]={0};             // ����32λͨ��Eѡλ��Ϊ48λ
    TablePermute(MiR,DatIn,E_Table,48); 
    Xor(MiR,DatKi,48);                   // ������Կ���
    S_Change(DatIn,MiR);                 // S�б任
    TablePermute(DatIn,DatIn,P_Table,32);   // P�û������
}



void SetKey(char KeyIn[8])               // ������Կ ��ȡ����ԿKi 
{
    int i=0;
    static bool KeyBit[64]={0};                // ��Կ�����ƴ洢�ռ� 
    static bool *KiL=&KeyBit[0],*KiR=&KeyBit[28];  // ǰ28,��28��56
    ByteToBit(KeyBit,KeyIn,64);                    // ����ԿתΪ�����ƴ���KeyBit 
    TablePermute(KeyBit,KeyBit,PC1_Table,56);      // PC1���û� 56��
    for(i=0;i<16;i++)
    {
        LoopMove(KiL,28,Move_Table[i]);       // ǰ28λ���� 
        LoopMove(KiR,28,Move_Table[i]);          // ��28λ���� 
         TablePermute(SubKey[i],KeyBit,PC2_Table,48); 
         // ��ά���� SubKey[i]Ϊÿһ����ʼ��ַ 
         // ÿ��һ��λ����PC2�û��� Ki 48λ 
    }        
}

void PlayDes(char MesOut[8],char MesIn[8])  // ִ��DES����
{                                           // �ֽ����� Bin���� Hex��� 
    int i=0;
    static bool MesBit[64]={0};        // ���Ķ����ƴ洢�ռ� 64λ
    static bool Temp[32]={0};
    static bool *MiL=&MesBit[0],*MiR=&MesBit[32]; // ǰ32λ ��32λ 
    ByteToBit(MesBit,MesIn,64);                 // �����Ļ��ɶ����ƴ���MesBit
    TablePermute(MesBit,MesBit,IP_Table,64);    // IP�û� 
    for(i=0;i<16;i++)                       // ����16�� 
    {
        BitsCopy(Temp,MiR,32);            // ��ʱ�洢
        F_Change(MiR,SubKey[i]);           // F�����任
        Xor(MiR,MiL,32);                  // �õ�Ri 
        BitsCopy(MiL,Temp,32);            // �õ�Li 
    }                    
    TablePermute(MesBit,MesBit,IPR_Table,64);
    BitToHex(MesOut,MesBit,64);
}

void KickDes(char MesOut[8],char MesIn[8])       // ִ��DES����
{                                                // Hex���� Bin���� �ֽ���� 
    int i=0;
    static bool MesBit[64]={0};        // ���Ķ����ƴ洢�ռ� 64λ
    static bool Temp[32]={0};
    static bool *MiL=&MesBit[0],*MiR=&MesBit[32]; // ǰ32λ ��32λ
    HexToBit(MesBit,MesIn,64);                 // �����Ļ��ɶ����ƴ���MesBit
    TablePermute(MesBit,MesBit,IP_Table,64);    // IP�û� 
    for(i=15;i>=0;i--)
    {
        BitsCopy(Temp,MiL,32);
        F_Change(MiL,SubKey[i]);
        Xor(MiL,MiR,32);
        BitsCopy(MiR,Temp,32);
    }    
    TablePermute(MesBit,MesBit,IPR_Table,64);
    BitToByte(MesOut,MesBit,64);        
} 
