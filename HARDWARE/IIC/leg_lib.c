#include "include.h" 


//转换腿局部坐标系到全局机体坐标系
void conver_legpos_to_barin(BRAIN_STRUCT *in,LEG_STRUCT * inl,u8 id)
{ u8 i;
	for(i=0;i<3;i++){
	inl->pos_now_brain[i].x=inl->pos_now[i].x+in->sys.leg_local[id].x;
  inl->pos_now_brain[i].y=inl->pos_now[i].y+in->sys.leg_local[id].y;
	inl->pos_now_brain[i].z=inl->pos_now[i].z+in->sys.leg_local[id].z;
	}
}
//判断脚到达初始化的位置
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
//跨腿重复保护器
u8 leg_repeat_protect(u8 id,u8 last_move_id,u8 last_last_move_id,float yaw,float yaw_trig){
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
						return trig_list_r[1];
						else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)
						return trig_list_l[1];
						else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))
						return trig_list_f[1];
						else
						return trig_list_b[1];
					else
						if(yaw_in>yaw_temp&&yaw_in<90+yaw_temp)
						return trig_list_r[i+1];
						else if(yaw_in<-yaw_temp&&yaw_in>-90-yaw_temp)
						return trig_list_l[i+1];
						else if((yaw_in<yaw_temp&&yaw_in>=0)||(yaw_in>-yaw_temp&&yaw_in<0))
						return trig_list_f[i+1];
						else
						return trig_list_b[i+1];
				}else	
        return id;
}

//跨腿重复保护器
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
					   if(i!=last_move_id||i!=last_last_move_id)
						 return id=i;
					 }
				}else	
        return id;
}

//点沿矢量对称缩放
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
	
*nx=cro_x+(cro_x-x)*k;	
*ny=cro_y+(cro_y-y)*k;	
}	
}

//两点求直线方程
void line_function_from_two_point(float x1,float y1,float x2,float y2,float *k,float *b)
{ 
	float k_temp=0;
  *k=k_temp=(y1-y2)/(x1-x2+0.000001);
  *b=y1-k_temp*x1;
}	

//矢量求直线方程
void line_function_from_arrow(float x,float y,float yaw,float *k,float *b)
{ 
	float tyaw=90-yaw+0.000011;
	float k_temp=0;
  *k=k_temp=tan(tyaw/57.3);
  *b=y-k_temp*x;
}	

//矢量垂线方程
void line_function90_from_arrow(float x,float y,float yaw,float *k,float *b)
{ 
	float tyaw=90-yaw+0.000011;
	float k_temp=0;
  *k=k_temp=-1/tan(tyaw/57.3);
  *b=y-k_temp*x;
}	

//点在直线上
u8 check_point_on_line(float x,float y,float k,float b,float err)
{ 
	float temp=k*x+b-y;
	if(ABS(temp)<err)
		return 1;
	else
		return 0;
}	

//两直线交点
u8 cross_point_of_lines(float k1,float b1,float k2,float b2,float *x,float *y)
{ 
	if(ABS(k1-k2)<0.001){
		*x=*y=0;
		return 0;}
	float x_temp;
	*x=x_temp=(b1-b2)/(k2-k1+0.00001);
	*y=k1*x_temp+b1;
	
	return 1;
}	

//点在点矢量方向前
u8 check_point_front_arrow(float x,float y,float cx,float cy,float yaw)
{ 
  float tyaw=90-yaw+0.000011;
	float kc_90=-1/tan(tyaw/57.3);
	float bc_90=cy-kc_90*cx;
	float cx_t=cx+sin(yaw/57.3)*1,cy_t=cy+cos(yaw/57.3)*1;
	int flag[2];
	flag[0]=kc_90*cx_t+bc_90-cy_t;
	flag[1]=kc_90*x+bc_90-y;
	if(flag[0]*flag[1]>0)
	return 1;
	else 
  return 0;
}	

//判断两点在线同一侧
u8 check_points_same_side(float x1,float y1,float x2,float y2,float k,float b)
{ 
	int flag[2];
	flag[0]=k*x1+b-y1;
	flag[1]=k*x2+b-y2;
	if(flag[0]*flag[1]>0)
	return 1;
	else 
  return 0;
}	

//点矢量与直线交点
u8 check_cross_arrow_line(float cx,float cy,float yaw,float k,float b,float *x,float *y)
{ 
  float tyaw=90-yaw+0.000011;
	float kc=tan(tyaw/57.3);
	float bc=cy-kc*cx;
	float cro_x,cro_y;
	u8 flag;	
	flag=cross_point_of_lines(k,b,kc,bc,&cro_x,&cro_y);
	*x=cro_x;
	*y=cro_y;
	
	if(flag==0||fabs(cro_y)>100||fabs(cro_x)>100)
		return 0;
	//有交点且在前方
	if(check_point_front_arrow(cro_x,cro_y, cx, cy, yaw))	
	return 1;
	else{
	*x=*y=0;	
	return 0;
	}
}	

//点矢量垂线与直线交点
u8 check_cross_arrow90_line(float cx,float cy,float yaw,float k,float b,float *x,float *y)
{ 
  float tyaw=90-yaw+0.000011;
	float kc=tan(tyaw/57.3);
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
//计算三角形重心坐标
void cal_center_of_trig(float x1,float y1,float x2,float y2,float x3,float x4,float *cx,float *cy)
{

}	

//计算两点距离
float cal_dis_of_points(float x1,float y1,float x2,float y2)
{
return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}	
//判断一个点在椭圆内部
u8 in_circle(float cx,float cy,float d_short,float d_long,float x,float y)
{
   float temp=pow(x-cx,2)/pow(d_short,2)+pow(y-cy,2)/pow(d_long,2);
   if(temp>1)//外面
		 return 0;
	 else 
		 return 1;
}

//点到直线距离
float dis_point_to_line(float x,float y,float k,float b)
{ 
  float k_90=-1/(k+0.000011);
	float b_90=y-k_90*x;
	float cx,cy;
	cross_point_of_lines(k,b,k_90,b_90,&cx,&cy);
	
	return cal_dis_of_points(x,y,cx,cy);
}	

//将机体坐标转换到全局坐标
void conver_body_to_global(float bx,float by,float *gx,float *gy)
{
 *gx=bx+brain.sys.leg_local[1].x-leg[4].pos_now[2].x+brain.sys.off_cor[Xr];
 *gy=by+brain.sys.leg_local[1].y-leg[4].pos_now[2].y+brain.sys.off_cor[Yr];
}

//移动区域限幅度 方框
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

//判断点在移动区域内部tangle
u8 check_in_move_range_tangle(u8 id,float x,float y,float cx,float cy,float min,float max)
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
	 
	 if(flag[0])
	  return 1;
	 else 
		return 0;
}

//判断点在移动区域内部
u8 check_in_move_range(u8 id,float x,float y,float cx,float cy,float min,float max)
{
		float tangle[4][2];
		float tar_x,tar_y;
		u8 flag[3];
	 //test 
//	 id=1;
//	 x=2;
//	 y=2;
//	 cx=0;
//	 cy=0;
//	 min=1;
//	 max=2;
	 //
	
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


//计算移动区域与矢量的两个交点
void cal_jiao_of_range_and_line(u8 id,float cx,float cy,float min,float max,float yaw,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y)
{

	 float jiaodiao[2][2]={0};
   float yaw_use;	
	 float tangle[4][2];
	 float tar_x=0,tar_y=0;
	 u8 flag[3];
	 //test 
//	 id=4;
//	 cx=0;
//	 cy=0;
//	 min=1;
//	 max=2;
//	 yaw=-135;
	 //
		 
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


//计算移动区域与矢量的两个交点 方框
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

//点矢量与四边形交点
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
	


//点矢量与三角形交点
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

//计算椭圆与矢量的两个交点
void cal_jiao_of_tuo_and_line(float cx,float cy,float d_short,float d_long,float yaw,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y)
{
   float tyaw=90-yaw+0.000011;
	 float tan_yaw=tan(tyaw/57.3);
	 float jiaodiao[2][2]={0};
	 //计算速度直线与椭圆交点
	 float temp=sqrt(pow(d_short,2)/(1+pow(d_short*tan_yaw/d_long,2)));
	 //判断速度方向交点符号
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

//计算三角形面积
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

//一个点在三角形内部
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
//计算当前稳态余量
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

//计算当前稳态余量4leg
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
//判断一个点在四边形内部
//			  y
//	d----------b          /\
// 	     |                |
//			 O                L
//			 |                |
//	a----------c   x			\/
u8 segmentsIntr(POS b,POS c,POS d,POS a,float *x,float *y){  
  
/** 1 解线性方程组, 求线段交点. **/  
// 如果分母为0 则平行或共线, 不相交  
    float denominator = (b.y - a.y)*(d.x - c.x) - (a.x - b.x)*(c.y - d.y);  
    if (denominator==0) {  
        return 0;  
    }  
   
// 线段所在直线的交点坐标 (x , y)      
     *x = ( (b.x - a.x) * (d.x - c.x) * (c.y - a.y)   
                + (b.y - a.y) * (d.x - c.x) * a.x   
                - (d.y - c.y) * (b.x - a.x) * c.x ) / denominator ;  
     *y = -( (b.y - a.y) * (d.y - c.y) * (c.x - a.x)   
                + (b.x - a.x) * (d.y - c.y) * a.y   
                - (d.x - c.x) * (b.y - a.y) * c.y ) / denominator;  
  
/** 2 判断交点是否在两条线段上 **/  
    if (  
        // 交点在线段1上  
        (*x - a.x) * (*x - b.x) <= 0 && (*y - a.y) * (*y - b.y) <= 0  
        // 且交点也在线段2上  
         && (*x - c.x) * (*x - d.x) <= 0 && (*y - c.y) * (*y - d.y) <= 0  
        ){  
  
        // 返回交点p  
        return 1;
    }  
    //否则不相交  
    return 0;  
  
}  
//一个点在四边形内部
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

//找到离xy最近的点
void find_closet_point(u8*min_id,float x, float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4,u8 num) {
	u8 i,j;
	float dis[4],dis_id[4]={0,1,2,3};
	  dis[0]=sqrt(pow(x-x1,2)+pow(y-y1,2));
	  dis[1]=sqrt(pow(x-x2,2)+pow(y-y2,2));
	  dis[2]=sqrt(pow(x-x3,2)+pow(y-y3,2));
	  dis[3]=sqrt(pow(x-x4,2)+pow(y-y4,2));
    for (i = 0; i < num; i++)  
    {  
        //每一次由底至上地上升  
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