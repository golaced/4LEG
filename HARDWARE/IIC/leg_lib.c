#include "include.h" 


//ת���Ⱦֲ�����ϵ��ȫ�ֻ�������ϵ
void conver_legpos_to_barin(BRAIN_STRUCT *in,LEG_STRUCT * inl,u8 id)
{ u8 i;
	for(i=0;i<3;i++){
	inl->pos_now_brain[i].x=inl->pos_now[i].x+in->sys.leg_local[id].x;
  inl->pos_now_brain[i].y=inl->pos_now[i].y+in->sys.leg_local[id].y;
	inl->pos_now_brain[i].z=inl->pos_now[i].z+in->sys.leg_local[id].z;
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
	float tyaw=90-yaw+0.0011;
	float k_temp=0;
  *k=k_temp=tan(tyaw/57.3);
  *b=y-k_temp*x;
}	

//ʸ�����߷���
void line_function90_from_arrow(float x,float y,float yaw,float *k,float *b)
{ 
	float tyaw=90-yaw+0.0011;
	float k_temp=0;
  *k=k_temp=-1/tan(tyaw/57.3);
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
	*y=k1*x_temp+b1;
	
	return 1;
}	

//���ڵ�ʸ������ǰ
u8 check_point_front_arrow(float x,float y,float cx,float cy,float yaw)
{ 
  float tyaw=90-yaw+0.0011;
	float kc_90=-1/tan(tyaw/57.3);
	float bc_90=cy-kc_90*cx;
	float cx_t=cx+sin(yaw/57.3),cy_t=cy+cos(yaw/57.3);
	int flag[2];
	flag[0]=kc_90*cx_t+bc_90-cy_t;
	flag[1]=kc_90*x+bc_90-y;
	if(flag[0]*flag[1]>0)
	return 1;
	else 
  return 0;
}	

//��ʸ����ֱ�߽���
u8 check_cross_arrow_line(float cx,float cy,float yaw,float k,float b,float *x,float *y)
{ 
  float tyaw=90-yaw+0.0011;
	float kc=tan(tyaw/57.3);
	float bc=cy-kc*cx;
	float cro_x,cro_y;
	u8 flag;	
	flag=cross_point_of_lines(k,b,kc,bc,&cro_x,&cro_y);
	*x=cro_x;
	*y=cro_y;
	
	if(flag==0)
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
  float tyaw=90-yaw+0.0011;
	float kc=tan(tyaw/57.3);
	float kc_90=-1/kc;
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
  float k_90=-1/(k+0.0011);
	float b_90=y-k_90*x;
	float cx,cy;
	cross_point_of_lines(k,b,k_90,b_90,&cx,&cy);
	
	return cal_dis_of_points(x,y,cx,cy);
}	

//����������ת����ȫ������
void conver_body_to_global(float bx,float by,float *gx,float *gy)
{
 *gx=bx+brain.sys.leg_local[1].x-leg[4].pos_now[2].x;
 *gy=by+brain.sys.leg_local[1].y-leg[4].pos_now[2].y;
}
//������Բ��ʸ���������Ƕ�
void cal_jiao_of_tuo_and_line(float cx,float cy,float d_short,float d_long,float yaw,float *jiao1_x,float *jiao1_y,float *jiao2_x,float *jiao2_y)
{
   float tyaw=90-yaw+0.0011;
	 float tan_yaw=tan(tyaw/57.3);
	 float jiaodiao[2][2]={0};
	 //�����ٶ�ֱ������Բ����
	 float temp=sqrt(pow(d_short,2)/(1+pow(d_short*tan_yaw/d_long,2)));
	 //�ж��ٶȷ��򽻵����
	 if(yaw+0.0011>0&&yaw+0.0011<180)
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