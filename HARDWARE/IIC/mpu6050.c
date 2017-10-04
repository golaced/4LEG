#include "include.h"
#include "mpu6050.h"
#include "imu.h"
#include "mpu9250.h"
u8 acc_3d_step;
MPU6050_STRUCT mpu6050,mpu6050_fc;

u8 mpu6050_buffer[14];
u8 mpu6050_ok;
void MPU6050_Read(void)
{
    MPU9250_ReadValue();
}

#include "cycle_cal_oldx.h"
s32 sum_temp[7]= {0,0,0,0,0,0,0};
float sum_temp_att[2]={0};
s32 sum_temp_3d[7]= {0,0,0,0,0,0,0};
u16 acc_sum_cnt = 0,acc_sum_cnt_3d=0,acc_smple_cnt_3d=0,gyro_sum_cnt = 0;
#define OFFSET_AV_NUM_ACC 50
void MPU6050_Data_Offset()
{


    if(mpu6050_fc.Acc_CALIBRATE == 1)
    {
       

        acc_sum_cnt++;
				if(mpu6050_fc.Cali_3d){
				  sum_temp_att[0]+=Pitch;
					sum_temp_att[1]+=Roll;
				}
				
				{
        sum_temp[A_X] += mpu6050_fc.Acc_I16.x;
        sum_temp[A_Y] += mpu6050_fc.Acc_I16.y;
        sum_temp[A_Z] += mpu6050_fc.Acc_I16.z - 65536/16;   // +-8G
				}
        sum_temp[TEM] += mpu6050_fc.Tempreature;

        if( acc_sum_cnt >= OFFSET_AV_NUM )
        {   
            mpu6050_fc.Acc_Offset.x = sum_temp[A_X]/OFFSET_AV_NUM;
            mpu6050_fc.Acc_Offset.y = sum_temp[A_Y]/OFFSET_AV_NUM;
            mpu6050_fc.Acc_Offset.z = sum_temp[A_Z]/OFFSET_AV_NUM;
					  mpu6050_fc.att_off[0]=(float)sum_temp_att[0]/OFFSET_AV_NUM;
					  mpu6050_fc.att_off[1]=(float)sum_temp_att[1]/OFFSET_AV_NUM;
            mpu6050_fc.Acc_Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
            acc_sum_cnt =0;
            mpu6050_fc.Acc_CALIBRATE = 0;
            WRITE_PARM();
            sum_temp[A_X] = sum_temp[A_Y] = sum_temp[A_Z] = sum_temp[TEM] = 0;
				  	sum_temp_att[1]=sum_temp_att[0]=0;
        }
    }
// 3d cal
		static xyz_f_t ACC_Reg;
		static u8 acc_3d_step_reg;
		float sphere_x,sphere_y,sphere_z,sphere_r;
		switch(acc_3d_step)
			{ 
			case 0:
				acc_smple_cnt_3d=acc_sum_cnt_3d=sum_temp_3d[A_X] = sum_temp_3d[A_Y] = sum_temp_3d[A_Z] = sum_temp_3d[TEM] = 0;
				cycle_init_oldx(&hml_lsq);
			break;
			default:
			if(hml_lsq.size<360&&(fabs(ACC_Reg.x-mpu6050_fc.Acc_I16.x)>0||fabs(ACC_Reg.y-mpu6050_fc.Acc_I16.y)>0||fabs(ACC_Reg.z-mpu6050_fc.Acc_I16.z)>0))
			{
			if(acc_3d_step>acc_3d_step_reg)
			acc_smple_cnt_3d=acc_sum_cnt_3d=sum_temp_3d[A_X] = sum_temp_3d[A_Y] = sum_temp_3d[A_Z] = sum_temp_3d[TEM] = 0;		
			acc_sum_cnt_3d++;
      sum_temp_3d[A_X] += mpu6050_fc.Acc_I16.x;
      sum_temp_3d[A_Y] += mpu6050_fc.Acc_I16.y;
      sum_temp_3d[A_Z] += mpu6050_fc.Acc_I16.z;   
				if(acc_sum_cnt_3d>OFFSET_AV_NUM_ACC){
					if(acc_smple_cnt_3d<12){
					acc_smple_cnt_3d++;	
					xyz_f_t data;	
					data.x = sum_temp_3d[A_X]/OFFSET_AV_NUM_ACC;
					data.y = sum_temp_3d[A_Y]/OFFSET_AV_NUM_ACC;
					data.z = sum_temp_3d[A_Z]/OFFSET_AV_NUM_ACC;	
					acc_sum_cnt_3d=sum_temp_3d[A_X] = sum_temp_3d[A_Y] = sum_temp_3d[A_Z] = sum_temp_3d[TEM] = 0;	
					cycle_data_add_oldx(&hml_lsq, (float)data.x/1000.,(float)data.y/1000.,(float)data.z/1000.);}
					else if(acc_3d_step==6){
					acc_3d_step=0;	
					cycle_cal_oldx(&hml_lsq, 666,0.001,  &sphere_x, &sphere_y, &sphere_z, &sphere_r);	
					mpu6050_fc.Off_3d.x=(hml_lsq.Off[0]*1000);
					mpu6050_fc.Off_3d.y=(hml_lsq.Off[1]*1000);
					mpu6050_fc.Off_3d.z=(hml_lsq.Off[2]*1000);
					mpu6050_fc.Gain_3d.x =  (hml_lsq.Gain[0]);
					mpu6050_fc.Gain_3d.y =  (hml_lsq.Gain[1]);
					mpu6050_fc.Gain_3d.z =  (hml_lsq.Gain[2]);	
          WRITE_PARM();						
					}		 		
				} 
			acc_3d_step_reg=acc_3d_step;	
			}
			break;
		
		}
		ACC_Reg.x=mpu6050_fc.Acc_I16.x;
	  ACC_Reg.y=mpu6050_fc.Acc_I16.y;
		ACC_Reg.z=mpu6050_fc.Acc_I16.z;


    if(mpu6050_fc.Gyro_CALIBRATE)
    {
        gyro_sum_cnt++;
        sum_temp[G_X] += mpu6050_fc.Gyro_I16.x;
        sum_temp[G_Y] += mpu6050_fc.Gyro_I16.y;
        sum_temp[G_Z] += mpu6050_fc.Gyro_I16.z;
        sum_temp[TEM] += mpu6050_fc.Tempreature;

        if( gyro_sum_cnt >= OFFSET_AV_NUM )
        {
            mpu6050_fc.Gyro_Offset.x = (float)sum_temp[G_X]/OFFSET_AV_NUM;
            mpu6050_fc.Gyro_Offset.y = (float)sum_temp[G_Y]/OFFSET_AV_NUM;
            mpu6050_fc.Gyro_Offset.z = (float)sum_temp[G_Z]/OFFSET_AV_NUM;
            mpu6050_fc.Gyro_Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
            gyro_sum_cnt =0;
            if(mpu6050_fc.Gyro_CALIBRATE == 1)
			{
               WRITE_PARM();
			}  
            mpu6050_fc.Gyro_CALIBRATE = 0;
            sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = sum_temp[TEM] = 0;
        }
    }
}

void Transform(float itx,float ity,float itz,float *it_x,float *it_y,float *it_z)
{
    *it_x = itx;
    *it_y = ity;
    *it_z = itz;

}

s16 FILT_BUF[ITEMS][(FILTER_NUM + 1)];
uint8_t filter_cnt = 0,filter_cnt_old = 0;

float mpu6050_tmp[ITEMS];
float mpu_fil_tmp[ITEMS];
float test_ang =0,test_ang_old=0,test_ang_d,test_fli_a,test_i;

void MPU6050_Data_Prepare(float T)
{
    u8 i;
    s32 FILT_TMP[ITEMS] = {0,0,0,0,0,0,0};
//	float auto_offset_temp[3];
    float Gyro_tmp[3];


    MPU6050_Data_Offset(); //校准函数

    /*读取buffer原始数据*/
		mpu6050_fc.Acc_I16.x=rawAccel[1].value;
		mpu6050_fc.Acc_I16.y=rawAccel[0].value;
		mpu6050_fc.Acc_I16.z=rawAccel[2].value;
		mpu6050_fc.Gyro_I16.x=rawGyro[1].value;
		mpu6050_fc.Gyro_I16.y=rawGyro[0].value;
		mpu6050_fc.Gyro_I16.z=rawGyro[2].value;	
    Gyro_tmp[0] = mpu6050_fc.Gyro_I16.x ;//
    Gyro_tmp[1] = mpu6050_fc.Gyro_I16.y ;//
    Gyro_tmp[2] = mpu6050_fc.Gyro_I16.z ;//
    mpu6050_fc.Tempreature=rawMPU6050Temperature.value;
    mpu6050_fc.TEM_LPF += 2 *3.14f *T *(mpu6050_fc.Tempreature - mpu6050_fc.TEM_LPF);
    mpu6050_fc.Ftempreature = mpu6050_fc.TEM_LPF/340.0f + 36.5f;

//======================================================================
    if( ++filter_cnt > FILTER_NUM )
    {
        filter_cnt = 0;
        filter_cnt_old = 1;
    }
    else
    {
        filter_cnt_old = (filter_cnt == FILTER_NUM)? 0 : (filter_cnt + 1);
    }
//10 170 4056
		if(fabs(mpu6050_fc.Off_3d.x)>10||fabs(mpu6050_fc.Off_3d.y)>10||fabs(mpu6050_fc.Off_3d.z)>10)
			mpu6050_fc.Cali_3d=1;
		int en_off_3d_off=0;
    /* 得出校准后的数据 */
	 if(mpu6050_fc.Cali_3d){
			  mpu6050_tmp[A_X] = (mpu6050_fc.Acc_I16.x - mpu6050_fc.Off_3d.x)*mpu6050_fc.Gain_3d.x - mpu6050_fc.Acc_Offset.x*en_off_3d_off;
        mpu6050_tmp[A_Y] = (mpu6050_fc.Acc_I16.y - mpu6050_fc.Off_3d.y)*mpu6050_fc.Gain_3d.y - mpu6050_fc.Acc_Offset.y*en_off_3d_off;
        mpu6050_tmp[A_Z] = (mpu6050_fc.Acc_I16.z - mpu6050_fc.Off_3d.z)*mpu6050_fc.Gain_3d.z - mpu6050_fc.Acc_Offset.z*en_off_3d_off;
	 }
   else{	 

        mpu6050_tmp[A_X] = (mpu6050_fc.Acc_I16.x - mpu6050_fc.Acc_Offset.x) ;
        mpu6050_tmp[A_Y] = (mpu6050_fc.Acc_I16.y - mpu6050_fc.Acc_Offset.y) ;
        mpu6050_tmp[A_Z] = (mpu6050_fc.Acc_I16.z - mpu6050_fc.Acc_Offset.z) ;
   
   
  }
    mpu6050_tmp[G_X] = Gyro_tmp[0] - mpu6050_fc.Gyro_Offset.x ;//
    mpu6050_tmp[G_Y] = Gyro_tmp[1] - mpu6050_fc.Gyro_Offset.y ;//
    mpu6050_tmp[G_Z] = Gyro_tmp[2] - mpu6050_fc.Gyro_Offset.z ;//


    /* 更新滤波滑动窗口数组 */
    FILT_BUF[A_X][filter_cnt] = mpu6050_tmp[A_X];
    FILT_BUF[A_Y][filter_cnt] = mpu6050_tmp[A_Y];
    FILT_BUF[A_Z][filter_cnt] = mpu6050_tmp[A_Z];
    FILT_BUF[G_X][filter_cnt] = mpu6050_tmp[G_X];
    FILT_BUF[G_Y][filter_cnt] = mpu6050_tmp[G_Y];
    FILT_BUF[G_Z][filter_cnt] = mpu6050_tmp[G_Z];

    for(i=0; i<FILTER_NUM; i++)
    {
        FILT_TMP[A_X] += FILT_BUF[A_X][i];
        FILT_TMP[A_Y] += FILT_BUF[A_Y][i];
        FILT_TMP[A_Z] += FILT_BUF[A_Z][i];
        FILT_TMP[G_X] += FILT_BUF[G_X][i];
        FILT_TMP[G_Y] += FILT_BUF[G_Y][i];
        FILT_TMP[G_Z] += FILT_BUF[G_Z][i];
    }


    mpu_fil_tmp[A_X] = (float)( FILT_TMP[A_X] )/(float)FILTER_NUM;
    mpu_fil_tmp[A_Y] = (float)( FILT_TMP[A_Y] )/(float)FILTER_NUM;
    mpu_fil_tmp[A_Z] = (float)( FILT_TMP[A_Z] )/(float)FILTER_NUM;


    mpu_fil_tmp[G_X] = (float)( FILT_TMP[G_X] )/(float)FILTER_NUM;
    mpu_fil_tmp[G_Y] = (float)( FILT_TMP[G_Y] )/(float)FILTER_NUM;
    mpu_fil_tmp[G_Z] = (float)( FILT_TMP[G_Z] )/(float)FILTER_NUM;


    /*坐标转换*/
    Transform(mpu_fil_tmp[A_X],mpu_fil_tmp[A_Y],mpu_fil_tmp[A_Z],&mpu6050_fc.Acc.x,&mpu6050_fc.Acc.y,&mpu6050_fc.Acc.z);
    Transform(mpu_fil_tmp[G_X],mpu_fil_tmp[G_Y],mpu_fil_tmp[G_Z],&mpu6050_fc.Gyro.x,&mpu6050_fc.Gyro.y,&mpu6050_fc.Gyro.z);

    mpu6050_fc.Gyro_deg.x = mpu6050_fc.Gyro.x *TO_ANGLE;
    mpu6050_fc.Gyro_deg.y = mpu6050_fc.Gyro.y *TO_ANGLE;
    mpu6050_fc.Gyro_deg.z = mpu6050_fc.Gyro.z *TO_ANGLE;

}
