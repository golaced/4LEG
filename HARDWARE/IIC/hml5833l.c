

#include "mpu9250.h"
#include "parameter.h"
#include "my_math.h"
#include "include.h"
#include "iic_soft.h"
#include "filter.h"
#include "hml5833l.h"

ak8975_t ak8975 = { {0,0,0},{-1,-1,-1},{1,0.8538,0.9389},{0,0,0} };

#define  IIR_ORDER     4      //使用IIR滤波器的阶数
static double b_IIR_hml[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
static double a_IIR_hml[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
static double InPut_IIR_hml[3][IIR_ORDER+1] = {0};
static double OutPut_IIR_hml[3][IIR_ORDER+1] = {0};
u8 ak8975_ok;
void ANO_AK8975_Read_Mag_Data(void)
{
	int16_t mag_temp[3];
//u8 ak8975_buffer[6]; //接收数据缓存
	
	mag_temp[0]=rawMag[0].value/10;
	mag_temp[1]=rawMag[1].value/10;
	mag_temp[2]=rawMag[2].value/10;
	
	ak8975.Mag_Adc.x= IIR_I_Filter(mag_temp[0], InPut_IIR_hml[0], OutPut_IIR_hml[0], b_IIR_hml, IIR_ORDER+1, a_IIR_hml, IIR_ORDER+1);
  ak8975.Mag_Adc.y= IIR_I_Filter(mag_temp[1], InPut_IIR_hml[1], OutPut_IIR_hml[1], b_IIR_hml, IIR_ORDER+1, a_IIR_hml, IIR_ORDER+1);
	ak8975.Mag_Adc.z= IIR_I_Filter(mag_temp[2], InPut_IIR_hml[2], OutPut_IIR_hml[2], b_IIR_hml, IIR_ORDER+1, a_IIR_hml, IIR_ORDER+1);
	ak8975.Mag_Val.x = (ak8975.Mag_Adc.x - ak8975.Mag_Offset.x) ;
	ak8975.Mag_Val.y = (ak8975.Mag_Adc.y - ak8975.Mag_Offset.y)*ak8975.Mag_Gain.y ;
	ak8975.Mag_Val.z = (ak8975.Mag_Adc.z - ak8975.Mag_Offset.z)*ak8975.Mag_Gain.z ;
	//磁力计中点矫正	
	ANO_AK8975_CalOffset_Mag();

}

u8 Mag_CALIBRATED = 0,Mag_CALIBRATED_R=0;;
//磁力计中点矫正

void ANO_AK8975_CalOffset_Mag(void)
{
	static xyz_f_t	MagMAX = { -100 , -100 , -100 }, MagMIN = { 100 , 100 , 100 }, MagSum;
	static uint16_t cnt_m=0;
	static u8 hml_cal_temp=0;
static u8 state_cal_hml;
	switch(state_cal_hml)
	{
		case 0:if(Mag_CALIBRATED_R!=hml_cal_temp)
		{				Mag_CALIBRATED=1;state_cal_hml=1;}break;
		case 1:if(Mag_CALIBRATED==0)
		{			hml_cal_temp=Mag_CALIBRATED_R;state_cal_hml=0;}break;
	}
	
	if(Mag_CALIBRATED)
	{	
		#if USE_CYCLE_HML_CAL
		if(ABS(ak8975.Mag_Adc.x)<500&&ABS(ak8975.Mag_Adc.y)<500&&ABS(ak8975.Mag_Adc.z)<500)
		    HMC_CAL_HML();
		
		#else
		if(ABS(ak8975.Mag_Adc.x)<500&&ABS(ak8975.Mag_Adc.y)<500&&ABS(ak8975.Mag_Adc.z)<500)
		{
			MagMAX.x = MAX(ak8975.Mag_Adc.x, MagMAX.x);
			MagMAX.y = MAX(ak8975.Mag_Adc.y, MagMAX.y);
			MagMAX.z = MAX(ak8975.Mag_Adc.z, MagMAX.z);
			
			MagMIN.x = MIN(ak8975.Mag_Adc.x, MagMIN.x);
			MagMIN.y = MIN(ak8975.Mag_Adc.y, MagMIN.y);
			MagMIN.z = MIN(ak8975.Mag_Adc.z, MagMIN.z);		
			
			if(cnt_m == CALIBRATING_MAG_CYCLES)
			{
				ak8975.Mag_Offset.x = (int16_t)((MagMAX.x + MagMIN.x) * 0.5f);
				ak8975.Mag_Offset.y = (int16_t)((MagMAX.y + MagMIN.y) * 0.5f);
				ak8975.Mag_Offset.z = (int16_t)((MagMAX.z + MagMIN.z) * 0.5f);
	
				MagSum.x = MagMAX.x - MagMIN.x;
				MagSum.y = MagMAX.y - MagMIN.y;
				MagSum.z = MagMAX.z - MagMIN.z;
				
				ak8975.Mag_Gain.y = MagSum.x / MagSum.y;
				ak8975.Mag_Gain.z = MagSum.x / MagSum.z;
				
			WRITE_PARM();//Param_SaveMagOffset(&ak8975.Mag_Offset);//param_Save();//保存数据
				cnt_m = 0;
				Mag_CALIBRATED = 0;
			}
		}
		#endif
		cnt_m++;
		
	}
	else
	{

	}
}

