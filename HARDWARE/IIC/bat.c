#include "bat.h"
#include "include.h"		 

//��ʼ��ADC															   
void  Adc_Init(void)
{    
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ��

  //�ȳ�ʼ��ADC1ͨ��5 IO��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//PA5 ͨ��5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��  
 
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1��λ
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//��λ����	 
 
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMAʧ��
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ɨ��ģʽ	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1��ת���ڹ��������� Ҳ����ֻת����������1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��
	
 
	ADC_Cmd(ADC1, ENABLE);//����ADת����	

}				  
float k_ad=0.00875;
float Get_Adc(u8 ch)   
{ 
	ch=7;
	  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ��			    
  
	ADC_SoftwareStartConv(ADC1);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1)*k_ad;	//�������һ��ADC1�������ת�����
}
//��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
//ch:ͨ�����
//times:��ȡ����
//����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		Delay_ms(5);
	}
	return temp_val/times;
} 
	 
BAT bat;
void Bat_protect(float dt)
{
static u8 state;
static u16 cnt[5];	
static  float temp,temp1;
	switch(state)
	{
		case 0:
			  if(Get_Adc(7)!=0){
			  temp+=Get_Adc(7);
				cnt[0]++;	
				}
				if(cnt[0]>20){
					state=1;
					temp1=temp/cnt[0];
					if(temp1>11.1-1&&temp1<12.6+1)
					  bat.bat_s=3;
					else if(temp1>14.8-1&&temp1<16.4+1)
						bat.bat_s=4;
					else
						bat.bat_s=0;
					bat.full=4.2*bat.bat_s;
					temp=0;
				}	  
		break;
	  case 1:
			 if(Get_Adc(7)!=0){
			  bat.origin=Get_Adc(7);
				temp+=bat.origin;
				cnt[1]++;	
				}
				if(cnt[1]>66){		
					bat.average=temp/cnt[1];cnt[1]=0;
          temp=0;		
          switch(bat.bat_s){
            case 3:
						 	if(bat.average<11.1-0.6||bat.average<11.1-0.6+(12.6-11.1)*bat.protect_percent)
							bat.low_bat=1;	
							else			
							bat.low_bat=0;
						break;
						case 4:
						if(bat.average<14.8-0.6||bat.average<14.8-0.6+(16.4-14.8)*bat.protect_percent)
							bat.low_bat_cnt++;
						else
							bat.low_bat_cnt=0;
						 if(bat.low_bat_cnt>2/0.05)
						  bat.low_bat=1;	
							else			
							bat.low_bat=0;
						break;;
 					}
				}	  
		break;
	
	}
}	









