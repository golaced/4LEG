#ifndef __FILTER_H
#define __FILTER_H

#include "include.h"
//float Moving_Average(u8 item,u8 width_num,float in);
void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out);
float Moving_Median(u8 item,u8 width_num,float in);
extern double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);
extern fp32 LPF_1st(fp32 oldData, fp32 newData, fp32 lpf_factor);
#define _xyz_f_t xyz_f_t
void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out);


#define NUMBER_OF_FIRST_ORDER_FILTERS 20
#define ACC_LOWPASS_X 0
#define ACC_LOWPASS_Y 1
#define ACC_LOWPASS_Z 2
#define BARO_LOWPASS   10
#define FLOW_LOWPASS_X   11
#define FLOW_LOWPASS_Y   12
typedef struct firstOrderFilterData {
  float   gx1;
  float   gx2;
  float   gx3;
  float   previousInput;
  float   previousOutput;
} firstOrderFilterData_t;

extern firstOrderFilterData_t firstOrderFilters[NUMBER_OF_FIRST_ORDER_FILTERS];

void initFirstOrderFilter(float T);
float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters,float T);
#endif
