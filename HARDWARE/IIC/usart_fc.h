#ifndef _USART_H
#define _USART_H

#include "stm32f4xx.h"
void UsartSend_LEG_BUF_BUF(u8 sel);
void LEG_POWER_OFF(u8 sel);
extern u8 Rx_Buf[];
void Usart2_Init(u32 br_num);
void Usart2_IRQ(void);
void Usart3_IRQ(void);
void Usart2_Send(unsigned char *DataToSend ,u8 data_num);

void Uart5_Init(u32 br_num);
void Uart5_IRQ(void);
void Uart5_Send(unsigned char *DataToSend ,u8 data_num);
void CPU_LINK_TASK(void);
typedef struct PID_STA{u16 OP,OI,OD,IP,II,ID,YP,YI,YD;}PID_STA;
extern PID_STA HPID,SPID,FIX_PID,NAV_PID;
void UsartSend_M100(uint8_t ch);
void Send_DJ11_12(void);
typedef struct int16_rcget{
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
				int16_t AUX1;
				int16_t AUX2;
				int16_t AUX3;
				int16_t AUX4;
				int16_t AUX5;
	      int16_t HEIGHT_MODE;
	      int16_t POS_MODE;
	      u8 update,Heart,Heart_rx,Heart_error;
	      u16 lose_cnt,lose_cnt_rx;
	      u8 connect;
				int16_t RST;}RC_GETDATA;

extern RC_GETDATA Rc_Get,Rc_Get_PWM,Rc_Get_SBUS,Rc_Wifi;//���յ���RC����,1000~2000
				
				
struct _float{
	      float x;
				float y;
				float z;};

struct _int16{
       int16_t x;
	     int16_t y;
	     int16_t z;};		

struct _trans{
     struct _int16 origin;  //ԭʼֵ
	   struct _float averag;  //ƽ��ֵ
	   struct _float histor;  //��ʷֵ
	   struct _int16 quiet;   //��ֵ̬
	   struct _float radian;  //����ֵ 
          };

struct _alt{
int32_t altitude;
float altitude_f;
int32_t Temperature;
int32_t Pressure;
int Temperat;
};
					
struct _sensor{   
	struct _trans acc;
	struct _trans gyro;
	struct _trans hmc;
	struct _alt alt;
              };

extern struct _sensor sensor;	

	

struct _speed{   
	int altitude;
	int bmp;
	int pitch;
	int roll;
	int gps;
	int filter;
	int sonar;
	int acc;
              };

struct _altitude{   
	int bmp;
	int sonar;
	int gps;
	int acc;
	int filter;
              };
struct _angle{   
float pitch;
float roll;
float yaw;
              };

							
struct _get{   
	struct _speed speed;
	struct _altitude altitude;
	struct _angle AngE;
              };
struct _plane{   
	struct _speed speed;
	struct _altitude altitude;
	struct _get get;
              };

extern struct _plane plane;	
							
							
struct _slam{   
	 int16_t spd[5];
	 int16_t dis[5];
              };
extern struct _slam slam;	
							
							
struct _SPEED_FLOW_NAV{
float west;
float east;
float x;
float y;
float x_f;
float y_f;
};	



struct _POS_GPS_NAV {
long J;
long W;
long X_O;
long Y_O;
long X_UKF;
long Y_UKF;
u8 gps_mode;
u8 star_num;
};

struct _FLOW_NAV{
struct _SPEED_FLOW_NAV speed;
struct _SPEED_FLOW_NAV speed_h;	
u8 rate;
};	

struct _IMU_NAV{   
struct _FLOW_NAV flow;
struct _POS_GPS_NAV gps;
	
};
extern struct _IMU_NAV imu_nav;
extern float ALT_POS_SONAR_HEAD;

struct _PID2{
float p;
float i;
float d;
float i_max;
float limit;
float dead;	
float dead2;	
};
struct _PID1{
struct _PID2 out;
struct _PID2 in;	
};
struct _PID_SET{
struct _PID1 nav;
struct _PID1 high;
struct _PID1 avoid;
struct _PID1 circle;	
};
extern struct _PID_SET pid;

typedef struct
{
u8 thr_fix;
u8 en_pid_out_pit;
u8 en_pid_out_rol;
u8 en_pid_out_yaw;
u8 en_pid_fuzzy_p;
u8 en_pid_sb_set;
u8 trig_flow_spd;
u8 return_home;
u8 trig_h_spd;
u8 px4_map;
u8 en_fuzzy_angle_pid;
u8 en_sensor_equal_flp;
u8 pit_rol_pid_out_flp;	
u8 en_pid_yaw_angle_control;	
u8 en_pid_yaw_control_by_pit_rol;	
u8 thr_add_pos;
u8 spid;
u8 mpu6050_bw_42;
u8 en_imu_q_update_mine;	
u8 en_moto_smooth;
u8 pid_mode;
u8 no_head;
u8 sonar_avoid;	
u8 yaw_sel;
u8 sb_smooth;
u8 use_px4_err;
u8 flow_hold_position;
u8 dji_sonar_high;
u8 auto_fly_up,auto_land;
u8 en_circle_nav,circle_miss_fly,en_track_forward,en_circle_locate;
u8 flow_hold_position_use_global;
u8 flow_hold_position_high_fix;
u8 height_safe;
u8 baro_lock;
u8 baro_f_use_ukfm;
u8 flow_f_use_ukfm;
u8 use_ano_bmp_spd;
u8 tune_ctrl_angle_offset;
u8 imu_use_mid_down;
u8 hunman_pid;
u8 yaw_imu_control;	
u8 cal_sel;
u8 flow_sel;
u8 height_in_speed;
u8 height_upload;
u8 en_h_mode_switch;
u8 en_dj_cal;
u8 en_sd_save;
u8 cal_rc;
u8 en_break;
u8 use_dji;
u8 en_marker;
u8 rc_control_flow_spd;
u8 rc_control_flow_pos;
u8 rc_control_flow_pos_sel;
u8 dj_by_hand;
u8 en_dj_control;
u8 dj_yaw_follow;
u8 dji_mode;
u8 en_visual_control;
u8 hold_use_flow;
u8 en_sonar_avoid;
u8 thr_fix_test;
u8 en_imu_ekf;
u8 att_pid_tune;
u8 high_pid_tune;
u8 dj_lock;
u8 en_eso;
u8 en_eso_h_out;
u8 en_eso_h_in;
u8 yaw_use_eso;
u8 flow_d_acc;
u8 en_hinf_height_spd;
u8 en_circle_control;
u8 save_video;
u8 en_h_inf;
u8 test1;
u8 test2;
u8 test3;
u8 test4;	
u8 mode_fly;
u8 h_is_fix;
u8 mems_state;
u8 att_ident1;
//flow
u8 en_flow_gro_fix;
u8 flow_size;
u8 show_qr_origin;
//test
u8 fc_test1;
}_MODE;

extern _MODE mode;
	

typedef struct{
	unsigned int x;//Ŀ���x����
	unsigned int y;//Ŀ���y����
	unsigned int w;//Ŀ��Ŀ��
	unsigned int h;//Ŀ��ĸ߶�
	
	u8 check;
}RESULT;//ʶ����
extern float angle_imu_dj[3];
extern RESULT color;
extern u8 LOCK, KEY[8],KEY_SEL[4],NAV_BOARD_CONNECT;
extern void GOL_LINK_TASK(void);
extern void SD_LINK_TASK(void);
extern void Usart1_Init(u32 br_num);//SD_board
extern void Usart4_Init(u32 br_num);//-------SD_board
extern void Usart3_Init(u32 br_num);//-------CPU_board
void Uart6_Init(u32 br_num);//-----odroid
extern u8 cnt_nav_board;
extern u16 data_rate_gol_link;
extern void UART2_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);
extern void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void SD_LINK_TASK2(u8 sel);
extern void UsartSend_GPS(uint8_t ch);
void Send_IMU_NAV(void);
#define SEND_IMU 0
#define SEND_FLOW 1
#define SEND_GPS 2
#define SEND_ALT 3
extern float rate_gps_board;
void Send_IMU_TO_GPS(void);
#define SEND_BUF_SIZE1 64*4	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.
extern u8 SendBuff1[SEND_BUF_SIZE1];	//�������ݻ�����

void data_per_uart1(u8 sel);
void clear_leg_uart(void);
extern u16 leg_uart_cnt;

#define SEND_BUF_SIZE2 40	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.
extern u8 SendBuff2[SEND_BUF_SIZE2];	//�������ݻ�����
void data_per_uart2(void);

#define SEND_BUF_SIZE3 32	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.
extern u8 SendBuff3[SEND_BUF_SIZE3];	//�������ݻ�����
void data_per_uart3(void);

#define SEND_BUF_SIZE4 64	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.
extern u8 SendBuff4[SEND_BUF_SIZE4];	//�������ݻ�����
void data_per_uart4(u8 sel);

void Send_LEG(u8 sel);
void GOL_LINK_TASK(void);


void ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,int16_t hx,int16_t hy,int16_t hz);
void ReportIMU(int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);

typedef struct {
        float pos_end[5][3];
	      u16 lose_cnt,lose_cnt_rx;
	      u8 face_check;
	      u16 fx,fy,fw,fh;
	      u8 connect;}Tinker;

extern Tinker tinker;//���յ���RC����,1000~2000
void Send_Speed(float spd[4]);
#endif
