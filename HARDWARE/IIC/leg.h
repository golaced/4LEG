
#include "include.h"
#define Xs 0
#define Ys 1
#define Zs 2

#define USE_UART_LEG 1
typedef struct 
{
	float x;
	float y;
	float z;
	
}POS;

typedef struct 
{
	float leg_up_high;
	u8 leg_switch_flag,leg_ground_force;
	float z;
	POS off_all;
	POS init_end_pos;
	POS limit,limit_min;
	float l1;
	float l2;
	float l3;
	u8 pwm_id[4];
	u16 PWM_OFF[4];
	u16 PWM_INIT[3];
	u16 PWM_MIN[3],PWM_MAX[3];
	float PWM_OUT[4];
	float PWM_PER_DEGREE[3];
	u8 use_ground_check;
	u8 leg_set_invert,leg_ground_cnt;
	float pos_tar_reg[3],off_local[2];
	u8 en_pwm_out;
	u16 leg_loss_cnt;
	float desire_time;
	int sita_flag[3];
	float dsita[3];
	float dsita_tar[3];
	float init_sita[3];
	float sita_tar[3];
	POS pos_tar_trig_test[3];
	u8 id,init_mode,leg_power_force,control_mode_focre;
	u16 leg_move_pass_cnt;
}LEG_SYS;


typedef struct 
{ u8 leg_connect,leg_ground,err;
	u8 leg_power,rst_leg;
	u8 need_move;	
	u8 curve_trig,control_mode;
	POS pos_now[3],pos_now_brain[3];
	POS pos_tar[3],pos_tar_trig[3];
	float sita[3];
	 //0->openloop 1->closeloop	
	float leg_end_force[4],leg_meme_angle[4];
	float deng[3];
	LEG_SYS sys;
}LEG_STRUCT;
//------------------------------BARIN-----

typedef struct 
{ u8 leg_connect;
	u16 leg_loss_cnt,dt_leg_min_trig_cnt;
	float att_off[2],k_spd_to_range,kp_center[2],k_center_fp,move_range_k,k_center_c[2];
	float desire_time;
	POS leg_local[5];
	u8 leg_use_ground;
	u8 init_mode;
	POS pos_tar_trig[5];
	u8 err,rst;
	float center_off_when_move[2];
	float leg_t,tar_spd[3];
	float leg_h;	
	float leg_move_range[2],leg_move_min_dt;//cm
	POS off_leg[5],center_off,center_off1,center_scale;
}BRAIN_SYS;

typedef struct 
{ u8 control_mode,power_all,rst_all,tabu;
	float steady_value;
	u8 force_stop,loss_center,ground_leg_num,can_move_leg;	
	u8 leg_move[5],leg_out_range[5];	
	float tar_att_force[3],tar_att[3],att[3],tar_w,spd,spd_d,spd_yaw;
  u8 center_stable,move_id,leg_move_state;
	float area_of_leg[2];
	POS center;
	double leg_ground_center[3],leg_ground_center_trig[3],leg_ground_center_trig_init[3],tar_center[2];
	float now_pos[3],now_spd[3],now_acc[3];
	BRAIN_SYS sys;
}BRAIN_STRUCT;
extern u8 last_move_leg;
extern LEG_STRUCT leg[5];
extern BRAIN_STRUCT brain;

#define S_IDLE 0
#define S_BODY_MOVE 1
#define S_LEG_TRIG 2
#define S_LEG_TRIG_ING 3
#define S_LEG_TRIG_ING1 4
#define S_LEG_TRIG_DONE 5
#define S_BODY_RETURN 6
void cal_sita_from_pos( LEG_STRUCT *in,float x_i,float y_i,float z_i,u8 out);
void cal_pos_from_sita(LEG_STRUCT * in,float sita1,float sita2,float sita3);
void leg_init( LEG_STRUCT *in,u8 id);
u8 pos_range_check(LEG_STRUCT * in,float x,float y,float z);
void leg_drive(LEG_STRUCT * in,float dt);

//brain
void conver_legpos_to_barin(BRAIN_STRUCT *in,LEG_STRUCT * inl,u8 id);
void estimate_center(BRAIN_STRUCT *in,float att[3],float spd_body[3],float acc_body[3],float spd_tar[3],float w_tar);
void barin_init(BRAIN_STRUCT *in);
void center_control(float dt);//PID
void cal_center_of_leg_ground(BRAIN_STRUCT *in);//PID
void check_leg_need_move(BRAIN_STRUCT *in,float spd_body[3],float spd_tar[3],float w_tar,float dt);
u8 inTrig(float x, float y,float x1,float y1,float x2,float y2,float x3,float y3); 
u8 inTrig2(float x, float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4) ;
void find_closet_point(u8*min_id,float x, float y,float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4,u8 num); 
void find_leg_need_move(float spd_tar[3],float str[4],float end[4]) ;
void leg_task1(float dt);
void cal_deng_from_spd(BRAIN_STRUCT *in);
void cal_target_of_center_move(BRAIN_STRUCT *in);
void leg_tar_est(BRAIN_STRUCT *in,LEG_STRUCT *leg,float spd_body[3],float spd_tar[3],float w_tar,u8 need_move,float dt);
void att_control(float dt);
extern float center_control_out[2],att_control_out[5];;
#define Xr 0
#define Yr 1
#define Zr 2