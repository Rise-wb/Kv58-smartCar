#ifndef PAR_H
#define PAR_H

#include "include.h"



#define MAXDUTY   900    //最大电机占空比  80%
#define MINDUTY   0        //最小电机占空比   0%

#define SERVOMAX  (servomid+390)     //最大舵机占空比
#define SERVOMIN  (servomid-420)      //最小舵机占空比

//会车区
extern int 		huiche_t;
extern int		Confluence_flag;
extern int		Confluence_count;
extern int 		stop_flag;
//红外模块
extern unsigned char  irtime;
extern uint8_t irpro_ok,irok;
extern unsigned char IRcord[4];
extern unsigned char irdata[33];
extern uint8_t 	f_HW;
extern int		Red_a;
extern int		f_HW_Show;

//测距模块
extern int		date,uwb_x,uwb_X;	
extern int		uwb_flag;
extern char   uwb_data[12];
extern int		uart_finish_flag;
extern int    counter_flag;
//-------------------------------------------------
extern  int N;
//-----------------------电机全局变量--------------


extern  int16_t  Motor_LOut;
extern  int16_t  Motor_ROut;
extern  int16_t  Motor_Max;
extern  int16_t  Motor_Min;
extern  int16_t  Motor_LOut_Show;
extern  int16_t  Motor_ROut_Show;
extern  int16_t  Speed_L[6];
extern  int16_t  Speed_R[6];
extern  uint16_t sum_max;
extern  uint16_t sum_max1;
extern  uint16_t sum_2;
extern  int16_t  Motor_L;
extern  int16_t  Motor_R;
extern  int  Motor_Lshow;
extern  int  Motor_Rshow;
extern  uint16_t Straight_Count;
extern  uint16_t ShaChe_Count;
extern  uint8_t  Speed_Flag;
extern  int  Speed_P;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
extern  int  Speed_I;
extern  int  Speed_D;
extern  int16_t  Speed_P_show;
extern  int16_t  Speed_I_show;
extern  int16_t  Speed_D_show;
extern  int32_t  v_D;
extern  int32_t  v_P;
extern  uint16_t wandao_Flag;
extern  uint16_t wandao_Flag1;
extern  uint16_t D_ZHI;
extern  uint16_t CCD_ZHI;
extern  uint16_t C_ZHI;
extern  uint16_t D_Zhi_Value;
extern  uint16_t C_Zhi_Value;
extern  uint16_t ss1; 
extern  uint16_t ss2;
extern  uint16_t ss3;
extern  uint16_t duanzhidao_Count;
extern  uint16_t changzhidao_Count;
extern  uint16_t CZsha_Count;
extern  int32_t  xx;
extern  int32_t  yy;
extern  int16_t Speed_ValueL,Speed_L_Last,Speed_R_Last,Speed_Value,Speed_ValueR,
               Speed_EL,Speed_ER;
//-----------------------------左电机输出------------------//
extern  int16_t  Speed_LError_Last;
extern  int16_t  Speed_LError_Prev;
extern  int16_t  Speed_LOut;
extern  int16_t  Speed_LOut_show;
extern  int16_t  Speed_LError_Now;                // 当前速度偏差
//-----------------------------右电机输出------------------//
extern  int16_t  Speed_RError_Last;
extern  int16_t  Speed_RError_Prev;
extern  int16_t  Speed_ROut;
extern  int16_t  Speed_ROut_show;
extern  int16_t  Speed_RError_Now;                // 当前速度偏差

// extern int16_t  Speed_L[5];
// extern int16_t  Speed_R[5];
 extern int PWM_value_1;
 extern int PWM_value_2;
 extern int P_L;
 extern int P_R;
 extern float Speed_Lmodu;
 extern float Speed_Rmodu;
 extern uint16_t Steer_L_Range;
 extern uint16_t Steer_R_Range;
 extern int bug_flag;
 extern int sum_flag;
 extern int speed_L;
 extern int speed_R;
 extern int speed_flag;
 extern int t;
 extern int huan_plus;
 extern int huan_time;
 extern int huan_time1;
 extern int huan_empty_time;
 extern int huan_flag;
 extern int huan_flag1;
 extern int huan_flag2;
 extern int huan_flag3;
 extern int huan_size;
 extern int huan_num;
 extern int ramp_flag;
 extern int ramp_flag1;
 extern int ramp_count;
 extern int ramp_time;
 extern int cross_flag;
 extern int pwm_dif;
 extern int ra;
 extern int te;
 extern int fan_k;
 extern	int L_sum;
 extern int R_sum;
 extern int Double_min;
 extern	int Single_min;
 extern	int Single_max;
 extern int Single_max_L;
 extern int    PWM_servo;
 extern int    ORG_servo;
 extern int    ORG_chaspeed;
 extern int    PID_cha_Answer;
 extern int 	 PID_Answer;
 extern int    AD_result[8];
 extern int    pAD_result[8];
 extern int    AD_result_temp[8];
 extern int    dif_W,  sum_W,  dif_L,  sum_L,  dif_V,  sum_V,  sum_all  ,  dif_huan,  sum_huan;
 extern int    Servoduty;     //servo duty has been initiated the servomid
 extern int    direction;   //方向，0左，1右
 extern int    EH,EHC,last_EH;
 extern int    E,EC,ECC,last_E;
 extern int    EL,ELC,last_EL;
 extern int    Em,EmC,last_Em;
 extern int    EW,dif_CW,dif_CCW;
 extern int    E2,E2C,E2CC;
 extern int    zhiDaoIndex;
 extern int    servomid;  //舵机中值
 extern int    speed;
 extern int    PWM_inc_max;//差速的最大值
 extern int    speed_mid;//基本速度
 extern int    speed_mid_turn;
 extern int    speed_mid_huan;
 extern int 	 servo_k;
 extern float    last_speed;
 extern float    ace;
 extern volatile int speed_temp;
 extern int    StraightC;
 extern int    time_s;      //runningtime
 extern int    speed_P,speed_P1;
 extern int    speed_D;
 extern int    P,  D;
 extern int    basicspeed;
 extern int    curvespeed;
 extern int    swm;
 extern int    swv;
 extern int    speed_Flag;
 extern int    speed_K;
 extern int    vol;
 extern int    PC;
 extern int    SD;
 extern int    WirelessTask;
 extern int    baV;
 extern int    circleTrcePoint;
 extern int    podaoflag;
 extern int    vp,vd;
 extern int    C_Zhi ;
 extern int    D_Zhi ;
 extern int    Podao_time;
 extern int    PWM_value_test;
 extern float  fftOutput[128];
 extern q15_t  fftInput[128];
 extern int    D_GYR_X;
 
 extern int    Ang_X,Ang_Y,Ang_Z;//角度
 extern int    podao_sum;
 
 extern int    changeCount;
 extern int    sideStopFlag;
 extern int    sideLowSpeedFlag;
 extern int    zhiDao52cmFlag;
 extern int    zhiDaoJiaSuFlag;
 extern int    anotherCar;
 extern int    singleRun;
 extern int    Straight_count;
 extern int    isInZhidaoFlag;
 extern int    hindCarFinishFlag ;
 extern int    sideRunCount ,sideLowSpeedCount ,sideRunCountEndFlag ;
 extern int    startCount;
 extern int    nowZhiChangeFinishFlag;
 extern int    EWset0Flag,hindEWset0Flag;
 extern int    hingCarActFlag;
 extern int    sideRunCountBreakFlag;
 
 extern uint8_t  circleTrceFlag ;
 
 extern int    isFrontCar_T;
 
 extern struct PID motor;
 extern struct PID *Mo;
 extern struct PID servo; 
 extern struct PID *Se;
 extern struct PID acce;
 extern struct PID *Ac;
 extern struct PID servo_t;
 extern struct PID *St;
 
 extern struct SMC_t smotor;
 extern struct SMC_t *SMo ; 
 extern struct kalman_par GyX;
 extern struct kalman_par GyY;
 extern struct kalman_par GyZ;

 extern struct kalman_par ACX;
 extern struct kalman_par ACY;
 extern struct kalman_par ACZ;
 extern struct kalman_par Ang;
 extern struct kalman_par Spe;
 
extern void Par_init(void);


#endif
