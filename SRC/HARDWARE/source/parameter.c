
#include "parameter.h"
//汇车区
int huiche_t=10;
int Confluence_flag=0;
int Confluence_count=50;
int stop_flag=0;
//红外模块
unsigned char  irtime;//红外用全局变量
unsigned char irpro_ok,irok;
unsigned char IRcord[4];
unsigned char irdata[33];
uint8_t f_HW=0;
int Red_a=0;
int f_HW_Show=0;
int counter_flag=0;

//测距模块
int date=0,uwb_x=0,uwb_X=0;
int uwb_flag=0;
char uwb_data[12];
int uart_finish_flag=1;
//--------------------------

int N=50;
//---------------------------
uint16_t Steer_R_Range =  200;
uint16_t Steer_L_Range =  200;
float  Speed_Rmodu=1.0,Speed_Lmodu=1.0;
//----------------------------
// int16_t  Speed_L[5] = {160,130,130,105,15};//{110,110,110,80,15};｛长直道，小直道，弯道，坡道，刹车｝
// int16_t  Speed_R[5] = {160,130,130,105,15};//{110,110,110,80,15};
// uint16_t sum_max = 0;
 int16_t  Motor_LOut = 0;
 int16_t  Motor_ROut = 0;
 int16_t  Motor_Max = 6500;     //电机最大值
 int16_t  Motor_Min = -3500;      //电机最小值
 int16_t  Motor_LOut_Show = 0;
 int16_t  Motor_ROut_Show = 0;
 int16_t  Speed_L[6] = {200,155,150,145,15,140};//{110,110,110,80,15};｛长直道，小直道，弯道，坡道，刹车, 环岛｝
 int16_t  Speed_R[6] = {200,155,150,145,15,140};//{110,110,110,80,15};
 uint16_t sum_max = 250;
 uint16_t sum_max1 = 300;
 uint16_t sum_2 = 0;
 int16_t  Motor_L = 0;
 int16_t  Motor_R = 0;
 int  Motor_Lshow = 0;
 int  Motor_Rshow = 0;
 uint16_t Straight_Count = 0;
 uint16_t ShaChe_Count = 0;
 uint8_t  Speed_Flag = 0;     //速度标志位
 int  Speed_P =256;      //56                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
 int  Speed_I =60;      //30
 int  Speed_D =20;      //20
 int16_t  Speed_P_show = 0;
 int16_t  Speed_I_show = 0;
 int16_t  Speed_D_show = 0;
 int32_t  v_D = 0;
 int32_t  v_P = 0;
 uint16_t wandao_Flag = 300;//300
 uint16_t wandao_Flag1 = 700;//450
 uint16_t D_ZHI = 0;
 uint16_t CCD_ZHI = 0;
 uint16_t C_ZHI = 0;
 uint16_t D_Zhi_Value = 8;
 uint16_t C_Zhi_Value = 12;
 uint16_t ss1 = 110; //60
 uint16_t ss2 = 70; //24 30
 uint16_t ss3 = 50;//14 18
 uint16_t duanzhidao_Count = 35;    //判断为短直道计数
 uint16_t changzhidao_Count = 50;  //判断为长直道计数
 uint16_t CZsha_Count = 280;//3m 280 6m 560     93/m
 int32_t  xx = 4;
 int32_t  yy = 4;
 int16_t  Speed_ValueL=0,Speed_L_Last=0,Speed_R_Last=0,Speed_Value=0,Speed_ValueR=0,
        Speed_EL=0,Speed_ER=0;
 //-----------------------------左电机输出------------------//
 int16_t Speed_LError_Last = 0;
 int16_t Speed_LError_Prev = 0;
 int16_t  Speed_LOut = 0;                      // PID计算出的增量值
 int16_t  Speed_LOut_show = 0;
 int16_t  Speed_LError_Now = 0;                // 当前速度偏差
 //-----------------------------右电机输出------------------//
 int16_t Speed_RError_Last = 0;
 int16_t Speed_RError_Prev = 0;
 int16_t  Speed_ROut = 0;                      // PID计算出的增量值
 int16_t  Speed_ROut_show = 0;
 int16_t  Speed_RError_Now = 0;                // 当前速度偏差
//----------------------------
int    PWM_value_1=0;
int    PWM_value_2=0;
int    P_L=0;
int    P_R=0;
int    bug_flag=0;
int    t=0;
int    sum_flag=800;
int    huan_plus=5;//负6正5
int    huan_time=260;
int    huan_time1=320;
int    huan_empty_time=300;
int    ramp_time=50;
int    ramp_flag=0;
int    ramp_flag1=0;
int    ramp_count=0;
int    huan_flag=10;
int    huan_flag1=0;
int    huan_flag3=0;
int    cross_flag=0;
int    huan_flag2=10;
int    huan_size=0;
int    huan_num=0 ;

int    pwm_dif=0;
int    ra=150;
int    te=150;
int    fan_k=10;//25;
int    PWM_servo=3500;
int 	 ORG_servo=0;
int    ORG_chaspeed=0;
int    PID_Answer=0;
int    PID_cha_Answer=0;
int    AD_result[ADN] = {0};
int    pAD_result[ADN] = {0};
int    AD_result_temp[ADN];
int    dif_W,  sum_W,  dif_L,  sum_L,  dif_V,  sum_V,  sum_all,  dif_huan,  sum_huan;
int    Servoduty=0;     //servo duty has been initiated the servomid
int    direction=0;   //方向，0左，1右
int    E=0,EC=0,ECC=0,last_E=0; 
int    EH=0,EHC=0,last_EH=0;
int    EL=0,ELC=0,last_EL=0;
int    Em=0,EmC=0,last_Em=0;
int    P=1,  D=1;
int    EW=0,dif_CW=0,dif_CCW=0;
int    E2=0,E2C=0,E2CC;
int    zhiDaoIndex = 0;
int    servomid=3183;  //舵机中值8720 15500 3070;
int    speed;
float    last_speed;
float    ace;
volatile int speed_temp;
float  fftOutput[128] = {0};
q15_t  fftInput[128] = {0};

int    StraightC = 0;
int    time_s=20;      //runningtime
int    speed_P=150;//超大弯道480  180
int    speed_P1=200;//普通弯道570  220
int    speed_D=100;//

int    PWM_inc_max=900;
int    speed_mid=3700;
int    speed_mid_turn=3700;
int    speed_mid_huan=3600;
int    servo_k=350;//830;

int L_sum=0;
int R_sum=0;
int Double_min=0;
int Single_min=0;
int Single_max=0;
int Single_max_L=0;

int    basicspeed=40; //80
int    curvespeed=20; //30
int    speed_K=130; //110

int    swm=0;          //normal switvh  1
int    swv=0;          //normal switvh  1

int		 speed_L=0;
int    speed_R=0;
int    speed_flag=0;

int    circleTrcePoint = 0;
int    speed_Flag=0;
int    C_Zhi = 20;
int    D_Zhi = 10;
int    Podao_time = 100;
int    PWM_value_test = 350;

int    vol=0;
int    PC=0;
int    SD=0;
int    WirelessTask =0;
int    baV=0;
int    podaoflag=0;
int    vp=0,vd=0;
int    mid_flag = 30;

int    Ang_X,Ang_Y,Ang_Z;
int    podao_sum;

int    changeCount = 0;
int    sideStopFlag = 0;
int    sideLowSpeedFlag = 0;
int    zhiDao52cmFlag = 0;
int    zhiDaoJiaSuFlag = 0;
int    anotherCar = 0;
int    singleRun = 0;
int    Straight_count=0;
int    isInZhidaoFlag = 0;
int    hindCarFinishFlag = 0;
int    sideRunCount = 0,sideLowSpeedCount = 0,sideRunCountEndFlag = 0;
int    startCount = 0;
int    nowZhiChangeFinishFlag = 1;
int    EWset0Flag = 0,hindEWset0Flag = 0;
int    hingCarActFlag = 0;
int    sideRunCountBreakFlag = 0;

uint8_t  circleTrceFlag = 0;

int    isFrontCar_T = POSFRONT;

struct PID motor;
struct PID *Mo=&motor;
struct PID servo; 
struct PID *Se=&servo;
struct PID acce;
struct PID *Ac=&acce;
struct PID servo_t;
struct PID *St=&servo_t;

struct SMC_t smotor;
struct SMC_t *SMo = &smotor; 

struct kalman_par GyX;  
struct kalman_par GyY;
struct kalman_par GyZ;

struct kalman_par ACX;
struct kalman_par ACY;
struct kalman_par ACZ;

struct kalman_par Spe;

//------------------initiate some parameter---------------
void Par_init()
{
    PIDInit(Mo);                //初始化PID控制器
    PIDInit(Se);
    
  Mo->Proportion=560;//20    //电机系数
  Mo->Integral=500;
  Mo->Derivative=200;//30
  
	St->Proportion=18;    // A40 舵机系数，无积分控制器   
  St->Derivative=3;     // A8
  St->Integral  =0;
	
  Se->Proportion=28; //11   // A40 舵机系数，无积分控制器   
  Se->Derivative=340; //5    // A8
  Se->Integral  =0;
  Servoduty=servomid;
  
  Ac->Proportion=40;
  Ac->Integral=20;
  Ac->Derivative=8;
  
  SMo->c = 1;
  SMo->b = 10;  //分母
  SMo->epc = 10;
  SMo->k = 70;
  
  //FFT_arm_init();
  
     kalman_fliter_init(&GyX);
     kalman_fliter_init(&GyY);
     kalman_fliter_init(&GyZ);
     kalman_fliter_init(&ACX);
     kalman_fliter_init(&ACY);
     kalman_fliter_init(&ACZ);
     kalman_fliter_init(&Spe);
     
    
  GyX.Q=1;  GyX.R=10;
  GyY.Q=1;  GyY.R=10;
  GyZ.Q=1;  GyZ.R=10;
  
  ACX.Q=1;  ACX.R=200;
  ACY.Q=1;  ACY.R=200;
  ACZ.Q=1;  ACZ.R=200;
  
  Spe.Q=10;  Spe.R=10;
}
