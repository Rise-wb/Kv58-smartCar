#ifndef __PID_H
#define __PID_H
#include  "include.h"
//#include "include.h"

struct PID 
{ 
	int  SetPoint;           //  设定目标 Desired Value
	int  LastPoint;
	int  PrevPoint;
    int  Proportion;         //  比例常数 Proportional Const
    int  Integral;           //  积分常数 Integral Const
    int  Derivative;         //  微分常数 Derivative Const
    int  LastError;          //  Error[-1]
    int  PrevError;          //  Error[-2]
    int  SumError;           //  Sums of Errors
};

struct Pid_wei
{
		int  SetPoint;           //  设定目标 Desired Value
	  int  LastPoint;
	  int  PrevPoint;
		double Kp;
	  double Ki;
		double Kd;
		int64_t now_error;
		int64_t last_error;
		int64_t integral;
		int64_t val;
		
};
extern struct Pid_wei w_pid;
void Pid_init(struct Pid_wei *w_Pid);
extern struct  PID  pid;
void PIDInit(struct PID *sptr);
int Standard_incremental_PID(struct PID *pp, int NextPoint);
int Errorset_incremental_PID(struct PID *pp, int E, int EC,int ECC);
int Bangbang_incremental_PID(struct PID *pp, int NextPoint);
int incremental_PID(struct PID *pp,int NextPoint);
int standard_position_PID(struct PID *pp,int NextPoint);
int position_PID(struct PID *pp,int NextPoint);
int Integral_separation_position_PID(struct PID *pp,int NextPoint,uint8_t Q);
int Integral_separation_incremental_PID(struct PID *pp, int NextPoint,uint8_t Q);
int Differential_forward_incremental_PID(struct PID *pp, int NextPoint);
int servo_position_PID1(struct PID *pp,int NextPoint);
int servo_position_PID(struct PID *pp,int NextPoint);
int my_PID_W(struct Pid_wei *pp,int error);

#endif
