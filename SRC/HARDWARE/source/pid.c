#include "include.h"
#include "pid.h"


//struct  PID  pid;
void PIDInit(struct PID *sptr) 
{  
	sptr->SumError = 0; 
	sptr->LastError = 0; //Error[-1]  
	sptr->PrevError = 0; //Error[-2] 
	sptr->Proportion = 0; //比例常数 Proportional Const  
	sptr->Integral = 0; //积分常数Integral Const  
	sptr->Derivative = 0; //微分常数 Derivative Const  
	sptr->SetPoint 	=0;
	sptr->LastPoint	=0;
	sptr->PrevPoint	=0;	
} 

void Pid_init(struct Pid_wei *w_Pid){
		w_Pid->Kp=0.080;
		w_Pid->Ki=0;
		w_Pid->Kd=0.095;
		w_Pid->now_error=0;
		w_Pid->last_error=0;
		w_Pid->integral=0;
		w_Pid->val=0;
}

/*====================================================================================================
   PID计算部分
=====================================================================================================*/

/******************************标准增量式PID************************************/
//标准增量式PID	 ，返回值为增量
int Standard_incremental_PID(struct PID *pp, int NextPoint)
{
    int  pidcalc,Error;
    Error = pp->SetPoint - NextPoint;     //偏差
	 
	pidcalc =  pp->Proportion * (Error-pp->LastError) / 100 // E[k]-E[k-1]项，增量计算
			 + pp->Integral * Error /100// E[k]项
			 + pp->Derivative * (Error-2*pp->LastError+pp->PrevError) / 100; //E[k]-2*E[k-1]+E[k-2]项	
    
	 //存储误差，用于下次计算
    pp->PrevError = pp->LastError;
    pp->LastError = Error;
 
	return (pidcalc); 		//返回增量值
} 
//给定偏差导数的增量式PID
int Errorset_incremental_PID(struct PID *pp, int E, int EC,int ECC)
{
    int  pidcalc;
    //Error = pp->SetPoint - NextPoint;     //偏差
	 
	pidcalc =  pp->Proportion * (EC) / 10 // E[k]-E[k-1]项，增量计算
			 + pp->Integral * (E) /100// E[k]项
			 + pp->Derivative * (ECC) / 10; //E[k]-2*E[k-1]+E[k-2]项	
    
	 //存储误差，用于下次计算
    //pp->PrevError = pp->LastError;
    //pp->LastError = Error;
 
	return (pidcalc); 		//返回增量值
} 
//棒棒增量式PID	 ，返回值为位置！
int Bangbang_incremental_PID(struct PID *pp, int NextPoint)
{
    //PID部分
    int  pidcalc,Error;
    static int value=0;
    Error = pp->SetPoint - NextPoint;     //偏差
	 
	pidcalc =  pp->Proportion * (Error-pp->LastError) // E[k]-E[k-1]项，增量计算
			 + pp->Integral * Error // E[k]项
			 + pp->Derivative * (Error-2*pp->LastError+pp->PrevError); //E[k]-2*E[k-1]+E[k-2]项	
    
	 //存储误差，用于下次计算
    pp->PrevError = pp->LastError;
    pp->LastError = Error;
    
    value+=pidcalc;
    
     //bang-bang
 
 
	return (value); 		//返回增量值
} 

/*****************************非标准增量式PID********************************/
//非标准增量式PID ，返回值为增量
int incremental_PID(struct PID *pp,int NextPoint)
{
    int  pidcalc,Error;
    Error = pp->SetPoint - NextPoint;		// 增量计算
	 
	pidcalc =  pp->Proportion * Error // E[k]项
			 - pp->Integral * pp->LastError // E[k-1]项
			 + pp->Derivative * pp->PrevError; // E[k-2]项
    
    pp->PrevError = pp->LastError;		 //存储误差，用于下次计算
    pp->LastError = Error;
 
	return (pidcalc);			//返回增量值
} 

/******************************非标准位置式PID*********************************/
int position_PID(struct PID *pp,int NextPoint)
{
	int dError,Error,gappid;
 
	Error = pp->SetPoint - NextPoint; 	 //	偏差计算
    dError = pp->LastError - pp->PrevError;     // 微分计算
	pp->SumError+=(Error/10);					   //累积误差
	gappid =(pp->Proportion*Error/10)
			  + (pp->Integral * pp->SumError/10)
			  + pp->Derivative*dError;

	gappid=gappid/10;
    pp->PrevError = pp->LastError;
    pp->LastError =Error;
	return gappid ;		
}

/***************************标准位置式PID*************************/
int standard_position_PID(struct PID *pp,int NextPoint)
{
	int last_gappid,Error,gappid;
	Error = pp->SetPoint - NextPoint; 	 //	偏差计算

	gappid =  last_gappid
		  	+ pp->Proportion * (Error-pp->LastError) // E[k]-E[k-1]项，增量计算
			+ pp->Integral * Error // E[k]项
		    + pp->Derivative * (Error-2*pp->LastError+pp->PrevError); //E[k]-2*E[k-1]+E[k-2]项	

    pp->PrevError = pp->LastError;
    pp->LastError =Error;
	last_gappid=gappid;
	return gappid ;			
}

/*********************积分分离标准位置式PID算法*************************/
int Integral_separation_position_PID(struct PID *pp,int NextPoint,uint8_t Q)
{
	int last_gappid,Error,gappid;
	Error = pp->SetPoint - NextPoint; 	 //	偏差计算

	gappid =  last_gappid
		  	+ pp->Proportion * (Error-pp->LastError) // E[k]-E[k-1]项，增量计算
			+Q * pp->Integral * Error // E[k]项			   //Q=0;积分分离，
		    + pp->Derivative * (Error-2*pp->LastError+pp->PrevError); //E[k]-2*E[k-1]+E[k-2]项	

    pp->PrevError = pp->LastError;
    pp->LastError =Error;
	last_gappid=gappid;
	return gappid ;			
}

/*******************积分分离标准增量式PID算法*****************************/
int Integral_separation_incremental_PID(struct PID *pp, int NextPoint,uint8_t Q)
{
	int  pidcalc,Error;
    Error = pp->SetPoint - NextPoint;     //偏差
	 
	pidcalc =  pp->Proportion * (Error-pp->LastError) // E[k]-E[k-1]项，增量计算
			 + Q * pp->Integral * Error // E[k]项		   Q=0,积分分离，
			 + pp->Derivative * (Error-2*pp->LastError+pp->PrevError); //E[k]-2*E[k-1]+E[k-2]项	
    
	 //存储误差，用于下次计算
    pp->PrevError = pp->LastError;
    pp->LastError = Error;
 
	return (pidcalc); 		//返回增量值
}

/**************************微分先行增量式PID算法********************************/
int Differential_forward_incremental_PID(struct PID *pp, int NextPoint)		   //应用于设定值频繁升降的场合
{
	int  pidcalc,Error;
    Error = pp->SetPoint - NextPoint;     //偏差
	 
	pidcalc =  pp->Proportion * (Error-pp->LastError) // E[k]-E[k-1]项，增量计算
			 + pp->Integral * Error // E[k]项		   Q=0,积分分离，
			 + pp->Derivative * (NextPoint-2*pp->LastPoint+pp->PrevPoint); //E[k]-2*E[k-1]+E[k-2]项	
    
	 //存储误差，用于下次计算
	pp->PrevPoint = pp->LastPoint;
	pp->LastPoint = NextPoint;

    pp->PrevError = pp->LastError;
    pp->LastError = Error;
 
	return (pidcalc); 		//返回增量值		
}

//我写的舵机位置式算法
int servo_position_PID(struct PID *pp,int NextPoint)
{
  int P,D,Error;
  
  Error = pp->SetPoint - NextPoint;    //偏差
  
  pp->PrevError = pp->LastError ;
  pp->LastError = Error;                  //存储误差
  
	
  P = (720-0.6*Single_min)/550.00 * pp->LastError * pp->Proportion ;//(800-Single_min)/600.00 *
  D =  pp->Derivative * (pp->LastError - pp->PrevError)/10; //600.00/(900-Single_min)*  500.00/(900-Single_min) *
      
  
  return (pp->SetPoint+P+D);             //返回位置
}

int servo_position_PID1(struct PID *pp,int NextPoint)
{
  int P,D,Error;
  
  Error = pp->SetPoint - NextPoint;    //偏差
  
  pp->PrevError = pp->LastError ;
  pp->LastError = Error;                  //存储误差
  
 
  P = pp->LastError * pp->Proportion /100.0;
  D = pp->Derivative * (pp->LastError - pp->PrevError)/100.0; 
      
  
  return (pp->SetPoint+P+D);             //返回位置
}

int my_PID_W(struct Pid_wei *w_Pid,int error)
{
		w_Pid->last_error=w_Pid->now_error;
		w_Pid->now_error=error;
		w_Pid->integral+=w_Pid->now_error;
		w_Pid->val=w_Pid->Kp*w_Pid->now_error+w_Pid->Ki*w_Pid->integral+w_Pid->Kd*(w_Pid->now_error-w_Pid->last_error);
 
		return (w_Pid->val); 		//返回增量值
}
  
  
  
  
