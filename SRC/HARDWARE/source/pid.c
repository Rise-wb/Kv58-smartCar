#include "include.h"
#include "pid.h"


//struct  PID  pid;
void PIDInit(struct PID *sptr) 
{  
	sptr->SumError = 0; 
	sptr->LastError = 0; //Error[-1]  
	sptr->PrevError = 0; //Error[-2] 
	sptr->Proportion = 0; //�������� Proportional Const  
	sptr->Integral = 0; //���ֳ���Integral Const  
	sptr->Derivative = 0; //΢�ֳ��� Derivative Const  
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
   PID���㲿��
=====================================================================================================*/

/******************************��׼����ʽPID************************************/
//��׼����ʽPID	 ������ֵΪ����
int Standard_incremental_PID(struct PID *pp, int NextPoint)
{
    int  pidcalc,Error;
    Error = pp->SetPoint - NextPoint;     //ƫ��
	 
	pidcalc =  pp->Proportion * (Error-pp->LastError) / 100 // E[k]-E[k-1]���������
			 + pp->Integral * Error /100// E[k]��
			 + pp->Derivative * (Error-2*pp->LastError+pp->PrevError) / 100; //E[k]-2*E[k-1]+E[k-2]��	
    
	 //�洢�������´μ���
    pp->PrevError = pp->LastError;
    pp->LastError = Error;
 
	return (pidcalc); 		//��������ֵ
} 
//����ƫ���������ʽPID
int Errorset_incremental_PID(struct PID *pp, int E, int EC,int ECC)
{
    int  pidcalc;
    //Error = pp->SetPoint - NextPoint;     //ƫ��
	 
	pidcalc =  pp->Proportion * (EC) / 10 // E[k]-E[k-1]���������
			 + pp->Integral * (E) /100// E[k]��
			 + pp->Derivative * (ECC) / 10; //E[k]-2*E[k-1]+E[k-2]��	
    
	 //�洢�������´μ���
    //pp->PrevError = pp->LastError;
    //pp->LastError = Error;
 
	return (pidcalc); 		//��������ֵ
} 
//��������ʽPID	 ������ֵΪλ�ã�
int Bangbang_incremental_PID(struct PID *pp, int NextPoint)
{
    //PID����
    int  pidcalc,Error;
    static int value=0;
    Error = pp->SetPoint - NextPoint;     //ƫ��
	 
	pidcalc =  pp->Proportion * (Error-pp->LastError) // E[k]-E[k-1]���������
			 + pp->Integral * Error // E[k]��
			 + pp->Derivative * (Error-2*pp->LastError+pp->PrevError); //E[k]-2*E[k-1]+E[k-2]��	
    
	 //�洢�������´μ���
    pp->PrevError = pp->LastError;
    pp->LastError = Error;
    
    value+=pidcalc;
    
     //bang-bang
 
 
	return (value); 		//��������ֵ
} 

/*****************************�Ǳ�׼����ʽPID********************************/
//�Ǳ�׼����ʽPID ������ֵΪ����
int incremental_PID(struct PID *pp,int NextPoint)
{
    int  pidcalc,Error;
    Error = pp->SetPoint - NextPoint;		// ��������
	 
	pidcalc =  pp->Proportion * Error // E[k]��
			 - pp->Integral * pp->LastError // E[k-1]��
			 + pp->Derivative * pp->PrevError; // E[k-2]��
    
    pp->PrevError = pp->LastError;		 //�洢�������´μ���
    pp->LastError = Error;
 
	return (pidcalc);			//��������ֵ
} 

/******************************�Ǳ�׼λ��ʽPID*********************************/
int position_PID(struct PID *pp,int NextPoint)
{
	int dError,Error,gappid;
 
	Error = pp->SetPoint - NextPoint; 	 //	ƫ�����
    dError = pp->LastError - pp->PrevError;     // ΢�ּ���
	pp->SumError+=(Error/10);					   //�ۻ����
	gappid =(pp->Proportion*Error/10)
			  + (pp->Integral * pp->SumError/10)
			  + pp->Derivative*dError;

	gappid=gappid/10;
    pp->PrevError = pp->LastError;
    pp->LastError =Error;
	return gappid ;		
}

/***************************��׼λ��ʽPID*************************/
int standard_position_PID(struct PID *pp,int NextPoint)
{
	int last_gappid,Error,gappid;
	Error = pp->SetPoint - NextPoint; 	 //	ƫ�����

	gappid =  last_gappid
		  	+ pp->Proportion * (Error-pp->LastError) // E[k]-E[k-1]���������
			+ pp->Integral * Error // E[k]��
		    + pp->Derivative * (Error-2*pp->LastError+pp->PrevError); //E[k]-2*E[k-1]+E[k-2]��	

    pp->PrevError = pp->LastError;
    pp->LastError =Error;
	last_gappid=gappid;
	return gappid ;			
}

/*********************���ַ����׼λ��ʽPID�㷨*************************/
int Integral_separation_position_PID(struct PID *pp,int NextPoint,uint8_t Q)
{
	int last_gappid,Error,gappid;
	Error = pp->SetPoint - NextPoint; 	 //	ƫ�����

	gappid =  last_gappid
		  	+ pp->Proportion * (Error-pp->LastError) // E[k]-E[k-1]���������
			+Q * pp->Integral * Error // E[k]��			   //Q=0;���ַ��룬
		    + pp->Derivative * (Error-2*pp->LastError+pp->PrevError); //E[k]-2*E[k-1]+E[k-2]��	

    pp->PrevError = pp->LastError;
    pp->LastError =Error;
	last_gappid=gappid;
	return gappid ;			
}

/*******************���ַ����׼����ʽPID�㷨*****************************/
int Integral_separation_incremental_PID(struct PID *pp, int NextPoint,uint8_t Q)
{
	int  pidcalc,Error;
    Error = pp->SetPoint - NextPoint;     //ƫ��
	 
	pidcalc =  pp->Proportion * (Error-pp->LastError) // E[k]-E[k-1]���������
			 + Q * pp->Integral * Error // E[k]��		   Q=0,���ַ��룬
			 + pp->Derivative * (Error-2*pp->LastError+pp->PrevError); //E[k]-2*E[k-1]+E[k-2]��	
    
	 //�洢�������´μ���
    pp->PrevError = pp->LastError;
    pp->LastError = Error;
 
	return (pidcalc); 		//��������ֵ
}

/**************************΢����������ʽPID�㷨********************************/
int Differential_forward_incremental_PID(struct PID *pp, int NextPoint)		   //Ӧ�����趨ֵƵ�������ĳ���
{
	int  pidcalc,Error;
    Error = pp->SetPoint - NextPoint;     //ƫ��
	 
	pidcalc =  pp->Proportion * (Error-pp->LastError) // E[k]-E[k-1]���������
			 + pp->Integral * Error // E[k]��		   Q=0,���ַ��룬
			 + pp->Derivative * (NextPoint-2*pp->LastPoint+pp->PrevPoint); //E[k]-2*E[k-1]+E[k-2]��	
    
	 //�洢�������´μ���
	pp->PrevPoint = pp->LastPoint;
	pp->LastPoint = NextPoint;

    pp->PrevError = pp->LastError;
    pp->LastError = Error;
 
	return (pidcalc); 		//��������ֵ		
}

//��д�Ķ��λ��ʽ�㷨
int servo_position_PID(struct PID *pp,int NextPoint)
{
  int P,D,Error;
  
  Error = pp->SetPoint - NextPoint;    //ƫ��
  
  pp->PrevError = pp->LastError ;
  pp->LastError = Error;                  //�洢���
  
	
  P = (720-0.6*Single_min)/550.00 * pp->LastError * pp->Proportion ;//(800-Single_min)/600.00 *
  D =  pp->Derivative * (pp->LastError - pp->PrevError)/10; //600.00/(900-Single_min)*  500.00/(900-Single_min) *
      
  
  return (pp->SetPoint+P+D);             //����λ��
}

int servo_position_PID1(struct PID *pp,int NextPoint)
{
  int P,D,Error;
  
  Error = pp->SetPoint - NextPoint;    //ƫ��
  
  pp->PrevError = pp->LastError ;
  pp->LastError = Error;                  //�洢���
  
 
  P = pp->LastError * pp->Proportion /100.0;
  D = pp->Derivative * (pp->LastError - pp->PrevError)/100.0; 
      
  
  return (pp->SetPoint+P+D);             //����λ��
}

int my_PID_W(struct Pid_wei *w_Pid,int error)
{
		w_Pid->last_error=w_Pid->now_error;
		w_Pid->now_error=error;
		w_Pid->integral+=w_Pid->now_error;
		w_Pid->val=w_Pid->Kp*w_Pid->now_error+w_Pid->Ki*w_Pid->integral+w_Pid->Kd*(w_Pid->now_error-w_Pid->last_error);
 
		return (w_Pid->val); 		//��������ֵ
}
  
  
  
  
