#include "SMC.h"
#include "math.h"


//符号函数
int32_t  sign(int32_t input)
{
	  if(input > 0)
	  	return 1;
	  else if(input < 0)
	  	return -1;
	  else
	  	return 0;
}

//饱和函数
float  sat(int32_t input)
{
    int32_t delta = 1;

    if(input > delta)
    	return 1.0;
    else if(input < -delta)
    	return -1.0;
    else 
    	return 1.0*input/delta;
}

//连续函数,采用继电器特性连续化
float  thta(int32_t input)
{
	  float sigma = 0.0;
	  float result;
	  //sigma是很小的正常数
      result =   1.0*input/(fabs(input)+sigma);
		return result;
}
//控制输出
/************************************
*根据电机电枢平衡方程推出如下公式
*
*f = (Peq*w+Beq*w'+Teq)/Jeq
*b = KT/Jeq
*
************************************/
int32_t  u_control(struct SMC_t* pp)
{
	       int32_t u = 0; //控制输出
           int32_t s = 0; //滑模面函数

           int32_t Peq = 1;
           int32_t Beq = 1;
           int32_t Teq = 0;
#define SGN(input)   sign(input)

      pp->dFeedPoint = pp->FeedPoint - pp->Last_FeedPoint;
      pp->ddFeedPoint = pp->FeedPoint - 2*pp->Last_FeedPoint + pp->Pre_FeedPoint; 
      
      pp->dSetPoint = pp->SetPoint - pp->Last_SetPoint;
      pp->ddSetPoint = pp->SetPoint - 2*pp->Last_SetPoint + pp->Pre_SetPoint;

      pp->Error = pp->SetPoint - pp->FeedPoint;
      pp->dError = pp->dSetPoint - pp->dFeedPoint;

      s = pp->c*pp->Error + pp->dError;
      u = (pp->epc*SGN(s) + pp->k*s + pp->c*pp->dError + pp->ddSetPoint + Peq*pp->FeedPoint + Beq*pp->dFeedPoint + Teq)/pp->b;  
      //f = Peq*pp->FeedPoint + Beq*pp->dFeedPoint + Teq

      pp->Pre_FeedPoint = pp->Last_FeedPoint;
      pp->Last_FeedPoint = pp->FeedPoint;

      pp->Pre_SetPoint = pp->Last_SetPoint;
      pp->Last_SetPoint = pp->SetPoint;

      return u;      

}
