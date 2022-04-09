//Compile by liyaxian 2016-05-03
//@All rights reserved
#include "kalman.h"

void kalman_fliter_init(struct kalman_par* pp)
{
	pp->nowdata_p = 20;
	pp->nowdata   = 0.0;
	pp->Finaldata = 0.0;
        pp->kg = 0.1;
}

void kalman_fliter(struct kalman_par* pp,int signal)
{
         /*
	      这个卡尔曼进行了优化，少了一个变量
        Q:过程噪声,Q增大,动态响应变快，收敛稳定性变坏
        R:测量噪声,R增大，动态响应变慢，收敛稳定性变好
	      
	      修改1.0
	      保存两此信号输入的结果
	  */
	  /*预测*/
	  pp->nowdata = pp->Finaldata;
	  /*协方差*/
	  pp->nowdata_p = pp->nowdata_p+pp->Q/10.0;
	  /*卡尔曼增益*/
	  pp->kg = pp->nowdata_p/(pp->nowdata_p+pp->R/10.0);
	  /*最优解*/
          pp->Finaldata = pp->nowdata+pp->kg*((float)signal - pp->nowdata);
	  /*更新协方差*/
	  pp->nowdata_p = (1-pp->kg)*pp->nowdata_p;
          
          pp->Finaldata_int = (int)pp->Finaldata;
}
