#ifndef SMC_H
#define SMC_H
#include "include.h"
/*************************************
*liyaxian @All rights reserved
*二阶非线性系统
*x'' = - f(x,t) + bu(t) + d(t)
*滑模控制规律
*u(t) = 1/b*(epc*sign(s) + ks + c*(xd' - x') + xd'' + f - d(t))
*************************************/
struct SMC_t
{
     int32_t  SetPoint;       //设定值
     int32_t  Last_SetPoint;
     int32_t  Pre_SetPoint;
     int32_t  dSetPoint;      //设定值的导数
     int32_t  ddSetPoint;     //二阶导数

     int32_t  FeedPoint;      //实际值
     int32_t  Last_FeedPoint;
     int32_t  Pre_FeedPoint;
     int32_t  dFeedPoint;     //实际值的导数
     int32_t  ddFeedPoint;    //二阶导数

     int32_t  Error;          //偏差
     int32_t  dError;         //偏差变化率

     int32_t   c;             //滑模面参数

     int32_t   epc;
     int32_t   b;
     int32_t   k;     
}; 

extern int32_t  u_control(struct SMC_t* pp);

#endif
