//Compile by liyaxian 2016-05-03
//@All rights reserved
#ifndef KAM_H
#define KAM_H
#include "include.h"
struct kalman_par
{
    float nowdata_p;  //协方差
    float nowdata;    //预测
    float kg;         //卡尔曼增益
    float Finaldata;  //最优解
    int   Finaldata_int;
    int Q;          //Q:过程噪声,Q增大,动态响应变快，收敛稳定性变坏  精度0.1 
    int R;          //R:测量噪声,R增大，动态响应变慢，收敛稳定性变好  精度0.1
};

extern void kalman_fliter_init(struct kalman_par* pp);
extern void kalman_fliter(struct kalman_par* pp,int signal);

#endif 
