#ifndef ANALYSE_H
#define ANALYSE_H

#include "include.h"
#include "math.h"
//---------------use for user--------------------
extern void set_Angle(void);
extern void set_Speed(void); 

//----------------use for inner-------------------
extern void normal(int* swm);    //normalize AD value;
extern void GetSiTa(void);
extern void guihua(int* swm);
extern void filter(void);              //ÂË²¨  use for inner
extern void GetPAD_result(void);
extern void ramp_process(void);
extern int signal2(void);              //singal analyse
extern void caculate_speed2(void);     //caculate speed
extern void timerset(int time_s);
extern void isDoubleWirelessTask(void);
extern void wireless_task(void);
extern void Speed_LControl(int16_t Speed_LSet,int16_t Speed_LBack);
extern void Speed_RControl(int16_t Speed_RSet,int16_t Speed_RBack);
#endif

