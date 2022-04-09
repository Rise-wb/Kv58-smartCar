#ifndef FFT_ARM_H
#define FFT_ARM_H

#include "include.h"



extern void FFT_arm_init(void);
extern void FFT_arm_cov(q15_t * input,float * output);

#endif
