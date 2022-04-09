#include "FFT_arm.h"
#include "arm_math.h"

//arm_rfft_fast_instance_f32 S;
arm_rfft_instance_q15 S;
/* 实数序列FFT长度 */
#define fftSize  128
/* 正变换 */
uint32_t  ifftFlag = 0;
/*码位倒序*/
uint32_t  doBitReverse = 1;
void FFT_arm_init()
{
	  /* 初始化结构体S中的参数 */      
      arm_rfft_init_q15(&S, fftSize, ifftFlag, doBitReverse);
}

void FFT_arm_cov(q15_t * input,float * output)
{
    q15_t inputTemp[fftSize];
    q15_t ouputTemp[fftSize];
    float f_ouputTemp[fftSize];
         
         arm_copy_q15(input,inputTemp ,fftSize );               //数组复制
	       arm_rfft_q15(&S,inputTemp,ouputTemp);                  //正变换         
         for(uint8_t i = 0; i<fftSize ; i++)
         {
								f_ouputTemp[i] = (float32_t)ouputTemp[i];         //定点数转换浮点数
         }
         arm_cmplx_mag_f32(f_ouputTemp, output, fftSize);         //求模
}
