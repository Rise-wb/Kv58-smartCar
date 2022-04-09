#ifndef SYSCAR_H
#define SYSCAR_H

#include "include.h"

extern void CtrlTask(void);  //控制任务
//定时器
extern void CtrlPeriodTimerInit(void);


//计数器
//#define QDPHA_pin  &PTB18 //正交解码
//#define QDPHB_pin  &PTB19
//#define QDALT      ALT6
//#define FTM_QD     FTM2

#define QDPHA_pin_L  &PTB18	//左轮
#define QDPHB_pin_L  &PTB19
#define QDPHA_pin_R  &PTA12	//右轮
#define QDPHB_pin_R  &PTA13
#define QDALT_L      ALT6	//PTB18 B19
#define QDALT_R      ALT7	//PTA12 A13
#define FTM_QD_L     FTM2	//PTB18 B19
#define FTM_QD_R     FTM1	//PTA12 B1A


extern void CounterInit(void);
extern int32_t CounterGet_L(void);	//计数器的取值函数（方向）
extern int32_t CounterGet_R(void);
extern inline void ConuterClean(void);  //计数器的复位函数

//extern int32_t CounterGet(void);
//extern inline void ConuterClean(void);
//电机PWM

#define MotorPwmFrequency   15000
#define MotorPwmDutyPrecision  10000
//#define MotorFTM_PWMpin_1  &PTA12
//#define MotorFTM_PWMpin_2	&PTA13
#define MotorFTM_PWMpin_3 &PTE7
#define MotorFTM_PWMpin_4	&PTE8
#define MotorFTM_PWMpin_5 &PTE9
#define MotorFTM_PWMpin_6	&PTE10
//#define MotorFTM_ALT       ALT3
#define MotorFTM_ALT1      ALT6
//#define MOTOR_FTM_CHANNEL_GO1    0  //PTA11
//#define MOTOR_FTM_CHANNEL_GO2  1	 //PTA13
#define MOTOR_FTM_CHANNEL_GO3  2  //PTA11
#define MOTOR_FTM_CHANNEL_GO4  3	 //PTA13
#define MOTOR_FTM_CHANNEL_GO5  4   //PTA11
#define MOTOR_FTM_CHANNEL_GO6  5	 //PTA13
//#define MotorFTM_BASEADDR  	FTM1
#define MotorFTM_BASEADDR1  	FTM3
extern void MotorPWMInit(void);
extern void MotorPWMDuty(int32_t duty);
extern void MotorPWMChange1(int32_t duty);
extern void MotorPWMChange2(int32_t duty);
extern void MotorPWMConctol(int32_t duty);
extern void Motorstop(int32_t duty);
//舵机PWM
#define ServoPwmFrequency   50
#define ServoPwmDutyPrecision  50000
#define ServoMidDuty1    servomid   
#define ServoMidDuty2    0 
#define ServoFTM_PWMpin_1  &PTC1
//#define ServoFTM_PWMpin_2	 &PTC3
#define ServoFTM_ALT       ALT4
#define Servo_FTM_CHANNEL_1    0 //PTC1
//#define Servo_FTM_CHANNEL_2    2	//PTC3
#define ServoFTM_BASEADDR  	FTM0
extern void ServoPWMInit(void);
extern void ServoPWMDuty1(int32_t duty);
extern void ServoPWMDuty2(int32_t duty);

//AD
#define ADN    8
extern HSADC_port * HSADchannel[ADN] ;
extern inline void ADC_init(void);
extern inline void ADC_get(uint32_t* ptr);
//串口
#define UartBase  UART0
#define baudRate  115200
#define TXpin     &PTB1
#define RXpin			&PTB0
#define UART_ALT  ALT7
extern void uartPut_init(void);
extern void vcan_sendware(int *data1,int *data2,int *data3,int *data4,int *data5,int *data6,int *data7,int *data8);
//蜂鸣器
#define Beer_CHN  &PTE19
#define Reed_CH1  &PTA16
#define Reed_CH2  &PTA17
#define Beer_ON    gpio_set(Beer_CHN, 1)//(PTXn_T(Beer_CHN,OUT) = 1 )
#define Beer_OFF   gpio_set(Beer_CHN, 0)//(PTXn_T(Beer_CHN,OUT) = 0 )
#define Reed1_GET   gpio_get(Reed_CH1)//(PTXn_T(Reed_CH1,IN))
#define Reed2_GET   gpio_get(Reed_CH2)//(PTXn_T(Reed_CH2,IN))
extern void Beer_init(void);
extern void std_io_init(void);

#endif
