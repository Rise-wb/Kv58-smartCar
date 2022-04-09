#include "SystemCar.h"

//-------------定时器中断-----------------
void CtrlPeriodTimerInit()
{
    /* Structure of initialize PIT */
    pit_config_t pitConfig;
    /*
     * pitConfig.enableRunInDebug = false;
     */
    PIT_GetDefaultConfig(&pitConfig);

    /* Init pit module */
    PIT_Init(PIT, &pitConfig);

    /* Set timer period for channel 0 */
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, MSEC_TO_COUNT(5U, CLOCK_GetFreq(kCLOCK_BusClk)));  //单位是ms

    /* Enable timer interrupts for channel 0 */
    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

    /* Enable at the NVIC */
    EnableIRQ(PIT0_IRQn);

    /* Start channel 0 */   
    PIT_StartTimer(PIT, kPIT_Chnl_0);
}

//void PIT0_IRQHandler()
//{
//	    /*控制任务*/
//			CtrlTask();
//			/*编码器复位*/
////			ConuterClean();
//			/* Clear interrupt flag.*/
//			PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
//}

void PIT0_IRQHandler()
{
			UART_Irq_Dis(UART_0);
			counter_flag++;
	    /*????*/
			CtrlTask();
			/*?????*/
//			ConuterClean();
			/* Clear interrupt flag.*/
			PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
//			if(debug_flag>0)
			//UART_Irq_En(UART_0);
}



void CounterInit()
{
//			port_init(QDPHA_pin,QDALT);
//	    port_init(QDPHB_pin,QDALT);
//			ftm_quad_init(FTM_QD);
			port_init(QDPHA_pin_L,QDALT_L);
	    port_init(QDPHB_pin_L,QDALT_L);
			port_init(QDPHA_pin_R,QDALT_R);
	    port_init(QDPHB_pin_R,QDALT_R);
			ftm_quad_init(FTM_QD_L);
			ftm_quad_init(FTM_QD_R);

}

//int32_t CounterGet()
//{
//			return (int32_t)ftm_quad_get(FTM_QD);
//}

//inline void ConuterClean()
//{
//			ftm_quad_clean(FTM_QD);
//}
int32_t CounterGet_L()
{
			return (int32_t)ftm_quad_get(FTM_QD_L);
}

int32_t CounterGet_R()
{
			return (int32_t)ftm_quad_get(FTM_QD_R);
}

inline void ConuterClean()
{
			ftm_quad_clean(FTM_QD_L);
			ftm_quad_clean(FTM_QD_R);
}


void MotorPWMInit()
{
     ftm_config_t ftmInfo;
		 ftm_chnl_pwm_signal_param_t ftmParam[2];
		 ftm_chnl_pwm_signal_param_t ftmParam1[4];
	
//		port_init(MotorFTM_PWMpin_1,MotorFTM_ALT );
//		port_init(MotorFTM_PWMpin_2,MotorFTM_ALT );	
		port_init(MotorFTM_PWMpin_3,MotorFTM_ALT1 );
		port_init(MotorFTM_PWMpin_4,MotorFTM_ALT1 );
		port_init(MotorFTM_PWMpin_5,MotorFTM_ALT1 );
		port_init(MotorFTM_PWMpin_6,MotorFTM_ALT1 );
		     /* Configure ftm params with frequency 15kHZ */
//    ftmParam[0].chnlNumber = (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO1;
//    ftmParam[0].level = kFTM_HighTrue;
//    ftmParam[0].dutyCyclePercent = 0U;
//    ftmParam[0].firstEdgeDelayPercent = 0U;

//    ftmParam[1].chnlNumber = (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO2;
//    ftmParam[1].level = kFTM_HighTrue;
//    ftmParam[1].dutyCyclePercent = 0U;
//    ftmParam[1].firstEdgeDelayPercent = 0U;
		ftmParam1[0].chnlNumber = (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO3;
    ftmParam1[0].level = kFTM_HighTrue;
    ftmParam1[0].dutyCyclePercent = 0U;
    ftmParam1[0].firstEdgeDelayPercent = 0U;

    ftmParam1[1].chnlNumber = (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO4;
    ftmParam1[1].level = kFTM_HighTrue;
    ftmParam1[1].dutyCyclePercent = 0U;
    ftmParam1[1].firstEdgeDelayPercent = 0U;	

    ftmParam1[2].chnlNumber = (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO5;
    ftmParam1[2].level = kFTM_HighTrue;
    ftmParam1[2].dutyCyclePercent = 0U;
    ftmParam1[2].firstEdgeDelayPercent = 0U;

    ftmParam1[3].chnlNumber = (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO6;
    ftmParam1[3].level = kFTM_HighTrue;
    ftmParam1[3].dutyCyclePercent = 0U;
    ftmParam1[3].firstEdgeDelayPercent = 0U;
	
			/*
     * ftmInfo.prescale = kFTM_Prescale_Divide_1;
     * ftmInfo.bdmMode = kFTM_BdmMode_0;
     * ftmInfo.pwmSyncMode = kFTM_SoftwareTrigger;
     * ftmInfo.reloadPoints = 0;
     * ftmInfo.faultMode = kFTM_Fault_Disable;
     * ftmInfo.faultFilterValue = 0;
     * ftmInfo.deadTimePrescale = kFTM_Deadtime_Prescale_1;
     * ftmInfo.deadTimeValue = 0;
     * ftmInfo.extTriggers = 0;
     * ftmInfo.chnlInitState = 0;
     * ftmInfo.chnlPolarity = 0;
     * ftmInfo.useGlobalTimeBase = false;
     */
    FTM_GetDefaultConfig(&ftmInfo);		
    /* Initialize FTM module */
    FTM_Init(MotorFTM_BASEADDR1, &ftmInfo);
		
		FTM_SetupPwm(MotorFTM_BASEADDR1, ftmParam1, 4U, kFTM_EdgeAlignedPwm, MotorPwmFrequency, CLOCK_GetFreq(kCLOCK_FastPeriphClk));//
    FTM_StartTimer(MotorFTM_BASEADDR1, kFTM_SystemClock);//
}
//保持符号
void MotorPWMDuty(int32_t duty)
{
	int32_t dutyConverted = duty*FTM_PWM_Precision/MotorPwmDutyPrecision;
	
			if(duty>0)
			{
						/* Start PWM mode with updated duty cycle */				        
//						FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR, (ftm_chnl_t)MOTOR_FTM_CHANNEL_BACK, kFTM_EdgeAlignedPwm,
//																	 0);
//						gpio_set(&PTA10,0);	
//						FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO1, kFTM_EdgeAlignedPwm,
//																	 (uint16_t)dutyConverted);
						/* Software trigger to update registers */
//						FTM_SetSoftwareTrigger(MotorFTM_BASEADDR, true);
//			}
//			else
//			{
//						/* Start PWM mode with updated duty cycle */				        
//						FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO, kFTM_EdgeAlignedPwm,
//																	 0);
//						gpio_set(&PTA11,0);
//						FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO2, kFTM_EdgeAlignedPwm,
//																	 (uint16_t)(dutyConverted));
//						/* Software trigger to update registers */
//						FTM_SetSoftwareTrigger(MotorFTM_BASEADDR, true);
								FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)4, kFTM_EdgeAlignedPwm,
																	 (uint16_t)3000);
	
		//gpio_set(&PTE10,0);	
								FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)5, kFTM_EdgeAlignedPwm,
																	 (uint16_t)0);
	  //gpio_set(&PTE11,0);	
								FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)6, kFTM_EdgeAlignedPwm,
																	 (uint16_t)3000);
	
		//gpio_set(&PTE12,0);	
								FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)7, kFTM_EdgeAlignedPwm,
																	 (uint16_t)0);
								FTM_SetSoftwareTrigger(MotorFTM_BASEADDR1, true);
			}
			else{
						FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)4, kFTM_EdgeAlignedPwm,
																	 (uint16_t)0);
	
		//gpio_set(&PTE10,0);	
						FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)5, kFTM_EdgeAlignedPwm,
																	 (uint16_t)3000);
	  //gpio_set(&PTE11,0);	
						FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)6, kFTM_EdgeAlignedPwm,
																	 (uint16_t)0);
	
		//gpio_set(&PTE12,0);	
						FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)7, kFTM_EdgeAlignedPwm,
																	 (uint16_t)3000);
						FTM_SetSoftwareTrigger(MotorFTM_BASEADDR1, true);
			}
}

void MotorPWMChange1(int32_t duty){
		int32_t dutyConverted = 0;
//		gpio_set(&PTA8,0);	
		if(duty>0){
				dutyConverted = duty*FTM_PWM_Precision/MotorPwmDutyPrecision;
				FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO3, kFTM_EdgeAlignedPwm,
																	 (uint16_t)dutyConverted);
				FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO4, kFTM_EdgeAlignedPwm,
																	 (uint16_t)0);		
				FTM_SetSoftwareTrigger(MotorFTM_BASEADDR1, true);
		}
		else{
				duty=-duty;
				dutyConverted = duty*FTM_PWM_Precision/MotorPwmDutyPrecision;
				FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO3, kFTM_EdgeAlignedPwm,
																	 (uint16_t)0);
				FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO4, kFTM_EdgeAlignedPwm,
																	 (uint16_t)dutyConverted);
			  FTM_SetSoftwareTrigger(MotorFTM_BASEADDR1, true);
		}
//		/* Software trigger to update registers */
//		FTM_SetSoftwareTrigger(MotorFTM_BASEADDR, true);
}

void MotorPWMChange2(int32_t duty){
		int32_t dutyConverted = 0;
//		gpio_set(&PTA8,0);	
		if(duty>0){
				dutyConverted = duty*FTM_PWM_Precision/MotorPwmDutyPrecision;
				FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO6, kFTM_EdgeAlignedPwm,
																	 (uint16_t)dutyConverted);
				FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO5, kFTM_EdgeAlignedPwm,
																	 (uint16_t)0);		
				FTM_SetSoftwareTrigger(MotorFTM_BASEADDR1, true);
		}
		else{
				duty=-duty;
				dutyConverted = duty*FTM_PWM_Precision/MotorPwmDutyPrecision;
				FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO6, kFTM_EdgeAlignedPwm,
																	 (uint16_t)0);
				FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO5, kFTM_EdgeAlignedPwm,
																	 (uint16_t)dutyConverted);
			  FTM_SetSoftwareTrigger(MotorFTM_BASEADDR1, true);
		}	

//		int32_t dutyConverted = duty*FTM_PWM_Precision/MotorPwmDutyPrecision;
//		gpio_set(&PTA9,0);	
//		FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO2, kFTM_EdgeAlignedPwm,
//																	 (uint16_t)dutyConverted);
//		/* Software trigger to update registers */
//		FTM_SetSoftwareTrigger(MotorFTM_BASEADDR, true);
}

void MotorPWMConctol(int32_t duty){
		int32_t v=0;
		int32_t dutyConverted1=0;
		int32_t dutyConverted2=0;
		if(t!=0){
				dutyConverted1 = (speed_mid_huan+duty)*FTM_PWM_Precision/MotorPwmDutyPrecision;
				dutyConverted2 = (speed_mid_huan-duty)*FTM_PWM_Precision/MotorPwmDutyPrecision;
		}
		else{
				if(Single_min>=380){
						dutyConverted1 = (speed_mid+duty)*FTM_PWM_Precision/MotorPwmDutyPrecision;
						dutyConverted2 = (speed_mid-duty)*FTM_PWM_Precision/MotorPwmDutyPrecision;
				}
				else{
						dutyConverted1 = (speed_mid_turn+duty)*FTM_PWM_Precision/MotorPwmDutyPrecision;
						dutyConverted2 = (speed_mid_turn-duty)*FTM_PWM_Precision/MotorPwmDutyPrecision;
				}
		}
//		gpio_set(&PTA11,0);	
//		FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO2, kFTM_EdgeAlignedPwm,
//																	 (uint16_t)dutyConverted2);
//	
//		gpio_set(&PTA10,0);	
//		FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO1, kFTM_EdgeAlignedPwm,
//																	 (uint16_t)dutyConverted1);
//		FTM_SetSoftwareTrigger(MotorFTM_BASEADDR, true);
}

void Motorstop(int32_t duty){
			FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO3, kFTM_EdgeAlignedPwm,
																	 (uint16_t)0);
			FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO4, kFTM_EdgeAlignedPwm,
																	 (uint16_t)0);
			FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO5, kFTM_EdgeAlignedPwm,
																	 (uint16_t)0);
			FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR1, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO6, kFTM_EdgeAlignedPwm,
																	 (uint16_t)0);
			 FTM_SetSoftwareTrigger(MotorFTM_BASEADDR1, true);
//		int32_t dutyConverted1 = (duty)*FTM_PWM_Precision/MotorPwmDutyPrecision;
//		int32_t dutyConverted2 = (duty)*FTM_PWM_Precision/MotorPwmDutyPrecision;
//		gpio_set(&PTA11,0);	
//		FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO2, kFTM_EdgeAlignedPwm,
//																	 (uint16_t)dutyConverted2);
//	
//		gpio_set(&PTA10,0);	
//		FTM_UpdatePwmDutycycle(MotorFTM_BASEADDR, (ftm_chnl_t)MOTOR_FTM_CHANNEL_GO1, kFTM_EdgeAlignedPwm,
//																	 (uint16_t)dutyConverted1);
//		FTM_SetSoftwareTrigger(MotorFTM_BASEADDR, true);
}


void ServoPWMInit()
{
		 ftm_config_t ftmInfo;
		 ftm_chnl_pwm_signal_param_t ftmParam[1];
	
		port_init(ServoFTM_PWMpin_1,ServoFTM_ALT );
		//port_init(ServoFTM_PWMpin_2,ServoFTM_ALT );	
		     /* Configure ftm params with frequency  */
    ftmParam[0].chnlNumber = (ftm_chnl_t)Servo_FTM_CHANNEL_1;
    ftmParam[0].level = kFTM_LowTrue;
    ftmParam[0].dutyCyclePercent = ServoMidDuty1;
    ftmParam[0].firstEdgeDelayPercent = 0U;

//    ftmParam[1].chnlNumber = (ftm_chnl_t)Servo_FTM_CHANNEL_2;
//    ftmParam[1].level = kFTM_LowTrue;
//    ftmParam[1].dutyCyclePercent = ServoMidDuty2;
//    ftmParam[1].firstEdgeDelayPercent = 0U;
	
			/*
     * ftmInfo.prescale = kFTM_Prescale_Divide_1;
     * ftmInfo.bdmMode = kFTM_BdmMode_0;
     * ftmInfo.pwmSyncMode = kFTM_SoftwareTrigger;
     * ftmInfo.reloadPoints = 0;
     * ftmInfo.faultMode = kFTM_Fault_Disable;
     * ftmInfo.faultFilterValue = 0;
     * ftmInfo.deadTimePrescale = kFTM_Deadtime_Prescale_1;
     * ftmInfo.deadTimeValue = 0;
     * ftmInfo.extTriggers = 0;
     * ftmInfo.chnlInitState = 0;
     * ftmInfo.chnlPolarity = 0;
     * ftmInfo.useGlobalTimeBase = false;
     */
    FTM_GetDefaultConfig(&ftmInfo);	
		ftmInfo.prescale = kFTM_Prescale_Divide_32;
    /* Initialize FTM module */
    FTM_Init(ServoFTM_BASEADDR, &ftmInfo);
		
		FTM_SetupPwm(ServoFTM_BASEADDR, ftmParam, 1U, kFTM_EdgeAlignedPwm, ServoPwmFrequency, CLOCK_GetFreq(kCLOCK_FastPeriphClk));//
    FTM_StartTimer(ServoFTM_BASEADDR, kFTM_SystemClock);//
}
//FTM PWM占空比精度已修改50000
void ServoPWMDuty1(int32_t duty)
{
						/* Start PWM mode with updated duty cycle */				        
						FTM_UpdatePwmDutycycle(ServoFTM_BASEADDR, (ftm_chnl_t)Servo_FTM_CHANNEL_1, kFTM_EdgeAlignedPwm,
																	 (uint16_t)duty);
						/* Software trigger to update registers */
						FTM_SetSoftwareTrigger(ServoFTM_BASEADDR, true);
}
void ServoPWMDuty2(int32_t duty)
{
//						/* Start PWM mode with updated duty cycle */				        
//						FTM_UpdatePwmDutycycle(ServoFTM_BASEADDR, (ftm_chnl_t)Servo_FTM_CHANNEL_2, kFTM_EdgeAlignedPwm,
//																	 (uint16_t)duty);
//						/* Software trigger to update registers */
//						FTM_SetSoftwareTrigger(ServoFTM_BASEADDR, true);
}
//------------------ADC接口-----------------------
ADC16_port * ADCchannel[ADN] = {
																		&ADC0_PTE11,
																		&ADC0_PTE12,
																		&ADC0_PTB4,
																		&ADC0_PTB5,
																		&ADC0_PTE18,
																		&ADC0_PTE16,
																		&ADC0_PTE19,
																		&ADC0_PTE17
															};

inline void ADC_get(uint32_t* ptr)
{
			for(uint8_t i=0;i<ADN;i++)
			{
				  *(ptr+i)= (uint32_t) ADC0_Once( (*ADCchannel[i]).ADC_channelNumber, ADC_10bit );
			}
}

//HSADC_port * HSADchannel[ADN] = {														
//																	&HSADC0_ACH2,
//																	&HSADC0_ACH3,
//																	&HSADC0_PTE29,
//																	&HSADC0_PTE30,
//																	&HSADC0_PTE16,
//																	&HSADC0_PTE17,
//																	&HSADC0_PTE20,
//																	&HSADC0_PTE21									
//																};
//inline void ADC_init()
//{
//			HSADC0A_init();
//}

//inline void ADC_get(uint32_t* ptr)
//{
//			static uint16_t AD_temp[8] = {0};
//			for(uint8_t i=0;i<10;i++)
//			{
//					HSADC0A_once(AD_temp);
//					for(uint8_t j=0;j<ADN;j++)
//				  *(ptr+j) += (uint32_t)*(AD_temp+HSADchannel[j]->sampleLot);				
//			}
//			for(uint8_t j=0;j<ADN;j++)
//				  *(ptr+j) /= 10;
//}

void uartPut_init()
{
	  port_init(TXpin,UART_ALT);
	  port_init(RXpin,UART_ALT);
		uart_init(UartBase, baudRate);
}
//----------------上位机发送-------------------
//不同的上位机，不同的命令
void vcan_sendware(int *data1,int *data2,int *data3,int *data4,int *data5,int *data6,int *data7,int *data8)
{
  uint8_t i;
#define CMD_WARE     3
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //

    uart_putbuff(UartBase, cmdf, sizeof(cmdf));    //先发送命令

    for(i=0;i<4;i++)
    uart_putchar(UartBase, *data1>>(i<<3));              //再发送图像
    for(i=0;i<4;i++)
    uart_putchar(UartBase, *data2>>(i<<3));
    for(i=0;i<4;i++)
    uart_putchar(UartBase, *data3>>(i<<3));
    for(i=0;i<4;i++)
    uart_putchar(UartBase, *data4>>(i<<3));
    for(i=0;i<4;i++)
    uart_putchar(UartBase, *data5>>(i<<3));
    for(i=0;i<4;i++)
    uart_putchar(UartBase, *data6>>(i<<3));
    for(i=0;i<4;i++)
    uart_putchar(UartBase, *data7>>(i<<3));
    for(i=0;i<4;i++)
    uart_putchar(UartBase, *data8>>(i<<3));

    uart_putbuff(UartBase, cmdr, sizeof(cmdr));    //先发送命令
}
//-----------------part of GPIO--------------------
void Beer_init()
{
    gpio_init (Beer_CHN, GPO ,0);
}

void std_io_init()
{
    gpio_init (Reed_CH1, GPI,0);//两个干簧管检测
    gpio_init (Reed_CH2, GPI,0);
}
