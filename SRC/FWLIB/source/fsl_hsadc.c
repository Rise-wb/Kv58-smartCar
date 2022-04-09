/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "fsl_hsadc.h"
#include "fsl_gpio.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*
 * Define the MACROs to help calculating the position of register field for specific sample index.
 */
/* ZXCTRL1 & ZXCTRL2. */
#define HSADC_ZXCTRL_ZCE_MASK(index) (uint16_t)(3U << (2U * ((uint16_t)(index))))
#define HSADC_ZXCTRL_ZCE(index, value) (uint16_t)(((uint16_t)(value)) << (2U * ((uint16_t)(index))))
/* CLIST1 & CLIST2 & CLIST3 & CLIST4 */
#define HSADC_CLIST_SAMPLE_MASK(index) (uint16_t)(0xFU << (4U * ((uint16_t)(index))))
#define HSADC_CLIST_SAMPLE(index, value) (uint16_t)(((uint16_t)(value)) << (4U * ((uint16_t)(index))))
//---------------HSADCModule  channelNumber  channel67MuxNumber sampleLot GPIOpin
//ADC0 Converter A
HSADC_port HSADC0_PTE16 = {	HSADC0,			0,				0,			0,	&PTE16	};
HSADC_port HSADC0_PTE17 = {	HSADC0,			1,				0,      1,  &PTE17  };
HSADC_port HSADC0_ACH2  = { HSADC0,     2,				0,      2,  NULL    };
HSADC_port HSADC0_ACH3  = { HSADC0,     3,				0,      3,  NULL    };
HSADC_port HSADC0_PTE29 = { HSADC0,     4,				0,      4,  &PTE29  };
HSADC_port HSADC0_PTE30 = { HSADC0,     5,				0,      5,  &PTE30  };
HSADC_port HSADC0_PTE20 = { HSADC0,     6,				1,		  6,	 &PTE20	 };
HSADC_port HSADC0_PTE21 = { HSADC0,     7,				1,		  7,	 &PTE21  };
//HSADC_port HSADC0_PTB2  = { HSADC0,			6,				4,			0,	&PTB2		};
//HSADC_port HSADC0_PTA17 = { HSADC0,			7,				4,			0,	&PTA17	};
//HSADC_port HSADC0_ASE7a= {  HSADC0,     6,				0,		0,		NULL		};
//HSADC_port HSADC0_ACH6 = {	HSADC0,  		6,				2,  	0,		NULL		};
//HSADC_port HSADC0_VREFH = {	HSADC0,			6,				5,		0,			NULL	};
//HSADC_port HSADC0_DAC0OUT={ HSADC0,     6,				6,		0,			NULL	};
//HSADC_port HSADC0_ASEb = {	HSADC0,			7,				0,		0,		  NULL	};
//HSADC_port HSADC0_ACH7 = {  HSADC0,			7,				2,		0,  		NULL	};
//ADC0 Converter B
HSADC_port HSADC0_PTE18 = {	HSADC0,			8,				0,		  8,	&PTE18	};
HSADC_port HSADC0_PTE19 = { HSADC0,			9,				0,		  9,	&PTE19	};
HSADC_port HSADC0_PTB0  = { HSADC0,			10,				0,			10,	&PTB0		};
HSADC_port HSADC0_PTB1  = {	HSADC0,			11,				0,			11,	&PTB1		};
HSADC_port HSADC0_PTE24 = {	HSADC0,			12,				0,			12,	&PTE24	};
HSADC_port HSADC0_PTE25 = {	HSADC0,			13,				0,			13,	&PTE25	};
HSADC_port HSADC0_PTB10 = {	HSADC0,			14,				0,			14,	&PTB10	};
HSADC_port HSADC0_PTB11 = {	HSADC0,			15,				0,			15,	&PTB11 	};
//HSADC_port HSADC0_PTC0  = {	HSADC0,			14,				1,		15,	&PTC0 	};
//HSADC_port HSADC0_PTB3  = {	HSADC0,			15,				4,		15, &PTB3 	};
//HSADC_port HSADC0_PTE2  = {	HSADC0,			14,				2,		15,		&PTE2 	};
//HSADC_port HSADC0_PTE0  = {	HSADC0,			14,				5,		15,		&PTE0 	};
//HSADC_port HSADC0_PTE3  = {	HSADC0,			15,				2,		15, 	&PTE3 	};
//HSADC_port HSADC0_PTC1  = {	HSADC0,			15,				1,		15,	  &PTC1 	};
//HSADC_port HSADC0_PTE1  = {	HSADC0,			15,				5,		15, 	&PTE1 	};
//ADC1 Converter A
HSADC_port HSADC1_PTE0 	=  { HSADC1,	 		0,			0,     	0,	 &PTE0	};
HSADC_port HSADC1_PTE1 	=  { HSADC1,	 		1,			0,     	1,	 &PTE1	};
HSADC_port HSADC1_ACH2	=  { HSADC1,	 		2,			0,     	2,	 NULL 	};
HSADC_port HSADC1_ACH3	=  { HSADC1,			3,			0,			3,	 NULL		};
HSADC_port HSADC1_PTE4 	=  { HSADC1,	 		4,			0,     	4,	 &PTE4	};
HSADC_port HSADC1_PTE5 	=  { HSADC1,	 		5,			0,     	5,	 &PTE5	};
HSADC_port HSADC1_PTE11 =  { HSADC1,			6,			0,			6,	 &PTE11	};
HSADC_port HSADC1_PTE21 =  { HSADC1,			7,			0,			7,	 &PTE21 };
//HSADC_port HSADC1_PTB6	=  { HSADC1,			6,			3,			0,	 &PTB6	};
//HSADC_port HSADC1_PTB7	=  { HSADC1,			7,			3,			0,	 &PTB7  };
//HSADC_port HSADC1_PTD5  =  { HSADC1,			6,			1,      0,	 &PTD5 	};
//HSADC_port HSADC1_PTD6  =  { HSADC1,			7,			1,  		0,	 &PTD6  };
//HSADC_port HSADC1_PTD1  =  { HSADC1,			7,			2,			0,	 &PTD1  };
//ADC1 COnverter B
HSADC_port HSADC1_PTE2	=	 { HSADC1,      8,      0,			8,   &PTE2  };
HSADC_port HSADC1_PTE3	=	 { HSADC1,      9,			0,			9,   &PTE3  };
HSADC_port HSADC1_BCH2  =  { HSADC1,			10,			0,			10,  NULL   };
HSADC_port HSADC1_BCH3	=  { HSADC1,			11,			0,			11,  NULL		};
HSADC_port HSADC1_PTE24 =  { HSADC1,			12,     0,			12,  &PTE24 };
HSADC_port HSADC1_PTE25 =  { HSADC1,      13,     0,      13,  &PTE25 };
HSADC_port HSADC1_PTA7  =  { HSADC1,			14,			1,      14,  &PTA7  };
HSADC_port HSADC1_PTA8	=  { HSADC1,			15,			1,			15,	 &PTA8  };
//HSADC_port HSADC1_PTE12 =  { HSADC1,      14,     0,      15,  &PTE12 };
//HSADC_port HSADC1_PTC9  =  { HSADC1,			14,     3,			15,  &PTC9  };
//HSADC_port HSADC1_PTE6	=  { HSADC1,			15,			0,			15,	 &PTE6  };
//HSADC_port HSADC1_PTC8	=  { HSADC1,			15,			2,			15,	 &PTC8  };
//HSADC_port HSADC1_PTC2	=  { HSADC1,      14,     2,			15,  &PTC2  };
//HSADC_port HSADC1_PTC11 =  { HSADC1,      14,     4,      15,  &PTC11 };
//HSADC_port HSADC1_PTC10 =  { HSADC1,      15,     3,			15,	 &PTC10 };

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
 void HSADC0A_init()
 {
			    hsadc_config_t hsadcConfigStruct;
					hsadc_converter_config_t hsadcConverterConfigStruct;
					hsadc_sample_config_t hsadcSampleConfigStruct;
					uint16_t sampleMask = 0u;
	 
					port_init(HSADC0_PTE16.GPIOpin,ALT0);
					port_init(HSADC0_PTE17.GPIOpin,ALT0);
				  port_init(HSADC0_PTE29.GPIOpin,ALT0);
				  port_init(HSADC0_PTE30.GPIOpin,ALT0);
				  port_init(HSADC0_PTE20.GPIOpin,ALT0);
				  port_init(HSADC0_PTE21.GPIOpin,ALT0);
//				  port_init(HSADC0_ACH2.GPIOpin,ALT0);
//				  port_init(HSADC0_ACH3.GPIOpin,ALT0);					//HSADC0_ACH2
					 
				hsadcConfigStruct.dualConverterScanMode = kHSADC_DualConverterWorkAsTriggeredSequential  ;//
				hsadcConfigStruct.enableSimultaneousMode = false;
	      hsadcConfigStruct.resolution = kHSADC_Resolution10Bit ;
				hsadcConfigStruct.DMATriggerSoruce = kHSADC_DMATriggerSourceAsEndOfScan;
	      hsadcConfigStruct.idleWorkMode = kHSADC_IdleKeepNormal;
	      hsadcConfigStruct.powerUpDelayCount = 18u;
	 
	      hsadcConverterConfigStruct.clockDivisor = 4u;
				hsadcConverterConfigStruct.samplingTimeCount = 0u;
	      hsadcConverterConfigStruct.powerUpCalibrationModeMask = 0;//kHSADC_CalibrationModeSingleEnded;//0u;
	    	      
				hsadcSampleConfigStruct.enableDifferentialPair = false;
				hsadcSampleConfigStruct.zeroCrossingMode = kHSADC_ZeroCorssingDisabled;
				hsadcSampleConfigStruct.highLimitValue = 0x7FF8U;
				hsadcSampleConfigStruct.lowLimitValue  = 0;
				hsadcSampleConfigStruct.offsetValue  = 0;
				hsadcSampleConfigStruct.enableWaitSync = false;
				
				/* Initialize the HSADC common digital control.*/
				HSADC_Init(HSADC0, &hsadcConfigStruct);
				/* Configure each converter. */
				HSADC_SetConverterConfig(HSADC0,  kHSADC_ConverterA , &hsadcConverterConfigStruct);
				/* Enable the power for each converter. */
				HSADC_EnableConverterPower(HSADC0,   kHSADC_ConverterA, true);
				while (
						(kHSADC_ConverterAPowerDownFlag ) ==
						((kHSADC_ConverterAPowerDownFlag ) & HSADC_GetStatusFlags(HSADC0)))
				{
				}				
				/* Make each converter exit stop mode. */
				HSADC_EnableConverter(HSADC0,kHSADC_ConverterA , true);
				/*config the samplelot struct*/
				hsadcSampleConfigStruct.channelNumber = HSADC0_PTE16.HSADC_channelNumber;   
	      hsadcSampleConfigStruct.channel67MuxNumber = HSADC0_PTE16.HSADC_channel67MuxNumber;
				HSADC_SetSampleConfig(HSADC0,HSADC0_PTE16.sampleLot, &hsadcSampleConfigStruct);
				
				hsadcSampleConfigStruct.channelNumber = HSADC0_PTE17.HSADC_channelNumber;   
	      hsadcSampleConfigStruct.channel67MuxNumber = HSADC0_PTE17.HSADC_channel67MuxNumber;
				HSADC_SetSampleConfig(HSADC0,HSADC0_PTE17.sampleLot, &hsadcSampleConfigStruct);
				
				hsadcSampleConfigStruct.channelNumber = HSADC0_ACH2.HSADC_channelNumber;   
	      hsadcSampleConfigStruct.channel67MuxNumber = HSADC0_ACH2.HSADC_channel67MuxNumber;
				HSADC_SetSampleConfig(HSADC0,HSADC0_ACH2.sampleLot, &hsadcSampleConfigStruct);
				
				hsadcSampleConfigStruct.channelNumber = HSADC0_ACH3.HSADC_channelNumber;   
	      hsadcSampleConfigStruct.channel67MuxNumber = HSADC0_ACH3.HSADC_channel67MuxNumber;
				HSADC_SetSampleConfig(HSADC0,HSADC0_ACH3.sampleLot, &hsadcSampleConfigStruct);
				
				hsadcSampleConfigStruct.channelNumber = HSADC0_PTE29.HSADC_channelNumber;   
	      hsadcSampleConfigStruct.channel67MuxNumber = HSADC0_PTE29.HSADC_channel67MuxNumber;
				HSADC_SetSampleConfig(HSADC0,HSADC0_PTE29.sampleLot, &hsadcSampleConfigStruct);
				
				hsadcSampleConfigStruct.channelNumber = HSADC0_PTE30.HSADC_channelNumber;   
	      hsadcSampleConfigStruct.channel67MuxNumber = HSADC0_PTE30.HSADC_channel67MuxNumber;
				HSADC_SetSampleConfig(HSADC0,HSADC0_PTE30.sampleLot, &hsadcSampleConfigStruct);
				
				hsadcSampleConfigStruct.channelNumber = HSADC0_PTE20.HSADC_channelNumber;   
	      hsadcSampleConfigStruct.channel67MuxNumber = HSADC0_PTE20.HSADC_channel67MuxNumber;
				HSADC_SetSampleConfig(HSADC0,HSADC0_PTE20.sampleLot, &hsadcSampleConfigStruct);
				
				hsadcSampleConfigStruct.channelNumber = HSADC0_PTE21.HSADC_channelNumber;   
	      hsadcSampleConfigStruct.channel67MuxNumber = HSADC0_PTE21.HSADC_channel67MuxNumber;
				HSADC_SetSampleConfig(HSADC0,HSADC0_PTE21.sampleLot, &hsadcSampleConfigStruct);
				
				hsadcSampleConfigStruct.channelNumber = HSADC0_ACH2.HSADC_channelNumber;   
	      hsadcSampleConfigStruct.channel67MuxNumber = HSADC0_ACH2.HSADC_channel67MuxNumber;
				HSADC_SetSampleConfig(HSADC0,HSADC0_ACH2.sampleLot, &hsadcSampleConfigStruct);
				
				hsadcSampleConfigStruct.channelNumber = HSADC0_ACH2.HSADC_channelNumber;   
	      hsadcSampleConfigStruct.channel67MuxNumber = HSADC0_ACH2.HSADC_channel67MuxNumber;
				HSADC_SetSampleConfig(HSADC0,HSADC0_ACH2.sampleLot, &hsadcSampleConfigStruct);				
				/*enable the sample lot*/				
				sampleMask = HSADC_SAMPLE_MASK(0)|HSADC_SAMPLE_MASK(1)|HSADC_SAMPLE_MASK(2)|HSADC_SAMPLE_MASK(3)
										 |HSADC_SAMPLE_MASK(4)|HSADC_SAMPLE_MASK(5)|HSADC_SAMPLE_MASK(6)|HSADC_SAMPLE_MASK(7);
				HSADC_EnableSample(HSADC0, sampleMask, true);
				HSADC_EnableSample(HSADC0, (uint16_t)(~sampleMask), false); /* Disable other sample slots. */		

				/* Calibrate the converter after power up period. */
				HSADC_DoAutoCalibration(HSADC0, (kHSADC_ConverterA ),(kHSADC_CalibrationModeSingleEnded ));
				/* Wait the calibration process complete. None End of Scan flag will be set after power up calibration process. */
				while ((kHSADC_ConverterAEndOfCalibrationFlag | kHSADC_ConverterAEndOfScanFlag) !=
           ((kHSADC_ConverterAEndOfCalibrationFlag | kHSADC_ConverterAEndOfScanFlag) &
            HSADC_GetStatusFlags(HSADC0)))
				{
				}
				HSADC_ClearStatusFlags(HSADC0, kHSADC_ConverterAEndOfCalibrationFlag  | kHSADC_ConverterAEndOfScanFlag );
 }
 
 void HSADC0A_once(uint16_t* HSADC_result)
 {
	          static uint16_t sampleMask = HSADC_SAMPLE_MASK(0)|HSADC_SAMPLE_MASK(1)|HSADC_SAMPLE_MASK(2)|HSADC_SAMPLE_MASK(3)
							                  |HSADC_SAMPLE_MASK(4)|HSADC_SAMPLE_MASK(5)|HSADC_SAMPLE_MASK(6)|HSADC_SAMPLE_MASK(7);
	      	      
						HSADC_DoSoftwareTriggerConverter(HSADC0, kHSADC_ConverterA);
						 /* Wait the conversion to be done. */
						while (kHSADC_ConverterAEndOfScanFlag !=
									 (kHSADC_ConverterAEndOfScanFlag & HSADC_GetStatusFlags(HSADC0)))
						{
						}
						 /* Read the result value. */
						if (sampleMask == (sampleMask & HSADC_GetSampleReadyStatusFlags(HSADC0)))
						{
								*HSADC_result 		= (HSADC_GetSampleResultValue(HSADC0, 0u)&0x7FFF)>>5;   //10bit 
                *(HSADC_result+1) = (HSADC_GetSampleResultValue(HSADC0, 1u)&0x7FFF)>>5;
								*(HSADC_result+2) = (HSADC_GetSampleResultValue(HSADC0, 2u)&0x7FFF)>>5;
								*(HSADC_result+3) = (HSADC_GetSampleResultValue(HSADC0, 3u)&0x7FFF)>>5;
								*(HSADC_result+4) = (HSADC_GetSampleResultValue(HSADC0, 4u)&0x7FFF)>>5;
								*(HSADC_result+5) = (HSADC_GetSampleResultValue(HSADC0, 5u)&0x7FFF)>>5;
							  *(HSADC_result+6) = (HSADC_GetSampleResultValue(HSADC0, 6u)&0x7FFF)>>5;
								*(HSADC_result+7) = (HSADC_GetSampleResultValue(HSADC0, 7u)&0x7FFF)>>5;
						}
						HSADC_ClearStatusFlags(HSADC0, kHSADC_ConverterAEndOfScanFlag);									
 }
/*!
 * @brief Get instance number for HSADC module.
 *
 * @param base HSADC peripheral base address.
 */
static uint32_t HSADC_GetInstance(HSADC_Type *base);

/*!
 * @brief Set channel 6/7's sub mux channel number.
 *
 * When channel number is 6/7, it represents to configure converterA's channel 6/7 sub multiplex channel number.
 * When channel number is 14/15, it represents to configure converterB's channel 6/7 sub multiplex channel number.
 * In other cases, this function won't be functional.
 *
 * @param base                   HSADC peripheral base address.
 * @param channelNumber          Channel number.
 * @param muxNumber              Channel 6/7's sub multiplex channel number.
 * @param enableDifferentialPair Enable channel 6/7 to be differential pair or not.
 */
static void HSADC_SetChannel67Mux(HSADC_Type *base,
                                  uint16_t channelNumber,
                                  uint16_t muxNumber,
                                  bool enableDifferentialPair);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to HSADC bases for each instance. */
static HSADC_Type *const s_hsadcBases[] = HSADC_BASE_PTRS;

/*! @brief Pointers to HSADC clocks for each instance. */
static const clock_ip_name_t s_hsadcClocks[] = HSADC_CLOCKS;

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t HSADC_GetInstance(HSADC_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < FSL_FEATURE_SOC_HSADC_COUNT; instance++)
    {
        if (s_hsadcBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < FSL_FEATURE_SOC_HSADC_COUNT);

    return instance;
}

void HSADC_Init(HSADC_Type *base, const hsadc_config_t *config)
{
    assert(NULL != config);
    assert(config->powerUpDelayCount <= (HSADC_PWR_PUDELAY_MASK >> HSADC_PWR_PUDELAY_SHIFT));

    uint16_t tmp16;

    /* Enable module clock. */
    CLOCK_EnableClock(s_hsadcClocks[HSADC_GetInstance(base)]);

    /* CTRL1 */
    tmp16 = (base->CTRL1 & ~HSADC_CTRL1_SMODE_MASK);
    tmp16 |= HSADC_CTRL1_SMODE(config->dualConverterScanMode);
    base->CTRL1 = tmp16;

    /* CTRL2 */
    tmp16 = (base->CTRL2 & ~HSADC_CTRL2_SIMULT_MASK);
    if (config->enableSimultaneousMode)
    {
        tmp16 |= HSADC_CTRL2_SIMULT_MASK;
    }
    base->CTRL2 = tmp16;

    /* CTRL3 */
    tmp16 = (base->CTRL3 & ~(HSADC_CTRL3_ADCRES_MASK | HSADC_CTRL3_DMASRC_MASK));
    tmp16 |= (HSADC_CTRL3_ADCRES(config->resolution) | HSADC_CTRL3_DMASRC(config->DMATriggerSoruce));
    base->CTRL3 = tmp16;

    /* PWR */
    tmp16 = (base->PWR & ~(HSADC_PWR_ASB_MASK | HSADC_PWR_APD_MASK | HSADC_PWR_PUDELAY_MASK));
    switch (config->idleWorkMode)
    {
        case kHSADC_IdleKeepNormal:
            break;
        case kHSADC_IdleAutoStandby:
            tmp16 |= HSADC_PWR_ASB_MASK;
            break;
        case kHSADC_IdleAutoPowerDown:
            tmp16 |= HSADC_PWR_APD_MASK;
            break;
        default:
            break;
    }
    tmp16 |= HSADC_PWR_PUDELAY(config->powerUpDelayCount);
    base->PWR = tmp16;
}

void HSADC_GetDefaultConfig(hsadc_config_t *config)
{
    assert(NULL != config);

    config->dualConverterScanMode = kHSADC_DualConverterWorkAsTriggeredParallel;
    config->enableSimultaneousMode = true;
    config->resolution = kHSADC_Resolution12Bit;
    config->DMATriggerSoruce = kHSADC_DMATriggerSourceAsEndOfScan;
    config->idleWorkMode = kHSADC_IdleKeepNormal;
    config->powerUpDelayCount = 18U;
}

void HSADC_Deinit(HSADC_Type *base)
{
    /* Power down both converter. */
    base->PWR &= ~(HSADC_PWR_PDA_MASK | HSADC_PWR_PDB_MASK);

    /* Disable module clock. */
    CLOCK_DisableClock(s_hsadcClocks[HSADC_GetInstance(base)]);
}

void HSADC_SetConverterConfig(HSADC_Type *base, uint16_t converterMask, const hsadc_converter_config_t *config)
{
    assert(NULL != config);
    /* Check the divisor value's range. */
    assert((config->clockDivisor >= 2U) &&
           (config->clockDivisor <= ((HSADC_CTRL2_DIVA_MASK >> HSADC_CTRL2_DIVA_SHIFT) + 1U)));

    uint16_t tmp16;

    if (kHSADC_ConverterA == (kHSADC_ConverterA & converterMask))
    {
        assert(config->samplingTimeCount <= (HSADC_SAMPTIM_SAMPT_A_MASK >> HSADC_SAMPTIM_SAMPT_A_SHIFT));

        /* CTRL2 */
        tmp16 = (base->CTRL2 & ~HSADC_CTRL2_DIVA_MASK);
        tmp16 |= HSADC_CTRL2_DIVA(config->clockDivisor - 1U);
        base->CTRL2 = tmp16;

        /* SAMPTIM */
        tmp16 = (base->SAMPTIM & ~HSADC_SAMPTIM_SAMPT_A_MASK);
        tmp16 |= HSADC_SAMPTIM_SAMPT_A(config->samplingTimeCount);
        base->SAMPTIM = tmp16;

        /* CALIB */
        tmp16 = (base->CALIB & ~(HSADC_CALIB_REQSINGA_MASK | HSADC_CALIB_REQDIFA_MASK));
        if (kHSADC_CalibrationModeSingleEnded & config->powerUpCalibrationModeMask)
        {
            tmp16 |= HSADC_CALIB_REQSINGA_MASK;
        }
        if (kHSADC_CalibrationModeDifferential & config->powerUpCalibrationModeMask)
        {
            tmp16 |= HSADC_CALIB_REQDIFA_MASK;
        }
        base->CALIB = tmp16;
    }

    if (kHSADC_ConverterB == (kHSADC_ConverterB & converterMask))
    {
        assert(config->samplingTimeCount <= (HSADC_SAMPTIM_SAMPT_B_MASK >> HSADC_SAMPTIM_SAMPT_B_SHIFT));

        /* PWR2 */
        tmp16 = (base->PWR2 & ~HSADC_PWR2_DIVB_MASK);
        tmp16 |= HSADC_PWR2_DIVB(config->clockDivisor - 1U);
        base->PWR2 = tmp16;

        /* SAMPTIM */
        tmp16 = (base->SAMPTIM & ~HSADC_SAMPTIM_SAMPT_B_MASK);
        tmp16 |= HSADC_SAMPTIM_SAMPT_B(config->samplingTimeCount);
        base->SAMPTIM = tmp16;

        /* CALIB */
        tmp16 = (base->CALIB & ~(HSADC_CALIB_REQSINGB_MASK | HSADC_CALIB_REQDIFB_MASK));
        if (kHSADC_CalibrationModeSingleEnded & config->powerUpCalibrationModeMask)
        {
            tmp16 |= HSADC_CALIB_REQSINGB_MASK;
        }
        if (kHSADC_CalibrationModeDifferential & config->powerUpCalibrationModeMask)
        {
            tmp16 |= HSADC_CALIB_REQDIFB_MASK;
        }
        base->CALIB = tmp16;
    }
}

void HSADC_GetDefaultConverterConfig(hsadc_converter_config_t *config)
{
    assert(NULL != config);

    config->clockDivisor = 5U;
    config->samplingTimeCount = 0U;
    config->powerUpCalibrationModeMask = 0U;
}

void HSADC_EnableConverter(HSADC_Type *base, uint16_t converterMask, bool enable)
{
    /* CTRL1 */
    if (kHSADC_ConverterA == (kHSADC_ConverterA & converterMask))
    {
        if (true == enable)
        {
            base->CTRL1 &= ~HSADC_CTRL1_STOPA_MASK;
        }
        else
        {
            base->CTRL1 |= HSADC_CTRL1_STOPA_MASK;
        }
    }
    /* CTRL2 */
    if (kHSADC_ConverterB == (kHSADC_ConverterB & converterMask))
    {
        if (true == enable)
        {
            base->CTRL2 &= ~HSADC_CTRL2_STOPB_MASK;
        }
        else
        {
            base->CTRL2 |= HSADC_CTRL2_STOPB_MASK;
        }
    }
}

void HSADC_EnableConverterSyncInput(HSADC_Type *base, uint16_t converterMask, bool enable)
{
    /* CTRL1 */
    if (kHSADC_ConverterA == (kHSADC_ConverterA & converterMask))
    {
        if (true == enable)
        {
            base->CTRL1 |= HSADC_CTRL1_SYNCA_MASK;
        }
        else
        {
            base->CTRL1 &= ~HSADC_CTRL1_SYNCA_MASK;
        }
    }
    /* CTRL2 */
    if (kHSADC_ConverterB == (kHSADC_ConverterB & converterMask))
    {
        if (true == enable)
        {
            base->CTRL2 |= HSADC_CTRL2_SYNCB_MASK;
        }
        else
        {
            base->CTRL2 &= ~HSADC_CTRL2_SYNCB_MASK;
        }
    }
}

void HSADC_EnableConverterPower(HSADC_Type *base, uint16_t converterMask, bool enable)
{
    /* PWR */
    if (kHSADC_ConverterA == (kHSADC_ConverterA & converterMask))
    {
        if (true == enable)
        {
            base->PWR &= ~HSADC_PWR_PDA_MASK;
        }
        else
        {
            base->PWR |= HSADC_PWR_PDA_MASK;
        }
    }
    if (kHSADC_ConverterB == (kHSADC_ConverterB & converterMask))
    {
        if (true == enable)
        {
            base->PWR &= ~HSADC_PWR_PDB_MASK;
        }
        else
        {
            base->PWR |= HSADC_PWR_PDB_MASK;
        }
    }
}

void HSADC_DoSoftwareTriggerConverter(HSADC_Type *base, uint16_t converterMask)
{
    /* CTRL1 */
    if (kHSADC_ConverterA == (kHSADC_ConverterA & converterMask))
    {
        base->CTRL1 |= HSADC_CTRL1_STARTA_MASK;
    }
    /* CTRL2 */
    if (kHSADC_ConverterB == (kHSADC_ConverterB & converterMask))
    {
        base->CTRL2 |= HSADC_CTRL2_STARTB_MASK;
    }
}

void HSADC_EnableConverterDMA(HSADC_Type *base, uint16_t converterMask, bool enable)
{
    /* CTRL1 */
    if (kHSADC_ConverterA == (kHSADC_ConverterA & converterMask))
    {
        if (true == enable)
        {
            base->CTRL1 |= HSADC_CTRL1_DMAENA_MASK;
        }
        else
        {
            base->CTRL1 &= ~HSADC_CTRL1_DMAENA_MASK;
        }
    }
    /* CTRL2 */
    if (kHSADC_ConverterB == (kHSADC_ConverterB & converterMask))
    {
        if (true == enable)
        {
            base->CTRL2 |= HSADC_CTRL2_DMAENB_MASK;
        }
        else
        {
            base->CTRL2 &= ~HSADC_CTRL2_DMAENB_MASK;
        }
    }
}

void HSADC_EnableInterrupts(HSADC_Type *base, uint16_t mask)
{
    uint16_t tmp16;

    /* CTRL1 */
    tmp16 = base->CTRL1;
    if (kHSADC_ZeroCrossingInterruptEnable == (kHSADC_ZeroCrossingInterruptEnable & mask))
    {
        tmp16 |= HSADC_CTRL1_ZCIE_MASK;
    }
    if (kHSADC_HighLimitInterruptEnable == (kHSADC_HighLimitInterruptEnable & mask))
    {
        tmp16 |= HSADC_CTRL1_HLMTIE_MASK;
    }
    if (kHSADC_LowLimitInterruptEnable == (kHSADC_LowLimitInterruptEnable & mask))
    {
        tmp16 |= HSADC_CTRL1_LLMTIE_MASK;
    }
    if (kHSADC_ConverterAEndOfScanInterruptEnable == (kHSADC_ConverterAEndOfScanInterruptEnable & mask))
    {
        tmp16 |= HSADC_CTRL1_EOSIEA_MASK;
    }
    base->CTRL1 = tmp16;

    /* CTRL2 */
    if (kHSADC_ConverterBEndOfScanInterruptEnable == (kHSADC_ConverterBEndOfScanInterruptEnable & mask))
    {
        base->CTRL2 |= HSADC_CTRL2_EOSIEB_MASK;
    }

    /* CALIB */
    tmp16 = base->CALIB;
    if (kHSADC_ConverterAEndOfCalibrationInterruptEnable == (kHSADC_ConverterAEndOfCalibrationInterruptEnable & mask))
    {
        tmp16 |= HSADC_CALIB_EOCALIEA_MASK;
    }
    if (kHSADC_ConverterBEndOfCalibrationInterruptEnable == (kHSADC_ConverterBEndOfCalibrationInterruptEnable & mask))
    {
        tmp16 |= HSADC_CALIB_EOCALIEB_MASK;
    }
    base->CALIB = tmp16;
}

void HSADC_DisableInterrupts(HSADC_Type *base, uint16_t mask)
{
    uint16_t tmp16;

    /* CTRL1 */
    tmp16 = base->CTRL1;
    if (kHSADC_ZeroCrossingInterruptEnable == (kHSADC_ZeroCrossingInterruptEnable & mask))
    {
        tmp16 &= HSADC_CTRL1_ZCIE_MASK;
    }
    if (kHSADC_HighLimitInterruptEnable == (kHSADC_HighLimitInterruptEnable & mask))
    {
        tmp16 &= HSADC_CTRL1_HLMTIE_MASK;
    }
    if (kHSADC_LowLimitInterruptEnable == (kHSADC_LowLimitInterruptEnable & mask))
    {
        tmp16 &= HSADC_CTRL1_LLMTIE_MASK;
    }
    if (kHSADC_ConverterAEndOfScanInterruptEnable == (kHSADC_ConverterAEndOfScanInterruptEnable & mask))
    {
        tmp16 &= HSADC_CTRL1_EOSIEA_MASK;
    }
    base->CTRL1 = tmp16;

    /* CTRL2 */
    if (kHSADC_ConverterBEndOfScanInterruptEnable == (kHSADC_ConverterBEndOfScanInterruptEnable & mask))
    {
        base->CTRL2 &= ~HSADC_CTRL2_EOSIEB_MASK;
    }

    /* CALIB */
    tmp16 = base->CALIB;
    if (kHSADC_ConverterAEndOfCalibrationInterruptEnable == (kHSADC_ConverterAEndOfCalibrationInterruptEnable & mask))
    {
        tmp16 &= HSADC_CALIB_EOCALIEA_MASK;
    }
    if (kHSADC_ConverterBEndOfCalibrationInterruptEnable == (kHSADC_ConverterBEndOfCalibrationInterruptEnable & mask))
    {
        tmp16 &= HSADC_CALIB_EOCALIEB_MASK;
    }
    base->CALIB = tmp16;
}

uint16_t HSADC_GetStatusFlags(HSADC_Type *base)
{
    uint16_t tmp16;
    uint16_t status = 0U;

    /* STAT */
    tmp16 = base->STAT;
    if (HSADC_STAT_ZCI_MASK == (tmp16 & HSADC_STAT_ZCI_MASK))
    {
        status |= kHSADC_ZeroCrossingFlag;
    }
    if (HSADC_STAT_HLMTI_MASK == (tmp16 & HSADC_STAT_HLMTI_MASK))
    {
        status |= kHSADC_HighLimitFlag;
    }
    if (HSADC_STAT_LLMTI_MASK == (tmp16 & HSADC_STAT_LLMTI_MASK))
    {
        status |= kHSADC_LowLimitFlag;
    }
    if (HSADC_STAT_EOSIA_MASK == (tmp16 & HSADC_STAT_EOSIA_MASK))
    {
        status |= kHSADC_ConverterAEndOfScanFlag;
    }
    if (HSADC_STAT_EOSIB_MASK == (tmp16 & HSADC_STAT_EOSIB_MASK))
    {
        status |= kHSADC_ConverterBEndOfScanFlag;
    }
    if (HSADC_STAT_EOCALIA_MASK == (tmp16 & HSADC_STAT_EOCALIA_MASK))
    {
        status |= kHSADC_ConverterAEndOfCalibrationFlag;
    }
    if (HSADC_STAT_EOCALIB_MASK == (tmp16 & HSADC_STAT_EOCALIB_MASK))
    {
        status |= kHSADC_ConverterBEndOfCalibrationFlag;
    }
    if (HSADC_STAT_CIPA_MASK == (tmp16 & HSADC_STAT_CIPA_MASK))
    {
        status |= kHSADC_ConverterAConvertingFlag;
    }
    if (HSADC_STAT_CIPB_MASK == (tmp16 & HSADC_STAT_CIPB_MASK))
    {
        status |= kHSADC_ConverterBConvertingFlag;
    }
    if (HSADC_STAT_DUMMYA_MASK == (tmp16 & HSADC_STAT_DUMMYA_MASK))
    {
        status |= kHSADC_ConverterADummyConvertingFlag;
    }
    if (HSADC_STAT_DUMMYB_MASK == (tmp16 & HSADC_STAT_DUMMYB_MASK))
    {
        status |= kHSADC_ConverterBDummyConvertingFlag;
    }
    if (HSADC_STAT_CALONA_MASK == (tmp16 & HSADC_STAT_CALONA_MASK))
    {
        status |= kHSADC_ConverterACalibratingFlag;
    }
    if (HSADC_STAT_CALONB_MASK == (tmp16 & HSADC_STAT_CALONB_MASK))
    {
        status |= kHSADC_ConverterBCalibratingFlag;
    }

    /* PWR */
    tmp16 = base->PWR;
    if (HSADC_PWR_PSTSA_MASK == (tmp16 & HSADC_PWR_PSTSA_MASK))
    {
        status |= kHSADC_ConverterAPowerDownFlag;
    }
    if (HSADC_PWR_PSTSB_MASK == (tmp16 & HSADC_PWR_PSTSB_MASK))
    {
        status |= kHSADC_ConverterBPowerDownFlag;
    }

    return status;
}

void HSADC_ClearStatusFlags(HSADC_Type *base, uint16_t mask)
{
    uint16_t tmp16;

    if (kHSADC_ZeroCrossingFlag == (mask & kHSADC_ZeroCrossingFlag))
    {
        base->ZXSTAT = 0xFFFFU;
    }
    if (kHSADC_HighLimitFlag == (mask & kHSADC_HighLimitFlag))
    {
        base->HILIMSTAT = 0xFFFFU;
    }
    if (kHSADC_LowLimitFlag == (mask & kHSADC_LowLimitFlag))
    {
        base->LOLIMSTAT = 0xFFFFU;
    }
    /* STAT */
    tmp16 = base->STAT;
    if (kHSADC_ConverterAEndOfScanFlag == (mask & kHSADC_ConverterAEndOfScanFlag))
    {
        tmp16 |= HSADC_STAT_EOSIA_MASK;
    }
    if (kHSADC_ConverterBEndOfScanFlag == (mask & kHSADC_ConverterBEndOfScanFlag))
    {
        tmp16 |= HSADC_STAT_EOSIB_MASK;
    }
    if (kHSADC_ConverterAEndOfCalibrationFlag == (mask & kHSADC_ConverterAEndOfCalibrationFlag))
    {
        tmp16 |= HSADC_STAT_EOCALIA_MASK;
    }
    if (kHSADC_ConverterBEndOfCalibrationFlag == (mask & kHSADC_ConverterBEndOfCalibrationFlag))
    {
        tmp16 |= HSADC_STAT_EOCALIB_MASK;
    }
    base->STAT = tmp16;
}

static void HSADC_SetChannel67Mux(HSADC_Type *base,
                                  uint16_t channelNumber,
                                  uint16_t muxNumber,
                                  bool enableDifferentialPair)
{
    uint16_t tmp16;

    /* MUX67_SEL */
    /* When channel number is 6/7, it represents converterA's channel 6/7. */
    /* When channel number is 14/15, it represents converterB's channel 6/7. */
    tmp16 = base->MUX67_SEL;
    /* For differential mode. */
    if (true == enableDifferentialPair)
    {
        switch (channelNumber)
        {
            case 6U:
            case 7U:
                tmp16 &= ~(HSADC_MUX67_SEL_CH6_SELA_MASK | HSADC_MUX67_SEL_CH7_SELA_MASK);
                tmp16 |= (HSADC_MUX67_SEL_CH6_SELA(muxNumber) | HSADC_MUX67_SEL_CH7_SELA(muxNumber));
                break;
            case 14U:
            case 15U:
                tmp16 &= ~(HSADC_MUX67_SEL_CH6_SELB_MASK | HSADC_MUX67_SEL_CH7_SELB_MASK);
                tmp16 |= (HSADC_MUX67_SEL_CH6_SELB(muxNumber) | HSADC_MUX67_SEL_CH7_SELB(muxNumber));
                break;
            default:
                break;
        }
    }
    else /* For single ended mode. */
    {
        switch (channelNumber)
        {
            case 6U:
                tmp16 &= ~HSADC_MUX67_SEL_CH6_SELA_MASK;
                tmp16 |= HSADC_MUX67_SEL_CH6_SELA(muxNumber);
                break;
            case 7U:
                tmp16 &= ~HSADC_MUX67_SEL_CH7_SELA_MASK;
                tmp16 |= HSADC_MUX67_SEL_CH7_SELA(muxNumber);
                break;
            case 14U:
                tmp16 &= ~HSADC_MUX67_SEL_CH6_SELB_MASK;
                tmp16 |= HSADC_MUX67_SEL_CH6_SELB(muxNumber);
                break;
            case 15U:
                tmp16 &= ~HSADC_MUX67_SEL_CH7_SELB_MASK;
                tmp16 |= HSADC_MUX67_SEL_CH7_SELB(muxNumber);
                break;
            default:
                break;
        }
    }
    base->MUX67_SEL = tmp16;
}

/*
 * Mask table for channel differential pair setting.
 * The item's value would be set into CTRL1[CHNCFG_L] or CTRL2[CHNCFG_H].
 */
const uint16_t g_HSADCChannelConfigDifferentialMask[] = {
    0x0010U, /* CHN0-1,   ANA0-ANA1, CTRL1[CHNCFG_L]. */
    0x0020U, /* CHN2-3,   ANA2-ANA3. CTRL1[CHNCFG_L]. */
    0x0080U, /* CHN4-5,   ANA4-ANA5. CTRL2[CHNCFG_H]. */
    0x0100U, /* CHN6-7,   ANA6-ANA7. CTRL2[CHNCFG_H]. */
    0x0040U, /* CHN8-9,   ANB0-ANB1. CTRL1[CHNCFG_L]. */
    0x0080U, /* CHN10-11, ANB2-ANB3. CTRL1[CHNCFG_L]. */
    0x0200U, /* CHN12-13, ANB4-ANB5. CTRL2[CHNCFG_H]. */
    0x0400U  /* CHN14-15, ANB6-ANB7. CTRL2[CHNCFG_H]. */
};
void HSADC_SetSampleConfig(HSADC_Type *base, uint16_t sampleIndex, const hsadc_sample_config_t *config)
{
    assert(sampleIndex < HSADC_RSLT_COUNT);
    assert(NULL != config);

    uint16_t tmp16;

    /* Configure the differential conversion channel. */
    if ((config->channelNumber < 4U) || ((config->channelNumber >= 8U) && (config->channelNumber < 12U)))
    {
        if (config->enableDifferentialPair)
        {
            base->CTRL1 |= g_HSADCChannelConfigDifferentialMask[config->channelNumber / 2U];
        }
        else
        {
            base->CTRL1 &= (uint16_t)(~g_HSADCChannelConfigDifferentialMask[config->channelNumber / 2U]);
        }
    }
    else if (((config->channelNumber >= 4U) && (config->channelNumber < 8U)) ||
             ((config->channelNumber >= 12U) && (config->channelNumber < 16U)))
    {
        if (config->enableDifferentialPair)
        {
            base->CTRL2 |= g_HSADCChannelConfigDifferentialMask[config->channelNumber / 2U];
        }
        else
        {
            base->CTRL2 &= (uint16_t)(~g_HSADCChannelConfigDifferentialMask[config->channelNumber / 2U]);
        }
    }
    else
    {
        /* To avoid MISRA rule 14.10 error. */
    }

    /* Configure the zero crossing mode. */
    if (sampleIndex < 8U)
    {
        tmp16 = base->ZXCTRL1 & (uint16_t)(~HSADC_ZXCTRL_ZCE_MASK(sampleIndex));
        tmp16 |= HSADC_ZXCTRL_ZCE(sampleIndex, config->zeroCrossingMode);
        base->ZXCTRL1 = tmp16;
    }
    else if (sampleIndex < 16U)
    {
        sampleIndex -= 8U;
        tmp16 = base->ZXCTRL2 & (uint16_t)(~HSADC_ZXCTRL_ZCE_MASK(sampleIndex));
        tmp16 |= HSADC_ZXCTRL_ZCE(sampleIndex, config->zeroCrossingMode);
        base->ZXCTRL2 = tmp16;
        sampleIndex += 8U;
    }
    else
    {
        /* To avoid MISRA rule 14.10 error. */
    }

    /* Fill the conversion channel into sample slot. */
    if (sampleIndex < 4U)
    {
        tmp16 = base->CLIST1 & (uint16_t)(~HSADC_CLIST_SAMPLE_MASK(sampleIndex));
        tmp16 |= HSADC_CLIST_SAMPLE(sampleIndex, config->channelNumber);
        base->CLIST1 = tmp16;
    }
    else if (sampleIndex < 8U)
    {
        sampleIndex -= 4U;
        tmp16 = base->CLIST2 & (uint16_t)(~HSADC_CLIST_SAMPLE_MASK(sampleIndex));
        tmp16 |= HSADC_CLIST_SAMPLE(sampleIndex, config->channelNumber);
        base->CLIST2 = tmp16;
        sampleIndex += 4U;
    }
    else if (sampleIndex < 12U)
    {
        sampleIndex -= 8U;
        tmp16 = base->CLIST3 & (uint16_t)(~HSADC_CLIST_SAMPLE_MASK(sampleIndex));
        tmp16 |= HSADC_CLIST_SAMPLE(sampleIndex, config->channelNumber);
        base->CLIST3 = tmp16;
        sampleIndex += 8U;
    }
    else if (sampleIndex < 16U)
    {
        sampleIndex -= 12U;
        tmp16 = base->CLIST4 & (uint16_t)(~HSADC_CLIST_SAMPLE_MASK(sampleIndex));
        tmp16 |= HSADC_CLIST_SAMPLE(sampleIndex, config->channelNumber);
        base->CLIST4 = tmp16;
        sampleIndex += 12U;
    }
    else
    {
        /* To avoid MISRA rule 14.10 error. */
    }

    /* Configure the hardware compare. */
    base->LOLIM[sampleIndex] = config->lowLimitValue;
    base->HILIM[sampleIndex] = config->highLimitValue;
    base->OFFST[sampleIndex] = config->offsetValue;

    /* Configure the hardware trigger. */
    /* SCTRL. */
    if (config->enableWaitSync)
    {
        base->SCTRL |= (1U << sampleIndex);
    }
    else
    {
        base->SCTRL &= ~(1U << sampleIndex);
    }

    /* Configure the channel 6/7's additional multiplex selector. */
    HSADC_SetChannel67Mux(base, config->channelNumber, config->channel67MuxNumber, config->enableDifferentialPair);
}

void HSADC_GetDefaultSampleConfig(hsadc_sample_config_t *config)
{
    assert(NULL != config);

    config->channelNumber = 0U;
    config->channel67MuxNumber = 0U;
    config->enableDifferentialPair = false;
    config->zeroCrossingMode = kHSADC_ZeroCorssingDisabled;
    config->highLimitValue = 0x7FF8U;
    config->lowLimitValue = 0U;
    config->offsetValue = 0U;
    config->enableWaitSync = false;
}

void HSADC_DoAutoCalibration(HSADC_Type *base, uint16_t converterMask, uint16_t calibrationModeMask)
{
    assert(calibrationModeMask);

    /* CALIB */
    /* Set the calibration mode.
     * Hardware requires that the bit REQSINGA, REQDIFA, REQA, REQSINGB, REQDIFB, REQB in CALIB register can't be set
     * at the same time. They must be set as the sequency description in the reference manual.
     */
    if (kHSADC_ConverterA == (kHSADC_ConverterA & converterMask))
    {
        if (kHSADC_CalibrationModeSingleEnded == (kHSADC_CalibrationModeSingleEnded & calibrationModeMask))
        {
            base->CALIB |= HSADC_CALIB_REQSINGA_MASK;
        }
        if (kHSADC_CalibrationModeDifferential == (kHSADC_CalibrationModeDifferential & calibrationModeMask))
        {
            base->CALIB |= HSADC_CALIB_REQDIFA_MASK;
        }
        base->CALIB |= HSADC_CALIB_CAL_REQA_MASK;
    }
    if (kHSADC_ConverterB == (kHSADC_ConverterB & converterMask))
    {
        if (kHSADC_CalibrationModeSingleEnded == (kHSADC_CalibrationModeSingleEnded & calibrationModeMask))
        {
            base->CALIB |= HSADC_CALIB_REQSINGB_MASK;
        }
        if (kHSADC_CalibrationModeDifferential == (kHSADC_CalibrationModeDifferential & calibrationModeMask))
        {
            base->CALIB |= HSADC_CALIB_REQDIFB_MASK;
        }
        base->CALIB |= HSADC_CALIB_CAL_REQB_MASK;
    }

    /* Trigger the calibration. */
    HSADC_DoSoftwareTriggerConverter(base, converterMask);
}

uint32_t HSADC_GetCalibrationResultValue(HSADC_Type *base)
{
    return (((uint32_t)(base->CALVAL_A) << 16U) + base->CALVAL_B);
}

void HSADC_EnableCalibrationResultValue(HSADC_Type *base, uint16_t converterMask, bool enable)
{
    /* CALIB */
    /* Enable means not to bypass the calibration operation. */
    if (kHSADC_ConverterA == (kHSADC_ConverterA & converterMask))
    {
        if (true == enable)
        {
            base->CALIB &= ~HSADC_CALIB_BYPA_MASK;
        }
        else
        {
            base->CALIB |= HSADC_CALIB_BYPA_MASK;
        }
    }
    if (kHSADC_ConverterB == (kHSADC_ConverterB & converterMask))
    {
        if (true == enable)
        {
            base->CALIB &= ~HSADC_CALIB_BYPB_MASK;
        }
        else
        {
            base->CALIB |= HSADC_CALIB_BYPB_MASK;
        }
    }
}
