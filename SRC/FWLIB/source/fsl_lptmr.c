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

#include "fsl_lptmr.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Gets the instance from the base address to be used to gate or ungate the module clock
 *
 * @param base LPTMR peripheral base address
 *
 * @return The LPTMR instance
 */
static uint32_t LPTMR_GetInstance(LPTMR_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to LPTMR bases for each instance. */
static LPTMR_Type *const s_lptmrBases[] = LPTMR_BASE_PTRS;

/*! @brief Pointers to LPTMR clocks for each instance. */
static const clock_ip_name_t s_lptmrClocks[] = LPTMR_CLOCKS;

/*******************************************************************************
 * Code
 ******************************************************************************/
 /*!
 *  @brief      LPTMR��ʱ������ms��
 *  @param      ms          LPTMR��ʱʱ��(0~65535)
 *  @since      v5.0
 *  Sample usage:       lptmr_delay_ms(32);     // LPTMR ��ʱ32ms
 */
void lptmr_delay_ms(uint16_t ms)
{
    if(ms == 0)
    {
        return;
    }

    SIM->SCGC5 |= SIM_SCGC5_LPTMR_MASK;    //ʹ��LPTģ��ʱ��

    LPTMR0->CSR = 0x00;                      //�ȹ���LPT���Զ����������ֵ

    LPTMR0->CMR = ms;                        //���ñȽ�ֵ������ʱʱ��

    //ѡ��ʱ��Դ
    LPTMR0->PSR  =   ( 0
                      | LPTMR_PSR_PCS(1)                  //ѡ��ʱ��Դ�� 0 Ϊ MCGIRCLK ��1Ϊ LPO��1KHz�� ��2Ϊ ERCLK32K ��3Ϊ OSCERCLK
                      | LPTMR_PSR_PBYP_MASK               //��· Ԥ��Ƶ/�����˲��� ,������ Ԥ��Ƶ/�����˲���(ע���˱�ʾʹ��Ԥ��Ƶ/�����˲���)
                      //| LPTMR_PSR_PRESCALE(1)           //Ԥ��Ƶֵ = 2^(n+1) ,n = 0~ 0xF
                    );

    //ʹ�� LPT
    LPTMR0->CSR  =  (0
                    //| LPTMR_CSR_TPS(1)        // ѡ������ܽ� ѡ��
                    //| LPTMR_CSR_TMS_MASK      // ѡ��������� (ע���˱�ʾʱ�����ģʽ)
                    //| ( cfg == LPT_Falling ?  LPTMR_CSR_TPP_MASK :   0  )  //���������������ʽѡ��0Ϊ�ߵ�ƽ��Ч�������ؼ�1
                    | LPTMR_CSR_TEN_MASK        //ʹ��LPT(ע���˱�ʾ����)
                    //| LPTMR_CSR_TIE_MASK      //�ж�ʹ��
                    //| LPTMR_CSR_TFC_MASK      //0:����ֵ���ڱȽ�ֵ�͸�λ��1�������λ��ע�ͱ�ʾ0��
                   );

    while (!(LPTMR0->CSR & LPTMR_CSR_TCF_MASK)); //�ȴ��Ƚ�ֵ�����ֵ��ȣ���ʱ�䵽��

    LPTMR0->CSR &= ~LPTMR_CSR_TEN_MASK;          //����Ƚϱ�־λ

    return;
}

static uint32_t LPTMR_GetInstance(LPTMR_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < FSL_FEATURE_SOC_LPTMR_COUNT; instance++)
    {
        if (s_lptmrBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < FSL_FEATURE_SOC_LPTMR_COUNT);

    return instance;
}

void LPTMR_Init(LPTMR_Type *base, const lptmr_config_t *config)
{
    assert(config);

    /* Ungate the LPTMR clock*/
    CLOCK_EnableClock(s_lptmrClocks[LPTMR_GetInstance(base)]);

    /* Configure the timers operation mode and input pin setup */
    base->CSR = (LPTMR_CSR_TMS(config->timerMode) | LPTMR_CSR_TFC(config->enableFreeRunning) |
                 LPTMR_CSR_TPP(config->pinPolarity) | LPTMR_CSR_TPS(config->pinSelect));

    /* Configure the prescale value and clock source */
    base->PSR = (LPTMR_PSR_PRESCALE(config->value) | LPTMR_PSR_PBYP(config->bypassPrescaler) |
                 LPTMR_PSR_PCS(config->prescalerClockSource));
}

void LPTMR_Deinit(LPTMR_Type *base)
{
    /* Disable the LPTMR and reset the internal logic */
    base->CSR &= ~LPTMR_CSR_TEN_MASK;
    /* Gate the LPTMR clock*/
    CLOCK_DisableClock(s_lptmrClocks[LPTMR_GetInstance(base)]);
}

void LPTMR_GetDefaultConfig(lptmr_config_t *config)
{
    assert(config);

    /* Use time counter mode */
    config->timerMode = kLPTMR_TimerModeTimeCounter;
    /* Use input 0 as source in pulse counter mode */
    config->pinSelect = kLPTMR_PinSelectInput_0;
    /* Pulse input pin polarity is active-high */
    config->pinPolarity = kLPTMR_PinPolarityActiveHigh;
    /* Counter resets whenever TCF flag is set */
    config->enableFreeRunning = false;
    /* Bypass the prescaler */
    config->bypassPrescaler = true;
    /* LPTMR clock source */
    config->prescalerClockSource = kLPTMR_PrescalerClock_1;
    /* Divide the prescaler clock by 2 */
    config->value = kLPTMR_Prescale_Glitch_0;
}
