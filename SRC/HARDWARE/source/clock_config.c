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

#include "fsl_common.h"
#include "fsl_smc.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*
 * How to setup clock using clock driver functions:
 *
 * 1. CLOCK_SetSimSafeDivs, to make sure core clock, bus clock, flexbus clock
 *    and flash clock are in allowed range during clock mode switch.
 *
 * 2. Call CLOCK_Osc0Init to setup OSC clock, if it is used in target mode.
 *
 * 3. Set MCG configuration, MCG includes three parts: FLL clock, PLL clock and
 *    internal reference clock(MCGIRCLK). Follow the steps to setup:
 *
 *    1). Call CLOCK_BootToXxxMode to set MCG to target mode.
 *
 *    2). If target mode is FBI/BLPI/PBI mode, the MCGIRCLK has been configured
 *        correctly. For other modes, need to call CLOCK_SetInternalRefClkConfig
 *        explicitly to setup MCGIRCLK.
 *
 *    3). Don't need to configure FLL explicitly, because if target mode is FLL
 *        mode, then FLL has been configured by the function CLOCK_BootToXxxMode,
 *        if the target mode is not FLL mode, the FLL is disabled.
 *
 *    4). If target mode is PEE/PBE/PEI/PBI mode, then the related PLL has been
 *        setup by CLOCK_BootToXxxMode. In FBE/FBI/FEE/FBE mode, the PLL could
 *        be enabled independently, call CLOCK_EnablePll0 explicitly in this case.
 *
 * 4. Call CLOCK_SetSimConfig to set the clock configuration in SIM.
 */

void BOARD_BootClockVLPR(void)
{
    /*
    * Core clock: 4MHz
    */
    const sim_clock_config_t simConfig = {
            .er32kSrc = 3U,         /* ERCLK32K selection, use LPO. */
            .clkdiv1 = 0x00070000U, /* SIM_CLKDIV1. */
    };

    CLOCK_SetSimSafeDivs();

    CLOCK_BootToBlpiMode(0U, kMCG_IrcFast, kMCG_IrclkEnable);

    CLOCK_SetSimConfig(&simConfig);

    SystemCoreClock = 4000000U;

    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeVlpr(SMC);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateVlpr)
    {
    }
}

void BOARD_BootClockRUN_300M(void)
{
    /*
    * Core clock: 300MHz  FastbusClock 150M FlesbusClock 150M BusClock 50M
    */
    const mcg_pll_config_t pll0Config = {
        .enableMode = 0U, .prdiv = 3U, .vdiv = 8U,//8   //PLL??????????  outClk = External / (prdiv+1) * (vdiv+16)
    };
    const sim_clock_config_t simConfig = {
            .pllFllSel = 1U,        /* PLLFLLSEL select PLL */
            .er32kSrc = 3U,         /* ERCLK32K selection, use LPO. */
            .clkdiv1 = 0x01150000U, /* SIM_CLKDIV1. */
    };

    CLOCK_SetSimSafeDivs();
    BOARD_InitOsc0();

    CLOCK_BootToPeeMode(kMCG_OscselOsc, kMCG_PllClkSelPll0, &pll0Config);

    CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcSlow, 0);
    CLOCK_SetSimConfig(&simConfig);

    SystemCoreClock = 300000000U;
}

void BOARD_BootClockRUN_250M(void)
{
    /*
    * Core clock: 250MHz  FastbusClock 125M FlesbusClock 125M BusClock 31.25M
    */
    const mcg_pll_config_t pll0Config = {
        .enableMode = 0U, .prdiv = 4U, .vdiv = 9U,   //PLL??????????  outClk = External / (prdiv+1) * (vdiv+16)
    };
    const sim_clock_config_t simConfig = {
            .pllFllSel = 1U,        /* PLLFLLSEL select PLL */
            .er32kSrc = 3U,         /* ERCLK32K selection, use LPO. */
            .clkdiv1 = 0x01170000U, /* SIM_CLKDIV1. */
    };

    CLOCK_SetSimSafeDivs();
    BOARD_InitOsc0();

    CLOCK_BootToPeeMode(kMCG_OscselOsc, kMCG_PllClkSelPll0, &pll0Config);

    CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcSlow, 0);
    CLOCK_SetSimConfig(&simConfig);

    SystemCoreClock = 250000000U;
}

void BOARD_BootClockRUN_240M(void)
{
    /*
    * Core clock: 240MHz  FastbusClock 120M FlesbusClock 120M BusClock 30M
    */
    const mcg_pll_config_t pll0Config = {
        .enableMode = 0U, .prdiv = 4U, .vdiv = 8U,   //PLL??????????  outClk = External / (prdiv+1) * (vdiv+16)
    };
    const sim_clock_config_t simConfig = {
            .pllFllSel = 1U,        /* PLLFLLSEL select PLL */
            .er32kSrc = 3U,         /* ERCLK32K selection, use LPO. */
            .clkdiv1 = 0x01170000U, /* SIM_CLKDIV1. */
    };

    CLOCK_SetSimSafeDivs();
    BOARD_InitOsc0();

    CLOCK_BootToPeeMode(kMCG_OscselOsc, kMCG_PllClkSelPll0, &pll0Config);

    CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcSlow, 0);
    CLOCK_SetSimConfig(&simConfig);

    SystemCoreClock = 240000000U;
}
void BOARD_BootClockRUN_220M(void)
{
    /*
    * Core clock: 220MHz  FastbusClock 110M FlesbusClock 110M BusClock 27.5M
    */
    const mcg_pll_config_t pll0Config = {
        .enableMode = 0U, .prdiv = 4U, .vdiv = 6U,   //PLL??????????  outClk = External / (prdiv+1) * (vdiv+16)
    };
    const sim_clock_config_t simConfig = {
            .pllFllSel = 1U,        /* PLLFLLSEL select PLL */
            .er32kSrc = 3U,         /* ERCLK32K selection, use LPO. */
            .clkdiv1 = 0x01170000U, /* SIM_CLKDIV1. */
    };

    CLOCK_SetSimSafeDivs();
    BOARD_InitOsc0();

    CLOCK_BootToPeeMode(kMCG_OscselOsc, kMCG_PllClkSelPll0, &pll0Config);

    CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcSlow, 0);
    CLOCK_SetSimConfig(&simConfig);

    SystemCoreClock = 220000000U;
}

void BOARD_BootClockRUN_200M(void)
{
    /*
    * Core clock: 200MHz  FastbusClock 100M FlesbusClock 100M BusClock 25M
    */
    const mcg_pll_config_t pll0Config = {
        .enableMode = 0U, .prdiv = 4U, .vdiv = 4U,   //PLL??????????  outClk = External / (prdiv+1) * (vdiv+16)
    };
    const sim_clock_config_t simConfig = {
            .pllFllSel = 1U,        /* PLLFLLSEL select PLL */
            .er32kSrc = 3U,         /* ERCLK32K selection, use LPO. */
            .clkdiv1 = 0x01170000U, /* SIM_CLKDIV1. */
    };

    CLOCK_SetSimSafeDivs();
    BOARD_InitOsc0();

    CLOCK_BootToPeeMode(kMCG_OscselOsc, kMCG_PllClkSelPll0, &pll0Config);

    CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcSlow, 0);
    CLOCK_SetSimConfig(&simConfig);

    SystemCoreClock = 200000000U;
}
void BOARD_BootClockRUN_180M(void)
{
    /*
    * Core clock: 180MHz  FastbusClock 90M FlesbusClock 90M BusClock 30M
    */
    const mcg_pll_config_t pll0Config = {
        .enableMode = 0U, .prdiv = 4U, .vdiv = 2U,   //PLL??????????  outClk = External / (prdiv+1) * (vdiv+16)
    };
    const sim_clock_config_t simConfig = {
            .pllFllSel = 1U,        /* PLLFLLSEL select PLL */
            .er32kSrc = 3U,         /* ERCLK32K selection, use LPO. */
            .clkdiv1 = 0x01150000U, /* SIM_CLKDIV1. */
    };

    CLOCK_SetSimSafeDivs();
    BOARD_InitOsc0();

    CLOCK_BootToPeeMode(kMCG_OscselOsc, kMCG_PllClkSelPll0, &pll0Config);

    CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcSlow, 0);
    CLOCK_SetSimConfig(&simConfig);

    SystemCoreClock = 180000000U;
}
void BOARD_BootClockHSRUN(void)
{
    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeHsrun(SMC);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateHsrun)
    {
    }
    CLOCK_SetSimSafeDivs();
    BOARD_InitOsc0();
    const sim_clock_config_t simConfig = {
            .pllFllSel = 1U,        /* PLLFLLSEL select PLL */
            .er32kSrc = 3U,         /* ERCLK32K selection, use LPO. */
            .clkdiv1 = 0x01390000U, /* SIM_CLKDIV1. */
    };

    const mcg_pll_config_t pll0Config = {
        .enableMode = 0U, .prdiv = 0x03U, .vdiv = 0x16U,    //PLL??????????  outClk = External / (prdiv+1) * (vdiv+16)
    };
    CLOCK_BootToPeeMode(kMCG_OscselOsc, kMCG_PllClkSelPll0, &pll0Config);

    CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcSlow, 0);
    CLOCK_SetSimConfig(&simConfig);
    SystemCoreClock = 237500000U;	
}

void BOARD_InitOsc0(void)
{
    /*
    * Core clock: 120MHz
    */
    const osc_config_t oscConfig = {.freq = BOARD_XTAL0_CLK_HZ,
                                    .capLoad = 0,
                                    .workMode = kOSC_ModeExt,
                                    .oscerConfig = {
                                        .enableMode = kOSC_ErClkEnable,
#if (defined(FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER) && FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER)
                                        .erclkDiv = 0U,
#endif
                                    }};

    CLOCK_InitOsc0(&oscConfig);

    /* Passing the XTAL0 frequency to clock driver. */
    CLOCK_SetXtal0Freq(BOARD_XTAL0_CLK_HZ);
}
