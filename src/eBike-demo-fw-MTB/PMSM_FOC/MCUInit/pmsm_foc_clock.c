/**
 * @file pmsm_foc_clock.c
 * @date 09 May, 2019
 *
 * @cond
 *********************************************************************************************************************
 * PMSM_FOC Motor Control Library
 *
 * Copyright (c) 2015-2020, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * @endcond
 ***********************************************************************************************************************/

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "../MCUInit/pmsm_foc_clock.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/**
 *  Data Structure initialization - Clock Configuration.
 */
XMC_SCU_CLOCK_CONFIG_t Clock_Config =
{
	#if(UC_SERIES == XMC14)
		.fdiv       = 0U,  /**< 8/10 Bit Fractional divider */
		.idiv       = 1U,  /**< 8 Bit integer divider */

		.dclk_src   = XMC_SCU_CLOCK_DCLKSRC_DCO1,
		.oschp_mode = XMC_SCU_CLOCK_OSCHP_MODE_DISABLED,
		.osclp_mode = XMC_SCU_CLOCK_OSCLP_MODE_DISABLED,
		.pclk_src   = XMC_SCU_CLOCK_PCLKSRC_DOUBLE_MCLK,
		.rtc_src    = XMC_SCU_CLOCK_RTCCLKSRC_DCO2
	#else
		.idiv       = 0x01U,
		.pclk_src   = XMC_SCU_CLOCK_PCLKSRC_DOUBLE_MCLK,
		.rtc_src    = XMC_SCU_CLOCK_RTCCLKSRC_DCO2
	#endif
};

/***********************************************************************************************************************
 * GLOBAL DATA
***********************************************************************************************************************/
/* Global variable, MCU Reset Status Information, reason of last reset */
uint32_t MCUResetStatus;    // Global variable. MCU Reset Status Information, reason of last reset.

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

/* API to initialize clock module and read reset status */
void PMSM_FOC_Clock_Init(void)
{
    uint32_t reset_status;

    /* Reset status, get reason of last reset */
    reset_status = XMC_SCU_RESET_GetDeviceResetReason();

    /* Record MCU Reset Status Information by a global variable */
    MCUResetStatus = reset_status;

    /* Clear reset status, to ensure a clear indication of cause of next reset */
    XMC_SCU_RESET_ClearDeviceResetReason();

    /* Enable reset triggered by critical events: Flash ECC error, loss of clock, 16kbytes SRAM parity error */
    XMC_SCU_RESET_EnableResetRequest((uint32_t)XMC_SCU_RESET_REQUEST_FLASH_ECC_ERROR |
                                     (uint32_t)XMC_SCU_RESET_REQUEST_CLOCK_LOSS |
                                     (uint32_t)XMC_SCU_RESET_REQUEST_SRAM_PARITY_ERROR);

    /* 32MHz MCLK, PCLK = 2 x MCLK = 64MHz, RTC clock is standby clock, Counter Adjustment = 1024 clock cycles */
    XMC_SCU_CLOCK_Init(&Clock_Config);
}
