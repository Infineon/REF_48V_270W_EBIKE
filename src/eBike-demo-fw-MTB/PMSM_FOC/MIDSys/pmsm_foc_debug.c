/**
 * @file pmsm_foc_debug.c
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
#include "../MIDSys/pmsm_foc_debug.h"

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

/***********************************************************************************************************************
 * GLOBAL DATA
***********************************************************************************************************************/

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
volatile int32_t Tmp_CRS;
#if((DEBUG_PWM_0_ENABLE == ENABLED) || (DEBUG_PWM_1_ENABLE == ENABLED))
/* API to use CCU4 Debug with 2 Outputs, P0.4 and P1.0 */
void PMSM_FOC_CCU4_Debug3output(int32_t In04, uint16_t In04_Flag, uint16_t In04_N, int32_t In10, uint16_t In10_Flag,
                       uint16_t In10_N)
{
  	//int32_t Tmp_CRS;                               /* Tmp for CCU4 debug. */
  	#if(DEBUG_PWM_0_ENABLE == ENABLED)
  	/* Update CCU40.OUT0 (P1.0) duty-cycle for debug. */
  	if (In10_Flag == 0U)
  	{
    	Tmp_CRS = ((In10 * DEBUG_PWM_PERIOD_CNTS) >> In10_N); /* In10 is a positive integer, In10 < 2^In10_N. */
  	}
  	else
  	{
    	/* In10 is a positive or negative integer, -2^In10_N < In10 < 2^In10_N. */
    	Tmp_CRS = ((In10 + (1U << In10_N)) * DEBUG_PWM_PERIOD_CNTS) >> (In10_N + 1U);
  	}

    if (Tmp_CRS < 0)
    {
        Tmp_CRS = REVERSE_CRS_OR_0;
    }

    XMC_CCU4_SLICE_SetTimerCompareMatch( DEBUG_PWM_0_SLICE, Tmp_CRS);		/* Update CCU41 Shadow Compare Register. */
    XMC_CCU4_EnableShadowTransfer(DEBUG_PWM_CCU4_MODULE, (uint32_t)DEBUG_PWM_0_SLICE_SHADOW_TRANS_ENABLE_Msk);
    #endif /* (DEBUG_PWM_0_ENABLE == 1) */

  	#if	(DEBUG_PWM_1_ENABLE == ENABLED)
  	/* Update CCU40.OUT1 (P0.4) duty-cycle for debug. */
  	if (In04_Flag == 0U)
  	{
    	Tmp_CRS = (In04 * DEBUG_PWM_PERIOD_CNTS) >> In04_N; /* In04 is a positive integer. */
  	}
  	else
  	{
    	/* In04 is a positive or negative integer. */
    	Tmp_CRS = ((In04 + (1U << In04_N)) * DEBUG_PWM_PERIOD_CNTS) >> (In04_N + 1U);
  	}

    if (Tmp_CRS < 0)
    {
        Tmp_CRS = REVERSE_CRS_OR_0;
    }
    XMC_CCU4_SLICE_SetTimerCompareMatch( DEBUG_PWM_1_SLICE, Tmp_CRS);		/* Update CCU41 Shadow Compare Register. */

    XMC_CCU4_EnableShadowTransfer(DEBUG_PWM_CCU4_MODULE, (uint32_t)DEBUG_PWM_1_SLICE_SHADOW_TRANS_ENABLE_Msk);
    #endif /* (DEBUG_PWM_1_ENABLE == 1) */

}	/* End of pmsm_foc_ccu4_debug3output () */

#endif /* #if((DEBUG_PWM_0_ENABLE == 1U) || (DEBUG_PWM_1_ENABLE == 1U)) */


