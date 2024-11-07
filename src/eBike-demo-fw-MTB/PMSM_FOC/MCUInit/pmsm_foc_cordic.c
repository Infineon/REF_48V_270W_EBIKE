/**
 * @file pmsm_foc_cordic.c
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
#include <PMSM_FOC/MCUInit/pmsm_foc_cordic.h>

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

#define VECTORING_MODE  0x62U   /*
                                 * For CORDIC control register: Circular Vectoring Mode (default).
                                 * MPS: Divide by 2 (default).
                                 */

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/* API to initialize MATH Module */
void PMSM_FOC_MATH_Init(void)
{
    /* Disable bit protection */
    XMC_MATH_Enable();

    /* Default CORDIC setting so far. No result chaining is selected */
    MATH->GLBCON = 0x00U;

    /* Setting to Vector Mode */
    MATH->CON = VECTORING_MODE;

    /* Data control: No Keep*/
    MATH->STATC = 0x00U; /* Data control: No Keep.*/
}
