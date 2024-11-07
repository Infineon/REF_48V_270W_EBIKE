/**
 * @file main.c
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

 ***********************************************************************************************************************/

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include <PMSM_FOC/Configuration/pmsm_foc_mcuhw_params.h>
#include <PMSM_FOC/Configuration/pmsm_foc_config.h>
#include <PMSM_FOC/Configuration/pmsm_foc_const.h>
#include <xmc_common.h>
#include "../PMSM_FOC/MIDSys/pmsm_foc_debug.h"
#include "../PMSM_FOC/ControlModules/pmsm_foc_functions.h"
#include "../PMSM_FOC/ControlModules/pmsm_foc_interface.h"
#include "PMSM_FOC/MCUInit/pmsm_foc_systick.h"

#if (SPI_LIB == ENABLED)
#include "../PMSM_FOC/IMD700A_SPI_LIB/IMD700_SPI_LIB.h"
#endif

/*******************************************************************************************************************
 * EXTERN
 ******************************************************************************************************************/

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
#if(RESET_BMI_ENABLE == 1)
void ChangeBMI(void)
{
	if ((XMC_GPIO_GetInput(TEST_PIN)) == IS_GPIO_LOW)
	{
		ROM_BmiInstallationReq(SWD0);
		while(1);
	}
}
#endif

int main(void)
{
    /* Initialize MCU and motor control peripherals */
    PMSM_FOC_Init();

	#if(RESET_BMI_ENABLE == 1)
    ChangeBMI();
	#endif

    /* Start the motor */
    PMSM_FOC_MotorStart();

    /* Placeholder for user application code. The while loop below can be replaced with user application code. */
    while (1)
    {
      
    }

    return 0;
}
/* End of main () */


