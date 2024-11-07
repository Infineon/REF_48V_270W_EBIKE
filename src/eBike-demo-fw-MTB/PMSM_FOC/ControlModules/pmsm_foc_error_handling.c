/**
 * @file pmsm_foc_error_handling.c
 * @date 09 May, 2019
 *
 * @cond
 *********************************************************************************************************************
 * PMSM FOC Motor Control Library
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
/**********************************************************************************************************************
 * HEADER FILES
 *********************************************************************************************************************/
#include "pmsm_foc_error_handling.h"
#include "pmsm_foc_interface.h"
#include "pmsm_foc_state_machine.h"
#include "../Configuration/pmsm_foc_config.h"

#if (SPI_LIB == ENABLED)
#include "../IMD700A_SPI_LIB/utility.h"
#endif

/**********************************************************************************************************************
 * MACRO's
 *********************************************************************************************************************/
#define PMSM_FOC_MAX_CYCLE_TIME              (7500)

/**********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * API IMPLEMENTATION
 *********************************************************************************************************************/
/* Error handling, e.g.: to handle CCU8 TRAP protection */
void PMSM_FOC_ErrorHandling(void)
{
	static uint32_t Cycle_Counter = 0;
    if ((PMSM_FOC_CTRL.error_status != 0U) && (SYSTEM_BE_IDLE))
    {
        /******************************************************************************************************************
         * Error handling for Over Current Protection
         ******************************************************************************************************************/
        if (PMSM_FOC_CTRL.error_status & PMSM_FOC_EID_OVER_CURRENT)
        {
            /* Reset the over current related variables & filters. */
            PMSM_FOC_OUTPUT.current_i_mag = 0;
            PMSM_FOC_OUTPUT.current_i_mag_filtered = 0;
			#if(USER_OCP_LEVELS_PROTECTION == ENABLED)
			for(int i=0; i<PMSM_FOC_CTRL.ocp.OCP_level_num; i++)
			{
				PMSM_FOC_CTRL.ocp.OCP_level_error[i] = FALSE;
			}
			#endif
		  /* Clear the error when pot reference goes to zero(system Idle) */
		  PMSM_FOC_CTRL.error_status &= (~PMSM_FOC_EID_OVER_CURRENT);
    	}

        /******************************************************************************************************************
         * Error handling for Over board over temperature
         ******************************************************************************************************************/
        if (PMSM_FOC_CTRL.error_status & PMSM_FOC_EID_INVERTER_OVER_TEMP)
        {
            /* Reset the error when temperature reduced. */
		        if(PMSM_FOC_CTRL.inverter_board_temp_degrees < (int16_t)USER_MAX_BOARD_TEMP_START_C)
            {
                /* Clear the error when pot reference goes to zero(system Idle) */
                PMSM_FOC_CTRL.error_status &= (~PMSM_FOC_EID_INVERTER_OVER_TEMP);
            }
        }

        /******************************************************************************************************************
         * Error handling for DC bus Under/Over Voltage
         ******************************************************************************************************************/
        #if(USER_VDC_UV_OV_PROTECTION_ENABLE == ENABLED)

        if(PMSM_FOC_CTRL.error_status & PMSM_FOC_EID_OVER_VOLT)
        {
            /* Check DC link voltage and clear error if its in range */
            if (ADC.adc_res_vdc_filtered < VDC_OVER_VOLTAGE_LIMIT)
            {
              PMSM_FOC_CTRL.error_status &= (~PMSM_FOC_EID_OVER_VOLT);
            }
        }

        if(PMSM_FOC_CTRL.error_status & PMSM_FOC_EID_UNDER_VOLT)
        {
          /* Check DC link voltage and clear error if its in range */
          if (ADC.adc_res_vdc_filtered > VDC_UNDER_VOLTAGE_LIMIT)
          {
            PMSM_FOC_CTRL.error_status &= (~PMSM_FOC_EID_UNDER_VOLT);
          }
        }
        #endif

        /******************************************************************************************************************
         * Error handling for Torque limit exceed
         *****************************************************************************************************************/
        if (PMSM_FOC_CTRL.error_status & PMSM_FOC_EID_TORQUE_LIMIT_EXCEED)
        {
            /* Clear the error when pot reference goes to zero(system Idle) */
            PMSM_FOC_CTRL.error_status &= (~PMSM_FOC_EID_TORQUE_LIMIT_EXCEED);
        }

        /******************************************************************************************************************
         * Error handling for 6EDL7141 Fault
         *****************************************************************************************************************/
        if (PMSM_FOC_CTRL.error_status & PMSM_FOC_EID_6EDL7141_FAULT)
        {
            /* Clear 6EDL7141 Faults */
            Driver_Faults_Clear();

            /* Clear the error when pot reference goes to zero(system Idle) */
            PMSM_FOC_CTRL.error_status &= (~PMSM_FOC_EID_6EDL7141_FAULT);
        }

        /******************************************************************************************************************
         * Error handling for Motor Stall
         *****************************************************************************************************************/
        if (PMSM_FOC_CTRL.error_status & PMSM_FOC_EID_STALL)
        {
            /* Clear the error when pot reference goes to zero(system Idle) */
            PMSM_FOC_CTRL.error_status &= (~PMSM_FOC_EID_STALL);
        }

        /******************************************************************************************************************
         * Error handling for CCU8 Trap
         *****************************************************************************************************************/
        /* If CCU8 TRAP has occurred, and system becomes idle (i.e.: PWM duty cycle or POT ADC too low), */

        if (PMSM_FOC_CTRL.error_status & PMSM_FOC_EID_CTRAP)
        {
        	Cycle_Counter++;
        	if (Cycle_Counter > PMSM_FOC_MAX_CYCLE_TIME)
        	{
                /* Clear 6EDL7141 Faults */
                Driver_Faults_Clear(); /*For ebike only*/

        		XMC_CCU8_SLICE_ClearEvent(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_EVENT2);
        	    XMC_CCU8_SLICE_ClearEvent(CCU8_SLICE_PHASE_V, XMC_CCU8_SLICE_IRQ_ID_EVENT2);
        	    XMC_CCU8_SLICE_ClearEvent(CCU8_SLICE_PHASE_W, XMC_CCU8_SLICE_IRQ_ID_EVENT2);

        		Cycle_Counter = 0;

        		/* Clear the error when pot reference goes to zero(system Idle) */
        		PMSM_FOC_CTRL.error_status &= (~PMSM_FOC_EID_CTRAP);
        	}
        }

        /******************************************************************************************************************
         * Error handling for IDC Overcurrent
         *****************************************************************************************************************/
        if (PMSM_FOC_CTRL.error_status & PMSM_FOC_EID_IDC_OVERCURRENT)
        {
            /* Clear the error when pot reference goes to zero(system Idle) */
            PMSM_FOC_CTRL.error_status &= (~PMSM_FOC_EID_IDC_OVERCURRENT);
        }

        /*****************************************************************************************************************
         * Error handling for Invalid Hall pattern
         *****************************************************************************************************************/
        if (PMSM_FOC_CTRL.error_status & PMSM_FOC_EID_INVALID_HALL_PAT)
        {
        	/* Clear the error when pot reference goes to zero(system Idle) */
        	PMSM_FOC_CTRL.error_status &= (~PMSM_FOC_EID_INVALID_HALL_PAT);
        }
    }

    /* If all errors are cleared then go to STOP state */
    if (PMSM_FOC_CTRL.error_status == 0)
    {
     	PMSM_FOC_CTRL.motor_stop_counter = 0;
    	/* Next go to Motor coasting function and wait for motor start command */
    	PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_STOP_MOTOR;

    }

    /* Set High side PWM duty to zero */
    PMSM_FOC_CCU8_SetDutyZero();

} /* End of pmsm_foc_error_handling () */
