/**
 * @file pmsm_foc_interface.c
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

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <PMSM_FOC/Configuration/pmsm_foc_mcuhw_params.h>
#include "pmsm_foc_interface.h"

/*********************************************************************************************************************
 * EXTERN
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * DATA
 ********************************************************************************************************************/
extern volatile int32_t   Speed_RPM_LPF;             /* uC_Probe variable */
/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * API Implementation
 ********************************************************************************************************************/
/* API to start the Motor. If motor is not in error state then only it will start when reference is above
 * threshold mentioned in the IDLE condition */
void PMSM_FOC_MotorStart(void)
{
    if (PMSM_FOC_CTRL.msm_state != PMSM_FOC_MSM_ERROR)
    {
        PMSM_FOC_CTRL.motor_start_flag = 1;
    }
}

/* API to stop the Motor. It has higher priority than reference.*/
void PMSM_FOC_MotorStop(void)
{
    PMSM_FOC_CTRL.motor_start_flag = 0;
    //TODO - Set the reference values to zero for corresponding control scheme for GUI.
}

/* API to set motor direction command from configured GPIO pin */
void PMSM_FOC_SetMotorDirection(void)
{
  /* Check if GPIO pin is configured for direction set */
  #if(USER_MOTOR_BI_DIRECTION_CTRL == ENABLED)
    #ifdef MOTOR_DIR_INPUT_PIN
    if ((XMC_GPIO_GetInput(MOTOR_DIR_INPUT_PIN)) == IS_GPIO_LOW)
    {
        PMSM_FOC_CTRL.rotation_dir = DIRECTION_INC;
    }
    else
    {
        PMSM_FOC_CTRL.rotation_dir = DIRECTION_DEC;
    }
  #else
    PMSM_FOC_CTRL.rotation_dir = DIRECTION_INC;
  #endif
  #else
  /* Set the default motor direction */
  PMSM_FOC_CTRL.rotation_dir = DIRECTION_INC;
  #endif
}

#if(E_BIKE_REF == ENABLED)
void E_BIKE_THROTTLE_mode_detection(void)
{
    /*E-bike throttle mode check*/
	switch(PMSM_FOC_CTRL.ebike_throttle_mode)
	{
	case E_BIKE_THROTTLE_ECO_MODE:
		PMSM_FOC_CTRL.ebike_throttle_mode_led1 = 1;
		PMSM_FOC_CTRL.ebike_throttle_mode_led2 = 0;
		PMSM_FOC_CTRL.ebike_throttle_mode_led3 = 0;
		break;
	case E_BIKE_THROTTLE_NORMAL_MODE:
		PMSM_FOC_CTRL.ebike_throttle_mode_led1 = 0;
		PMSM_FOC_CTRL.ebike_throttle_mode_led2 = 1;
		PMSM_FOC_CTRL.ebike_throttle_mode_led3 = 0;
		break;
	case E_BIKE_THROTTLE_SPORT_MODE:
		PMSM_FOC_CTRL.ebike_throttle_mode_led1 = 0;
		PMSM_FOC_CTRL.ebike_throttle_mode_led2 = 0;
		PMSM_FOC_CTRL.ebike_throttle_mode_led3 = 1;
		break;
    default:
    	break;
	}
//	PMSM_FOC_VQ_RAMP_GEN.ramp_up_step = PMSM_FOC_CTRL.E_bike_mode_slope_rate;

}
#endif
