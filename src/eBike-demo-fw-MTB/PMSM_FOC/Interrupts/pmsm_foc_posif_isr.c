/**
 * @file pmsm_foc_posif_isr.c
 * @Firmware PMSM_FOC_SL_XMC14_V1_5_8
 * @Modified date: 2019-01-10
 *
 * @cond
 ****************************************
 * PMSM FOC Motor Control Library
 *
 * Copyright (c) 2015-2019, Infineon Technologies AG
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
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes
 * with Infineon Technologies AG (dave@infineon.com).
 ******************************************
 *
 * @file pmsm_foc_posif_isr.c
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * @endcond
 *
 */

/*********************************************************************************************************************
 * HEADER FILES
 ***************************************/
#include "..\MCUInit\pmsm_foc_posif.h"
#include "..\ControlModules\pmsm_foc_functions.h"
//#include "..\MIDSys\pmsm_foc_speed_pos_hall.h"

/*********************************************************************************************************************
 * GLOBAL DATA
 ***************************************/

/*******************************************************************************
* Function Name: POSIF_IRQHandler: XMC_POSIF_IRQ_EVENT_CHE
********************************************************************************
* Summary:
*  POSIF_IRQHandler interrupt handler function will occur for every
*  correct hall pattern. Calculate the timing between two correct hall events.
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/

#if(MOTOR_CTRL_SCHEME == PMSM_HALL_FOC)
static void pmsm_foc_posif_wrong_hall_event(void);
static void pmsm_foc_posif_correct_hall_event(void);

/**
 * @param None
 * @return none <br>
 *
 * \par<b>Description:</b><br>
 * Event is mapped to correct and wrong hall event of POSIF.\n
 * Correct hall - used ONLY in STATE_IDENTIFICATION state to get the speed, position and direction of the freely running motor.\n
 * Wrong hall - used to find direction reversal or real wrong hall event (hall failure) \n
 * For reverse hall, apply next hall pattern based on the intended direction.
 * For hall failure, stop the motor and change the state to ERROR.
 */
__RAM_FUNC void POSIF_IRQHandler(void)
{
	if (hall_sensor_data.run_hall_pat_dir_id == DISABLED)
	{
		/* Wrong hall event */
		if (XMC_POSIF_GetEventStatus(POSIF_MODULE, XMC_POSIF_IRQ_EVENT_WHE) == (uint8_t)1)
		{
			/* handle reverse hall event and correct the pattern */
			pmsm_foc_posif_wrong_hall_event();
		}
		/* Correct hall event */
		else
		{
			pmsm_foc_posif_correct_hall_event();
			PMSM_FOC_THROTTLE_AVG.hall_change_flag = 1;
		}
	}
	else
	{
	  if ( hall_sensor_data.open_loop_stable_counter > USER_HALL_EDGE_COUNT)
	  {
		  pmsm_foc_speed_pos_get_hall_pos(&hall_sensor_data.cur_hall_pos);
	  }
	  else
	  {
		  hall_sensor_data.open_loop_stable_counter++;
	  }
	}
}


/*
 * Find the direction of the motor in free running condition.
 * This event is disabled once control is switched to closed loop.
 */
static void pmsm_foc_posif_correct_hall_event(void)
{
#if(HALL_ISR_DEBUG_IO == ENABLED)
  //XMC_GPIO_ToggleOutput(HALL_ISR_DEBUG_PIN);
  XMC_GPIO_PORT0->OMR = 0x10001U << 4;
#endif
//  XMC_GPIO_ToggleOutput(TEST_PIN);
//  XMC_GPIO_SetOutputHigh(TEST_PIN);

  /* Get captured timer value on rising edge */
  hall_sensor_data.event_counter = HALL_CAPTURE_SLICE->CV[1];		//XMC_CCU4_SLICE_GetCaptureRegisterValue(HALL_CAPTURE_SLICE, 1U);

  hall_sensor_data.prev_hall_pos = hall_sensor_data.cur_hall_pos;

  pmsm_foc_speed_pos_get_hall_pos(&hall_sensor_data.cur_hall_pos);
  hall_sensor_data.hall_angleQ31 = hall_pat_angle[hall_sensor_data.cur_hall_pos];

  if(hall_sensor_data.first_edge_flag == 0U)
  {
    pmsm_foc_hall_foc_struc_init();
    hall_sensor_data.first_edge_flag = 1U;
    hall_sensor_data.rotor_angleQ31 = hall_sensor_data.hall_angleQ31;
  }
  else
  {
    if(hall_sensor_data.whe_flag == 1)
    {
      hall_sensor_data.rotor_angleQ31 = hall_sensor_data.hall_angleQ31;
      hall_sensor_data.correct_edge = 0U;
    }
    else
    {
      /* Leave the rotor speed estimation in high speed loop */
      hall_sensor_data.correct_edge = 1U;
    }

  } //end of if(hall_sensor_data.first_edge_flag == 0U)

  /* Set che_flag to 1 */
  hall_sensor_data.che_flag = 1;
  /* Set whe_flag to 0 */
  hall_sensor_data.whe_flag = 0;

  /* Configure current and next expected hall patterns */
  POSIF_MODULE->HALPS = (uint32_t)(pmsm_foc_hall_pattern.hall_pattern_pos[hall_sensor_data.cur_hall_pos] & (POSIF_HALPS_HCPS_Msk | POSIF_HALPS_HEPS_Msk));
  /* Update hall pattern */
  POSIF_MODULE->MCMS = (uint32_t)POSIF_MCMS_STHR_Msk;	//XMC_POSIF_HSC_UpdateHallPattern(POSIF0);
  /* Clear pending event */
  XMC_POSIF_ClearEvent(POSIF_MODULE, XMC_POSIF_IRQ_EVENT_CHE);

  /* Reset stall counter on correct hall event*/
  hall_sensor_data.stall_counter = 0;

  /* For GUI debug */
  PMSM_FOC_MC_PARA.rotor_speed = hall_sensor_data.speed;

#if(HALL_ISR_DEBUG_IO == ENABLED)
  //XMC_GPIO_ToggleOutput(HALL_ISR_DEBUG_PIN);
  XMC_GPIO_PORT0->OMR = 0x10001U << 4;
#endif

#if(USER_MOTOR_STALL_DETECTION_ENABLE == ENABLED)
  /* Reset stall counter on correct hall event*/
  hall_sensor_data.stall_counter = 0;
#endif
}

/**
 * @}
 */
/**
 * @}
 */

/*
 * if sampled hall pattern matches the pattern in the reverse direction, apply the next
 * hall and multi-channel pattern for the intended direction and perform forced shadow transfer.
 * Restart capture timer and calculate the speed.
 *
 * if hall failure, stop the motor and change the state to ERROR.
 */
static void pmsm_foc_posif_wrong_hall_event(void)
{
#ifdef TEST_PIN
  //XMC_GPIO_SetOutputHigh(TEST_PIN);
  XMC_GPIO_PORT4->OMR = (uint32_t)0x1U << 8;
#endif

  uint8_t reverse_pat;      /* Expected reverse direction pattern based on last sample pattern */
  uint8_t correct_pat;      /* Expected correct direction pattern based on last sample pattern */

  hall_sensor_data.prev_hall_pos = hall_sensor_data.cur_hall_pos;
  /* Read the Hall input GPIO pins */
  pmsm_foc_speed_pos_get_hall_pos(&hall_sensor_data.cur_hall_pos);

	/*Get the sampled hall pattern*/
  reverse_pat = pmsm_foc_hall_pattern.hall_pattern_neg[hall_sensor_data.prev_hall_pos];
  correct_pat = pmsm_foc_hall_pattern.hall_pattern_pos[hall_sensor_data.prev_hall_pos];;

  /* Check if the pattern matches the correct or reverse hall pattern */
  if (( reverse_pat == hall_sensor_data.cur_hall_pos) || ( correct_pat == hall_sensor_data.cur_hall_pos))
  {
      /* Reverse hall event detected, so update actual motor direction */
    /* Set whe_flag to 1 */
    hall_sensor_data.whe_flag = 1;
    /* Set che_flag to 0 */
    hall_sensor_data.che_flag = 0;
//
    hall_sensor_data.hall_angleQ31 = hall_pat_angle[hall_sensor_data.cur_hall_pos];
    hall_sensor_data.rotor_angleQ31 = hall_sensor_data.hall_angleQ31;
//
    /* Configure current and next expected hall patterns */
    POSIF_MODULE->HALPS = (uint32_t)(pmsm_foc_hall_pattern.hall_pattern_pos[hall_sensor_data.cur_hall_pos] & (POSIF_HALPS_HCPS_Msk | POSIF_HALPS_HEPS_Msk));
    /* Update hall pattern */
    POSIF_MODULE->MCMS = (uint32_t)POSIF_MCMS_STHR_Msk;	//XMC_POSIF_HSC_UpdateHallPattern(POSIF0);

    hall_sensor_data.event_counter = 999;
//
  #if(USER_MOTOR_STALL_DETECTION_ENABLE == ENABLED)
    /* Reset stall counter on wrong hall event*/
    hall_sensor_data.stall_counter = 0;
  #endif
  }
  /* Hall failure - sampled hall pattern does not match with either direction pattern */
  else
  {
    /* TODO: wrong hall signal, stop the motor */
  }

  /* Clear pending event */
  XMC_POSIF_ClearEvent(POSIF_MODULE, XMC_POSIF_IRQ_EVENT_WHE);

#ifdef TEST_PIN
  //XMC_GPIO_SetOutputLow(TEST_PIN);
  XMC_GPIO_PORT4->OMR = 0x10000U << 8;
#endif

}
#endif		/* #if(MOTOR_CTRL_SCHEME == PMSM_HALL_FOC) */
