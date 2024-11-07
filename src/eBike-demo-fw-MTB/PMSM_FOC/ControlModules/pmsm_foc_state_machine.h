/**
 * @file pmsm_foc_state_machine.h
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
#ifndef PMSM_FOC_STATE_MACHINE_H_
#define PMSM_FOC_STATE_MACHINE_H_
/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <PMSM_FOC/FOCLib/BMCLIB/bmclib_v_hz_profile.h>
#include <PMSM_FOC/FOCLib/BMCLIB/bmclib_limiters.h>

#include <PMSM_FOC/Configuration/pmsm_foc_config.h>

#include "../ControlModules/pmsm_foc_error_handling.h"
#include "../ControlModules/pmsm_foc_speed_current_ctrl.h"
#include "../ControlModules/pmsm_foc_vq_voltage_ctrl.h"
#include "../ControlModules/pmsm_foc_functions.h"

#include "../MCUInit/pmsm_foc_posif.h"

#include "../MCUInit/pmsm_foc_ccu8.h"

#if (SPI_LIB == ENABLED)
#include "../IMD700A_SPI_LIB/utility.h"
#endif
/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup ControlModules
 * @{
 */

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * ENUMS
 ********************************************************************************************************************/
/**
 * @brief This enumerates the motor control state machine.
 */
typedef enum
{
    PMSM_FOC_MSM_CLOSED_LOOP          = (1<<0),    /*!< 0001 - FOC CLOSED LOOP  */
	PMSM_FOC_MSM_HALL_CALIB_PROCESS   = (1<<1),		 /*!< 0002 - ROTOT IPD   */
    PMSM_FOC_MSM_STOP_MOTOR           = (1<<3),		 /*!< 0008 - MOTOR STOP  */
    PMSM_FOC_MSM_VF_OPENLOOP          = (1<<4),		 /*!< 0016 - V/F OPEN LOOP */
    PMSM_FOC_MSM_TRANSITION           = (1<<5),		 /*!< 0032 - TRANSITION TO CLOSE LOOP  */
    PMSM_FOC_MSM_PRE_POSITIONING      = (1<<6),		 /*!< 0064 - ROTOR ALIGNMENT  */
    PMSM_FOC_MSM_ERROR                = (1<<7),		 /*!< 0128 - ERROR  */
    PMSM_FOC_MSM_MOTOR_BOOTSTRAP      = (1<<8),		 /*!< 0256 - BOOTSTRAP \n */
	PMSM_FOC_MSM_MOTOR_COASTING		  = (1<<9)
} PMSM_FOC_MSM_STATES_t;


/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * LOCAL API PROTOTYPES
 ********************************************************************************************************************/
__STATIC_FORCEINLINE void PMSM_FOC_MSM_VF_OPENLOOP_Func(BMCLIB_VF_OPEN_LOOP_t* const vf_open_loop_ptr);
__STATIC_FORCEINLINE void PMSM_FOC_MSM_STOP_MOTOR_Func(void);
__STATIC_FORCEINLINE void PMSM_FOC_MSM_ERROR_Func(void);
__STATIC_FORCEINLINE void PMSM_FOC_MSM_TRANSITION_Func(void);
__STATIC_FORCEINLINE void PMSM_FOC_MSM_CLOSED_LOOP_Func(void);
__STATIC_FORCEINLINE void PMSM_FOC_MSM_MOTOR_BOOTSTRAP_Func(void);
__STATIC_FORCEINLINE void PMSM_FOC_MSM_HALL_CALIB_PROCESS_Func(void);
__STATIC_FORCEINLINE void PMSM_FOC_MSM_MOTOR_COASTING_Func(void);


#if(USER_ROTOR_IPD_METHOD == ROTOR_IPD_PRE_ALIGNMENT)
__STATIC_FORCEINLINE void PMSM_FOC_MSM_PRE_POSITIONING_Func(void);
#endif

/*********************************************************************************************************************
 * API PROTOTYPES
 ********************************************************************************************************************/
extern void PMSM_FOC_SetMotorDirection(void);
/*********************************************************************************************************************
 * FUNCTION DEFINATION
 ********************************************************************************************************************/
/**
 * @brief Motor control state machine.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_FORCEINLINE void PMSM_FOC_MSM(void)
{
    switch (PMSM_FOC_CTRL.msm_state)
    {
        case PMSM_FOC_MSM_CLOSED_LOOP:
            PMSM_FOC_MSM_CLOSED_LOOP_Func();
            break;

        case PMSM_FOC_MSM_HALL_CALIB_PROCESS:
        	PMSM_FOC_MSM_HALL_CALIB_PROCESS_Func();
        	break;

        case PMSM_FOC_MSM_TRANSITION :
            PMSM_FOC_MSM_TRANSITION_Func();
            break;

        case PMSM_FOC_MSM_VF_OPENLOOP:
            PMSM_FOC_MSM_VF_OPENLOOP_Func(&PMSM_FOC_VF_OPEN_LOOP_CTRL);
            break;
            
       case PMSM_FOC_MSM_MOTOR_BOOTSTRAP:
            PMSM_FOC_MSM_MOTOR_BOOTSTRAP_Func();
            break;

       case PMSM_FOC_MSM_MOTOR_COASTING:
    	   PMSM_FOC_MSM_MOTOR_COASTING_Func();
    	   break;

       case PMSM_FOC_MSM_STOP_MOTOR:
    	   PMSM_FOC_MSM_STOP_MOTOR_Func();
    	   break;

	   #if(USER_ROTOR_IPD_METHOD == ROTOR_IPD_PRE_ALIGNMENT)
       case PMSM_FOC_MSM_PRE_POSITIONING:
    	   PMSM_FOC_MSM_PRE_POSITIONING_Func();
    	   break;
	   #endif

       case PMSM_FOC_MSM_ERROR:
    	   PMSM_FOC_MSM_ERROR_Func();
    	   break;

       default:
    	   break;
    }

}

/*********************************************************************************************************************
 * STATE MACHINE Functions
 ********************************************************************************************************************/
/**
 * @brief This function is called when Motor control state machine is set to PMSM_FOC_MSM_CLOSED_LOOP.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_FORCEINLINE void PMSM_FOC_MSM_CLOSED_LOOP_Func(void)
{
    /* Call control scheme - Ex : Vq voltage control or Speed inner current control as per user configurations */
    PMSM_FOC_CTRL.ctrl_scheme_fun_ptr();

    #if(USER_VDC_VOLT_COMPENSATION == ENABLED)
    #define COMPENSATION_FACTOR              (4U)	/* 0 to 8, adjustable for best performance */
    /* Unsigned mode,After division the result will be shifted by COMPENSATION_FACTOR */
    MATH->DIVCON = (0x00008004 | (COMPENSATION_FACTOR << MATH_DIVCON_QSCNT_Pos));
    MATH->DVD = PMSM_FOC_OUTPUT.svm_vref_16 * (VADC_DCLINK_T << COMPENSATION_FACTOR); // Dividend
    MATH->DVS = ADC.adc_res_vdc;//Divisor
    #endif

    /* Miscellaneous works in FOC, such input reference ramp control. */
    PMSM_FOC_RefRampCtrl();

    /* Wait for division operation to get over */
    #if(USER_VDC_VOLT_COMPENSATION == ENABLED)
    while(IS_MATH_DIV_BUSY);
    PMSM_FOC_OUTPUT.svm_vref_16 = BMCLIB_MAX_LIMIT( (MATH->QUOT),SVPWM_MAX_VREF);
    #endif

    /* Update SVM PWM. Execution time: ~5.4us */
    BMCLIB_SVPWM_Update(PMSM_FOC_OUTPUT.svm_vref_16, PMSM_FOC_OUTPUT.svm_angle_16, &PMSM_FOC_SVPWM);
}

__STATIC_FORCEINLINE void PMSM_FOC_MSM_MOTOR_COASTING_Func(void)
{
	if (PMSM_FOC_CTRL.motor_stop_counter == 0U)
	{
#if (SPI_LIB == ENABLED)
       /* Activate brake */
        XMC_GPIO_SetOutputLow(nBRAKE_PIN);
#else
		/* Motor coasting */
		pmsm_foc_disable_inverter();
#endif
	}
	else
	{
		/* If Motor is within x time, check for ref input */
		if (ADC.adc_res_pot > USER_TH_POT_ADC_START)
		{
			/* FOC variables initialization */
			PMSM_FOC_VariablesInit();

			/* Make sure restart using six step commutation */
			PMSM_FOC_INPUT.ref_iq = 0U;
			PMSM_FOC_INPUT.ref_iq_temp = 0U;

			PMSM_FOC_CTRL.motor_stop_counter = 0;
			PMSM_FOC_MC_PARA.rotor_angle_q31 = hall_sensor_data.hall_angleQ31;

			/* Direct go to closed loop */
			PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_CLOSED_LOOP;

			/* Enable inverter and PWM pins */
			pmsm_foc_enable_inverter();
		}
	}
	PMSM_FOC_CTRL.motor_stop_counter++;

	/* Check speed below or approximately zero, go to motor stop */
	if ((hall_sensor_data.speed < BRAKE_MIN_SPEED) || (PMSM_FOC_CTRL.motor_stop_counter >=200))
	{
		PMSM_FOC_CTRL.motor_stop_counter = 0;
		PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_STOP_MOTOR;		//BOOTSTRAP state motor jerk happen
	}
}

/**********************************************************************************************************************
 Stop the Motor, check PWM or POT ADC (for adjusting motor speed)
 -----------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief This function is called when Motor control state machine is set to PMSM_FOC_MSM_STOP_MOTOR.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_FORCEINLINE void PMSM_FOC_MSM_STOP_MOTOR_Func(void)
{
    if (PMSM_FOC_CTRL.error_status == 0)
    {
#if (E_BIKE_REF == ENABLED)
    	if (SYSTEM_BE_IDLE || PMSM_FOC_CTRL.ebike_brake == 1)
#else
    	if (SYSTEM_BE_IDLE)
#endif
        {
#if (SPI_LIB == ENABLED)
    		/* Activate brake */
    		XMC_GPIO_SetOutputLow(nBRAKE_PIN);
#else
            /* If system is idle, i.e.: Reference or POT ADC too low.*/
        	pmsm_foc_disable_inverter();
#endif

        	/* In the case motor stop via emergeny stop button in GUI */
        	PMSM_FOC_CTRL.motor_start_flag = 1;
        	ADC.adc_res_pot = 0;

        	pmsm_foc_get_current_bias();
        }
        else
        {
            PMSM_FOC_CTRL.motor_stop_counter = 0;

#if (SPI_LIB == ENABLED)
            /* Make sure all gate driver faults are clear before motor start */
            Driver_Faults_Clear();
#endif

            /* Reset phase offset calibration counter */
            phase_offset_calib_counter = 0;

            /* Next go to Motor coasting function and wait for motor start command */
            PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_MOTOR_BOOTSTRAP;
        }
    }
    else
    {
#if (SPI_LIB == ENABLED)
        /* Activate brake */
        XMC_GPIO_SetOutputLow(nBRAKE_PIN);
#else
        pmsm_foc_disable_inverter();
#endif//SPI_LIB=ENABLED

        /* Next go to error state */
        PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_ERROR;
    }
}

/**********************************************************************************************************************
 Rotor initial preposition/alignment
 -----------------------------------------------------------------------------------------------------------------------*/
#if(USER_ROTOR_IPD_METHOD == ROTOR_IPD_PRE_ALIGNMENT)
/**
 * @brief This function is called when Motor control state machine is set to PMSM_FOC_MSM_PRE_POSITIONING.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_FORCEINLINE void PMSM_FOC_MSM_PRE_POSITIONING_Func(void)
{
    int32_t angle;
    uint32_t i_mag;
    
    if (PMSM_FOC_CTRL.alignment_counter == 0)
    {
        /* Align to SVM 0 degree*/
        PMSM_FOC_OUTPUT.svm_angle_16 = 0;
        PMSM_FOC_OUTPUT.svm_vref_16 = 0;
        
        PMSM_FOC_VADC_PhCurrentInit();
        
        /* Update SVM PWM. */
        BMCLIB_SVPWM_Update(PMSM_FOC_OUTPUT.svm_vref_16, PMSM_FOC_OUTPUT.svm_angle_16,&PMSM_FOC_SVPWM);
        
        /* Rotor preposition/alignment counter ++. */
        PMSM_FOC_CTRL.alignment_counter++;
    }
    else if (PMSM_FOC_CTRL.alignment_counter < ROTOR_PRE_ALIGNMENT_COUNT)
    {
        /* Reads phase current from VADC,Removes the offset and convert as per scaling system */
        PMSM_FOC_CurrentReconstruction(&ADC, &PMSM_FOC_SVPWM, &PMSM_FOC_CTRL, &PMSM_FOC_INPUT);

        /* Calculate current magnitude */
        BMCLIB_ClarkeTransform(PMSM_FOC_INPUT.i_u,PMSM_FOC_INPUT.i_v,PMSM_FOC_INPUT.i_w,&PMSM_FOC_CLARKE_TRANSFORM);

        BMCLIB_CartToPolar(PMSM_FOC_CLARKE_TRANSFORM.alpha, PMSM_FOC_CLARKE_TRANSFORM.beta,0);
        BMCLIB_CartToPolarGetResult(&i_mag, &angle);
        
        PMSM_FOC_OUTPUT.current_i_mag = (uint16_t)(i_mag>>CORDIC_SHIFT);

        /* Vref increases gradually. */
        if( PMSM_FOC_OUTPUT.current_i_mag < PMSM_FOC_INPUT.ref_rotor_pre_align_current)
        {
            if(PMSM_FOC_OUTPUT.svm_vref_16 < SVPWM_MAX_VREF)
            {
              PMSM_FOC_OUTPUT.svm_vref_16++;
            }
        }
        else
        {
            if(PMSM_FOC_OUTPUT.svm_vref_16 > 0)
            {
              PMSM_FOC_OUTPUT.svm_vref_16--;
            }
        }

        /* Update SVM PWM. */
        BMCLIB_SVPWM_Update(PMSM_FOC_OUTPUT.svm_vref_16, PMSM_FOC_OUTPUT.svm_angle_16,&PMSM_FOC_SVPWM);

        /* Rotor preposition/alignment counter++. */
        PMSM_FOC_CTRL.alignment_counter++;
    }
    else
    {
        /* Clear counter. */
        PMSM_FOC_CTRL.alignment_counter = 0U;
        PMSM_FOC_MC_PARA.rotor_angle_q31 = PMSM_FOC_ANGLE_000_DEGREE_Q31;
        
        #if((USER_MOTOR_STARTUP_METHOD == MOTOR_STARTUP_DIRECT_FOC) && (USER_FOC_CTRL_SCHEME != VF_OPEN_LOOP_CTRL))
            /* Next go to transition state to enter into close loop */
            PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_TRANSITION;
        #else         
            /* V/F open loop initialization */
            PMSM_FOC_VF_OpenLoopInit();
            PMSM_FOC_VF_OPEN_LOOP_CTRL.vref_angle = (int32_t)(((int16_t)PMSM_FOC_OUTPUT.svm_vref_16)<<16);
        #endif
    }
}
#endif

/**********************************************************************************************************************
 HALL zero degree identification closely related to preposition/alignment
 -----------------------------------------------------------------------------------------------------------------------*/
__STATIC_FORCEINLINE void PMSM_FOC_MSM_HALL_CALIB_PROCESS_Func(void)
{
	static uint8_t idx =0;

	if (hall_sensor_data.run_hall_pat_dir_id == DISABLED)
	{
		if (hall_sensor_data.zero_deg_id_counter < ROTOR_ZERO_DEG_ID_COUNT)
		{
			PMSM_FOC_OUTPUT.svm_angle_16 = 0U;

			/* Vref increases gradually. */
			if(PMSM_FOC_OUTPUT.svm_vref_16 < ROTOR_ZERO_DEG_ID_SVPWM_REF)
			{
				PMSM_FOC_OUTPUT.svm_vref_16 += ROTOR_ZERO_DEG_ID_RAMPUP_STEP;
			}

			/* Update SVM PWM. */
			BMCLIB_SVPWM_Update(PMSM_FOC_OUTPUT.svm_vref_16, PMSM_FOC_OUTPUT.svm_angle_16,&PMSM_FOC_SVPWM);

			hall_sensor_data.zero_deg_id_counter ++;
		}
		else
		{
			pmsm_foc_speed_pos_get_hall_pos(&hall_sensor_data.zero_hall_pos);
			hall_sensor_data.zero_deg_id_counter = 0;

			/* Proceed with hall pattern direction identification */
			hall_sensor_data.run_hall_pat_dir_id = ENABLED;

			/* V/F open loop initialization */
			PMSM_FOC_VF_OpenLoopInit();

			/* Configure current and expected hall patterns as 0U so that interrupt is triggered on any hall edge change */
			XMC_POSIF_HSC_SetHallPatterns(POSIF_MODULE, pmsm_foc_hall_pattern.hall_pattern_pos[0]);

			/* Make sure consecutive HALL_PAT_DIR_ID state, element zero has correct continuous hall state */
			hall_pat_open_run_previous = hall_sensor_data.cur_hall_pos;

			/* Overwrite some V/F initializaton for hall pattern direction id */
			PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_HALL_CALIB_PROCESS;

			/* Set the hall direction identification status to in progress before start id process */
			hall_sensor_data.status = HALL_CALIB_DIR_ID_STATUS_IN_PROGRESS;
		}
	}
	else
	{
		/* Record the hall pattern seq */
		if ((hall_sensor_data.cur_hall_pos != hall_pat_open_run_previous) && (idx<=15))
		{
			hall_pat_open_run_previous = hall_sensor_data.cur_hall_pos;
			hall_pat_open_run[idx++] = hall_sensor_data.cur_hall_pos;
		}
		else if(idx > 15U)
		{
			idx = 0;
			hall_sensor_data.status = HALL_CALIB_DIR_ID_STATUS_COMPLETED;
		}

		/* Run open loop */
		if(hall_sensor_data.status == HALL_CALIB_DIR_ID_STATUS_IN_PROGRESS)
		{
			BMCLIB_VHzProfileGen(&PMSM_FOC_VF_OPEN_LOOP_CTRL);

			/* Assign open loop speed to FOC estimator for GUI display */
			PMSM_FOC_MC_PARA.rotor_speed = PMSM_FOC_VF_OPEN_LOOP_CTRL.vf_motor_speed;
			/*  Update SVM PWM. */
			PMSM_FOC_OUTPUT.svm_vref_16 = PMSM_FOC_VF_OPEN_LOOP_CTRL.vref_mag;
			PMSM_FOC_OUTPUT.svm_angle_16 = (uint16_t)(PMSM_FOC_VF_OPEN_LOOP_CTRL.vref_angle >> 16);
			BMCLIB_SVPWM_Update(PMSM_FOC_OUTPUT.svm_vref_16,PMSM_FOC_OUTPUT.svm_angle_16,&PMSM_FOC_SVPWM);
		}
		else
		{
			/* restart the hall zero angle id */
			hall_sensor_data.run_hall_pat_dir_id = DISABLED;
			ADC.adc_res_pot = 0U;						/* to stay in MOTOR_STOP after id Func */

			/* Determine the pos/neg hall pattern in U,V,W pos direction*/
			hall_sensor_data.hall_pat_dir = pmsm_foc_hall_pattern_id();

			/* Reset the default hall pattern initialization based on identified direction */
			if (hall_sensor_data.hall_pat_dir != DIRECTION_INVALID)
			{
				pmsm_foc_reset_hall_pattern();
				pmsm_foc_speed_pos_get_hall_pos(&hall_sensor_data.cur_hall_pos);
				/* Configure current and expected hall patterns */
				XMC_POSIF_HSC_SetHallPatterns(POSIF_MODULE, pmsm_foc_hall_pattern.hall_pattern_pos[hall_sensor_data.cur_hall_pos]);
				/* Update hall pattern */
				XMC_POSIF_HSC_UpdateHallPattern(POSIF_MODULE);

				/* Reconfigure hall pattern angle provided the direction information */
				pmsm_foc_hall_pat_reinit(hall_sensor_data.zero_hall_pos);
			}

			PMSM_FOC_CTRL.motor_stop_counter = 0;

			/* Next go to Motor stop function and wait for motor start command */
			PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_STOP_MOTOR;
		}

		/* Assign open loop speed to FOC estimator for GUI display */
		PMSM_FOC_MC_PARA.rotor_speed = PMSM_FOC_VF_OPEN_LOOP_CTRL.vf_motor_speed;
		/*  Update SVM PWM. */
		PMSM_FOC_OUTPUT.svm_vref_16 = PMSM_FOC_VF_OPEN_LOOP_CTRL.vref_mag;
		PMSM_FOC_OUTPUT.svm_angle_16 = (uint16_t)(PMSM_FOC_VF_OPEN_LOOP_CTRL.vref_angle >> 16);
		BMCLIB_SVPWM_Update(PMSM_FOC_OUTPUT.svm_vref_16,PMSM_FOC_OUTPUT.svm_angle_16,&PMSM_FOC_SVPWM);
	}
}

/**********************************************************************************************************************
 V/f Open Loop Ramp Up with rotor initial preposition/alignment
 -----------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief This function is called when Motor control state machine is set to PMSM_FOC_MSM_VF_OPENLOOP.
 *
 * @param None
 *
 * @retval None
 */

__STATIC_FORCEINLINE void PMSM_FOC_MSM_VF_OPENLOOP_Func(BMCLIB_VF_OPEN_LOOP_t* const vf_open_loop_ptr)
{
    uint32_t cart2pol_mag;
    int32_t cart2pol_angle_vf;

    /* Reads phase current from VADC,Removes the offset and convert as per scaling system */
    PMSM_FOC_CurrentReconstruction(&ADC, &PMSM_FOC_SVPWM, &PMSM_FOC_CTRL, &PMSM_FOC_INPUT);

    /* To get I_Alpha and I_Beta of last PWM cycle. */
    BMCLIB_ClarkeTransform(PMSM_FOC_INPUT.i_u,PMSM_FOC_INPUT.i_v,PMSM_FOC_INPUT.i_w,&PMSM_FOC_CLARKE_TRANSFORM);

    /* CORDIC - I_mag = sqrt(i_alpha^2 + i_beta^2) */
    BMCLIB_CartToPolar(PMSM_FOC_CLARKE_TRANSFORM.alpha,PMSM_FOC_CLARKE_TRANSFORM.beta,0);

    /* Get CORDIC result */
    BMCLIB_CartToPolarGetResult(&cart2pol_mag,&cart2pol_angle_vf);
    PMSM_FOC_OUTPUT.current_i_mag = (uint16_t)(cart2pol_mag>>CORDIC_SHIFT);

    if(vf_open_loop_ptr->status == V_HZ_PROFILE_GEN_STATUS_IN_PROGRESS)
    {
        BMCLIB_VHzProfileGen(&PMSM_FOC_VF_OPEN_LOOP_CTRL);
    }
    else
    {
        PMSM_FOC_MC_PARA.rotor_angle_q31  = vf_open_loop_ptr->vref_angle;
        
        /* Init PLL speed to open loop transition speed for smooth startup.*/
        PMSM_FOC_MC_PARA.rotor_speed = vf_open_loop_ptr->vf_motor_speed;
              
        PMSM_FOC_TORQUE_PI.uk = vf_open_loop_ptr->vref_mag;
        PMSM_FOC_TORQUE_PI.ik = (int32_t)(PMSM_FOC_TORQUE_PI.uk << PMSM_FOC_TORQUE_PI.scale_kpki);

        /* motor reference speed of FOC close loop. */
        PMSM_FOC_INPUT.ref_speed = vf_open_loop_ptr->vf_motor_speed;

        PMSM_FOC_INPUT.ref_vq = vf_open_loop_ptr->vref_mag;

        /* Next, go to FOC closed-loop. */
        PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_CLOSED_LOOP;
    }

    /* Check if Motor stop is requested */
    if (SYSTEM_BE_IDLE)
    {
        /* Next, go to Motor Stop. */
    	PMSM_FOC_CTRL.motor_stop_counter = 0;		/* For avoid the pmsm_foc_ccu8_duty_set_zero */

    	/* Next go to Motor coasting function and wait for motor start command */
    	PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_STOP_MOTOR;
    }

    /* Assign open loop speed to FOC estimator for GUI display */
    PMSM_FOC_MC_PARA.rotor_speed = vf_open_loop_ptr->vf_motor_speed;
    /*  Update SVM PWM. */
    PMSM_FOC_OUTPUT.svm_vref_16 = vf_open_loop_ptr->vref_mag;
    PMSM_FOC_OUTPUT.svm_angle_16 = (uint16_t)(vf_open_loop_ptr->vref_angle >> 16);
    BMCLIB_SVPWM_Update(PMSM_FOC_OUTPUT.svm_vref_16,PMSM_FOC_OUTPUT.svm_angle_16,&PMSM_FOC_SVPWM);

}

/**********************************************************************************************************************
 To brake the Motor, charge gate driver bootstrap capacitors (if any)
 -----------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief This function is called when Motor control state machine is set to PMSM_FOC_MSM_MOTOR_BOOTSTRAP.
 *
 * @param None
 *
 * @retval None
 */

/***********************************************************************************************************************/
__STATIC_FORCEINLINE void PMSM_FOC_MSM_MOTOR_BOOTSTRAP_Func(void)
{
	if (PMSM_FOC_CTRL.motor_stop_counter == 0U)
	{
		/* ENABLE gate driver. */
		#ifdef INVERTER_EN_PIN
		pmsm_foc_enable_inverter();
		#endif
    }
    else if (PMSM_FOC_CTRL.motor_stop_counter > BOOTSTRAP_PRECHARGE_TIME)
    {
    	if (ADC.adc_res_pot > USER_TH_POT_ADC_START)
    	{
            /* Make sure all gate driver faults are clear before motor start */
            Driver_Faults_Clear();

    		/* Before motor start read the direction command */
    		PMSM_FOC_SetMotorDirection();

    		/* FOC variables initialization */
    		PMSM_FOC_VariablesInit();
    		pmsm_foc_hall_foc_varialbe_init();

            /* Disable low side braking in 6EDL7141 gate driver through nBRAKE pin */
            XMC_GPIO_SetOutputHigh(nBRAKE_PIN);

#if (MOTOR_CTRL_SCHEME == PMSM_HALL_FOC && USER_FOC_CTRL_SCHEME != VF_OPEN_LOOP_CTRL)
    		if(hall_sensor_data.run_zero_angle_id == ENABLED)
    		{
    			PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_HALL_CALIB_PROCESS;
    		}
    		else
    		{
    			PMSM_FOC_MC_PARA.rotor_angle_q31 = hall_sensor_data.hall_angleQ31;
    			PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_CLOSED_LOOP;
    		}

#else
#if(USER_ROTOR_IPD_METHOD == ROTOR_IPD_PRE_ALIGNMENT)
    		/* Next, go to the rotor pre-position startup. */
    		PMSM_FOC_CTRL.alignment_counter = 0;
    		PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_PRE_POSITIONING;
#endif //End of #if(USER_ROTOR_IPD_METHOD == ROTOR_IPD_PRE_ALIGNMENT)
#endif

    		/* Clear counters. */
    		PMSM_FOC_CTRL.motor_stop_counter = 0U;

    		//TODO Check if this is required.
    		PMSM_FOC_CTRL.motor_start_flag = 1U;
    	}
    }

    PMSM_FOC_CTRL.motor_stop_counter++;

    if (PMSM_FOC_CTRL.motor_stop_counter > SYSTEM_IDLE_TIME_COUNT)
    {
    	/* Clear counters. */
    	PMSM_FOC_CTRL.motor_stop_counter = 0U;

    	/* Next go to PMSM_FOC_MSM_IDLE_MOTOR. Temp */
    	PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_STOP_MOTOR;
    }
}

/**********************************************************************************************************************
 Error handling function
 -----------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief This function is called when Motor control state machine is set to PMSM_FOC_MSM_ERROR.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_FORCEINLINE void PMSM_FOC_MSM_ERROR_Func(void)
{
    /* Reset variables */
    PMSM_FOC_INPUT.ref_id = 0;
    PMSM_FOC_INPUT.ref_iq = 0;
    PMSM_FOC_INPUT.ref_iq_temp = 0;
    PMSM_FOC_INPUT.ref_speed = 0;
    PMSM_FOC_INPUT.ref_vq = 0;

    PMSM_FOC_MC_PARA.rotor_speed = 0;
    PMSM_FOC_MC_PARA.rotor_angle_q31 = 0;

    PMSM_FOC_ErrorHandling();
}

/**********************************************************************************************************************
Transition - V/F to close loop function
-----------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief This function is called when motor control state machine is set to PMSM_FOC_MSM_TRANSITION.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_FORCEINLINE void PMSM_FOC_MSM_TRANSITION_Func(void)
{
    uint32_t cart2pol_mag;
    int32_t angle_q31;

    #if(USER_ROTOR_IPD_METHOD != ROTOR_IPD_NONE)
    if(PMSM_FOC_CTRL.first_kick_counter == 0U)
    {
        /* Initialize SVPWM initial angle to start motor in correct direction */
		if(PMSM_FOC_CTRL.rotation_dir == DIRECTION_DEC)
		{
        PMSM_FOC_MC_PARA.rotor_angle_q31 = -PMSM_FOC_MC_PARA.rotor_angle_q31;
		}

		PMSM_FOC_OUTPUT.svm_angle_16 = (uint16_t)((PMSM_FOC_MC_PARA.rotor_angle_q31 + PMSM_FOC_ANGLE_090_DEGREE_Q31)>>16);


		PMSM_FOC_CTRL.first_kick_counter++;
    }
    else
    {
        /* Minimum 2 counts are required to synchronize SVPWM & ADC readings to enter into close loop */
        if(PMSM_FOC_CTRL.first_kick_counter >= TRANSITION_FIRST_KICK_COUNT)
        {
            /* next go to close loop */
            PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_CLOSED_LOOP;
            PMSM_FOC_CTRL.first_kick_counter = 0U;

            /*reset throttle hall_change_chnt for closed-loop moving average*/
            PMSM_FOC_THROTTLE_AVG.hall_change_cnt = 0;
            PMSM_FOC_THROTTLE_AVG.hall_change_flag = 0;
        }
        else
        {
            PMSM_FOC_CTRL.first_kick_counter++;
        }
    }

	BMCLIB_SVPWM_Update(PMSM_FOC_INPUT.ref_vq, PMSM_FOC_OUTPUT.svm_angle_16, &PMSM_FOC_SVPWM);
	PMSM_FOC_SVPWM.previous_sector_num = (PMSM_FOC_SVPWM.current_sector_num + 5)%6;

	/* current reconstruction uses PMSM_FOC_SVPWM.previous_sector_num */
    PMSM_FOC_CurrentReconstruction(&ADC, &PMSM_FOC_SVPWM, &PMSM_FOC_CTRL, &PMSM_FOC_INPUT);

    /* Clarke Transform */
    BMCLIB_ClarkeTransform(PMSM_FOC_INPUT.i_u,PMSM_FOC_INPUT.i_v,PMSM_FOC_INPUT.i_w,&PMSM_FOC_CLARKE_TRANSFORM);

    /* CORDIC - I_mag = sqrt(i_alpha^2 + i_beta^2) */
    BMCLIB_CartToPolar(PMSM_FOC_CLARKE_TRANSFORM.alpha,PMSM_FOC_CLARKE_TRANSFORM.beta,0);

    /**************************************************************************************************************/
    /* Parallel computation - CPU(PMSM_FOC_RefRampCtrl) and CORDIC(I_mag = sqrt(i_alpha^2 + i_beta^2)) */
    /**************************************************************************************************************/
    PMSM_FOC_RefRampCtrl();

    /* Get CORDIC result */
    BMCLIB_CartToPolarGetResult(&cart2pol_mag,&angle_q31);

    PMSM_FOC_OUTPUT.current_i_mag = (uint16_t)(cart2pol_mag >> CORDIC_SHIFT);
    PMSM_FOC_OUTPUT.current_i_mag_filtered += (uint16_t)((PMSM_FOC_OUTPUT.current_i_mag - PMSM_FOC_OUTPUT.current_i_mag_filtered) >> 2);

    #endif
}

/*********************************************************************************************************************
 * STATE MACHINE Functions                                                                                     - END
 ********************************************************************************************************************/

/**
 * @}
 */

/**
 * @}
 */

#endif

