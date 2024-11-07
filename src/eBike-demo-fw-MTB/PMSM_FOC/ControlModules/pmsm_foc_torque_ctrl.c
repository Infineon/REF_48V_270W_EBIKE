/**
 * @file pmsm_foc_torque_ctrl.c
 * @date 16 Aug, 2023
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

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "pmsm_foc_speed_current_ctrl.h"
#include "pmsm_foc_functions.h"
#include "pmsm_foc_state_machine.h"
#include "../FOCLib/BMCLIB/bmclib_limiters.h"

/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/
 #define LPF_I_FILT_COEFF (2U)     /* Low pass filter to remove high frequency current noise */
 #define SPEED_LPF_DQ_DECOUPLING_COEFF (4)  /* Low pass filtering for DQ decoupling components */
/*********************************************************************************************************************
 * LOCAL DATA
 ********************************************************************************************************************/
static int32_t decoupling_wlqiq_lpf = 0;
static int32_t decoupling_wldid_bemf_lpf = 0;

/* ******************************************************************************************************************
 * FOC controller LIB, calculated once in each PWM cycle.
 * *****************************************************************************************************************/
PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_TorqueCtrl (void)
{
    uint32_t cart2pol_mag;
    int32_t angle_q31;
    int32_t hypermag_res;
    int32_t id_temp;
    int32_t iq_temp;
    #if(USER_DQ_DECOUPLING == ENABLED)
    int32_t decoupling_wlqiq = 0;
    int32_t decoupling_wldid_bemf = 0;
    int32_t back_emf = 0;
    #endif

    /* Reads phase current from VADC,Removes the offset and convert as per scaling system */
    PMSM_FOC_CurrentReconstruction(&ADC, &PMSM_FOC_SVPWM, &PMSM_FOC_CTRL, &PMSM_FOC_INPUT);

    /* Clarke transform  - Execute on CPU */
    BMCLIB_ClarkeTransform(PMSM_FOC_INPUT.i_u,PMSM_FOC_INPUT.i_v,PMSM_FOC_INPUT.i_w,&PMSM_FOC_CLARKE_TRANSFORM);

    /**************************************************************************************************************/
    /* Parallel computation #1 - CPU(--) and CORDIC(PARK Transform) */
    /**************************************************************************************************************/
    /* Park transform  - Execute on CORDIC */
    BMCLIB_ParkTransform(PMSM_FOC_CLARKE_TRANSFORM.alpha,PMSM_FOC_CLARKE_TRANSFORM.beta,PMSM_FOC_MC_PARA.rotor_angle_q31);

    /* Read CORDIC result */
    BMCLIB_ParkTransformGetResult(&iq_temp, &id_temp);

    /* update PMSM_FOC_OUTPUT structure */
    PMSM_FOC_OUTPUT.flux_id = (int16_t)(id_temp >> CORDIC_SHIFT);
    PMSM_FOC_OUTPUT.torque_iq = (int16_t)(iq_temp >> CORDIC_SHIFT);

    /**************************************************************************************************************/
    /* Parallel computation #2 - CPU(Decoupling) and CORDIC(Vs Calculations) */
    /**************************************************************************************************************/
    /* CORDIC - sqrt(vd^2+vq^2) */
    BMCLIB_CartToPolar((int32_t)(PMSM_FOC_FLUX_PI.uk<<CORDIC_SHIFT),(int32_t)(PMSM_FOC_TORQUE_PI.uk<<CORDIC_SHIFT),0);

    /* CPU - Filtered iq current component */
    PMSM_FOC_OUTPUT.iq_filtered += (int16_t)((PMSM_FOC_OUTPUT.torque_iq - PMSM_FOC_OUTPUT.iq_filtered)>> LPF_I_FILT_COEFF);

	/* CPU - Decoupling Flux and Torque PI Control */
    #if(USER_DQ_DECOUPLING == ENABLED)
    if(1) /*TODO: conditions to be changed later*/
    {
    decoupling_wlqiq = (int32_t)(((int32_t)((PMSM_FOC_MC_PARA.rotor_speed * (int32_t)PMSM_FOC_OUTPUT.torque_iq ) >> 15) * PMSM_FOC_MC_PARA.phase_inductance_lq) >> PMSM_FOC_MC_PARA.phase_inductance_scale);

    back_emf = (int32_t)((PMSM_FOC_MC_PARA.rotor_speed * MOTOR_FLUX_LINKAGE_CONSTANT) >> MOTOR_SCALE_OF_FLUX_LINKAGE_CONSTANT);

    decoupling_wldid_bemf = back_emf + (int32_t)(((int32_t)((PMSM_FOC_MC_PARA.rotor_speed * PMSM_FOC_OUTPUT.flux_id) >> 15) * PMSM_FOC_MC_PARA.phase_inductance_ld) >> PMSM_FOC_MC_PARA.phase_inductance_scale);

    /* Limit min/max decoupling components */
    decoupling_wlqiq = MIN_MAX_LIMIT(decoupling_wlqiq, 32767, -32767);
    decoupling_wldid_bemf = MIN_MAX_LIMIT(decoupling_wldid_bemf, 32767, -32767);

    /* Low pass filter to dq decoupling */
    decoupling_wlqiq_lpf += (int32_t)((decoupling_wlqiq - decoupling_wlqiq_lpf)>>SPEED_LPF_DQ_DECOUPLING_COEFF);
    decoupling_wldid_bemf_lpf += (int32_t)((decoupling_wldid_bemf - decoupling_wldid_bemf_lpf)>>SPEED_LPF_DQ_DECOUPLING_COEFF);
    }
    #endif

    /* Read CORDIC result */
    BMCLIB_CartToPolarGetResult(&cart2pol_mag,&angle_q31);

    /**************************************************************************************************************/
    /* Parallel computation #4 - CPU(Flux PI) and CORDIC(Iq_max = sqrt(i_max^2 - id^2)) */
    /**************************************************************************************************************/
     /* CORDIC - Iq_max = sqrt(i_max^2 - id^2)*/
    if(PMSM_FOC_INPUT.ref_id != 0)
    {
        BMCLIB_HyperMag((int32_t)(PMSM_FOC_INPUT.limit_max_is << CORDIC_SHIFT),(int32_t)(PMSM_FOC_INPUT.ref_id << CORDIC_SHIFT));
    }

    /* CPU - Flux PI Controller */
    BMCLIB_PI_Controller(PMSM_FOC_INPUT.ref_id, PMSM_FOC_OUTPUT.flux_id, (int16_t)(-decoupling_wlqiq_lpf), &PMSM_FOC_FLUX_PI);

    hypermag_res = BMCLIB_HyperMagGetResult();

    /* Limit the Speed PI output limit(ref_iq) dynamically based upon ref_id */
    if(PMSM_FOC_INPUT.ref_id != 0)
    {
        PMSM_FOC_CTRL.limit_max_iq = (uint16_t)(hypermag_res >> CORDIC_SHIFT);
    }
    else
    {
        PMSM_FOC_CTRL.limit_max_iq = PMSM_FOC_INPUT.limit_max_is;
    }

    /**************************************************************************************************************/
    /* Parallel computation #5 - CPU(Torque PI) and CORDIC(Vq_max = sqrt(Vs_max^2 - Vd^2)) */
    /**************************************************************************************************************/
    /* CORDIC - Vq_max = sqrt(Vs_max^2 - Vd^2) */
    BMCLIB_HyperMag((int32_t)(PMSM_FOC_INPUT.limit_max_vref << CORDIC_SHIFT),(int32_t)(PMSM_FOC_OUTPUT.flux_vd  << CORDIC_SHIFT));

    /*====================== Max assist torque limit & Power LIMIT CHECK to user input ref iq ======================*/
    #if(MAX_ASSIST_TORQUE_LIMIT_EN_DIS == ENABLED)
    /* Update iq reference based upon user pedal input & the assist torque current limits */
    PMSM_FOC_INPUT.assist_limit_ref_iq = MIN(PMSM_FOC_INPUT.ref_iq_temp, PMSM_FOC_CTRL.ebike_assist_limit_iq);
    #endif

    #if(MAX_POWER_LIMIT_EN_DIS == ENABLED)
    /* Update iq reference based upon the previous compared iq reference & user input power max torque current limit */
    PMSM_FOC_INPUT.power_limit_ref_iq = MIN(PMSM_FOC_INPUT.ref_iq_temp, PMSM_FOC_CTRL.power_limit_iq);
    #endif
    /*============================================================================================================ */
    /* CPU - Update iq reference based upon user input & the current limits */
    //	PMSM_FOC_INPUT.ref_iq = MIN(PMSM_FOC_INPUT.ref_iq, PMSM_FOC_CTRL.limit_max_iq);
    PMSM_FOC_INPUT.ref_iq = MIN(PMSM_FOC_INPUT.ref_iq_temp, PMSM_FOC_CTRL.limit_max_iq);

    /*====================== Max assist torque limit & Power LIMIT CHECK ======================*/
    #if(MAX_ASSIST_TORQUE_LIMIT_EN_DIS == ENABLED)
    /* Update iq reference based upon the previous compared iq reference & assist torque current limit */
    PMSM_FOC_INPUT.ref_iq = MIN(PMSM_FOC_INPUT.ref_iq, PMSM_FOC_INPUT.assist_limit_ref_iq);
    #endif

    #if(MAX_POWER_LIMIT_EN_DIS == ENABLED)
    /* Update iq reference based upon the previous compared iq reference & user input power max torque current limit */
    PMSM_FOC_INPUT.ref_iq = MIN(PMSM_FOC_INPUT.ref_iq, PMSM_FOC_INPUT.power_limit_ref_iq);
    #endif
	/*============================================================================================================ */

    BMCLIB_PI_Controller(PMSM_FOC_INPUT.ref_iq, PMSM_FOC_OUTPUT.torque_iq, (decoupling_wldid_bemf_lpf), &PMSM_FOC_TORQUE_PI);

    /* Get CORDIC result */
    hypermag_res = BMCLIB_HyperMagGetResult();
    PMSM_FOC_CTRL.limit_max_vq = (uint16_t)(hypermag_res >> CORDIC_SHIFT);

    /* Limit Vq such that it will remain within maximum voltage limit(Vs_max) */
    PMSM_FOC_OUTPUT.torque_vq = BMCLIB_MAX_LIMIT(PMSM_FOC_TORQUE_PI.uk,PMSM_FOC_CTRL.limit_max_vq);

    /**************************************************************************************************************/
    /* Parallel computation #6 - CPU(PMSM_FOC_MC_PARA) and CORDIC(I_mag = sqrt(i_alpha^2 + i_beta^2)) */
    /**************************************************************************************************************/
    /* CORDIC - I_mag = sqrt(i_alpha^2 + i_beta^2) */
    BMCLIB_CartToPolar(PMSM_FOC_CLARKE_TRANSFORM.alpha,PMSM_FOC_CLARKE_TRANSFORM.beta,0);

    #if(MOTOR_CTRL_SCHEME == PMSM_HALL_FOC)
    #define TRAP_OPERATION  0
    #if(TRAP_OPERATION == 1)
    PMSM_FOC_MC_PARA.rotor_angle_q31 = hall_sensor_data.hall_angleQ31 + PMSM_FOC_ANGLE_030_DEGREE_Q31 + HALL_OFFSET_COMPENSATION_ANGLE_INIT;
	  #else
    if(hall_sensor_data.hall_valid_cnt < USER_HALL_EDGE_COUNT)
    {
    	PMSM_FOC_MC_PARA.rotor_angle_q31 = hall_sensor_data.hall_angleQ31 + PMSM_FOC_ANGLE_030_DEGREE_Q31; /*Center*/
    }
    else
    {
    	// Compute rotor angle using hall sensor
    	pmsm_foc_hall_rotor_angle();
    	PMSM_FOC_MC_PARA.rotor_angle_q31 = hall_sensor_data.rotor_angleQ31;
    }
    PMSM_FOC_MC_PARA.rotor_angle_q31 = PMSM_FOC_MC_PARA.rotor_angle_q31 + HALL_OFFSET_COMPENSATION_ANGLE_INIT;
    #endif
    #endif

    PMSM_FOC_OUTPUT.flux_vd = PMSM_FOC_FLUX_PI.uk;

    /* Get CORDIC result */
    BMCLIB_CartToPolarGetResult(&cart2pol_mag, &angle_q31);
    /**************************************************************************************************************/
    /* Parallel computation #7 - CPU(LIMIT_I_Mag) and CORDIC(cart2polar) */
    /**************************************************************************************************************/
    BMCLIB_CartToPolar((int32_t)(PMSM_FOC_OUTPUT.flux_vd << CORDIC_SHIFT),(int32_t)(PMSM_FOC_OUTPUT.torque_vq<<CORDIC_SHIFT), PMSM_FOC_MC_PARA.rotor_angle_q31);

    PMSM_FOC_OUTPUT.current_i_mag = (uint16_t)(cart2pol_mag >> CORDIC_SHIFT);
    PMSM_FOC_OUTPUT.current_i_mag_filtered += (uint16_t)((PMSM_FOC_OUTPUT.current_i_mag - PMSM_FOC_OUTPUT.current_i_mag_filtered) >> LPF_I_FILT_COEFF);

    /* Update PMSM_FOC_OUTPUT structure */
    BMCLIB_CartToPolarGetResult(&cart2pol_mag,&angle_q31);

    /**************************************************************************************************************/
    PMSM_FOC_OUTPUT.svm_vref_16 = (uint16_t)(cart2pol_mag >> CORDIC_SHIFT);
    PMSM_FOC_OUTPUT.svm_angle_16 = (uint16_t)(angle_q31 >> 16);
}
