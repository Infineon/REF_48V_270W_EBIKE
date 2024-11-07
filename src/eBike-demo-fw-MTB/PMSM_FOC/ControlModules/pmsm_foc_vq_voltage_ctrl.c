/**
 * @file pmsm_foc_vq_voltage_ctrl.c
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

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "pmsm_foc_vq_voltage_ctrl.h"
#include "pmsm_foc_functions.h"
#include "../FOCLib/BMCLIB/bmclib_limiters.h"

/*********************************************************************************************************************
 * MACRO
 ********************************************************************************************************************/
#define LPF_I_FILT_COEFF (8)     /* Low pass filter to remove high frequency current noise */
#define VQ_LPF_DQ_DECOUPLING_COEFF (4)  /* Low pass filtering for DQ decoupling components */
/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * FUNCTIONS
 ********************************************************************************************************************/
static int32_t decoupling_wlqiq_lpf = 0;

PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_VqVoltageCtrl(void)
{
    uint32_t cart2pol_mag;
    int32_t angle_q31;
    int32_t hypermag_res;
    int32_t id_temp;
    int32_t iq_temp; 
    int16_t id_fw = 0;
    int16_t id_mtpa = 0;

    /* Reads phase current from VADC,Removes the offset and convert as per scaling system */
    PMSM_FOC_CurrentReconstruction(&ADC, &PMSM_FOC_SVPWM, &PMSM_FOC_CTRL, &PMSM_FOC_INPUT);

    /* Clarke Transform */
    BMCLIB_ClarkeTransform(PMSM_FOC_INPUT.i_u,PMSM_FOC_INPUT.i_v,PMSM_FOC_INPUT.i_w,&PMSM_FOC_CLARKE_TRANSFORM);

	/* Vq voltage control */
#if(DEMO_SETUP == ENABLED)
    if(ADC.adc_res_pot < 2048)
    {
    	PMSM_FOC_INPUT.ref_vq = 0;
    }
    else
    {
    	PMSM_FOC_INPUT.ref_vq = VQ_REF_FIXED_VALUE;
    }
#endif

	PMSM_FOC_TORQUE_PI.uk = PMSM_FOC_INPUT.ref_vq;

    /**************************************************************************************************************/
    /* Parallel computation #1 - CPU(Flux Weakening) and CORDIC(PARK Transform) */
    /**************************************************************************************************************/
	/* CORDIC - Park Transform */
	BMCLIB_ParkTransform(PMSM_FOC_CLARKE_TRANSFORM.alpha, PMSM_FOC_CLARKE_TRANSFORM.beta,PMSM_FOC_MC_PARA.rotor_angle_q31);

	/* CPU - Filtered iq current component */
	PMSM_FOC_OUTPUT.iq_filtered += (int16_t)((PMSM_FOC_OUTPUT.torque_iq - PMSM_FOC_OUTPUT.iq_filtered)>> LPF_I_FILT_COEFF);

	PMSM_FOC_INPUT.ref_id = id_fw + id_mtpa;

	/* CORDIC - Park Transform Get Result */
	BMCLIB_ParkTransformGetResult(&iq_temp, &id_temp);

	/**************************************************************************************************************/
	/* Update PMSM_FOC_OUTPUT structure */
	PMSM_FOC_OUTPUT.flux_id = (int16_t)(id_temp >> CORDIC_SHIFT);
	PMSM_FOC_OUTPUT.torque_iq = (int16_t)(iq_temp >> CORDIC_SHIFT);

  /* For constant Vq reference DQ decoupling is done only for the Flux PI current(Id) control output. */
   #if(USER_DQ_DECOUPLING == ENABLED)
   int32_t decoupling_wlqiq = 0;

   if(1) /*TODO: conditions to be changed later*/
   {
	  decoupling_wlqiq = (int32_t)(((int32_t)((PMSM_FOC_MC_PARA.rotor_speed * PMSM_FOC_OUTPUT.torque_iq ) >> 15) * PMSM_FOC_MC_PARA.phase_inductance_lq) >> PMSM_FOC_MC_PARA.phase_inductance_scale);
	  decoupling_wlqiq = MIN_MAX_LIMIT(decoupling_wlqiq, 32767, -32767);
   }
   decoupling_wlqiq_lpf += (int32_t)((decoupling_wlqiq - decoupling_wlqiq_lpf)>>VQ_LPF_DQ_DECOUPLING_COEFF);
   #endif //End of #if(USER_DQ_DECOUPLING == ENABLED)

   BMCLIB_PI_Controller(PMSM_FOC_INPUT.ref_id, PMSM_FOC_OUTPUT.flux_id, (-decoupling_wlqiq_lpf), &PMSM_FOC_FLUX_PI);
   PMSM_FOC_OUTPUT.flux_vd = PMSM_FOC_FLUX_PI.uk;

    /**************************************************************************************************************/
    /* Parallel computation #2 - CPU(Iq Filter ) and CORDIC(Vq_max = sqrt(Vs_max^2 - Vd^2)) */
    /**************************************************************************************************************/
    /* CORDIC - Vq_max = sqrt(Vs_max^2 - Vd^2) */
    BMCLIB_HyperMag((int32_t)(PMSM_FOC_INPUT.limit_max_vref << CORDIC_SHIFT),(int32_t)(PMSM_FOC_OUTPUT.flux_vd << CORDIC_SHIFT));

    /* CPU - Filtered iq current component */
    PMSM_FOC_OUTPUT.iq_filtered += (int16_t)((PMSM_FOC_OUTPUT.torque_iq - PMSM_FOC_OUTPUT.iq_filtered)>> LPF_I_FILT_COEFF);
        
    /* Get CORDIC result */
    hypermag_res = BMCLIB_HyperMagGetResult();
    /**************************************************************************************************************/
    PMSM_FOC_CTRL.limit_max_vq = (uint16_t)(hypermag_res >> CORDIC_SHIFT);

    /* Limit Vq such that it will remain within maximum voltage limit(Vs_max) */
    PMSM_FOC_OUTPUT.torque_vq = BMCLIB_MAX_LIMIT(PMSM_FOC_TORQUE_PI.uk,PMSM_FOC_CTRL.limit_max_vq);

    /**************************************************************************************************************/
    /* Parallel computation #3 - CPU(PMSM_FOC_MC_PARA) and CORDIC(I_mag = sqrt(i_d^2 + i_q^2)) */
    /**************************************************************************************************************/
    /* CORDIC - I_mag = sqrt(i_d^2 + i_q^2) */
    BMCLIB_CartToPolar(PMSM_FOC_CLARKE_TRANSFORM.alpha,PMSM_FOC_CLARKE_TRANSFORM.beta,0);

    //AMCLIB_PLL_GetPosSpeed(PMSM_FOC_OUTPUT.torque_iq,PMSM_FOC_OUTPUT.flux_id,PMSM_FOC_INPUT.ref_id,PMSM_FOC_OUTPUT.flux_vd);

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
    //PMSM_FOC_MC_PARA.rotor_angle_q31 = PMSM_FOC_MC_PARA.rotor_angle_q31 + HALL_OFFSET_COMPENSATION_ANGLE_INIT;
    #endif
    #endif

    /* Get CORDIC result */
    BMCLIB_CartToPolarGetResult(&cart2pol_mag,&angle_q31);
    /**************************************************************************************************************/
    /* Parallel computation #4 - CPU(Imag Filter,PLL check) and CORDIC(cart2pol_mag = sqrt(v_d^2 + v_q^2)) */
    /**************************************************************************************************************/
    BMCLIB_CartToPolar((int32_t)(PMSM_FOC_OUTPUT.flux_vd<<CORDIC_SHIFT),(int32_t)(PMSM_FOC_OUTPUT.torque_vq<<CORDIC_SHIFT),PMSM_FOC_MC_PARA.rotor_angle_q31);
    PMSM_FOC_OUTPUT.current_i_mag = (uint16_t)(cart2pol_mag >> CORDIC_SHIFT);
    PMSM_FOC_OUTPUT.current_i_mag_filtered += (uint16_t)((PMSM_FOC_OUTPUT.current_i_mag - PMSM_FOC_OUTPUT.current_i_mag_filtered) >> LPF_I_FILT_COEFF);

    /* Update PMSM_FOC_OUTPUT structure */
    BMCLIB_CartToPolarGetResult(&cart2pol_mag,&angle_q31);

    #if(TRAP_OPERATION == 1)
    angle_q31 = PMSM_FOC_MC_PARA.rotor_angle_q31 + PMSM_FOC_ANGLE_090_DEGREE_Q31;
    PMSM_FOC_OUTPUT.svm_vref_16 = PMSM_FOC_INPUT.ref_vq;
    #else
    /**************************************************************************************************************/
    PMSM_FOC_OUTPUT.svm_vref_16 = (uint16_t)(cart2pol_mag >> CORDIC_SHIFT);
    #endif

    PMSM_FOC_OUTPUT.svm_angle_16 = (uint16_t)(angle_q31 >> 16);

}
