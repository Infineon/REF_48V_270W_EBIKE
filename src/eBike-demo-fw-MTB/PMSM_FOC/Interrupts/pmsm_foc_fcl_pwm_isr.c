/**
 * @file pmsm_foc_fcl_pwm_isr.c
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

#include "../ControlModules/pmsm_foc_error_handling.h"
#include "../ControlModules/pmsm_foc_functions.h"
#include "../ControlModules/pmsm_foc_interface.h"
#include "../ControlModules/pmsm_foc_speed_current_ctrl.h"
#include "../ControlModules/pmsm_foc_state_machine.h"
#include "../Configuration/pmsm_foc_mcuhw_params.h"

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup Interrupts
 * @{
 */

/*********************************************************************************************************************
 * EXTERN
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * API DEFINATION
 ********************************************************************************************************************/
/**
 * @brief Fast Control Loop(FCL)  - Interrupt occurs every PWM period
 *
 * @param None
 *
 * @retval None
 */
volatile int32_t iu,iv,iw;
volatile int32_t ialpha,ibeta;
volatile int32_t counter;
void PMSM_FOC_FCL_ISR(void)
{
    /* Trigger scan for the VADC measurement of non critical measurements */
    XMC_VADC_GROUP_ScanTriggerConversion(VADC_G0);
    XMC_VADC_GROUP_ScanTriggerConversion(VADC_G1);

    /* Rotor speed estimation */
    pmsm_foc_hall_rotor_speed();

    /* Motor control state machine */
    PMSM_FOC_MSM();

    iu = PMSM_FOC_INPUT.i_u;
    iv = PMSM_FOC_INPUT.i_v;
    iw = PMSM_FOC_INPUT.i_w;

    ialpha = PMSM_FOC_CLARKE_TRANSFORM.alpha;
    ibeta = PMSM_FOC_CLARKE_TRANSFORM.beta;

    /* Debug purpose only */
    #if ((DEBUG_PWM_0_ENABLE == ENABLED) || (DEBUG_PWM_1_ENABLE == ENABLED))
//    PMSM_FOC_CCU4_Debug3output(((int16_t)PMSM_FOC_MC_PARA.rotor_angle_q31 >> 16),1,15, ((int16_t)(PMSM_FOC_MC_PARA.rotor_angle_q31 >> 16)),1,15);		//16, 32, 31, 15
//    PMSM_FOC_CCU4_Debug3output(((int16_t)PMSM_FOC_MC_PARA.rotor_angle_q31 >> 16),1,15, ((int16_t)(PMSM_FOC_SVPWM.current_sector_num)),0,3);        //16, 32, 31, 15
//    PMSM_FOC_CCU4_Debug3output(((int16_t)PMSM_FOC_MC_PARA.rotor_angle_q31 >> 16),1,15, ((int16_t)(hall_sensor_data.cur_hall_pos)),0,3);        //16, 32, 31, 15
//    PMSM_FOC_CCU4_Debug3output(((int16_t)PMSM_FOC_MC_PARA.rotor_angle_q31 >> 16),1,15, ((int16_t)(PMSM_FOC_INPUT.i_w)),1,15);        //16, 32, 31, 15
    PMSM_FOC_CCU4_Debug3output(((int16_t)PMSM_FOC_MC_PARA.rotor_angle_q31 >> 16),1,15, ((int16_t)(PMSM_FOC_OUTPUT.svm_angle_16)),1,16);        //16, 32, 31, 15
    #endif

    /* System monitoring to detect errors, execution time ~1.4uSec */
    PMSM_FOC_SysMonitoring();

    /* ucProbe sampling function for ucProbe oscilloscope feature */
    #if (USER_UCPROBE_OSCILLOSCOPE == ENABLED)
    ProbeScope_Sampling();
    #endif

    PMSM_FOC_MiscWorks();

}

/**
 * @}
 */

/**
 * @}
 */
