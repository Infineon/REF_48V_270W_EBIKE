/**
 * @file pmsm_foc_current_sense_3S.h
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
#ifndef PMSM_FOC_CURRENT_SENSE_3S_H_
#define PMSM_FOC_CURRENT_SENSE_3S_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "../ControlModules/pmsm_foc_functions.h"
#include "../Configuration/pmsm_foc_mcuhw_params.h"

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup MIDSys
 * @{
 */
static uint32_t phase_offset_calib_counter = 0;

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * EXTERN
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
__STATIC_FORCEINLINE void pmsm_foc_get_current_bias(void)
{
	if(phase_offset_calib_counter == 0)
	{
		PMSM_FOC_VADC_PhCurrentInit();

	} else if(phase_offset_calib_counter < 21)
	{

		/* Capture Current sense amplifier offset */
#if(USER_MOTOR_PH_OFFSET_CALIBRATION == ENABLED)
		/* Read phase currents */
#if (USER_CURRENT_SENSING == THREE_SHUNT_SYNC_CONV)
		ADC.adc_res_iw = XMC_VADC_GROUP_GetResult(VADC_I1_GROUP,VADC_I1_RESULT_REG);
		ADC.adc_res_iv = XMC_VADC_GROUP_GetResult(VADC_I2_GROUP,VADC_I2_RESULT_REG);
		ADC.adc_res_iu = XMC_VADC_GROUP_GetResult(VADC_I3_GROUP,VADC_I3_RESULT_REG);
#else
		ADC.adc_res_iu = XMC_VADC_GROUP_GetResult(VADC_IU_GROUP,VADC_IU_RESULT_REG);
		ADC.adc_res_iv = XMC_VADC_GROUP_GetResult(VADC_IV_GROUP,VADC_IV_RESULT_REG);
		ADC.adc_res_iw = XMC_VADC_GROUP_GetResult(VADC_IW_GROUP,VADC_IW_RESULT_REG);
#endif // End of #if (USER_CURRENT_SENSING == THREE_SHUNT_SYNC_CONV)

		/* Read Iu ADC bias */
		ADC.adc_bias_iu += (int32_t)((ADC.adc_res_iu - ADC.adc_bias_iu)>> SHIFT_BIAS_LPF);
		/* Read Iv ADC bias */
		ADC.adc_bias_iv += (int32_t)((ADC.adc_res_iv - ADC.adc_bias_iv)>> SHIFT_BIAS_LPF);
		/* Read Iw ADC bias */
		ADC.adc_bias_iw += (int32_t)((ADC.adc_res_iw - ADC.adc_bias_iw)>> SHIFT_BIAS_LPF);

#endif
	}
	phase_offset_calib_counter++;
}

/**
 * @param Previous_SVM_SectorNo previous SVM sector number \n
 * @param New_SVM_SectorNo      next SVM sector number \n
 * @param  HandlePtr pointer to an object of ADC Current.\n
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Read ADC result of 3-phase motor current <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
#if (USER_CURRENT_SENSING ==  THREE_SHUNT_SYNC_CONV)
/* API to read ADC result of the 3 shunt current */
__STATIC_FORCEINLINE void PMSM_FOC_VADC_GetPhasecurrent(uint16_t previous_svm_sector_num,
    uint16_t new_svm_sector_num, ADC_t * const foc_input_ptr)
{
    uint16_t I1;
    uint16_t I2;
    uint16_t I3;

    /* Read current ADC (ADC synchronous conversion) */
    I1 = XMC_VADC_GROUP_GetResult(VADC_I1_GROUP,VADC_I1_RESULT_REG);
    I2 = XMC_VADC_GROUP_GetResult(VADC_I2_GROUP,VADC_I2_RESULT_REG);
    I3 = XMC_VADC_GROUP_GetResult(VADC_I3_GROUP,VADC_I3_RESULT_REG);

    #if((USER_ADC_CALIBRATION == ENABLED) && (UC_SERIES == XMC13))
    /* Clear offset calibration values*/
    CLEAR_OFFSET_CALIB_VALUES;
    #endif

    /* 3-phase current reconstruction */
    switch (previous_svm_sector_num)
    {
        case 0:
        case 5:
          /* Sectors A and F. ADC sequences - Iw -> Iv -> Iu */
          foc_input_ptr->adc_res_iw = I2;
          foc_input_ptr->adc_res_iv = I1;
          foc_input_ptr->adc_res_iu = I3;
          break;

        case 1:
        case 2:
            /* Sectors B and C. ADC sequences - Iw -> Iu -> Iv */
            foc_input_ptr->adc_res_iu = I1;
            foc_input_ptr->adc_res_iv = I3;
            foc_input_ptr->adc_res_iw = I2;

            #if (USER_MOTOR_BI_DIRECTION_CTRL == ENABLED )
            /* May need swap SVPWM CCU8 duty cycles of phase V and W, so that motor reverses */
            if (PMSM_FOC_CTRL.rotation_dir == DIRECTION_DEC)
            {
                foc_input_ptr->adc_res_iv = I2;
                foc_input_ptr->adc_res_iw = I3;
            }
            #endif
            break;

        default:
            /* Process for all other cases, Sectors D and E. ADC sequences - Iu -> Iv -> Iw */
            foc_input_ptr->adc_res_iu = I1;
            foc_input_ptr->adc_res_iv = I2;
            foc_input_ptr->adc_res_iw = I3;

            #if (USER_MOTOR_BI_DIRECTION_CTRL == ENABLED )
            /* May need swap SVPWM CCU8 duty cycles of phase V and W, so that motor reverses */
            if (PMSM_FOC_CTRL.rotation_dir == DIRECTION_DEC)
            {
                foc_input_ptr->adc_res_iv = I3;
                foc_input_ptr->adc_res_iw = I2;
            }
            #endif
            break;
    }

    /* If SVM sector changed update the synchronous current sensing configuration as per the SVPWM */
    if (new_svm_sector_num != previous_svm_sector_num)
    {
        PMSM_FOC_VADC_PhCurrentSyncConfig(new_svm_sector_num);
    }
}
#else
__STATIC_FORCEINLINE void PMSM_FOC_VADC_GetPhasecurrent(uint16_t previous_svm_sector_num, uint16_t new_svm_sector_num, ADC_t* const foc_input_ptr)
{
    foc_input_ptr->adc_res_iu = XMC_VADC_GROUP_GetResult(VADC_IU_GROUP,VADC_IU_RESULT_REG);
    foc_input_ptr->adc_res_iv = XMC_VADC_GROUP_GetResult(VADC_IV_GROUP,VADC_IV_RESULT_REG);
    foc_input_ptr->adc_res_iw = XMC_VADC_GROUP_GetResult(VADC_IW_GROUP,VADC_IW_RESULT_REG);
}
#endif

/**
 * @brief 3-Shunt 3-Phase Current Reconstruction, ADC values are from last PWM cycle
 *    ADCs of 2or3-Shunt are triggered by CCU83 CR1S
 *    It also considers the motor direction to swap the currents as per the CCU8 swap.
 *
 * @param vadc_res_iu
 *    vadc_res_iv
 *    vadc_res_iw
 *
 *@retval   *PMSM_FOC_INPUT_t
 */
__STATIC_FORCEINLINE void PMSM_FOC_CurrentReconstruction(ADC_t * const adc_ptr, BMCLIB_SVPWM_t * const svpwm_ptr,
                                                               PMSM_FOC_CTRL_t * const foc_ctrl_ptr, PMSM_FOC_INPUT_t * const foc_input_ptr)
{
    /* Read three phase currents */
    PMSM_FOC_VADC_GetPhasecurrent(svpwm_ptr->previous_sector_num, svpwm_ptr->current_sector_num, adc_ptr);

#if(INVERTERCARD_TYPE == LEV_4KW_INVERTER)

    /* Motor phase current, Iu, Iv, Iw*/
    if (foc_ctrl_ptr->rotation_dir == DIRECTION_INC)
    {
    	foc_input_ptr->i_v = (int16_t)((adc_ptr->adc_res_iv - adc_ptr->adc_bias_iv) * ADC_I15_RATIO);
    	foc_input_ptr->i_w = (int16_t)((adc_ptr->adc_res_iw - adc_ptr->adc_bias_iw) * ADC_I15_RATIO);

    	foc_input_ptr->i_u = -foc_input_ptr->i_w - foc_input_ptr->i_v;  /* TJ Calculating I_W (I_W + I_V + I_U = 0)*/
    }
    else
    {
    	foc_input_ptr->i_v = (int16_t)((adc_ptr->adc_res_iw - adc_ptr->adc_bias_iw) * ADC_I15_RATIO);
    	foc_input_ptr->i_w = (int16_t)((adc_ptr->adc_res_iv - adc_ptr->adc_bias_iv) * ADC_I15_RATIO);

    	foc_input_ptr->i_u = -foc_input_ptr->i_w - foc_input_ptr->i_v;
    }
#else
    /* Motor phase current, Iu, Iv, Iw*/
    foc_input_ptr->i_u = (int16_t)((adc_ptr->adc_bias_iu - adc_ptr->adc_res_iu) << 4);

    if (foc_ctrl_ptr->rotation_dir == DIRECTION_INC)
    {
    	foc_input_ptr->i_v = (int16_t)((adc_ptr->adc_bias_iv - adc_ptr->adc_res_iv) << 4);
    	foc_input_ptr->i_w = (int16_t)((adc_ptr->adc_bias_iw - adc_ptr->adc_res_iw) << 4);
    }
    else
    {
    	/* Swap the  V &  W phase currents */
    	foc_input_ptr->i_v = (int16_t)((adc_ptr->adc_bias_iw - adc_ptr->adc_res_iw) << 4);
    	foc_input_ptr->i_w = (int16_t)((adc_ptr->adc_bias_iv - adc_ptr->adc_res_iv) << 4);
    }
#endif

}


/**
 * @}
 */

/**
 * @}
 */

#endif /* PMSM_FOC_CURRENT_SENSE_3S_H_ */
