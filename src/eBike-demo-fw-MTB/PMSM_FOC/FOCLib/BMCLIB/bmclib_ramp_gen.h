/**
 * @file bmclib_ramp_gen.h
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
#ifndef BMCLIB_RAMP_GEN_H_
#define BMCLIB_RAMP_GEN_H_
/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <xmc_common.h>

/*********************************************************************************************************************
 * DEFINES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
typedef struct BMCLIB_LINEAR_RAMP_GEN
{
    uint16_t ramp_up_step;               /*!< CONFIG - ramp up step for each cycle */
    uint16_t ramp_down_step;             /*!< CONFIG - ramp down step for each cycle */
    uint16_t ramp_cycle_skip_count;      /*!< CONFIG - ramp cycle skip count */
    uint16_t ramp_cycle_skip_counter;    /*!< Internal variable - ramp cycle skip counter  */
    uint8_t  ramp_up_enable;             /*!< CONFIG - ramp-up enable/disable CTRL */
    uint8_t  ramp_down_enable;           /*!< CONFIG - ramp-down enable/disable CTRL */
    uint8_t  ramp_status_flag;           /*!< Status flag to indicate ramp up/down/no ramp in present cycle.[0- no ramp in ,1-ramp down,2 -ramp up] */
}BMCLIB_LINEAR_RAMP_GEN_t;

/*********************************************************************************************************************
 * FUNCTION DECLARATION
 ********************************************************************************************************************/
/**
 * @param ref_set_val reference set value
 * @param set_val set value (pass by reference)
 * @param linear_ramp_gen_ptr BMCLIB_LINEAR_RAMP_GEN_t structure address (pass by reference)
 *
 * @return None<BR>
 *
 * \par Description:
 * To perform Linear Ramp Generation, based upon values in BMCLIB_LINEAR_RAMP_GEN_t <BR>\n
 *
 * \par Related APIs:
 * None.
 */
/* This function performs Linear Ramp Generation */
__STATIC_FORCEINLINE void BMCLIB_LinearRampGenerator(int32_t ref_set_val, int32_t * const set_val, BMCLIB_LINEAR_RAMP_GEN_t * const linear_ramp_gen_ptr)
{
    if (*set_val != ref_set_val)
    {
        linear_ramp_gen_ptr->ramp_cycle_skip_counter++;
        if(linear_ramp_gen_ptr->ramp_cycle_skip_counter >= linear_ramp_gen_ptr->ramp_cycle_skip_count)
        {
            linear_ramp_gen_ptr->ramp_cycle_skip_counter = 0;
            /* Ramp up if set value is less than reference set value */
            if(*set_val < ref_set_val)
            {
                *set_val += (int32_t)(linear_ramp_gen_ptr->ramp_up_step * linear_ramp_gen_ptr->ramp_up_enable);
                /* after ramp up if the set value is higher than the reference value then clamp it to reference value */
                if(*set_val > ref_set_val)
                {
                    *set_val = ref_set_val;
                }
                /* Set ramp flag to 2 to indicate ramp up is active */
                linear_ramp_gen_ptr->ramp_status_flag = 2U;
            }
            else
            {
                /* Ramp down if set value is higher than the reference set value */
                if(*set_val > ref_set_val)
                {
                    *set_val -= (int32_t)(linear_ramp_gen_ptr->ramp_down_step * linear_ramp_gen_ptr->ramp_down_enable);
                    /* after ramp down if the set value is lower than the reference value then clamp it to reference value */
                    if(*set_val < ref_set_val)
                    {
                        *set_val = ref_set_val;
                    }
                    /* Set ramp flag to 1 to indicate ramp down is active */
                    linear_ramp_gen_ptr->ramp_status_flag = 1U;
                }
            }
        }
    }
    else
    {
        /* No ramp is active */
        linear_ramp_gen_ptr->ramp_status_flag = 0U;
    }
}
#endif // BMCLIB_RAMP_GEN_H_
