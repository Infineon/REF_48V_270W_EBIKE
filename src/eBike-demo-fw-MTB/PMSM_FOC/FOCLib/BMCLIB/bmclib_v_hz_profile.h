/**
 * @file bmclib_v_hz_profile.h
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
#ifndef BMCLIB_V_HZ_PROFILE_H_
#define BMCLIB_V_HZ_PROFILE_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
typedef enum V_HZ_PROFILE_GEN_STATUS
{
    V_HZ_PROFILE_GEN_STATUS_COMPLETED,
    V_HZ_PROFILE_GEN_STATUS_IN_PROGRESS
} V_HZ_PROFILE_GEN_STATUS_t;

/**  @brief V/F open loop */
typedef struct
{
    V_HZ_PROFILE_GEN_STATUS_t status;                /*!< V/f open loop status - In progress or Completed */
    uint32_t vf_speed_to_angle_conv_factor;          /*!< Speed to angle conversion factor */ 
    int32_t vref_angle;                              /*!<  V/F open loop angle */
    uint16_t vf_offset;                              /*!<  V/F open loop offset */
    uint16_t vf_constant;                            /*!<  V/F constant */
    uint16_t vf_speed_ramp_up_rate;                  /*!<  V/F speed ramp up rate */
    uint16_t vf_speed_ramp_counter;                  /*!<  ramp counter to track ramp up rate */
    uint16_t vf_transition_speed;                    /*!<  V/F transition to close loop speed */
    uint16_t vf_motor_speed;                         /*!<  V/F open loop motor speed */
    uint16_t vref_mag;                               /*!<  V/F open loop voltage amplitude for SVPWM */
    uint16_t stablization_count;                     /*!< Time before jump to close loop */
    uint16_t stablization_counter;                   /*!< Counter to track time before jump to close loop */
    uint16_t vf_speed_to_angle_conv_factor_scale;    /*!< Speed to angle conversion factor scale */
    uint8_t dont_exit_open_loop_flag;                /*!< If flag set to 1 ctrl will not go to close loop - Useful for debugging only */ 
    
} BMCLIB_VF_OPEN_LOOP_t;

/**********************************************************************************************************************
* EXTERN
**********************************************************************************************************************/

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * API IMPLEMENTATIONS
 ********************************************************************************************************************/
/**
 * @brief V/F open loop voltage & angle generator.
 *
 * @param *vf_open_loop_ptr (pass by reference)
 *
 * @retval None
 */
__STATIC_FORCEINLINE void BMCLIB_VHzProfileGen(BMCLIB_VF_OPEN_LOOP_t * vf_open_loop_ptr)
{
    /* Update V/f voltage amplitude, Vref = Offset + Kω. */
    vf_open_loop_ptr->vref_mag = vf_open_loop_ptr->vf_offset + (vf_open_loop_ptr->vf_constant * vf_open_loop_ptr->vf_motor_speed);

    /* Limit vref */
    if (vf_open_loop_ptr->vref_mag > SVPWM_MAX_VREF)
    {
        /*  Limit |Vref| maximum value.*/
        vf_open_loop_ptr->vref_mag = SVPWM_MAX_VREF;
    }

    /* θ[k] = θ[k-1] + ω[k]. */
    vf_open_loop_ptr->vref_angle += (int32_t)((vf_open_loop_ptr->vf_motor_speed * vf_open_loop_ptr->vf_speed_to_angle_conv_factor)>>vf_open_loop_ptr->vf_speed_to_angle_conv_factor_scale); /* θ[k] = θ[k-1] + ω[k]. */

    if (vf_open_loop_ptr->status == V_HZ_PROFILE_GEN_STATUS_IN_PROGRESS)
    {
        /* check if motor speed is reached to transition speed */
        if (vf_open_loop_ptr->vf_motor_speed < vf_open_loop_ptr->vf_transition_speed)
        {
            /* Speed ramp counter ++. */
            vf_open_loop_ptr->vf_speed_ramp_counter++;

            if (vf_open_loop_ptr->vf_speed_ramp_counter > vf_open_loop_ptr->vf_speed_ramp_up_rate)
            {
                /* Increment motor speed. */
                vf_open_loop_ptr->vf_motor_speed++;

                /* Clear ramp counter.*/
                vf_open_loop_ptr->vf_speed_ramp_counter = 0;
            }
        }
        else
        {
            if (vf_open_loop_ptr->dont_exit_open_loop_flag == FALSE)
            {
                /* motor run at V/f constant speed for a while.*/
                vf_open_loop_ptr->stablization_counter++;

                if (vf_open_loop_ptr->stablization_counter > vf_open_loop_ptr->stablization_count)
                {
                    /* Change flag: V/f ramp-up completed and motor is in stable state */
                    vf_open_loop_ptr->status = V_HZ_PROFILE_GEN_STATUS_COMPLETED;
                }
            }
        }
    }
}

#endif /* BMCLIB_V_HZ_PROFILE_H_ */
