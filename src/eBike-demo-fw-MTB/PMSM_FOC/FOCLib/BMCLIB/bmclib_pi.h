/**
 * @file bmclib_pi.h
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
#ifndef BMCLIB_PI_H_
#define BMCLIB_PI_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#ifndef MIN_MAX_LIMIT                                    /*!< Limits the input as per limits */
#define MIN_MAX_LIMIT(Input,LimitH,LimitL) (((Input) > (LimitH)) ? (LimitH) : (((Input) < (LimitL))? (LimitL): (Input)))
#endif

#ifndef MIN
#define MIN(a, b)          (((a) < (b)) ? (a) : (b))   /*!< macro returning smallest input */
#endif

#ifndef MAX
#define MAX(a, b)          (((a) > (b)) ? (a) : (b))   /*!< macro returning biggest input */
#endif

#define ABS(a)             (((a)< 0) ? (-(a)) : (a))    /*!< returns the absolute number */

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
typedef struct BMCLIB_PI_CTRL
{
    int32_t error;                              /*!< PI error signal (reference value � feedback value), error[k] */
    int32_t uk;                                 /*!< PI output U[k] */
    int32_t ik;                                 /*!< Integral result I[k] */
    int32_t ik_limit_min;                       /*!< Integral buffer limit  - minimum */
    int32_t ik_limit_max;                       /*!< Integral buffer limit  - maximum */
    int32_t uk_limit_min;                       /*!< PI output limit  - minimum */
    int32_t uk_limit_max;                       /*!< PI output limit  - maximum */
    int32_t uk_limit_max_scaled;                /*!< Internal variable - PI output limit scaled - maximum */
    int32_t uk_limit_min_scaled;                /*!< Internal variable - PI output limit scaled - minimum */
    int16_t kp;                                 /*!< Proportional gain Kp */
    int16_t ki;                                 /*!< Integral gain Ki */
    int16_t scale_kpki;                         /*!< Scale-up Kp,ki by 2^Scale_Kpki */
    int16_t enable_antiwindup;                  /*!< Antiwindup enable/disable control */
} BMCLIB_PI_CTRL_t;

/**********************************************************************************************************************
* EXTERN
**********************************************************************************************************************/

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/

/* Updates the PI control output limits */
/**
 * @brief Updates the PI control output limits
 *
 * @param
 *
 * @retval
 */

/*********************************************************************************************************************
 * API IMPLEMENTATIONS
 ********************************************************************************************************************/
/**
 * @brief PI controller with anti-windup and dynamic clampling and feed forward.
 *      U(t)=Kp x e(t) + (Ki/Ts) x ∫e(t)dt, where Ts is sampling period, e.g.: Ts = 50us.
 *      I[k] = I[k-1] + Ki * error[k]
 *      U[k] = Kp * error[k] + I[k]
 *
 * @param reference
 * @param feedback
 * @param feedforward
 * @param *PI controller (pass by reference)
 *
 * @retval None
 */
__STATIC_FORCEINLINE void BMCLIB_PI_Controller(int32_t reference, int32_t feedback, int16_t feedforward, \
                                                                         BMCLIB_PI_CTRL_t *pi_handle_ptr)
{
    int32_t temp_uk;
    int32_t temp_kp_ff_error;

    /* Error = Reference - Feedback */
    pi_handle_ptr->error = MIN_MAX_LIMIT((reference - feedback),32767,-32767);

    temp_kp_ff_error = (int32_t)(pi_handle_ptr->kp * pi_handle_ptr->error);
    temp_kp_ff_error += (int32_t)(feedforward<< pi_handle_ptr->scale_kpki);

    /* Integral output I[k] = I[k-1] + Ki * error[k] */
    pi_handle_ptr->ik += (int32_t)(pi_handle_ptr->ki * pi_handle_ptr->error);

    /* Check if antiwindup feature is enabled */
    if(pi_handle_ptr->enable_antiwindup == 1)
    {
        /* Dynamic error integral limit */
        pi_handle_ptr->ik_limit_max = (MAX((pi_handle_ptr->uk_limit_max_scaled - temp_kp_ff_error), 0));
        pi_handle_ptr->ik_limit_min = (MIN((pi_handle_ptr->uk_limit_min_scaled - temp_kp_ff_error), 0));
    }

    /* Limit the integral buffer as per dynamic error integral limit  */
    pi_handle_ptr->ik = MIN_MAX_LIMIT(pi_handle_ptr->ik, pi_handle_ptr->ik_limit_max, pi_handle_ptr->ik_limit_min);

    /* PI output U[k] = Kp * error[k] + FF + I[k] */
    temp_uk = (int32_t)((temp_kp_ff_error + pi_handle_ptr->ik) >> pi_handle_ptr->scale_kpki);

    /* Check U[k] output limit */
    pi_handle_ptr->uk = MIN_MAX_LIMIT(temp_uk, pi_handle_ptr->uk_limit_max, pi_handle_ptr->uk_limit_min);
}

#endif /* BMCLIB_PI_H_ */
