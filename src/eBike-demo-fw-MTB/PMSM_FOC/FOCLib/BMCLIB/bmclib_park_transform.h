/**
 * @file bmclib_park_transform.h
 * @date 09 May, 2019
 *
 * @cond
 *********************************************************************************************************************
 * BMCLIB Motor Control Library
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
#ifndef BMCLIB_PARK_TRANSFORM_H_
#define BMCLIB_PARK_TRANSFORM_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#define PARK_CORDIC_CIRCULAR_ROTATION_MODE       (0x6A)          /*!< CORDIC: Circular Rotation Mode. MPS: Divide by 2 (default). */
#define PARK_IS_CORDIC_BUSY                      (MATH->STATC & 0x01) /*!< Returns 1 if CORDIC is busy */
#define PARK_CORDIC_MPS_BY_K_SCALE               (8)             /*!< CORDIC MPS/K scaling factor */
#define PARK_CORDIC_CIRCULAR_MPS_BY_K_SCALED     (311)           /*!< CORDIC MPS/K ->(2/1.64676)* 2^CORDIC_MPS_BY_K_SCALE */

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************************************************/

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
 * @brief CORDIC - Park Transform - Transforms orthogonal stationary system into orthogonal rotating reference frame.
 * Iq = K[I_Beta cos(φ)-I_Alpha sin(φ)]/MPS   * Iq = Xfinal = K[X cos(Z) - Y sin(Z)] / MPS, where K = 1.646760258121.
 * Id = K[I_Alpha cos(φ)+I_Beta sin(φ)]/MPS   * Id = Yfinal = K[Y cos(Z) + X sin(Z)] / MPS      (Zfinal = 0).
 *
 * @param alpha
 * @param beta
 * @param rotor_angle_q31
 *
 *@retval None
 *
 */
__STATIC_FORCEINLINE void BMCLIB_ParkTransform(int32_t alpha, int32_t beta, int32_t angle)
{
    /* General control of CORDIC Control Register */
    MATH->CON = PARK_CORDIC_CIRCULAR_ROTATION_MODE;
    /* Z = φ, Hall rotor angle, or estimated rotor angle of last PWM cycle from PLL */
    MATH->CORDZ = angle;
    /* Y = I_Alpha */
    MATH->CORDY = alpha;
    /* X = I_Beta. Input CORDX data, and auto start of CORDIC calculation (~62 kernel clock cycles) */
    MATH->CORDX = beta;
}

/**
 * @brief Get CORDIC Result from Park Transform
 *
 * @param *qs
 * @param *ds
 *
 * @retval None
 */
__STATIC_FORCEINLINE void BMCLIB_ParkTransformGetResult(int32_t * const qs, int32_t * const ds)
{
    int32_t qs_temp;
    int32_t ds_temp;

    /* Wait if CORDIC is still running calculation */
    while (PARK_IS_CORDIC_BUSY);

    /* Read CORDIC results qs and ds - 32-bit. */
    qs_temp = MATH->CORRX;
    ds_temp = MATH->CORRY;

    /* x MPS/K */
    *qs = (int32_t)(((int32_t)(qs_temp >> PARK_CORDIC_MPS_BY_K_SCALE)) * PARK_CORDIC_CIRCULAR_MPS_BY_K_SCALED);
    *ds = (int32_t)(((int32_t)(ds_temp >> PARK_CORDIC_MPS_BY_K_SCALE)) * PARK_CORDIC_CIRCULAR_MPS_BY_K_SCALED);
}

#endif /* BMCLIB_PARK_TRANSFORM_H_ */
