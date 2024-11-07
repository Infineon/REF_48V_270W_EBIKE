/**
 * @file bmclib_cart2pol.h
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
#ifndef BMCLIB_CART2POL_H_
#define BMCLIB_CART2POL_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <xmc_common.h>

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#define CART2POL_IS_CORDIC_BUSY                      (MATH->STATC & 0x01) /*!< Returns 1 if CORDIC is busy */
#define CART2POL_CORDIC_CIRCULAR_VECTORING_MODE      (0x62)          /*!< CORDIC: Circular Vectoring Mode (default). MPS: Divide by 2 (default). */
#define CART2POL_CORDIC_CIRCULAR_MPS_BY_K_SCALED     (311)           /*!< CORDIC MPS/K ->(2/1.64676)* 2^CORDIC_MPS_BY_K_SCALE */
#define CART2POL_CORDIC_MPS_BY_K_SCALE               (8)             /*!< CORDIC MPS/K scaling factor */
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
 * @brief Converts Cartesian Coordinates to Polar Coordinates.If angle is non zero value then it considers as an offset.
 * @param x       Input X co-ordinate value - MSB aligned 24 bit. Signed -> [-2^23,(2^23-1)],Unsigned ->[0,(2^24)]
 * @param y       Input Y co-ordinate value - MSB aligned 24 bit. Signed -> [-2^23,(2^23-1)],Unsigned ->[0,(2^24)]
 * @param angle   Input angle if any offset required - MSB aligned 24 bit
 * @return   None <BR>
 *
 * \par Description
 * This function calculates the magnitude of resultant vector in circular mode.\n
 * Magnitude  = SQRT(x*x+y*y); where k = 1.646760258121 \n
 * Angle      = atan(y/x) + angle_offset;
 *
 * \par Note
 * On XMC14 device this API will use HW CORDIC module for computation of resultant magnitude.\n
 */
__STATIC_FORCEINLINE void BMCLIB_CartToPolar(int32_t x, int32_t y, int32_t angle_offset)
{
    /*Clearing previous values of CORDX,CORDY and CORDZ Registers*/
    MATH->STATC = 0;
    /* General control of CORDIC Control Register */
    MATH->CON = CART2POL_CORDIC_CIRCULAR_VECTORING_MODE;
    /* Z = φ. Θ = atan(y/x) + angle φ */
    MATH->CORDZ = angle_offset;
    /* Y */
    MATH->CORDY = y;
    /* Input CORDX data, and auto start of CORDIC calculation */
    MATH->CORDX = x;
}

/**
 * @brief Read the CORDIC result register.
 * @param * resultant_ptr    Resultant magnitude will be update here.Signed -> [-2^23,(2^23-1)],Unsigned ->[0,(2^24)]
 * @param * angle_ptr        Resultant angle will be updated here.MSB aligned 24 bit [-2^23,(2^23-1)] represents [-π to π]
 * @return   None <BR>
 *
 * \par Description
 * This function reads the CORDIC result register and update the Magnitude & Angle.\n
 * Magnitude  = SQRT(x*x+y*y); where k = 1.646760258121 \n
 * Angle      = atan(y/x) + angle_q31;
 *
 * \par Note
 * On XMC14 device this API will use HW CORDIC module for computation of resultant magnitude.\n
 */
__STATIC_FORCEINLINE void BMCLIB_CartToPolarGetResult(uint32_t * const resultant_ptr, int32_t * const angle_ptr)
{
    uint32_t mag;

    while (CART2POL_IS_CORDIC_BUSY);
    /* Read CORDIC result |Vref| - 32-bit unsigned  and scale down to get real value */
    mag = MATH->CORRX;
    /* Angle addition by CORDIC directly, where Θ = atan(Vq/Vd), φ is rotor angle */
    *angle_ptr = (int32_t)MATH->CORRZ;

    /* Get real values by scaling down */
    mag = (uint32_t)(mag >> CART2POL_CORDIC_MPS_BY_K_SCALE);
    *resultant_ptr = (uint32_t)(mag * CART2POL_CORDIC_CIRCULAR_MPS_BY_K_SCALED); // x MPS/K.
}

#endif /* BMCLIB_CART2POL_H_ */
