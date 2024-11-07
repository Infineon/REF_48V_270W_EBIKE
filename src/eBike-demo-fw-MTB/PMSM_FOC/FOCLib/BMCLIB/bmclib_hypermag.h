/**
 * @file bmclib_hypermag.h
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
#ifndef BMCLIB_HYPERMAG_H_
#define BMCLIB_HYPERMAG_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#define HYPERMAG_IS_CORDIC_BUSY                     (MATH->STATC & 0x01) /*!< Returns 1 if CORDIC is busy */
#define HYPERMAG_CORDIC_HYPERBOLIC_VECTORING_MODE    (0x66)          /*!< CORDIC: Hyperbolic Vectoring Mode. MPS: Divide by 2 (default).*/
#define HYPERMAG_CORDIC_HYPERBOLIC_MPS_BY_K_SCALED   (618)           /*!< CORDIC MPS/K ->(2/0.828159360960)* 2^CORDIC_MPS_BY_K_SCALE */
#define HYPERMAG_CORDIC_MPS_BY_K_SCALE               (8)             /*!< CORDIC MPS/K scaling factor */
#define ABS(a)             (((a)< 0) ? (-(a)) : (a))    			 /*!< returns the absolute number */
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
 * @brief Calculates magnitude of resultant vector in hyperbolic mode.
 * @param x  Input X co-ordinate value
 * @param y  Input Y co-ordinate value
 * @return   None <BR>
 *
 * \par Description
 * This function calculates the magnitude of resultant vector in hyperbolic mode.\n
 * Magnitude  = SQRT(x*x-y*y); where k = 0.828159360960 \n
 *
 * \par Note
 * On XMC14 device this API will use HW CORDIC module for computation of trigonometric, hyperbolic and linear functions. \n
 * x should be greater than y.
 */
__STATIC_FORCEINLINE void BMCLIB_HyperMag(int32_t x, int32_t y)
{
   /* Useful domain for CORDIC in this mode is |x|>|y| */
    if(ABS(x) < ABS(y))
    {
      y=0;
      x=0;
    }
    /* General control of CORDIC Control Register */
    MATH->CON = HYPERMAG_CORDIC_HYPERBOLIC_VECTORING_MODE;
  	/* Z = T = atan(y/x) */
    MATH->CORDZ = 0;
  	/* Input CORDY data = y */
    MATH->CORDY = y;
    /* Input CORDX data, and auto start of CORDIC calculation */
    MATH->CORDX = x;
}

/**
 * @brief Reads the results of magnitude of resultant vector in hyperbolic mode.
 *
 * @param None
 *
 * @retval resultant magnitude
 */
__STATIC_FORCEINLINE  int32_t BMCLIB_HyperMagGetResult(void)
{
    int32_t resultant_magnitude;
    while (HYPERMAG_IS_CORDIC_BUSY);
    /* Read CORDIC result - 32-bit unsigned  and scale down to get real value */
    resultant_magnitude = MATH->CORRX;
    resultant_magnitude = (int32_t)(resultant_magnitude >> HYPERMAG_CORDIC_MPS_BY_K_SCALE);
    /* Get real values by scaling down */
    resultant_magnitude = (int32_t)(resultant_magnitude * HYPERMAG_CORDIC_HYPERBOLIC_MPS_BY_K_SCALED); // x MPS/K.
    return (resultant_magnitude);
}

#endif /* BMCLIB_HYPERMAG_H_ */
