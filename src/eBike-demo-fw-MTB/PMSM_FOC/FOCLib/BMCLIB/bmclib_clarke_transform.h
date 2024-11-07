/**
 * @file bmclib_clarke_transform.h
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
#ifndef BMCLIB_CLARKE_TRANSFORM_H_
#define BMCLIB_CLARKE_TRANSFORM_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#define BMCLIB_DIV_3_Q14            (5461)         /* ((int16_t)((1/3) * (1<<SCALE_DIV_3))) */
#define BMCLIB_DIV_SQRT3_Q14        (9459)         /* (1/√3)*2^14 */
/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/**  @brief Clarke Transform structure */
typedef struct
{
    int32_t alpha;     /*!< Output quadrature quantity - α  in 2^14 scaled format for higher resolution */
    int32_t beta;      /*!< Output quadrature quantity - β  in 2^14 scaled format for higher resolution */
} BMCLIB_CLARKE_TRANSFORM_t;

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
 * @brief Converts balanced three phase quantities(us,vs,ws) into balanced two phase quadrature quantities(alpha,beta).
 * Alpha = (2 * I_U - (I_V + I_W))/3
 * Beta = (I_U + 2 * I_V)/√3 = (I_V - I_W)/√3
 *
 * @param us
 * @param vs
 * @param ws
 * @param BMCLIB_CLARKE_TRANSFORM_t
 *
 *@retval *PMSM_FOC_CLARKE_TRANSFORM_t clarke_transform_ptr
 */
__STATIC_FORCEINLINE void BMCLIB_ClarkeTransform(int16_t us,int16_t vs, int16_t ws, BMCLIB_CLARKE_TRANSFORM_t* const clarke_transform_ptr)
{
    /* Use all three phase current measurements to calculate alpha,beta */
    /* Alpha = (2 * Us - (Vs + Ws))/3 */
    clarke_transform_ptr->alpha = (int32_t)(((us << 1) - (vs + ws)) * BMCLIB_DIV_3_Q14);
    /*  Beta = (Vs - Ws)/√3 in 1Q31 */
    clarke_transform_ptr->beta = (int32_t)((vs - ws) * BMCLIB_DIV_SQRT3_Q14);
}

#endif /* BMCLIB_CLARKE_TRANSFORM_H_ */
