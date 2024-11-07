/**
 * @file bmclib_pt1_filter.h
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
#ifndef BMCLIB_FILTER_PT1_H_
#define BMCLIB_FILTER_PT1_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/**
 * This structure holds the values of PT1 filter parameters.
 */
typedef struct BMCLIB_PT1_FILTER
{
    int32_t  z1_q15;           /*!< Filter time constant in scaled format  (fc*2^15/fsample)*/
    int32_t  pt1_out_val;      /*!< filter output */
} BMCLIB_PT1_FILTER_t;
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
 * @brief PT1 is a low pass filter.Low pass filtering will be done as per configured cutoff frequency.
 * @param input_val            Input value for filtering
 * @param pt1_filter_ptr       PT1_filter structure pointer (pass by reference)
 * @return   None <BR>
 *
 * \par Description
 * This function calculates the magnitude of resultant vector in circular mode.\n
 * Y[n+1] = Y[n] + (((X[n+1] - Y[n]) * z1_q15) >> 15)
 *
 */
__STATIC_FORCEINLINE void BMCLIB_PT1_Filter(int32_t input_val, BMCLIB_PT1_FILTER_t * const pt1_filter_ptr)
{
    pt1_filter_ptr->pt1_out_val += (int32_t)((pt1_filter_ptr->z1_q15 * (input_val - pt1_filter_ptr->pt1_out_val)) >> 15);
}

#endif /* BMCLIB_FILTER_PT1_H_ */
