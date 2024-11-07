/**
 * @file bmclib_limiters.h
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
#ifndef BMCLIB_LIMITERS_H_
#define BMCLIB_LIMITERS_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

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

#define BMCLIB_MIN_MAX_LIMIT(Buffer,LimitL,LimitH) (((Buffer) > (LimitH)) ? (LimitH) : (((Buffer) < (LimitL))? (LimitL): (Buffer))) /*!< macro to check within Maximum and Minimum limit */
#define BMCLIB_MAX_LIMIT(Buffer,LimitH) (((Buffer) > (LimitH)) ? (LimitH) : (Buffer)) /*!< macro to check within Maximum limit */
#define BMCLIB_MIN_LIMIT(Buffer,LimitL) (((Buffer) < (LimitL)) ? (LimitL) : (Buffer)) /*!< macro to check within Minimum limit */

#define BMCLIB_MIN(a, b)                           (((a) < (b)) ? (a) : (b))   /*!< macro returning smallest input */
#define BMCLIB_MAX(a, b)                           (((a) > (b)) ? (a) : (b))   /*!< macro returning biggest input */

#endif /* BMCLIB_LIMITERS_H_ */
