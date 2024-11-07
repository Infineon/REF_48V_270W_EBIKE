/**
 * @file pmsm_foc_error_handling.h
 * @date 09 May, 2019
 *
 * @cond
 *********************************************************************************************************************
 * PMSM FOC Motor Control Library
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

#ifndef PMSM_FOC_ERROR_HANDLING_H_
#define PMSM_FOC_ERROR_HANDLING_H_

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup ControlModules
 * @{
 */

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * MACRO's
 ********************************************************************************************************************/
#define PMSM_FOC_EID_UNDER_VOLT_POS          (0U)
#define PMSM_FOC_EID_OVER_VOLT_POS           (1U)
#define PMSM_FOC_EID_STALL_POS               (2U)
#define PMSM_FOC_EID_INVERTER_OVER_TEMP_POS  (3U)
#define PMSM_FOC_EID_TORQUE_LIMIT_EXCEED_POS (4U)
#define PMSM_FOC_EID_OVER_CURRENT_POS        (5U)
#define PMSM_FOC_EID_IDC_OVERCURRENT_POS     (6U)
#define PMSM_FOC_EID_CTRAP_POS               (7U)
#define PMSM_FOC_EID_INVALID_HALL_PAT_POS	 (8U)
#define PMSM_FOC_EID_6EDL7141_POS            (9U)


/**********************************************************************************************************************
 * ENUMS
 **********************************************************************************************************************/
/**
 * @brief This enumerates the error codes of the control which can occur during run-time.
 */
typedef enum PMSM_FOC_EID
{
    PMSM_FOC_EID_NO_ERROR             = 0,                                          /*!< Error ID 000 - NO ERROR */
    PMSM_FOC_EID_UNDER_VOLT           = (1U<<PMSM_FOC_EID_UNDER_VOLT_POS),          /*!< Error ID 001 - DC BUS UNDER VOLTAGE */
    PMSM_FOC_EID_OVER_VOLT            = (1U<<PMSM_FOC_EID_OVER_VOLT_POS),           /*!< Error ID 002 - DC BUS OVER VOLTAGE */
    PMSM_FOC_EID_STALL                = (1U<<PMSM_FOC_EID_STALL_POS),               /*!< Error ID 003 - ROTOR STALLED */
    PMSM_FOC_EID_INVERTER_OVER_TEMP   = (1U<<PMSM_FOC_EID_INVERTER_OVER_TEMP_POS),  /*!< Error ID 004 - INVERTER OVER TEMPERATURE */
    PMSM_FOC_EID_TORQUE_LIMIT_EXCEED  = (1U<<PMSM_FOC_EID_TORQUE_LIMIT_EXCEED_POS), /*!< Error ID 005 - TORQUE LIMIT EXCEED */
    PMSM_FOC_EID_OVER_CURRENT         = (1U<<PMSM_FOC_EID_OVER_CURRENT_POS),        /*!< Error ID 006 - OVER CURRENT */
    PMSM_FOC_EID_IDC_OVERCURRENT      = (1U<<PMSM_FOC_EID_IDC_OVERCURRENT_POS),     /*!< Error ID 007 - IDC OVER CURRENT */
	PMSM_FOC_EID_CTRAP				  = (1U<<PMSM_FOC_EID_CTRAP_POS),				/*!< Error ID 008 - CCU8 TRAP */
	PMSM_FOC_EID_INVALID_HALL_PAT	  = (1U<<PMSM_FOC_EID_INVALID_HALL_PAT_POS),
	PMSM_FOC_EID_6EDL7141_FAULT       = (1U<<PMSM_FOC_EID_6EDL7141_POS)
} PMSM_FOC_EID_t;

/***********************************************************************************************************************
 * API Prototypes
 **********************************************************************************************************************/
/**
 * @brief	Check if any errors are cleared and change the control accordingly.
 *
 * @param	None
 *
 *@retval	None
 */
void PMSM_FOC_ErrorHandling(void);

/**
 * @}
 */

/**
 * @}
 */
#endif /* PMSM_FOC_INTERRUPTS_PMSM_FOC_ERROR_HANDLING_H_ */

