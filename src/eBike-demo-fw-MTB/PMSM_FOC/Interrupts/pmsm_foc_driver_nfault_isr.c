/**
    @file: pmsm_foc_driver_nfault_isr.c

    Implementation of pmsm_foc_driver_nfault_isr
*/

/* ===========================================================================
** Copyright (C) 2021-2023 Infineon Technologies AG
** All rights reserved.
** ===========================================================================
**
** ===========================================================================
** This document contains proprietary information of Infineon Technologies AG.
** Passing on and copying of this document, and communication of its contents
** is not permitted without Infineon's prior written authorisation.
** ===========================================================================
*/

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "../ControlModules/pmsm_foc_error_handling.h"
#include "../ControlModules/pmsm_foc_functions.h"
#include "../ControlModules/pmsm_foc_interface.h"
#include "../ControlModules/pmsm_foc_state_machine.h"
#include "../IMD700A_SPI_LIB/utility.h"

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup Interrupts
 * @{
 */

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/**
 * @brief nFAULT pin interrupt from 6EDL7141 gate driver.
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_DRIVER_nFAULT_ISR(void)
{
    /* Activate brake */
    XMC_GPIO_SetOutputLow(nBRAKE_PIN);

    /* Read the fault status register of 6EDL7141 */
    Driver_GetFaultStatus();

    PMSM_FOC_CTRL.error_status |= PMSM_FOC_EID_6EDL7141_FAULT;

    /* Next go to ERROR state & wait until it get cleared */
    PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_ERROR;

} /* End of PMSM_FOC_DRIVER_nFAULT_ISR () */

/**
 * @}
 */

/**
 * @}
 */

