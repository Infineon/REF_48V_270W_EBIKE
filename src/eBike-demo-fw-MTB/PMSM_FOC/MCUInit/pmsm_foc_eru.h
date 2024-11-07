/**
    @file: pmsm_foc_eru.h

    Header file of pmsm_foc_eru
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

#ifndef PMSM_FOC_ERU_H_
#define PMSM_FOC_ERU_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <PMSM_FOC/Configuration/pmsm_foc_const.h>
#include <PMSM_FOC/Configuration/pmsm_foc_mcuhw_params.h>
#include <xmc_eru.h>

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup MCUInit
 * @{
 */
/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Configures the ERU for the Pin Interrupt. <BR>\n
 *
 */
void PMSM_FOC_PIN_INTERRUPT_Init(void);

/**
 * @}
 */

/**
 * @}
 */
#endif /* PMSM_FOC_MCUINIT_PMSM_FOC_WDT_H_ */
