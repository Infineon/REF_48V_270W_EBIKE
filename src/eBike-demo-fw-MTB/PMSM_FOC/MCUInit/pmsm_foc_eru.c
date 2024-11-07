/**
    @file: pmsm_foc_eru.c

    Implementation of pmsm_foc_eru
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
#include <PMSM_FOC/MCUInit/pmsm_foc_eru.h>

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/*
 * Data Structure initialization - ERU Configuration.
 */

XMC_ERU_ETL_CONFIG_t XMC_ERU_ETL_CONFIG =
{
    .input_a = (uint32_t)XMC_ERU_ETL_INPUT_A3, /* Event input selection for A(0-3) */
    .input_b = (uint32_t)XMC_ERU_ETL_INPUT_B0, /* Event input selection for B(0-3) */
    .enable_output_trigger = (uint32_t)1,
    .edge_detection = XMC_ERU_ETL_EDGE_DETECTION_FALLING, /* Select the edge to convert as event */
    .output_trigger_channel = XMC_ERU_ETL_OUTPUT_TRIGGER_CHANNEL3, /* Select the source for event */
    .source = XMC_ERU_ETL_SOURCE_A
};
/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

/* API to initialize the Fault pin interrupt with ERU */
void PMSM_FOC_PIN_INTERRUPT_Init(void)
{
    /* ERU Event Trigger Logic Hardware initialization */
    XMC_ERU_ETL_Init(XMC_ERU1, 2U, &XMC_ERU_ETL_CONFIG);

    /* OGU is configured to generate event on configured trigger edge */
    XMC_ERU_OGU_SetServiceRequestMode(XMC_ERU1, 3U, XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER);

    NVIC_SetPriority (FAULT_PIN_ERU_IRQn, PMSM_FOC_FAULT_NVIC_PRIO);
    XMC_SCU_SetInterruptControl(FAULT_PIN_ERU_IRQn, XMC_SCU_IRQCTRL_ERU1_SR3_IRQ6);
    NVIC_EnableIRQ(FAULT_PIN_ERU_IRQn);
}

