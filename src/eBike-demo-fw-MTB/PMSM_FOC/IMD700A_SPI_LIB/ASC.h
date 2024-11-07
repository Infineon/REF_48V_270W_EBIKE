/**
    @file: ASC.h

    Header file of ASC
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

#ifndef ASC_H_
#define ASC_H_

 /*****************************************************************************/
#include <PMSM_FOC/IMD700A_SPI_LIB/SPI.h>
#include "xmc_usic.h"

#if (DEVICE == IMD700A)
#define UART_CH			    XMC_UART1_CH0
#define UART_TX_MODE    XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT6 //tx->P4.4
#define RXD_MUX         3    // USIC1_CH0.DX0D->P4.5
#define USIC_CH         USIC1_CH0
#define UART_TX_PIN     P4_4 //
#define UART_RX_PIN     P4_5 //

#elif (DEVICE == SPI_LINK)
#define UART_CH			XMC_UART0_CH1	//
#define UART_TX_MODE    XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7 //tx->P1.2
#define RXD_MUX         0    // USIC0_CH1.DX0A->P1.3
#define USIC_CH         USIC0_CH1
#define UART_TX_PIN     P1_2 //
#define UART_RX_PIN     P1_3 //
#endif

void ASC_Init(void);
#endif /* ASC_H_ */
