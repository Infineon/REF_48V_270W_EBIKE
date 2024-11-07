/**
    @file: SPI.h

    Header file of SPI
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
   
#ifndef SPI_H_
#define SPI_H_

 /*****************************************************************************/
#include <PMSM_FOC/IMD700A_SPI_LIB/ASC.h>
#include "xmc_usic.h"
#include "xmc_spi.h"
#include <xmc_common.h>
#include <PMSM_FOC/Configuration/pmsm_foc_const.h>


#define SPI_MAS_CH  XMC_SPI1_CH1
#define SCLK_PIN 		P0_3             //ALT8, P0_3, USIC1_CH1.SCLKOUT,ALT8
#define SELO_PIN 		P0_4             //ALT8, P0_4, USIC1_CH1.SELO0, ALT8

// SPI MISO and MOSI pinout definition
#if (DEVICE == SPI_LINK)
#define MISO_PIN 		P0_0             //P0_0, DX0A
#define MOSI_PIN 		P0_1             //ALT9, P0_1, USIC1_CH1.DOUT0,ALT9
#define INPUT_MUX       USIC1_C1_DX0_P0_0
#elif (DEVICE == IMD700A)
#define MISO_PIN 		P0_1             //P0_1, DX0B
#define MOSI_PIN 		P0_0             //ALT9, P0_1, USIC1_CH1.DOUT0,ALT9
#define INPUT_MUX       USIC1_C1_DX0_P0_1
#endif

#define SPI_BAUD_RATE      4000000
#define SPI_WORD_LENGTH    (8U)
#define SPI_FRAME_LENGTH   (24U)

#define LED0 			   P0_0

void SPI_Master_Init(XMC_USIC_CH_t *const channel);


#endif /* SPI_H_ */
