/**
    @file: ASC.c

    Implementation of ASC
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

/******************************************************************************
* 
*  ASC communication programms
*
 *****************************************************************************/

#include <PMSM_FOC/IMD700A_SPI_LIB/ASC.h>
#include "xmc_gpio.h"
#include "xmc_uart.h"

/* MACROS */

#define UART_BAUD_RATE       (115200)

/* UART configuration */
XMC_UART_CH_CONFIG_t     uart_config =
{
    .baudrate = UART_BAUD_RATE,
    .data_bits = 8U,
    .frame_length = 8U,
    .stop_bits = 1U,
    .parity_mode = XMC_USIC_CH_PARITY_MODE_NONE
};

/* UART Tx and Rx Pins */
XMC_GPIO_CONFIG_t rx_pin_config =
{
    .mode = XMC_GPIO_MODE_INPUT_PULL_UP,   //Rx input mode configure
};

XMC_GPIO_CONFIG_t asc_tx_pin_config =
{
   .mode = UART_TX_MODE,  // Tx output mode configure
};


void ASC_Init()
{
    /* Initialize GPIO */
    XMC_GPIO_Init(UART_RX_PIN, &rx_pin_config);
    XMC_GPIO_Init(UART_TX_PIN, &asc_tx_pin_config);

    /* Configure UART channel */
    XMC_UART_CH_Init(UART_CH, &uart_config);
    /* Configure input multiplexor */
    XMC_UART_CH_SetInputSource(UART_CH,XMC_UART_CH_INPUT_RXD, RXD_MUX); //input select
    /* Initialize FIFO */
    XMC_USIC_CH_RXFIFO_Configure(UART_CH, 8, XMC_USIC_CH_FIFO_SIZE_8WORDS, 4);//limit is 4, when FIFO is full with 4 words and a 5 is received, the event happnes.
    //this is used in main to while until 3 words are received
    XMC_USIC_CH_TXFIFO_Configure(UART_CH, 0, XMC_USIC_CH_FIFO_SIZE_8WORDS, 1);

    /* Enable Standard Receive Event */
    //XMC_UART_CH_EnableEvent(XMC_UART0_CH1, XMC_UART_CH_EVENT_STANDARD_RECEIVE);

    /* Start UART channel */
    XMC_UART_CH_Start(UART_CH);
}






