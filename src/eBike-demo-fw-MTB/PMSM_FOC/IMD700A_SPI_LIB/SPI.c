/**
    @file: SPI.c

    Implementation of SPI
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

#include <PMSM_FOC/IMD700A_SPI_LIB/SPI.h>
#include <PMSM_FOC/IMD700A_SPI_LIB/utility.h>
#include "xmc_gpio.h"
#include "xmc_spi.h"

/* SPI configuration structure */
XMC_SPI_CH_CONFIG_t spi_config =
{
    .baudrate = SPI_BAUD_RATE,
    .bus_mode = XMC_SPI_CH_BUS_MODE_MASTER,
    .selo_inversion = XMC_SPI_CH_SLAVE_SEL_INV_TO_MSLS,  //set Slave select active low
    .parity_mode = XMC_USIC_CH_PARITY_MODE_NONE
};

/* GPIO SPI TX pin configuration */
XMC_GPIO_CONFIG_t tx_pin_config =
{
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT9		//SPI TX pin configure
};

/* GPIO SPI DX pin configuration */
XMC_GPIO_CONFIG_t dx_pin_config =
{
    .mode = XMC_GPIO_MODE_INPUT_PULL_UP                // SPI RX input pin configure
};

/* GPIO SPI SELO pin configuration */
XMC_GPIO_CONFIG_t selo_pin_config =
{
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT8,       // SPI Select pin configure
};

/* GPIO SPI SCLKOUT pin configuration */
XMC_GPIO_CONFIG_t clk_pin_config =
{
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT8,        // SPI Clock pin configure
};

void SPI_Master_Init(XMC_USIC_CH_t *const channel)
{
    /* Initialize SPI master*/
    XMC_SPI_CH_Init(channel, &spi_config);
    XMC_SPI_CH_SetWordLength(channel, SPI_WORD_LENGTH);
    XMC_SPI_CH_SetFrameLength(channel, SPI_FRAME_LENGTH);
    XMC_SPI_CH_SetBitOrderMsbFirst(channel);
    XMC_SPI_CH_EnableSlaveSelect(channel, XMC_SPI_CH_SLAVE_SELECT_0);
    #if (GATE_DRIVER == IFX_6EDL7141)
    XMC_SPI_CH_ConfigureShiftClockOutput(channel, XMC_SPI_CH_BRG_SHIFT_CLOCK_PASSIVE_LEVEL_0_DELAY_DISABLED, XMC_SPI_CH_BRG_SHIFT_CLOCK_OUTPUT_SCLK);  //added by tys falling sclk sampling and no polarity inversion
    XMC_SPI_CH_SetSlaveSelectPolarity (channel, XMC_SPI_CH_SLAVE_SEL_INV_TO_MSLS);  //added by tys, Slave select is active low to communicate with 6EDL7141
    #elif (GATE_DRIVER == IFX_NN)
    XMC_SPI_CH_ConfigureShiftClockOutput(channel, XMC_SPI_CH_BRG_SHIFT_CLOCK_PASSIVE_LEVEL_0_DELAY_ENABLED, XMC_SPI_CH_BRG_SHIFT_CLOCK_OUTPUT_SCLK);  //added by tys rising sclk sampling and no polarity inversion
    XMC_SPI_CH_SetSlaveSelectPolarity (channel, XMC_SPI_CH_SLAVE_SEL_SAME_AS_MSLS);  //added by tys, Slave select is active high to communicate with IFX_NN
    #endif

    /* Initialize FIFO */
    XMC_USIC_CH_RXFIFO_Configure(channel, 40, XMC_USIC_CH_FIFO_SIZE_8WORDS, 2);//receive from gate driver: limit is 1, when FIFO is full with 2 word and a 3nd is received, the event happens.
    //this is used in main to while until 3 words are received
    XMC_USIC_CH_TXFIFO_Configure(channel, 32, XMC_USIC_CH_FIFO_SIZE_8WORDS, 0);

    /* Configure input multiplexor */
    XMC_SPI_CH_SetInputSource(channel, XMC_SPI_CH_INPUT_DIN0, INPUT_MUX);		//SPI input stage source

    /* Start operation. */
    XMC_SPI_CH_Start(channel);

    /* Initialize GPIO */
    XMC_GPIO_Init(MOSI_PIN, &tx_pin_config);
    XMC_GPIO_Init(MISO_PIN, &dx_pin_config);
    XMC_GPIO_Init(SELO_PIN, &selo_pin_config);
    XMC_GPIO_Init(SCLK_PIN, &clk_pin_config);
}
