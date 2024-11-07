/**
    @file: IMD700_SPI_LIB.h

    Header file of IMD700_SPI_LIB
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
   
#ifndef IMD700_SPI_LIB_H_
#define IMD700_SPI_LIB_H_

#include <PMSM_FOC/IMD700A_SPI_LIB/ASC.h>
#include <PMSM_FOC/IMD700A_SPI_LIB/SPI.h>
#include <PMSM_FOC/IMD700A_SPI_LIB/utility.h>
#include <xmc_gpio.h>
#include <xmc_spi.h>
#include <xmc_uart.h>
#include <xmc_common.h>


/*********************************************************************************************************************
 * API Prototype
*********************************************************************************************************************/
//void ASC_Init(void);
//void SendByte(uint8_t data);
//void device_id_read(void);
//void bauderate_change_response(XMC_USIC_CH_t *const channel, uint32_t baudrate, uint32_t oversampling);
void poll_gui_message(void);
//uint8_t checksum (uint8_t *msg, uint16_t MsgSize);
void Config_Driver_6EDL7141(void);

#endif /* End of #ifndef IMD700_SPI_LIB_H_ */
