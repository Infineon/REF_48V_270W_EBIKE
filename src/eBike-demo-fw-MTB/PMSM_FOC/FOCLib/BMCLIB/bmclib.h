/**
    @file: bmclib.h

    Header file of bmclib in general
*/

/* ===========================================================================
** Copyright (C) 2022 Infineon Technologies AG
** All rights reserved.
** ===========================================================================
**
** ===========================================================================
** This document contains proprietary information of Infineon Technologies AG.
** Passing on and copying of this document, and communication of its contents
** is not permitted without Infineon's prior written authorisation.
** ===========================================================================
*/

#ifndef BMCLIB_H
#define BMCLIB_H

#include <xmc_common.h>

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/
//BMCLIB firmware version
#define BMCLIB_FIRMWARE_VERSION               (0x00000100)      /*!< MSB to LSB description MSB:Nil, Major=0x00, Minor=0x01 , Patch=0x00 */

/***********************************************************************************************************************
 * API Prototypes
 **********************************************************************************************************************/
/**
 * This function returns BMCLIB Firmware Version.
 * @return BMCLIB Firmware Version
 */
uint32_t BMCLIB_GetFirmwareVersion(void);

#endif
