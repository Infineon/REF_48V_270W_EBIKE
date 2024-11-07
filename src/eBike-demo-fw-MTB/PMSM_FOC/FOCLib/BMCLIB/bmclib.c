/**
    @file: bmclib.c

    Implementation of bmclib general function
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

#include <PMSM_FOC/FOCLib/BMCLIB/bmclib.h>

/* Doxygen comment in respective header */
uint32_t BMCLIB_GetFirmwareVersion(void)
{
    uint32_t temp;
    temp = (uint32_t)BMCLIB_FIRMWARE_VERSION;
    return temp;
}
