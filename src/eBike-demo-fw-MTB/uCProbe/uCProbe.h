/**
 * @file pmsm_foc_ucProbe.h
 * @date 09 May, 2019
 *
 * @cond
 *********************************************************************************************************************
 * PMSM_FOC Motor Control Library
 *
 * Copyright (c) 2015-2020, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * @endcond
 ***********************************************************************************************************************/
#ifndef UCPROBE_UCPROBE_H_
#define UCPROBE_UCPROBE_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <PMSM_FOC/Configuration/pmsm_foc_config.h>
#include "../uCProbe/ProbeScope/probe_scope.h"
#include "../uCProbe/ProbeScope/probe_scope_cfg.h"

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup uCProbe
 * @{
 */
/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
typedef union
{
  uint32_t user_config_array[50];
  struct
  {
    uint32_t config_valid_id; /*!< Valid Configuration ID */

    uint32_t pi_speed_kp;
    uint32_t pi_speed_ki;
    uint32_t pi_speed_scale_kpki;
    int32_t pi_speed_ik_limit_min;
    int32_t pi_speed_ik_limit_max;
    int32_t pi_speed_uk_limit_min;
    int32_t pi_speed_uk_limit_max;
    int32_t pi_speed_antiwindup_enable;

    uint32_t pi_torque_kp;
    uint32_t pi_torque_ki;
    uint32_t pi_torque_scale_kpki;
    int32_t pi_torque_ik_limit_min;
    int32_t pi_torque_ik_limit_max;
    int32_t pi_torque_uk_limit_min;
    int32_t pi_torque_uk_limit_max;
    int32_t pi_torque_antiwindup_enable;

    uint32_t pi_flux_kp;
    uint32_t pi_flux_ki;
    uint32_t pi_flux_scale_kpki;
    int32_t pi_flux_ik_limit_min;
    int32_t pi_flux_ik_limit_max;
    int32_t pi_flux_uk_limit_min;
    int32_t pi_flux_uk_limit_max;
    int32_t pi_flux_antiwindup_enable;

#if(MOTOR_CTRL_SCHEME != PMSM_HALL_FOC)
    uint32_t pi_pll_kp;
    uint32_t pi_pll_ki;
    uint32_t pi_pll_scale_kpki;
    int32_t pi_pll_ik_limit_min;
    int32_t pi_pll_ik_limit_max;
    int32_t pi_pll_uk_limit_min;
    int32_t pi_pll_uk_limit_max;
#endif

    uint32_t pi_fw_kp;
    uint32_t pi_fw_ki;
    uint32_t pi_fw_scale_kpki;
    int32_t pi_fw_ik_limit_min;
    int32_t pi_fw_ik_limit_max;
    int32_t pi_fw_uk_limit_min;
    int32_t pi_fw_uk_limit_max;
    int32_t pi_fw_antiwindup_enable;

  };

} USER_CONFIG_t;

/***********************************************************************************************************************
 * EXTERN
 **********************************************************************************************************************/
extern volatile uint32_t         ucProbe_cmd;
extern USER_CONFIG_t USER_CONFIG;

/***********************************************************************************************************************
 * API's PROTOTYPE
 **********************************************************************************************************************/
void PMSM_FOC_uCProbe_Init(void);
void PMSM_FOC_ucProbe_ReadFlash(void);
void PMSM_FOC_ucProbe_WriteFlash(void);  /* To write to the flash the updated user config data from u-inspector command processing */
void PMSM_FOC_ucProbe_CmdProcessing(void);

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/
#define PMSM_SL_FOC_UCPROBE_MAX_PARAMETER                (50U)                        /* Maximum number of parameter to be saved in Flash*/
#define PMSM_SL_FOC_UCPROBE_FLASH_VALID_ID               (0x1235)                     /* 16 bit Validation ID to check whether flash has any valid data*/

/* Command used to communicate with GUI */
#define UCPROBE_CMD_WRITE_USERPIPARAM_FLASH              (11U)
#define UCPROBE_CMD_READ_USERPIPARAM_FLASH               (12U)
#define UCPROBE_CMD_WRITE_DFLT_PIPARAM_FLASH             (13U)
#define UCPROBE_CMD_READ_DFLT_PIPARAM_FLASH              (14U)
#define UCPROBE_CMD_RESET                                (15U)
#define UCPROBE_CMD_MOTOR_STOP                           (16U)

/**
 * @}
 */

/**
 * @}
 */

#endif /* UCPROBE_UCPROBE_H_ */
