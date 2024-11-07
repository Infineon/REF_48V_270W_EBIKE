/**
 * @file pmsm_foc_ucProbe.c
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
 /*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

#include "xmc1_flash.h"
#include "uCProbe.h"
#include "../PMSM_FOC/ControlModules/pmsm_foc_functions.h"
#include "../PMSM_FOC/ControlModules/pmsm_foc_interface.h"
#include "../PMSM_FOC/ControlModules/pmsm_foc_state_machine.h"

#if(USER_UCPROBE_GUI == ENABLED)
/*********************************************************************************************************************
 * DEFINATIONS
 ********************************************************************************************************************/
#define SPEED_RPM_LPF                (2U)
#define USER_CONFIG_START_ADDRESS    ((uint32_t*)(0x10020C00)) /* Address taken from linker script */
/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/
volatile uint32_t ucProbe_cmd;
extern PMSM_FOC_CTRL_t PMSM_FOC_CTRL;

/*********************************************************************************************************************
 * Local DATA
 ********************************************************************************************************************/
volatile int32_t   Speed_RPM_LPF;             /* uC_Probe variable */
volatile int32_t   VDC_mV_LPF;                /* uC_Probe variable */

/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/

/***********************************************************************************************************************/
/* This function will read the user configuration data from the flash and load the USER_CONFIG variable with the values read from the flash */
void PMSM_FOC_uCProbe_Init(void)
{
  #if (USER_READ_CONFIG_FROM_FLASH == ENABLED)
    uint32_t *motor_conf_addr;
    motor_conf_addr = USER_CONFIG_START_ADDRESS;

    if (*motor_conf_addr == PMSM_SL_FOC_UCPROBE_FLASH_VALID_ID)
    {
      /*Read configuration from flash*/
      PMSM_FOC_ucProbe_ReadFlash();
    }
    else  /* If there is no data in the flash, then loading of the USER_CONFIG variable with default values is taken place in PMSM_FOC_UserConfig_Init() function */
    {
      PMSM_FOC_UserConfig_Init();
    }
  #else /* If the USER_READ_CONFIG_FROM_FLASH is disabled, load the default values */
    PMSM_FOC_UserConfig_Init();
  #endif

}

/***********************************************************************************************************************/
void PMSM_FOC_ucProbe_ReadFlash(void)
{
  uint32_t count = 0;
  uint32_t *motor_conf_addr;
  motor_conf_addr = USER_CONFIG_START_ADDRESS;

  for (count = 0; count < PMSM_SL_FOC_UCPROBE_MAX_PARAMETER; count++)
  {
    USER_CONFIG.user_config_array[count] = (uint32_t)(*motor_conf_addr++);
  }
}

/* This function will overwrite the flash contents with the values updated from the micro-inspector */
/* This function will be called from the uCProbe command processing function */
/* Whenever this function is called, the flash's configuration region needs to be overwritten with the updated values */
void PMSM_FOC_ucProbe_WriteFlash(void)
{
  uint32_t count;
  uint32_t *motor_conf_addr;
  motor_conf_addr = USER_CONFIG_START_ADDRESS;
  uint32_t user_config[PMSM_SL_FOC_UCPROBE_MAX_PARAMETER];
  USER_CONFIG.config_valid_id = PMSM_SL_FOC_UCPROBE_FLASH_VALID_ID;

  /* Update user parameters */
  for (count = 0; count < PMSM_SL_FOC_UCPROBE_MAX_PARAMETER; count++)
    user_config[count] = USER_CONFIG.user_config_array[count];

  /* Write only when motor is in STOP state */
  if (PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_STOP_MOTOR)
  {
    /*Erase and write 256 byte of data*/
    //XMC_FLASH_ProgramVerifyPage(motor_conf_addr, (const uint32_t*) &user_config); /*Address, data*/
  }
}

/***********************************************************************************************************************/
/* This function process the commands received from the ucProbe/MicroInspector GUI and updates FOC input configurations */
void PMSM_FOC_ucProbe_CmdProcessing(void)
{
  int32_t Speed_RPM;

  /* uC Probe Processing Handling */
  /* uC Speed Display */
  Speed_RPM = (int32_t)(((int32_t)PMSM_FOC_MC_PARA.rotor_speed * CONVERT_SPEED_TO_RPM ) >> SPEED_TO_RPM_SCALE);
  Speed_RPM_LPF += (int32_t)((Speed_RPM - Speed_RPM_LPF)>> SPEED_RPM_LPF);

  VDC_mV_LPF = (int32_t)((VDC_ADC_TO_mV * ADC.adc_res_vdc_filtered) >> VDC_ADC_TO_mV_SCALE);

  switch (ucProbe_cmd)
  {
    case UCPROBE_CMD_WRITE_USERPIPARAM_FLASH:
      /* Write User Parameters to FLASH */
      /* To check if the motor is in stop state */
      if (PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_STOP_MOTOR)
      {
        /* Disable all the interrupts before writing to the flash */
        __disable_irq();
      /* Update the USER_CONFIG variable with the latest control parameters' values before writing to flash */
      USER_CONFIG.pi_speed_kp = PMSM_FOC_SPEED_PI.kp;
      USER_CONFIG.pi_speed_ki = PMSM_FOC_SPEED_PI.ki;
      USER_CONFIG.pi_speed_scale_kpki = PMSM_FOC_SPEED_PI.scale_kpki;
      USER_CONFIG.pi_speed_ik_limit_max = PMSM_FOC_SPEED_PI.ik_limit_max;
      USER_CONFIG.pi_speed_ik_limit_min = PMSM_FOC_SPEED_PI.ik_limit_min;
      USER_CONFIG.pi_speed_uk_limit_max = PMSM_FOC_SPEED_PI.uk_limit_max;
      USER_CONFIG.pi_speed_uk_limit_min = PMSM_FOC_SPEED_PI.uk_limit_min;
      USER_CONFIG.pi_speed_antiwindup_enable = PMSM_FOC_SPEED_PI.enable_antiwindup;

      USER_CONFIG.pi_torque_kp = PMSM_FOC_TORQUE_PI.kp;
      USER_CONFIG.pi_torque_ki = PMSM_FOC_TORQUE_PI.ki;
      USER_CONFIG.pi_torque_scale_kpki = PMSM_FOC_TORQUE_PI.scale_kpki;
      USER_CONFIG.pi_torque_ik_limit_max = PMSM_FOC_TORQUE_PI.ik_limit_max;
      USER_CONFIG.pi_torque_ik_limit_min = PMSM_FOC_TORQUE_PI.ik_limit_min;
      USER_CONFIG.pi_torque_uk_limit_max = PMSM_FOC_TORQUE_PI.uk_limit_max;
      USER_CONFIG.pi_torque_uk_limit_min = PMSM_FOC_TORQUE_PI.uk_limit_min;
      USER_CONFIG.pi_torque_antiwindup_enable = PMSM_FOC_TORQUE_PI.enable_antiwindup;

      USER_CONFIG.pi_flux_kp = PMSM_FOC_FLUX_PI.kp;
      USER_CONFIG.pi_flux_ki = PMSM_FOC_FLUX_PI.ki;
      USER_CONFIG.pi_flux_scale_kpki = PMSM_FOC_FLUX_PI.scale_kpki;
      USER_CONFIG.pi_flux_ik_limit_max = PMSM_FOC_FLUX_PI.ik_limit_max;
      USER_CONFIG.pi_flux_ik_limit_min = PMSM_FOC_FLUX_PI.ik_limit_min;
      USER_CONFIG.pi_flux_uk_limit_max = PMSM_FOC_FLUX_PI.uk_limit_max;
      USER_CONFIG.pi_flux_uk_limit_max = PMSM_FOC_FLUX_PI.uk_limit_min;
      USER_CONFIG.pi_flux_antiwindup_enable = PMSM_FOC_FLUX_PI.enable_antiwindup;

        /* Update to flash */
        PMSM_FOC_ucProbe_WriteFlash();
        /* Enable all IRQs */
        __enable_irq();
        /* Command is processed */
        ucProbe_cmd = 0;
      }
      break;

    case UCPROBE_CMD_READ_USERPIPARAM_FLASH:
      /* Read User Parameters from FLASH */
	  if(PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_STOP_MOTOR)
	  {
        /* Disable all interrupts before accessing the flash */
        __disable_irq();
      /* Update to flash */
      PMSM_FOC_ucProbe_ReadFlash();
      /* After reading from flash, the latest data will be loaded to the PI controller's variables */
      PMSM_FOC_SPEED_PI.kp = USER_CONFIG.pi_speed_kp;
      PMSM_FOC_SPEED_PI.ki = USER_CONFIG.pi_speed_ki;
      PMSM_FOC_SPEED_PI.scale_kpki = USER_CONFIG.pi_speed_scale_kpki;
      PMSM_FOC_SPEED_PI.ik_limit_max = USER_CONFIG.pi_speed_ik_limit_max;
      PMSM_FOC_SPEED_PI.ik_limit_min = USER_CONFIG.pi_speed_ik_limit_min;
      PMSM_FOC_SPEED_PI.uk_limit_max = USER_CONFIG.pi_speed_uk_limit_max;
      PMSM_FOC_SPEED_PI.uk_limit_min = USER_CONFIG.pi_speed_uk_limit_min;
      PMSM_FOC_SPEED_PI.enable_antiwindup = USER_CONFIG.pi_speed_antiwindup_enable;

      PMSM_FOC_TORQUE_PI.kp = USER_CONFIG.pi_torque_kp;
      PMSM_FOC_TORQUE_PI.ki = USER_CONFIG.pi_torque_ki;
      PMSM_FOC_TORQUE_PI.scale_kpki = USER_CONFIG.pi_torque_scale_kpki;
      PMSM_FOC_TORQUE_PI.ik_limit_max = USER_CONFIG.pi_torque_ik_limit_max;
      PMSM_FOC_TORQUE_PI.ik_limit_min = USER_CONFIG.pi_torque_ik_limit_min;
      PMSM_FOC_TORQUE_PI.uk_limit_max = USER_CONFIG.pi_torque_uk_limit_max;
      PMSM_FOC_TORQUE_PI.uk_limit_min = USER_CONFIG.pi_torque_uk_limit_min;
      PMSM_FOC_TORQUE_PI.enable_antiwindup = USER_CONFIG.pi_torque_antiwindup_enable;

      PMSM_FOC_FLUX_PI.kp = USER_CONFIG.pi_flux_kp;
      PMSM_FOC_FLUX_PI.ki = USER_CONFIG.pi_flux_ki;
      PMSM_FOC_FLUX_PI.scale_kpki = USER_CONFIG.pi_flux_scale_kpki;
      PMSM_FOC_FLUX_PI.ik_limit_max = USER_CONFIG.pi_flux_ik_limit_max;
      PMSM_FOC_FLUX_PI.ik_limit_min = USER_CONFIG.pi_flux_ik_limit_min;
      PMSM_FOC_FLUX_PI.uk_limit_max = USER_CONFIG.pi_flux_uk_limit_max;
      PMSM_FOC_FLUX_PI.uk_limit_min = USER_CONFIG.pi_flux_uk_limit_min;
      PMSM_FOC_FLUX_PI.enable_antiwindup = USER_CONFIG.pi_flux_antiwindup_enable;

        /* Enable IRQs */
        __enable_irq();
        /* Command is processed */
        ucProbe_cmd = 0;
      }
      break;

    case UCPROBE_CMD_RESET:
      /* Reset MCU */
      NVIC_SystemReset();
      break;

    case UCPROBE_CMD_WRITE_DFLT_PIPARAM_FLASH:  /* This command will not be implemented */
      /* STOP motor */
      PMSM_FOC_MotorStop();
      /* Update Default PI Values in Flash with user configured PI Values. */
      /* Disable CCU8 interrupt to enable update of user parameter */
      NVIC_DisableIRQ(CCU80_0_IRQn);

#if(MOTOR_CTRL_SCHEME != PMSM_HALL_FOC)
      USER_CONFIG.pi_pll_kp = PI_PLL_KP;
      USER_CONFIG.pi_pll_ki = PI_PLL_KI;
      USER_CONFIG.pi_pll_scale_kpki = PI_PLL_SCALE_KPKI;
      USER_CONFIG.pi_pll_ik_limit_max = PI_PLL_IK_LIMIT_MAX;
      USER_CONFIG.pi_pll_ik_limit_min = PI_PLL_IK_LIMIT_MIN;
      USER_CONFIG.pi_pll_uk_limit_max = PI_PLL_UK_LIMIT_MAX;
      USER_CONFIG.pi_pll_uk_limit_min = PI_PLL_UK_LIMIT_MIN;
#endif

      USER_CONFIG.pi_flux_kp = PI_FLUX_KP;
      USER_CONFIG.pi_flux_ki = PI_FLUX_KI;
      USER_CONFIG.pi_flux_scale_kpki = PI_FLUX_SCALE_KPKI;
      USER_CONFIG.pi_flux_ik_limit_max = PI_FLUX_IK_LIMIT_MAX;
      USER_CONFIG.pi_flux_ik_limit_min = PI_FLUX_IK_LIMIT_MIN;
      USER_CONFIG.pi_flux_uk_limit_max = PI_FLUX_UK_LIMIT_MAX;
      USER_CONFIG.pi_flux_uk_limit_min = PI_FLUX_UK_LIMIT_MIN;

      USER_CONFIG.pi_torque_kp = PI_TORQUE_KP;
      USER_CONFIG.pi_torque_ki = PI_TORQUE_KI;
      USER_CONFIG.pi_torque_scale_kpki = PI_TORQUE_SCALE_KPKI;
      USER_CONFIG.pi_torque_ik_limit_max = PI_TORQUE_IK_LIMIT_MAX;
      USER_CONFIG.pi_torque_ik_limit_min = PI_TORQUE_IK_LIMIT_MIN;
      USER_CONFIG.pi_torque_uk_limit_max = PI_TORQUE_UK_LIMIT_MAX;
      USER_CONFIG.pi_torque_uk_limit_min = PI_TORQUE_UK_LIMIT_MIN;

      USER_CONFIG.pi_speed_kp = PI_SPEED_KP;
      USER_CONFIG.pi_speed_ki = PI_SPEED_KI;
      USER_CONFIG.pi_speed_scale_kpki = PI_SPEED_SCALE_KPKI;
      USER_CONFIG.pi_speed_ik_limit_max = PI_SPEED_IK_LIMIT_MAX;
      USER_CONFIG.pi_speed_ik_limit_min = PI_SPEED_IK_LIMIT_MIN;
      USER_CONFIG.pi_speed_uk_limit_max = PI_SPEED_UK_LIMIT_MAX;
      USER_CONFIG.pi_speed_uk_limit_min = PI_SPEED_UK_LIMIT_MIN;

      /* Update to flash */
      PMSM_FOC_ucProbe_WriteFlash();
      /* Enable IRQ */
      NVIC_EnableIRQ(CCU80_0_IRQn);
      /* Command is processed */
      ucProbe_cmd = 0;
      break;

    case UCPROBE_CMD_READ_DFLT_PIPARAM_FLASH:

      PMSM_FOC_SPEED_PI.kp = PI_SPEED_KP;
      PMSM_FOC_SPEED_PI.ki = PI_SPEED_KI;
      PMSM_FOC_SPEED_PI.scale_kpki = PI_SPEED_SCALE_KPKI;
      PMSM_FOC_SPEED_PI.ik_limit_max = PI_SPEED_IK_LIMIT_MAX;
      PMSM_FOC_SPEED_PI.ik_limit_min = PI_SPEED_IK_LIMIT_MIN;
      PMSM_FOC_SPEED_PI.uk_limit_max = PI_SPEED_UK_LIMIT_MAX;
      PMSM_FOC_SPEED_PI.uk_limit_min = PI_SPEED_UK_LIMIT_MIN;

      PMSM_FOC_TORQUE_PI.kp = PI_TORQUE_KP;
      PMSM_FOC_TORQUE_PI.ki = PI_TORQUE_KI;
      PMSM_FOC_TORQUE_PI.scale_kpki = PI_TORQUE_SCALE_KPKI;
      PMSM_FOC_TORQUE_PI.ik_limit_max = PI_TORQUE_IK_LIMIT_MAX;
      PMSM_FOC_TORQUE_PI.ik_limit_min = PI_TORQUE_IK_LIMIT_MIN;
      PMSM_FOC_TORQUE_PI.uk_limit_max = PI_TORQUE_UK_LIMIT_MAX;
      PMSM_FOC_TORQUE_PI.uk_limit_min = PI_TORQUE_UK_LIMIT_MIN;

      PMSM_FOC_FLUX_PI.kp = PI_FLUX_KP;
      PMSM_FOC_FLUX_PI.ki = PI_FLUX_KI;
      PMSM_FOC_FLUX_PI.scale_kpki = PI_FLUX_SCALE_KPKI;
      PMSM_FOC_FLUX_PI.ik_limit_max = PI_FLUX_IK_LIMIT_MAX;
      PMSM_FOC_FLUX_PI.ik_limit_min = PI_FLUX_IK_LIMIT_MIN;
      PMSM_FOC_FLUX_PI.uk_limit_max = PI_FLUX_UK_LIMIT_MAX;
      PMSM_FOC_FLUX_PI.uk_limit_min = PI_FLUX_UK_LIMIT_MIN;


      /* Command is processed */
      ucProbe_cmd = 0;
      break;

    case UCPROBE_CMD_MOTOR_STOP:
      /* set a CTRAP to stop motor*/
      CCU8_SLICE_PHASE_U->INS2 &= (0xFFFFFBFF); //to set CTRAP trigger level EV2LM to active high bit24
      CCU8_SLICE_PHASE_V->INS2 &= (0xFFFFFBFF); //to set CTRAP trigger level EV2LM to active high bit24
      CCU8_SLICE_PHASE_W->INS2 &= (0xFFFFFBFF); //to set CTRAP trigger level EV2LM to active hign bit24
      PMSM_FOC_CTRL.motor_start_flag = 0;
      PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_STOP_MOTOR;
      break;

    default:
      break;
  }
}

#endif //#if(USER_UCPROBE_GUI == ENABLED)
