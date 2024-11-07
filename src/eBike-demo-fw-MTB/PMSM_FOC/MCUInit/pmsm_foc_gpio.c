/**
 * @file pmsm_foc_gpio.c
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
#include <PMSM_FOC/Configuration/pmsm_foc_mcuhw_params.h>
#include "../MCUInit/pmsm_foc_gpio.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/**
 *  Data Structure initialization - GPIO Configuration for Gate Driver enable pin .
 */
XMC_GPIO_CONFIG_t Inverter_Enable_Pin_Config  =
{
    .mode            = (XMC_GPIO_MODE_t)XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level    = (XMC_GPIO_OUTPUT_LEVEL_t)USER_INVERTER_DISABLE_LEVEL
};

/**
 *  Data Structure initialization - GPIO Configuration for Brake enable pin .
 */
#ifdef nBRAKE_PIN
XMC_GPIO_CONFIG_t Brake_Enable_Pin_Config  =
{
    .mode            = (XMC_GPIO_MODE_t)XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level    = (XMC_GPIO_OUTPUT_LEVEL_t)USER_BRAKE_DISABLE_LEVEL
};
#endif

XMC_GPIO_CONFIG_t DRIVER_CS_AZ_Pin_Config  =
{
    .mode            = (XMC_GPIO_MODE_t)XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2,
    .output_level    = (XMC_GPIO_OUTPUT_LEVEL_t)XMC_GPIO_OUTPUT_LEVEL_HIGH
};

#ifdef nFAULT_PIN
XMC_GPIO_CONFIG_t nFault_Pin_Config  =
{
    .mode            = (XMC_GPIO_MODE_t)XMC_GPIO_MODE_INPUT_TRISTATE,
    .input_hysteresis= XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};
#endif

/**
 *  Data Structure initialization - GPIO Configuration for motor Direction pin .
 */
XMC_GPIO_CONFIG_t Motor_Dir_Pin_Config  =
{
    .mode            = (XMC_GPIO_MODE_t)XMC_GPIO_MODE_INPUT_PULL_DOWN,
    .input_hysteresis= XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};

/** GPIO Init handle for Phase U High Pin */
const XMC_GPIO_CONFIG_t PMSM_FOC_GPIO_PhU_High_Config =
{
    .mode             = PHASE_U_HS_ALT_SELECT,
    #if(USER_CCU8_PASSIVE_LEVEL_OUT0 == CCU8_PASSIVE_LOW)
    .output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW,
    #else
    .output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
    #endif
    .input_hysteresis = (XMC_GPIO_INPUT_HYSTERESIS_t)0
};

/** GPIO Init handle for Phase U Low Pin */
const XMC_GPIO_CONFIG_t PMSM_FOC_GPIO_PhU_Low_Config =
{
    .mode             = PHASE_U_LS_ALT_SELECT,
    #if(USER_CCU8_PASSIVE_LEVEL_OUT0 == CCU8_PASSIVE_LOW)
    .output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW,
    #else
    .output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
    #endif
    .input_hysteresis = (XMC_GPIO_INPUT_HYSTERESIS_t)0
};

/** GPIO Init handle for Phase V High Pin */
const XMC_GPIO_CONFIG_t PMSM_FOC_GPIO_PhV_High_Config =
{
    .mode             = PHASE_V_HS_ALT_SELECT,
    #if(USER_CCU8_PASSIVE_LEVEL_OUT0 == CCU8_PASSIVE_LOW)
    .output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW,
    #else
    .output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
    #endif
   .input_hysteresis = (XMC_GPIO_INPUT_HYSTERESIS_t)0
};

/** GPIO Init handle for Phase V Low Pin */
const XMC_GPIO_CONFIG_t PMSM_FOC_GPIO_PhV_Low_Config =
{
    .mode             = PHASE_V_LS_ALT_SELECT,
    #if(USER_CCU8_PASSIVE_LEVEL_OUT0 == CCU8_PASSIVE_LOW)
    .output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW,
    #else
    .output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
    #endif
    .input_hysteresis = (XMC_GPIO_INPUT_HYSTERESIS_t)0
};

/** GPIO Init handle for Phase W High Pin */
const XMC_GPIO_CONFIG_t PMSM_FOC_GPIO_PhW_High_Config =
{
    .mode             = PHASE_W_HS_ALT_SELECT,
    #if(USER_CCU8_PASSIVE_LEVEL_OUT0 == CCU8_PASSIVE_LOW)
    .output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW,
    #else
    .output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
    #endif
    .input_hysteresis = (XMC_GPIO_INPUT_HYSTERESIS_t)0
};

/** GPIO Init handle for Phase W Low Pin */
const XMC_GPIO_CONFIG_t PMSM_FOC_GPIO_PhW_Low_Config =
{
    .mode             = PHASE_W_LS_ALT_SELECT,
    #if(USER_CCU8_PASSIVE_LEVEL_OUT0 == CCU8_PASSIVE_LOW)
    .output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW,
    #else
    .output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
    #endif
    .input_hysteresis = (XMC_GPIO_INPUT_HYSTERESIS_t)0
};

#if(RESET_BMI_ENABLE == 1)
/** GPIO Init handle for BMI reset after MCU is set to production mode */
XMC_GPIO_CONFIG_t Reset_BMI_Pin  =
{
    .mode            = (XMC_GPIO_MODE_t)XMC_GPIO_MODE_INPUT_PULL_DOWN,
    .input_hysteresis= XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};
#endif

/** GPIO Init handle for hall sensor input pins */
const XMC_GPIO_CONFIG_t GPIO_Hall_Config  =
{
  .mode            = (XMC_GPIO_MODE_t)XMC_GPIO_MODE_INPUT_TRISTATE,
  .output_level    = (XMC_GPIO_OUTPUT_LEVEL_t)XMC_GPIO_OUTPUT_LEVEL_LOW,
  .input_hysteresis = (XMC_GPIO_INPUT_HYSTERESIS_t)0
};

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/* API to initialize GPIO pins used */
void PMSM_FOC_GPIO_Init(void)
{
    #ifdef INVERTER_EN_PIN
    /* Gate driver enable/disable pin */
    XMC_GPIO_Init(INVERTER_EN_PIN, &Inverter_Enable_Pin_Config);
    #endif

    #ifdef nBRAKE_PIN
    /* Brake enable/disable pin */
    XMC_GPIO_Init(nBRAKE_PIN, &Brake_Enable_Pin_Config);
    #endif

    #ifdef DRIVER_CS_AZ_PIN
    /* 6EDL7141 current sense amplifier auto zero pin  */
    XMC_GPIO_Init(DRIVER_CS_AZ_PIN, &DRIVER_CS_AZ_Pin_Config);
    #endif

    #ifdef nFAULT_PIN
    /* 6EDL7141 nFault pin  */
    XMC_GPIO_Init(nFAULT_PIN, &nFault_Pin_Config);
    #endif

    #ifdef MOTOR_DIR_INPUT_PIN
    /* motor direction control PIN */
    XMC_GPIO_Init(MOTOR_DIR_INPUT_PIN, &Motor_Dir_Pin_Config);
    #endif

    /* Phase U High Side PWM Output */
    XMC_GPIO_Init(PHASE_U_HS_PIN, &PMSM_FOC_GPIO_PhU_High_Config);
    /* Phase U Low Side PWM Output */
    XMC_GPIO_Init(PHASE_U_LS_PIN, &PMSM_FOC_GPIO_PhU_Low_Config);

    /* Phase V High Side PWM Output */
    XMC_GPIO_Init(PHASE_V_HS_PIN, &PMSM_FOC_GPIO_PhV_High_Config);
    /* Phase V Low Side PWM Output */
    XMC_GPIO_Init(PHASE_V_LS_PIN, &PMSM_FOC_GPIO_PhV_Low_Config);

    /* Phase W High Side PWM Output */
    XMC_GPIO_Init(PHASE_W_HS_PIN, &PMSM_FOC_GPIO_PhW_High_Config);
    /* Phase W Low Side PWM Output */
    XMC_GPIO_Init(PHASE_W_LS_PIN, &PMSM_FOC_GPIO_PhW_Low_Config);


    /* Debugging only - ADC trigger */
    //XMC_GPIO_SetMode(PHASE_ADC_TRIG_PIN, PHASE_ADC_TRIG_PIN_ALT_SELECT);

    #ifdef TRAP_PIN
    /* Trap input, internal pull-up */
    XMC_GPIO_SetMode(TRAP_PIN, XMC_GPIO_MODE_INPUT_PULL_UP);
    #endif

    /* Fault LED indicator */
    #ifdef FAULT_LED3
    XMC_GPIO_SetMode (FAULT_LED3,XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetOutputHigh(FAULT_LED3);
    #endif

    /* Flux weakening Active indicator */
    #ifdef FW_ACTIVE_LED4
    XMC_GPIO_SetMode (FW_ACTIVE_LED4,XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetOutputHigh(FW_ACTIVE_LED4);
    #endif

    /* Test pin - GPIO */
    #ifdef TEST_PIN
	#if(RESET_BMI_ENABLE == 1)
    XMC_GPIO_Init(TEST_PIN, &Reset_BMI_Pin);
	#else
    XMC_GPIO_SetMode (TEST_PIN,XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetOutputLow(TEST_PIN);
	#endif
    #endif

	#ifdef HALL_ISR_DEBUG_PIN
    XMC_GPIO_SetMode (HALL_ISR_DEBUG_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetOutputLow(HALL_ISR_DEBUG_PIN);
	#endif

//#if(E_BIKE_REF == ENABLED)
//    XMC_GPIO_SetMode (V_brake_switch, XMC_GPIO_MODE_INPUT_PULL_DOWN);
//#endif
}

/*
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Configures GPIO as input pins to the POSIF to sense HALL feedback.
 */
void pmsm_foc_gpio_hall_init(void)
{
  XMC_GPIO_Init(HALL_0_PIN, &GPIO_Hall_Config);
  XMC_GPIO_Init(HALL_1_PIN, &GPIO_Hall_Config);
  XMC_GPIO_Init(HALL_2_PIN, &GPIO_Hall_Config);
}

