/**
 * @file pmsm_foc_ccu4.c
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
#include <PMSM_FOC/MCUInit/pmsm_foc_ccu4.h>
#include "../Configuration/pmsm_foc_mcuhw_params.h"
#include "../Configuration/pmsm_foc_config.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/

#if((DEBUG_PWM_0_ENABLE == ENABLED) || (DEBUG_PWM_1_ENABLE == ENABLED))
/**
 *  Data Structure initialization - CCU4 Slice Configuration.
 */
XMC_CCU4_SLICE_COMPARE_CONFIG_t Debug_CCU4_SliceConfig =
{
    .timer_mode = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
    .monoshot = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear = 0U,
    .dither_timer_period = 0U,
    .dither_duty_cycle = 0U,
    .prescaler_mode = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_enable = 0U,
    .prescaler_initval = 0U, /* divide by 1--> 96MHz (as fast as possible) */
    .float_limit = 0U,
    .dither_limit = 0U,
    .passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH,
    .timer_concatenation = false
};
#endif /* ((DEBUG_PWM_0_ENABLE) | (DEBUG_PWM_1_ENABLE)) */

#if(MOTOR_CTRL_SCHEME == PMSM_HALL_FOC)
/*************************************  HALL SENSOR  ******************************************************************/
/****************************Start: Phase Delay Slice *****************************************************************/
/**
 * CCU4 slice used for blanking delay and phase delay
 * Single shot mode, Compare mode
 * Starts with any hall edge detected (POSIF.out0)
 * Normal pre-scaler mode
 * Pre-scaler: XMC_CCU4_SLICE_PRESCALER_32
 */
XMC_CCU4_SLICE_COMPARE_CONFIG_t DELAY_0_compare_config =
{
	.timer_mode = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
	.monoshot = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_SINGLE,
	.shadow_xfer_clear = false,
	.dither_timer_period = false,
	.dither_duty_cycle = false,
	.prescaler_mode = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
	.mcm_enable = false,
	.prescaler_initval = XMC_CCU4_SLICE_PRESCALER_2, /* in this case, 96MHz/2= 48MHz*/
	.float_limit = XMC_CCU4_SLICE_PRESCALER_32768,
	.dither_limit = 0U,
	.passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
	.timer_concatenation = false,
};

/* Capture Slice configuration */
XMC_CCU4_SLICE_CAPTURE_CONFIG_t CAPTURE_0_capture_config =
{
	.fifo_enable = false,
	.timer_clear_mode = XMC_CCU4_SLICE_TIMER_CLEAR_MODE_CAP_LOW,
	.same_event = false,
	.ignore_full_flag = true,//MTB code: true,  UM P22-40
	.prescaler_mode = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
	.prescaler_initval = HALL_CLK_PRE_SCALE, /* in this case, fCCU4 = 96Mhz/32 = 3MHz */
	.float_limit = XMC_CCU4_SLICE_PRESCALER_32768,
	.timer_concatenation = false,
};

/* Event 0: Start the slice on rising edge of POSIF0.OUT0 */
XMC_CCU4_SLICE_EVENT_CONFIG_t start_event0_config = //off time capture
{
  .mapped_input = XMC_CCU4_SLICE_INPUT_AE, //CAPTURE on POSIF0.OUT0
  .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
  .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
  .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED
};

/* Event 0: Capture on rising edge of POSIF0.OUT1 */
XMC_CCU4_SLICE_EVENT_CONFIG_t capture_event0_config = //off time capture
{
  .mapped_input = XMC_CCU4_SLICE_INPUT_AF, //CAPTURE on POSIF0.OUT1
  .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
  .level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
  .duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED
};

/********HALL SENSOR**********/
#endif

/**
 *  Data Structure initialization - CCU4 Slice Configuration for AZ Signal generation.
 */
XMC_CCU4_SLICE_COMPARE_CONFIG_t GDRIVER_AZ_CCU4_SliceConfig =
{
    .timer_mode = XMC_CCU4_SLICE_TIMER_COUNT_MODE_CA,
    .monoshot = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear = 0U,
    .dither_timer_period = 0U,
    .dither_duty_cycle = 0U,
    .prescaler_mode = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_enable = 0U,
    .prescaler_initval = 0U, /* divide by 1--> 96MHz (as fast as possible) */
    .float_limit = 0U,
    .dither_limit = 0U,
    .passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
    .timer_concatenation = false
};

XMC_CCU4_SLICE_EVENT_CONFIG_t CCU4_Input_Event0_Config =
{
    .mapped_input = XMC_CCU4_SLICE_INPUT_AI,
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE
};

XMC_CCU4_SLICE_EVENT_CONFIG_t CCU4_Input_Event1_Config =
{
    .mapped_input = XMC_CCU4_SLICE_INPUT_AI,
    .edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_FALLING_EDGE
};
/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

/* API to initialize CCU4 module to outputs debug information. Outputs P1. 0, P0.4, P1.2, P1.3 */
void PMSM_FOC_CCU4_Init(void)
{
	#ifdef DRIVER_CS_AZ_PIN
    /* Initialization for CCU4 timer for AZ signal generation for the gate driver */
    XMC_CCU4_Init(GDRIVER_AZ_PWM_CCU4_MODULE, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
    XMC_CCU4_SetModuleClock(GDRIVER_AZ_PWM_CCU4_MODULE, XMC_CCU4_CLOCK_SCU);

    /* Get the slice out of idle mode */
    XMC_CCU4_EnableClock(GDRIVER_AZ_PWM_CCU4_MODULE, GDRIVER_AZ_PWM_SLICE_NUM);
    /* Initialize the Slice */
    XMC_CCU4_SLICE_CompareInit(GDRIVER_AZ_PWM_SLICE, &GDRIVER_AZ_CCU4_SliceConfig);

    XMC_CCU4_SLICE_SetTimerCompareMatch(GDRIVER_AZ_PWM_SLICE, ((CCU8_PERIOD_REG>>1) + ADC_TRIGGER_DELAY));
    XMC_CCU4_SLICE_SetTimerPeriodMatch(GDRIVER_AZ_PWM_SLICE, (CCU8_PERIOD_REG>>1) + ADC_TRIGGER_DELAY + SVPWM_T0_THRESHOLD);

    XMC_CCU4_SLICE_SetTimerValue(GDRIVER_AZ_PWM_SLICE, 0);
    XMC_CCU4_EnableShadowTransfer(GDRIVER_AZ_PWM_CCU4_MODULE, (uint32_t) GDRIVER_AZ_PWM_SLICE_SHADOW_TRANS_ENABLE_Msk);

    /* Slice timer starts in synchronization with CCU8 PWM generation modules */
    XMC_CCU4_SLICE_ConfigureEvent(GDRIVER_AZ_PWM_SLICE, XMC_CCU4_SLICE_EVENT_0, &CCU4_Input_Event0_Config);
    /* Configure CCU4 timer start event */
    XMC_CCU4_SLICE_StartConfig(GDRIVER_AZ_PWM_SLICE, XMC_CCU4_SLICE_EVENT_0, XMC_CCU4_SLICE_START_MODE_TIMER_START_CLEAR);

    /* Slice timer starts in synchronization with CCU8 PWM generation modules */
    XMC_CCU4_SLICE_ConfigureEvent(GDRIVER_AZ_PWM_SLICE, XMC_CCU4_SLICE_EVENT_1, &CCU4_Input_Event1_Config);
    /* Configure CCU4 timer stop event */
    XMC_CCU4_SLICE_StopConfig(GDRIVER_AZ_PWM_SLICE, XMC_CCU4_SLICE_EVENT_1, XMC_CCU4_SLICE_END_MODE_TIMER_STOP_CLEAR);

    /* Disable Global Start Control CCU40 */
    XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC40_Msk);
	#endif

  	#if(DEBUG_PWM_0_ENABLE == ENABLED)
//    XMC_CCU4_Init(DEBUG_PWM_CCU4_MODULE, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);

 	/* Init Debug PWM Slice */
  	/* Get the slice out of idle mode */
  	XMC_CCU4_EnableClock(DEBUG_PWM_CCU4_MODULE, DEBUG_PWM_0_SLICE_NUM);
  	/* Initialize the Slice */
  	XMC_CCU4_SLICE_CompareInit(DEBUG_PWM_0_SLICE, &Debug_CCU4_SliceConfig);

    XMC_CCU4_SLICE_SetTimerCompareMatch( DEBUG_PWM_0_SLICE, DEBUG_PWM_50_PERCENT_DC_CNTS);
    XMC_CCU4_SLICE_SetTimerPeriodMatch( DEBUG_PWM_0_SLICE, DEBUG_PWM_PERIOD_CNTS);
    XMC_CCU4_SLICE_SetTimerValue( DEBUG_PWM_0_SLICE, 0U);
    XMC_CCU4_EnableShadowTransfer(DEBUG_PWM_CCU4_MODULE, (uint32_t) DEBUG_PWM_0_SLICE_SHADOW_TRANS_ENABLE_Msk);

    /* Setup the I/O Pin */
    XMC_GPIO_SetMode(DEBUG_PWM_0_PORT, DEBUG_PWM_0_PIN, DEBUG_PWM_0_ALT_OUT);
    XMC_CCU4_SLICE_StartTimer(DEBUG_PWM_0_SLICE);
    #endif /* (DEBUG_PWM_0_ENABLE == 1) */

  	#if(DEBUG_PWM_1_ENABLE == ENABLED)
    /* Init Debug PWM Slice */
    /* Get the slice out of idle mode */
    XMC_CCU4_EnableClock(DEBUG_PWM_CCU4_MODULE, DEBUG_PWM_1_SLICE_NUM);
    /* Initialize the Slice */
    XMC_CCU4_SLICE_CompareInit(DEBUG_PWM_1_SLICE, &Debug_CCU4_SliceConfig);

    XMC_CCU4_SLICE_SetTimerCompareMatch( DEBUG_PWM_1_SLICE, DEBUG_PWM_50_PERCENT_DC_CNTS);
    XMC_CCU4_SLICE_SetTimerPeriodMatch( DEBUG_PWM_1_SLICE, DEBUG_PWM_PERIOD_CNTS);
    XMC_CCU4_SLICE_SetTimerValue(DEBUG_PWM_1_SLICE, 0U);
    XMC_CCU4_EnableShadowTransfer(DEBUG_PWM_CCU4_MODULE, (uint32_t) DEBUG_PWM_1_SLICE_SHADOW_TRANS_ENABLE_Msk);

    /* Setup the I/O Pin */
    XMC_GPIO_SetMode(DEBUG_PWM_1_PORT, DEBUG_PWM_1_PIN, DEBUG_PWM_1_ALT_OUT);

    XMC_CCU4_SLICE_StartTimer(DEBUG_PWM_1_SLICE);
    #endif /* (DEBUG_PWM_1_ENABLE == 1) */

    XMC_CCU4_StartPrescaler(DEBUG_PWM_CCU4_MODULE);
}

#if(MOTOR_CTRL_SCHEME == PMSM_HALL_FOC)
void pmsm_foc_ccu4_hall_init()
{
  XMC_CCU4_Init(HALL_CCU4_MODULE, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
  XMC_CCU4_StartPrescaler(HALL_CCU4_MODULE);
  XMC_CCU4_SLICE_CompareInit(HALL_DELAY_SLICE, &DELAY_0_compare_config);
  XMC_CCU4_SLICE_SetTimerCompareMatch(HALL_DELAY_SLICE, 50U);   /* 5 counts with 48MHz ~ 0.1us*/
  XMC_CCU4_SLICE_SetTimerPeriodMatch(HALL_DELAY_SLICE, 100U);
  XMC_CCU4_SetMultiChannelShadowTransferMode(HALL_CCU4_MODULE, XMC_CCU4_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE0);
  XMC_CCU4_SLICE_DisableCascadedShadowTransfer(HALL_DELAY_SLICE);
  XMC_CCU4_EnableShadowTransfer(HALL_CCU4_MODULE,
  XMC_CCU4_SHADOW_TRANSFER_SLICE_0 |
  XMC_CCU4_SHADOW_TRANSFER_DITHER_SLICE_0 |
  XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_0 );
  XMC_CCU4_SLICE_ConfigureEvent(HALL_DELAY_SLICE, XMC_CCU4_SLICE_EVENT_0, &start_event0_config);
  XMC_CCU4_SLICE_StartConfig(HALL_DELAY_SLICE, XMC_CCU4_SLICE_EVENT_0, XMC_CCU4_SLICE_START_MODE_TIMER_START_CLEAR);
  XMC_CCU4_EnableClock(HALL_CCU4_MODULE, HALL_DELAY_SLICE_NUM);
  XMC_CCU4_SLICE_SetTimerValue(HALL_DELAY_SLICE, 0U);

  XMC_CCU4_SLICE_CaptureInit(HALL_CAPTURE_SLICE, &CAPTURE_0_capture_config);
  XMC_CCU4_SLICE_SetTimerPeriodMatch(HALL_CAPTURE_SLICE, 65535U);
  XMC_CCU4_SetMultiChannelShadowTransferMode(HALL_CCU4_MODULE, XMC_CCU4_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE1);
  XMC_CCU4_SLICE_DisableCascadedShadowTransfer(HALL_CAPTURE_SLICE);
  XMC_CCU4_EnableShadowTransfer(HALL_CCU4_MODULE,
  XMC_CCU4_SHADOW_TRANSFER_SLICE_1 |
  XMC_CCU4_SHADOW_TRANSFER_DITHER_SLICE_1 |
  XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_1 );
  XMC_CCU4_SLICE_ConfigureEvent(HALL_CAPTURE_SLICE, XMC_CCU4_SLICE_EVENT_0, &capture_event0_config);
  XMC_CCU4_SLICE_Capture0Config(HALL_CAPTURE_SLICE, XMC_CCU4_SLICE_EVENT_0);
  XMC_CCU4_EnableClock(HALL_CCU4_MODULE, HALL_CAPTURE_SLICE_NUM);
  XMC_CCU4_SLICE_SetTimerValue(HALL_CAPTURE_SLICE, 0U);

}
#endif
