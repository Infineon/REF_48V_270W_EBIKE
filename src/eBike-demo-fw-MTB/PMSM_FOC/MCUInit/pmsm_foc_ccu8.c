/**
 * @file pmsm_foc_ccu8.c
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
#include <PMSM_FOC/MCUInit/pmsm_foc_ccu8.h>
#include "uCProbe/uCProbe.h"
#include "xmc_common.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/**
 *  Data Structure initialization - CC8 Slices configuration for all three pwm phase.
 */
XMC_CCU8_SLICE_COMPARE_CONFIG_t CCU8_Phase_Init =
{
    .timer_mode           = (uint32_t)XMC_CCU8_SLICE_TIMER_COUNT_MODE_EA,
    .monoshot             = (uint32_t)XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear    = 1U,     /* CLST = 1b, Enable a shadow transfer whenever a clearing action by SW or external event */
    .dither_timer_period  = 0U,
    .dither_duty_cycle    = 0U,
    .prescaler_mode       = (uint8_t)XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_ch1_enable       = 0U,
    .mcm_ch2_enable       = 0U,
    .slice_status         = (uint8_t)XMC_CCU8_SLICE_STATUS_CHANNEL_1,
    .passive_level_out0   = USER_CCU8_PASSIVE_LEVEL_OUT0,
    .passive_level_out1   = USER_CCU8_PASSIVE_LEVEL_OUT1,
    .asymmetric_pwm       = (uint8_t)true,
    #if(UC_SERIES == XMC14)
    .selector_out0         = XMC_CCU8_SOURCE_OUT0_INV_ST1,
    .selector_out1         = XMC_CCU8_SOURCE_OUT1_ST1,
    .selector_out2         = DISABLED,
    .selector_out3         = DISABLED,
    #else
    .invert_out0          = 1U,
    .invert_out1          = 0U,
    #endif
    .prescaler_initval    = (uint8_t)0,
    .float_limit          = 0U,
    .dither_limit         = 0U,
    .timer_concatenation  = 0U
};

/**
 *  Data Structure initialization - CC8 Slice configuration to trigger ADC conversion start.
 */
XMC_CCU8_SLICE_COMPARE_CONFIG_t CCU8_VADC_Trigger_Init =
{
    .timer_mode           = (uint32_t)XMC_CCU8_SLICE_TIMER_COUNT_MODE_EA,
    .monoshot             = (uint32_t)XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT,
    .shadow_xfer_clear    = 1U,
    .dither_timer_period  = 0U,
    .dither_duty_cycle    = 0U,
    .prescaler_mode       = (uint8_t)XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
    .mcm_ch1_enable       = 0U,
    .mcm_ch2_enable       = 0U,
    .slice_status         = (uint8_t)XMC_CCU8_SLICE_STATUS_CHANNEL_1,
    .passive_level_out0   = 0,
    .passive_level_out1   = 0,
    .asymmetric_pwm       = (uint8_t)true,
    #if(UC_SERIES == XMC14)
    .selector_out0         = XMC_CCU8_SOURCE_OUT0_ST1,
    .selector_out1         = XMC_CCU8_SOURCE_OUT1_INV_ST1,
    .selector_out2         = DISABLED,
    .selector_out3         = DISABLED,
    #else
    .invert_out0          = 0U,
    .invert_out1          = 1U,
    #endif
    .prescaler_initval    = (uint8_t)0,
    .float_limit          = 0U,
    .dither_limit         = 0U,
    .timer_concatenation  = 0U
};

/**
 *  Data Structure initialization - CC8 Slice dead time configuration.
 */

XMC_CCU8_SLICE_DEAD_TIME_CONFIG_t CCU8_Deadtime_Config =
 {
       {
          .enable_dead_time_channel1 = 1U, /**< Enable dead time for Compare Channel-1 */
          .enable_dead_time_channel2 = 1U, /**< Enable dead time for Compare Channel-2 */
          .channel1_st_path = 1U,          /**< Should dead time be applied to ST output of Compare Channel-1? */
          .channel1_inv_st_path = 1U,      /**< Should dead time be applied to inverse ST output of Compare Channel-1? */
          .channel2_st_path = 1U,          /**< Should dead time be applied to ST output of Compare Channel-2? */
          .channel2_inv_st_path = 1U,      /**< Should dead time be applied to inverse ST output of Compare Channel-2? */
          .div = 0U,                       /**< Dead time prescaler divider value.*/
       },

      {
          .channel1_st_rising_edge_counter = CCU8_DEADTIME_RISE, /**< CH1 dead time rising edge */
          .channel1_st_falling_edge_counter = CCU8_DEADTIME_FALL , /**< CH1 dead time falling edge */
      },

      {
          .channel2_st_rising_edge_counter = CCU8_DEADTIME_RISE, /**< CH2 dead time rising edge */
          .channel2_st_falling_edge_counter = CCU8_DEADTIME_FALL , /**< CH2 dead time falling edge */
      },
};

/**
 *  Data Structure initialization - CC8 Slice event 0 set to sync start.
 */
XMC_CCU8_SLICE_EVENT_CONFIG_t CCU8_Input_Event0_Config =
{
    #if(UC_SERIES == XMC14)
    .mapped_input = XMC_CCU8_SLICE_INPUT_AH,
    #else
    .mapped_input = XMC_CCU8_SLICE_INPUT_H,
    #endif
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE
};

/**
 *  Data Structure initialization - CC8 Slice event 1 set to sync stop.
 */
XMC_CCU8_SLICE_EVENT_CONFIG_t CCU8_Input_Event1_Config =
{
    #if(UC_SERIES == XMC14)
    .mapped_input = XMC_CCU8_SLICE_INPUT_AH,
    #else
    .mapped_input = XMC_CCU8_SLICE_INPUT_H,
    #endif
    .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_FALLING_EDGE
};

/**
 *  Data Structure initialization - CC8 Slice trap event configuration.
 */
XMC_CCU8_SLICE_EVENT_CONFIG_t CCU8_Input_trap_config =
{
#if(UC_SERIES == XMC14)
  .mapped_input = XMC_CCU8_SLICE_INPUT_AA,
#else
  .mapped_input = XMC_CCU8_SLICE_INPUT_A,
#endif
  .level = USER_CCU8_INPUT_TRAP_LEVEL,
  .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
  .duration = XMC_CCU8_SLICE_EVENT_FILTER_7_CYCLES
};

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/* API to initialize PWM duty cycle to zero. */
void PMSM_FOC_CCU8_SetDutyZero(void)
{
    /* Set High side PWM duty to zero. -  Inverted Configuration */
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_SLICE_PHASE_U, 0);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_SLICE_PHASE_U, CCU8_PERIOD_REG+1);

    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_SLICE_PHASE_V, 0);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_SLICE_PHASE_V, CCU8_PERIOD_REG+1);

    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_SLICE_PHASE_W, 0);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_SLICE_PHASE_W, CCU8_PERIOD_REG+1);

    /* Trigger shadow transfer */
    CCU8_MODULE->GCSS |= (uint32_t) (XMC_CCU8_SHADOW_TRANSFER_SLICE_0 | XMC_CCU8_SHADOW_TRANSFER_SLICE_1 |
                          XMC_CCU8_SHADOW_TRANSFER_SLICE_2);
}
/* API to initialize CCU8 module for 3 phase pwm generation. Trap functionality enable */
void PMSM_FOC_CCU8_Init(void)
{
    XMC_CCU8_DisableModule(CCU8_MODULE);
    XMC_CCU8_Init(CCU8_MODULE, XMC_CCU8_SLICE_MCMS_ACTION_TRANSFER_PR_CR);

    XMC_CCU8_SLICE_CompareInit(CCU8_SLICE_PHASE_U, &CCU8_Phase_Init);
    XMC_CCU8_SLICE_CompareInit(CCU8_SLICE_PHASE_V, &CCU8_Phase_Init);
    XMC_CCU8_SLICE_CompareInit(CCU8_SLICE_PHASE_W, &CCU8_Phase_Init);
    XMC_CCU8_SLICE_CompareInit(CCU8_SLICE_ADC_TR, &CCU8_VADC_Trigger_Init);

    /* Set High side PWM duty to zero */
    PMSM_FOC_CCU8_SetDutyZero();

    /* Timer Period Shadow Value */
    XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU8_SLICE_PHASE_U, CCU8_PERIOD_REG);
    XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU8_SLICE_PHASE_V, CCU8_PERIOD_REG);
    XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU8_SLICE_PHASE_W, CCU8_PERIOD_REG);
    XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU8_SLICE_ADC_TR, CCU8_PERIOD_REG);

    /* For ADC trigger  - Pulse created with 3 counts */
//    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_SLICE_ADC_TR, ((CCU8_PERIOD_REG >> 1U) + ADC_TRIGGER_DELAY));
//    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_SLICE_ADC_TR, ((CCU8_PERIOD_REG >> 1U) + ADC_TRIGGER_DELAY +3));
//    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_SLICE_ADC_TR, ((CCU8_PERIOD_REG >> 1U) + GDRIVER_IC_DELAY));
//    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_SLICE_ADC_TR, ((CCU8_PERIOD_REG >> 1U) + GDRIVER_IC_DELAY +20));
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_SLICE_ADC_TR, (CCU8_PERIOD_REG >> 1U));
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_SLICE_ADC_TR, ((CCU8_PERIOD_REG >> 1U) + 1));


    /* Dead time enabled, No prescaler for the dead time counters */
    /* Dead time could cause motor phase current distortion, especially at low motor speed */
    XMC_CCU8_SLICE_DeadTimeInit(CCU8_SLICE_PHASE_U, &CCU8_Deadtime_Config);
    XMC_CCU8_SLICE_DeadTimeInit(CCU8_SLICE_PHASE_V, &CCU8_Deadtime_Config);
    XMC_CCU8_SLICE_DeadTimeInit(CCU8_SLICE_PHASE_W, &CCU8_Deadtime_Config);
//    XMC_CCU8_SLICE_DeadTimeInit(CCU8_SLICE_ADC_TR, &CCU8_Deadtime_Config);


    /* Enable shadow transfer for slice 0,1,2,3 for CCU80 Kernel */
    XMC_CCU8_EnableShadowTransfer(CCU8_MODULE, ((uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_0 |
                  (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_1 |
                  (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_2 |
                  (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_3));

    /*  Enable interrupt for CCU80 Period Match Enable */
    XMC_CCU8_SLICE_EnableEvent(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
    XMC_CCU8_SLICE_SetInterruptNode(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU8_SLICE_SR_ID_0);

    /* CCU80_0_IRQn = 25. CCU80 SR0 Interrupt has lower priority than ADC IRQ. CCU80_0_IRQHandler */
    NVIC_SetPriority (CCU80_0_IRQn, PMSM_FOC_FCL_NVIC_PRIO);
    XMC_SCU_SetInterruptControl(CCU80_0_IRQn, XMC_SCU_IRQCTRL_CCU80_SR0_IRQ25);
    NVIC_ClearPendingIRQ(CCU80_0_IRQn);
    NVIC_EnableIRQ(CCU80_0_IRQn);

    /* Configuring CCU80 CC8xINS - Input Selector Configuration to SCU.GSC80 */
    /* Event0 -> INyH (SCU.GSC80, Global Start Control CCU80) for EV0IS */
    /* INyH for EV0IS */
    XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_EVENT_0, &CCU8_Input_Event0_Config);
    XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_PHASE_V, XMC_CCU8_SLICE_EVENT_0, &CCU8_Input_Event0_Config);
    XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_PHASE_W, XMC_CCU8_SLICE_EVENT_0, &CCU8_Input_Event0_Config);
    XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_ADC_TR, XMC_CCU8_SLICE_EVENT_0, &CCU8_Input_Event0_Config);

    /* Setup Event0 for external start trigger */
    XMC_CCU8_SLICE_StartConfig(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
    XMC_CCU8_SLICE_StartConfig(CCU8_SLICE_PHASE_V, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
    XMC_CCU8_SLICE_StartConfig(CCU8_SLICE_PHASE_W, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
    XMC_CCU8_SLICE_StartConfig(CCU8_SLICE_ADC_TR, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);

    /* Configure Event1 for external stop trigger */
    XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_EVENT_1, &CCU8_Input_Event1_Config);
    XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_PHASE_V, XMC_CCU8_SLICE_EVENT_1, &CCU8_Input_Event1_Config);
    XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_PHASE_W, XMC_CCU8_SLICE_EVENT_1, &CCU8_Input_Event1_Config);
    XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_ADC_TR, XMC_CCU8_SLICE_EVENT_1, &CCU8_Input_Event1_Config);

    /* Setup Event1 for external stop trigger */
    XMC_CCU8_SLICE_StopConfig(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_EVENT_1, XMC_CCU8_SLICE_END_MODE_TIMER_STOP_CLEAR);
    XMC_CCU8_SLICE_StopConfig(CCU8_SLICE_PHASE_V, XMC_CCU8_SLICE_EVENT_1, XMC_CCU8_SLICE_END_MODE_TIMER_STOP_CLEAR);
    XMC_CCU8_SLICE_StopConfig(CCU8_SLICE_PHASE_W, XMC_CCU8_SLICE_EVENT_1, XMC_CCU8_SLICE_END_MODE_TIMER_STOP_CLEAR);
    XMC_CCU8_SLICE_StopConfig(CCU8_SLICE_ADC_TR, XMC_CCU8_SLICE_EVENT_1, XMC_CCU8_SLICE_END_MODE_TIMER_STOP_CLEAR);

     /* Disable Global Start Control CCU80 */
    XMC_SCU_SetCcuTriggerLow(PMSM_FOC_CCU8_CCU4_SYNC_START);

    /* Clear idle bit slice 0,1,2,3 for CCU80 Kernel */
    XMC_CCU8_EnableMultipleClocks(CCU8_MODULE, XMC_CCU8_GIDLC_CLOCK_MASK);

    /* Trap functionality enable */
    /* LPF2M = 11b, Event2 LPF -> 7 clock cycles of fCCU8 */
    /* EV2LM = 1b, Event2 (active LOW) -> TRAP -> CCU8x.INyA -> P0.12. Note trap function is level active */
//#if (SPI_LIB == DISABLED)
    XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_EVENT_2, &CCU8_Input_trap_config);
    XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_PHASE_V, XMC_CCU8_SLICE_EVENT_2, &CCU8_Input_trap_config);
    XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_PHASE_W, XMC_CCU8_SLICE_EVENT_2, &CCU8_Input_trap_config);

    /* CCU8 TRAP functionality enabled so PWM outputs react on state of an input pin. For over-current protection */
    /* TS = 1, TRAP function enabled and connected to CCU80-CC8x Event2 */
    /* TRPSW = TRPSE = 1b, TRAP state only exited by SW, synch with PWM */
    /* TRAPE3/2/1/0 = 1b, TRAP affects CCU8x.OUTy0/1/2/3 */
    XMC_CCU8_SLICE_TrapConfig(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_TRAP_EXIT_MODE_SW, true);
    XMC_CCU8_SLICE_EnableTrap(CCU8_SLICE_PHASE_U, XMC_CCU8_TC_TRAPSE_MASK);

    XMC_CCU8_SLICE_TrapConfig(CCU8_SLICE_PHASE_V, XMC_CCU8_SLICE_TRAP_EXIT_MODE_SW, true);
    XMC_CCU8_SLICE_EnableTrap(CCU8_SLICE_PHASE_V, XMC_CCU8_TC_TRAPSE_MASK);

    XMC_CCU8_SLICE_TrapConfig(CCU8_SLICE_PHASE_W, XMC_CCU8_SLICE_TRAP_EXIT_MODE_SW, true);
    XMC_CCU8_SLICE_EnableTrap(CCU8_SLICE_PHASE_W, XMC_CCU8_TC_TRAPSE_MASK);

    /* Enable interrupt for CCU80-CC80 Event2 */
    XMC_CCU8_SLICE_EnableEvent(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_EVENT2);

    /* Event2 interrupt forward to CC8ySR1 */
    XMC_CCU8_SLICE_SetInterruptNode(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_EVENT2, XMC_CCU8_SLICE_SR_ID_1);

    /* CCU80_1_IRQn = 26. Trap protection interrupt. CCU80_1_IRQHandler Trap protection interrupt has highest priority */

#if(UC_SERIES == XMC14)
    NVIC_SetPriority (IRQ26_IRQn, 0U);
    XMC_SCU_SetInterruptControl(IRQ26_IRQn, XMC_SCU_IRQCTRL_CCU80_SR1_IRQ26);
    NVIC_EnableIRQ(IRQ26_IRQn);
#else
    NVIC_SetPriority (CCU80_1_IRQn, 0U);
    NVIC_EnableIRQ(CCU80_1_IRQn);
#endif

    /* Interrupt Status Clear, for interrupts of Period Match, Trap Flag, and Event2 */
    CCU8_SLICE_PHASE_U->SWR |= 0x00000C01U;
//#endif// (SPI_LIB == DISABLED)
}



