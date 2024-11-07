/**
 * @file pmsm_foc_mcuhw_params.h
 * @date 09 May, 2019
 *
 * @cond
 *********************************************************************************************************************
 * PMSM FOC Motor Control Library
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

#ifndef PMSM_FOC_MCUCARD_PARAMETERS_H_
#define PMSM_FOC_MCUCARD_PARAMETERS_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <PMSM_FOC/Configuration/pmsm_foc_config.h>
#include <xmc_vadc.h>
#include <xmc_ccu4.h>
#include <xmc_ccu8.h>
#include <xmc_scu.h>
#include <xmc_gpio.h>
#include <xmc_math.h>
#include <xmc_wdt.h>
#include "xmc1_gpio_map.h"

#if (SPI_LIB == ENABLED)
#include "../IMD700A_SPI_LIB/utility.h"
#endif

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup Configuration
 * @{
 */
/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * GPIO Resources Configuration
 ********************************************************************************************************************/
#if(MCUTYPE == MCU_CARD_XMC1404)
  #define TRAP_PIN               P0_12
  #define INVERTER_EN_PIN        P3_0   			/* Active High */

  #if(USER_MOTOR_BI_DIRECTION_CTRL == ENABLED)
  #define MOTOR_DIR_INPUT_PIN    P4_10
  #endif

  /*********************************************************************************************************************
   * PWM gate signal configuration macros
   *****************************	*********/
  #define PHASE_U_HS_PIN        											P0_0
  #define PHASE_U_HS_ALT_SELECT 											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5
                                											
  #define PHASE_U_LS_PIN        											P0_1
  #define PHASE_U_LS_ALT_SELECT 											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5
                                											
  #define PHASE_V_HS_PIN        											P0_7
  #define PHASE_V_HS_ALT_SELECT 											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5
                                											
  #define PHASE_V_LS_PIN        											P0_6
  #define PHASE_V_LS_ALT_SELECT 											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5
                                											
  #define PHASE_W_HS_PIN        											P0_8
  #define PHASE_W_HS_ALT_SELECT 											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5
                                											
  #define PHASE_W_LS_PIN        											P0_9
  #define PHASE_W_LS_ALT_SELECT 											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5
                                											
  /*********************************************************************************************************************
   * Misc pin  macros
   ***************************************/
  #define FAULT_LED3              										P4_7   /* FAULT LED indicator */
  #define FW_ACTIVE_LED4          										P4_6   /* Flux Weakening Active LED indicator */
  #define TEST_PIN                										P4_0   /* Test pin for timing measurement */
  #define HALL_ISR_DEBUG_PIN      										P0_4
                                  										

  #define USER_INVERTER_ENABLE_LEVEL                  XMC_GPIO_OUTPUT_LEVEL_HIGH     /*!< Inverter Enable - required pin level for PWM output*/
  #define USER_INVERTER_DISABLE_LEVEL                 XMC_GPIO_OUTPUT_LEVEL_LOW      /*!< Inverter Disable - required pin level for PWM output */

  /*********************************************************************************************************************
   * CCU8 Resources Configuration
   ***************************************/
  #define CCU8_MODULE                                 CCU80
  #define CCU8_SLICE_PHASE_U                          CCU80_CC80
  #define CCU8_SLICE_PHASE_V                          CCU80_CC81
  #define CCU8_SLICE_PHASE_W                          CCU80_CC82
  #define CCU8_SLICE_ADC_TR                           CCU80_CC83
  #define USER_CCU8_PASSIVE_LEVEL_OUT0                CCU8_PASSIVE_LOW               /*!< PWM output passive level required for driver IC for high side */
  #define USER_CCU8_PASSIVE_LEVEL_OUT1                CCU8_PASSIVE_LOW               /*!< PWM output passive level required for driver IC for low side */

  /*********************************************************************************************************************
   * VADC Resources Configuration
   ********************************************************************************************************************/
  /* For simultaneous sampling */
  #define VADC_I1_GROUP         VADC_G1
  #define VADC_I1_CHANNEL       (0U)
  #define VADC_I1_RESULT_REG    (0U)

  #define VADC_I3_GROUP         VADC_G1
  #define VADC_I3_CHANNEL       (1U)
  #define VADC_I3_RESULT_REG    (1U)

  #define VADC_I2_GROUP         VADC_G0
  #define VADC_I2_CHANNEL       (0U)
  #define VADC_I2_RESULT_REG    (0U)

  #define VADC_I4_GROUP         VADC_G0
  #define VADC_I4_CHANNEL       (1U)
  #define VADC_I4_RESULT_REG    (1U)

  /* Motor Phase U VADC define */
  #define VADC_IU_G1_CHANNEL    										(3U)        /* P2.11, VADC group1 channel 3 */
  #define VADC_IU_G0_CHANNEL    										(4U)        /* P2.11, VADC group0 channel 4 */

  /* Motor Phase V VADC define */
  #define VADC_IV_G1_CHANNEL    										(2U)       /* P2.10, VADC group1 channel 2 */
  #define VADC_IV_G0_CHANNEL    										(3U)       /* P2.10, VADC group0 channel 3 */

  /* Motor Phase W VADC define */
  #define VADC_IW_G1_CHANNEL    										(4U)       /* P2.9, VADC group1 channel 4 */
  #define VADC_IW_G0_CHANNEL    										(2U)       /* P2.9, VADC group0 channel 2 */

  /* VADC Group 0 Alias channel 0 and channel 1 */
  #define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
  #define VADC_G0_CHANNEL_ALIAS1  VADC_IDC_CHANNEL

  /* VADC Group 1 Alias channel 0 and channel 1 */
  #define VADC_G1_CHANNEL_ALIAS0  VADC_IW_G1_CHANNEL
  #define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL

  #define VADC_IU_GROUP         										VADC_G1
  #define VADC_IU_GROUP_NO      										(1U)
  #define VADC_IU_CHANNEL       										(3U)       /* P2.11, VADC group1 channel 3 */
  #define VADC_IU_RESULT_REG    										(3U)

  #define VADC_IV_GROUP             									VADC_G1
  #define VADC_IV_GROUP_NO          									(1U)
  #define VADC_IV_CHANNEL           									(2U)       /* P2.10, VADC group1 channel 2 */
  #define VADC_IV_RESULT_REG        									(2U)
                                    									
  #define VADC_IW_GROUP             									VADC_G1
  #define VADC_IW_GROUP_NO          									(1U)
  #define VADC_IW_CHANNEL           									(4U)       /* P2.9, VADC group1 channel 4 */
  #define VADC_IW_RESULT_REG        									(4U)
                                    									
  /* DC link voltage VADC define */ 									
  #define VADC_VDC_GROUP            									VADC_G1
  #define VADC_VDC_GROUP_NO         									(1U)
  #define VADC_VDC_CHANNEL          									(5U)      /* P2.3 VADC group1 channel 5 */
  #define VADC_VDC_RESULT_REG       									(5U)
                                    									
  /* DC link average current VADC define */
  #define VADC_IDC_GROUP            									VADC_G1
  #define VADC_IDC_GROUP_NO         									(1U)
  #define VADC_IDC_CHANNEL          									(6U)       /* P2.4 VADC group1 channel 6 */
  #define VADC_IDC_RESULT_REG       									(6U)
                                    									
  /* Potentiometer VADC define*/    									
  #define VADC_POT_GROUP            									VADC_G1
  #define VADC_POT_GROUP_NO         									(1U)
  #define VADC_POT_CHANNEL          									(7U)      /* P2.5 VADC group1 channel 7 */
  #define VADC_POT_RESULT_REG       									(7U)
                                    									
  /* Temperature pin VADC define*/  									
  #define VADC_TEMP_GROUP           									VADC_G0
  #define VADC_TEMP_GROUP_NO        									(0U)
  #define VADC_TEMP_CHANNEL         									(6U)      /* P2.1 VADC group0 channel 6 (temporary) */
  #define VADC_TEMP_RESULT_REG      									(6U)

  #if(PMSM_FOC_HARDWARE_KIT == LEV_4KW_KIT)
  /* 1.65V Vref VADC define*/
  #define VADC_VREF_GROUP        										VADC_G0
  #define VADC_VREF_GROUP_NO     										(0U)
  #define VADC_VREF_CHANNEL      										(5U)      /* P2.0 VADC group0 channel 5 */
  #define VADC_VREF_RESULT_REG   										(5U)
  #endif
                                    									
  /*************************** 6EDL7141 configuration *****************************************************************/
  #define USER_IU_ADC_BIAS                            (2048)                         /*!< Motor phase U current bias -default*/
  #define USER_IV_ADC_BIAS                            (2048)                         /*!< Motor phase V current bias -default*/
  #define USER_IW_ADC_BIAS                            (2048)                         /*!< Motor phase W current bias -default*/

  /*********************************************************************************************************************
   * POSIF configuration macros
   ***************************************/
  /*
  Input signal  Ports
  Hall Input 1  Port P0.13    POSIF0.IN0B
  Hall Input 2  Port P1.1     POSIF0.IN1A
  Hall Input 3  Port P0.15    POSIF0.IN2B
  */                                    							
  #define HALL_0_PIN                    							P0_13   /* POSIF0.IN0B */
  #define HALL_1_PIN                    							P1_1    /* POSIF0.IN1A */
  #define HALL_2_PIN                    							P0_15   /* POSIF0.IN2B */
                                        							
  #define POSIF_MODULE                  							(POSIF0)
                                        							
  #define POSIF_HALL_0_INSEL            							(XMC_POSIF_INPUT_PORT_B)
  /** Refer POSIF PCONF.INSEL1 in reference manual*/
  #define POSIF_HALL_1_INSEL            							(XMC_POSIF_INPUT_PORT_A)
  /** Refer POSIF PCONF.INSEL2 in reference manual*/
  #define POSIF_HALL_2_INSEL            							(XMC_POSIF_INPUT_PORT_B)
                                        							
  #define POSIF_PATTERN_UPDATE_SEL      							(XMC_POSIF_INPUT_PORT_A)
  #define POSIF_PWM_SYNC_SIGNAL_SEL     							(XMC_POSIF_INPUT_PORT_A)
                                        							
  #define CCU8_ONEMATCH_SR              							(XMC_CCU8_SLICE_SR_ID_0)
                                        							
  #define HALL_CCU4_MODULE              							CCU40
  #define HALL_CCU4_MODULE_NUM          							0U
  #define HALL_DELAY_SLICE              							CCU40_CC40
  #define HALL_DELAY_SLICE_NUM          							0U
  #define HALL_CAPTURE_SLICE            							CCU40_CC41
  #define HALL_CAPTURE_SLICE_NUM        							1U

  /* ********************************************************************************************************************/
  /* NVIC Interrupt Resources Configuration */
  /* ********************************************************************************************************************/
  #define CCU80_0_IRQn                                IRQ25_IRQn          /*!< CCU80 SR0 Interrupt  */
  #define PMSM_FOC_FCL_ISR                            IRQ25_Handler       /*!< Fast Control Loop(FCL) - PWM Period Match*/

  #define TRAP_IRQn                                   IRQ26_IRQn          /*!< CCU80 SR1 Interrupt  */
  #define PMSM_FOC_CTRAP_ISR                          IRQ26_Handler       /*!< CTRAP - CCU8 - PWM */

//  #define FAULT_PIN_ERU_IRQn                          IRQ6_IRQn           /*!< ERU */
//  #define PMSM_FOC_DRIVER_nFAULT_ISR                  IRQ6_Handler        /*!< ERU - 6EDL7141 nFault pin interrupt handler */

  #define POSIF_IRQn                                  IRQ27_IRQn
  #define POSIF_IRQHandler                            IRQ27_Handler

  /* NVIC ISR handler mapping */
  #define PMSM_FOC_SCL_ISR                            SysTick_Handler     /*!< Slow Control Loop(SCL) - Systick */


  /* ********************************************************************************************************************/
  /* NVIC Interrupt Priority Configuration */
  /* ********************************************************************************************************************/
  /* Interrupt priority configurations - 0 is the highest priority and 3 is the lowest */
  #define PMSM_FOC_FCL_NVIC_PRIO                      (1U)  /*!< FAST Control loop - Executed every PWM period */
  #define PMSM_FOC_SCL_NVIC_PRIO                      (2U)  /*!< Slow Control loop  - SysTick */
  #define PMSM_FOC_CTRAP_NVIC_PRIO                    (0U)  /*!< CTRAP   */
  #define PMSM_FOC_FAULT_NVIC_PRIO                    (1U)  /*!< nFault from 6EDL7141 */
  #define PMSM_FOC_POSIF_NVIC_PRIO                    (0U)

/*********************************************************************************************************************
   * Configuration specific to LEV 4KW Inverter (temporary)
   ******************************************************************************************************************/
#if(INVERTERCARD_TYPE == LEV_4KW_INVERTER)
/*      --------------------------------------------------- Motor Phase Current Measurement ---------------------------------------- */
  #define USER_MAX_ADC_VDD_V                          (5.0f)                   /* VDD5, maximum voltage at ADC */
  #define CIRCUIT_GAIN_FACTOR                         (0.0049f) 			// 1A = CIRCUIT_GAIN_FACTOR (v)

  #define I_MAX_A                                     (1.65f/CIRCUIT_GAIN_FACTOR)
  #define ADC_I15_RATIO                               (int32_t)(USER_MAX_ADC_VDD_V * 32767 /(I_MAX_A * CIRCUIT_GAIN_FACTOR * 4095))
  #define USER_IDC_MAXCURRENT_A                       (90.0f)

  #define USER_CCU8_INPUT_TRAP_LEVEL	XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH

/* 		IDC Max limit to USER defined IDC Max Current 		*/
  #define IDC_MAX_LIMIT                               (uint32_t)(((((USER_IDC_MAXCURRENT_A/149)*CURRENT_VOLTAGE_RANGE)+CURRENT_OFFSET_VOLTAGE)/USER_MAX_ADC_VDD_V)*USER_MAX_ADC_POT_HEX) //1392 @10A

  #define I_PHASEOCP_A								  (244U)		// defined software phase current OCP value
  #define USER_MAX_ADC_POT_HEX                        (4096)                /* VDD5, maximum hex reading value at ADC Potentiometer */                  /* VDD5, maximum voltage at ADC */
  #define CURRENT_OFFSET_VOLTAGE                      (1.65f) 		// In Volt ->Current Offset Voltage -> 0A=1.65V//
  #define CURRENT_VOLTAGE_RANGE                       (1.2f)// In Volt ->Current Range in Voltage -> 2.85V(162A)-1.65V(0A)=1.2V//
  #define CURRENT_CONVERSION                          (float)(I_MAX_A/CURRENT_VOLTAGE_RANGE)//convert back to Current variable in Ampere//

  #define CURRENT_THRESHOLD	 						  (float)((I_PHASEOCP_A/I_MAX_A)*CURRENT_OFFSET_VOLTAGE)
  #define PHASEOCP_NEG								  DISABLED					/*1. ENABLED       2. DISABLED*/
  #define I_OCP_POS                                   (uint32_t)(((CURRENT_OFFSET_VOLTAGE+CURRENT_THRESHOLD)/USER_MAX_ADC_VDD_V)*USER_MAX_ADC_POT_HEX) //@20A phase 1432, @100A 1754
  #define I_OCP_NEG                                   (uint32_t)(((CURRENT_OFFSET_VOLTAGE-CURRENT_THRESHOLD)/USER_MAX_ADC_VDD_V)*USER_MAX_ADC_POT_HEX)
  #define I_OCP_SW									  (4835U) 		//4835=150A phase, used when RECON_OCP is enabled
  #define ADCLPF    								  (5U)

#else
  #define USER_CCU8_INPUT_TRAP_LEVEL	XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
#endif // if(INVERTERCARD_TYPE == LEV_4KW_INVERTER)

#elif(MCUTYPE == MCU_IMD701)
  #define INVERTER_EN_PIN        											P0_7 /* Active High */
  #define nBRAKE_PIN             											P1_3 /* Active Low */
  #define DRIVER_CS_AZ_PIN       											P1_2 /* Active High */
  #define nFAULT_PIN             											P3_4 /* Active Low */
                                 											
  #if(USER_MOTOR_BI_DIRECTION_CTRL == ENABLED)
  #define MOTOR_DIR_INPUT_PIN    											P4_10
  #endif                         											
                                 											
  #define PHASE_U_HS_PIN         											P3_3
  #define PHASE_U_HS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5
                                 											
  #define PHASE_U_LS_PIN         											P3_2
  #define PHASE_U_LS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5
                                 											
  #define PHASE_V_HS_PIN         											P3_1
  #define PHASE_V_HS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5
                                 											
  #define PHASE_V_LS_PIN         											P3_0
  #define PHASE_V_LS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5
                                 											
  #define PHASE_W_HS_PIN         											P1_0
  #define PHASE_W_HS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5
                                 											
  #define PHASE_W_LS_PIN         											P1_1
  #define PHASE_W_LS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

  #define FAULT_LED3              										P4_7   /* FAULT LED indicator */
  #define FW_ACTIVE_LED4          										P4_6   /* Flux Weakening Active LED indicator */
                                  										
  #define TEST_PIN                										P4_0   /* Test pin for timing measurement */
                                  										
  #define USER_INVERTER_ENABLE_LEVEL                  XMC_GPIO_OUTPUT_LEVEL_HIGH     /*!< Inverter Enable - required pin level for PWM output*/
  #define USER_INVERTER_DISABLE_LEVEL                 XMC_GPIO_OUTPUT_LEVEL_LOW      /*!< Inverter Disable - required pin level for PWM output */

  #define USER_BRAKE_ENABLE_LEVEL                     XMC_GPIO_OUTPUT_LEVEL_LOW      /*!< Brake Enable - required pin level to enable braking */
  #define USER_BRAKE_DISABLE_LEVEL                    XMC_GPIO_OUTPUT_LEVEL_HIGH     /*!< Brake Disable - required pin level to enable braking*/

  /*********************************************************************************************************************
   * CCU8 Resources Configuration
   ********************************************************************************************************************/
  #define CCU8_MODULE             										CCU80
  #define CCU8_SLICE_PHASE_U      										CCU80_CC81
  #define CCU8_SLICE_PHASE_V      										CCU80_CC82
  #define CCU8_SLICE_PHASE_W      										CCU80_CC80
                                  										
  #define CCU8_SLICE_ADC_TR       										CCU80_CC83
                                  										
  #define USER_CCU8_PASSIVE_LEVEL_OUT0                CCU8_PASSIVE_LOW               /*!< PWM output passive level required for driver IC for high side */
  #define USER_CCU8_PASSIVE_LEVEL_OUT1                CCU8_PASSIVE_LOW               /*!< PWM output passive level required for driver IC for low side */

//  #define USER_CCU8_INPUT_TRAP_LEVEL	XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
  #define USER_CCU8_INPUT_TRAP_LEVEL                  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH /*!< Trap signal input level selection for ctrap to occur */
  /*********************************************************************************************************************
   * VADC Resources Configuration
   ********************************************************************************************************************/
  /* For simultaneous sampling */
  #define VADC_I1_GROUP         											VADC_G1
  #define VADC_I1_CHANNEL       											(0U)
  #define VADC_I1_RESULT_REG    											(0U)
                                											
  #define VADC_I2_GROUP         											VADC_G0
  #define VADC_I2_CHANNEL       											(0U)
  #define VADC_I2_RESULT_REG    											(0U)
                                											
  #define VADC_I3_GROUP         											VADC_G1
  #define VADC_I3_CHANNEL       											(1U)
  #define VADC_I3_RESULT_REG    											(1U)
                                											
  #define VADC_I4_GROUP         											VADC_G0     /* P2.8 VADC group0 channel 1 */
  #define VADC_I4_CHANNEL       											(1U)
  #define VADC_I4_RESULT_REG    											(1U)
                                											
  /* Motor Phase U VADC define */
  #define VADC_IU_G1_CHANNEL    											(3U)        /* P2.11, VADC group1 channel 3 */
  #define VADC_IU_G0_CHANNEL    											(4U)        /* P2.11, VADC group0 channel 4 */
                                											
  /* Motor Phase V VADC define */
  #define VADC_IV_G1_CHANNEL    											(2U)       /* P2.10, VADC group1 channel 2 */
  #define VADC_IV_G0_CHANNEL    											(3U)       /* P2.10, VADC group0 channel 3 */
                                											
  /* Motor Phase W VADC define */
  #define VADC_IW_G1_CHANNEL    											(4U)       /* P2.9, VADC group1 channel 4 */
  #define VADC_IW_G0_CHANNEL    											(2U)       /* P2.9, VADC group0 channel 2 */
                                											
  /* Sectors A. ADC sequences - Iv -> Iw -> Iu */
  /* VADC Group 0 Alias channel 0 and channel 1 */
  #define VADC_G0_CHANNEL_ALIAS0  										VADC_IW_G0_CHANNEL
  #define VADC_G0_CHANNEL_ALIAS1  										VADC_I4_CHANNEL
                                  										
  /* VADC Group 1 Alias channel 0 and channel 1 */
  #define VADC_G1_CHANNEL_ALIAS0  										VADC_IV_G1_CHANNEL
  #define VADC_G1_CHANNEL_ALIAS1  										VADC_IU_G1_CHANNEL

  #define VADC_IU_GROUP         											VADC_G1
  #define VADC_IU_GROUP_NO      											(1U)
  #define VADC_IU_CHANNEL       											(3U)       /* P2.11, VADC group1 channel 3 */
  #define VADC_IU_RESULT_REG    											(3U)
                                											
  #define VADC_IV_GROUP         											VADC_G1
  #define VADC_IV_GROUP_NO      											(1U)
  #define VADC_IV_CHANNEL       											(2U)       /* P2.10, VADC group1 channel 2 */
  #define VADC_IV_RESULT_REG    											(2U)
                                											
  #define VADC_IW_GROUP         											VADC_G1
  #define VADC_IW_GROUP_NO      											(1U)
  #define VADC_IW_CHANNEL       											(4U)       /* P2.9, VADC group1 channel 4 */
  #define VADC_IW_RESULT_REG    											(4U)
                                											
  /* DC link voltage VADC define */
  #define VADC_VDC_GROUP        											VADC_G1
  #define VADC_VDC_GROUP_NO     											(1U)
  #define VADC_VDC_CHANNEL      											(5U)       /* P2.3 VADC group1 channel 5 */
  #define VADC_VDC_RESULT_REG   											(5U)
                                											
  /* T_Sense Temp sensor VADC define*/
  #define VADC_TEMP_GROUP       											VADC_G1
  #define VADC_TEMP_GROUP_NO    											(1U)
  #define VADC_TEMP_CHANNEL     											(6U)       /* P2.4 VADC group1 channel 6 */
  #define VADC_TEMP_RESULT_REG  											(6U)
                                											
  /* Potentiometer VADC define*/											
  #define VADC_POT_GROUP        											VADC_G1
  #define VADC_POT_GROUP_NO     											(1U)
  #define VADC_POT_CHANNEL      											(7U)       /* P2.5 VADC group1 channel 7 */
  #define VADC_POT_RESULT_REG   											(7U)
                                											

  /*************************** 6EDL7141 configuration *****************************************************************/
  #define USER_IU_ADC_BIAS                            (2048)                         /*!< Motor phase U current bias -default*/
  #define USER_IV_ADC_BIAS                            (2048)                         /*!< Motor phase V current bias -default*/
  #define USER_IW_ADC_BIAS                            (2048)                         /*!< Motor phase W current bias -default*/

  /*************************** AZ for 6EDL7141 through CCU4 ***********************************************************/
  #define GDRIVER_AZ_PWM_CCU4_MODULE                        (CCU40)
  #define GDRIVER_AZ_PWM_SLICE                              (CCU40_CC42)
  #define GDRIVER_AZ_PWM_SLICE_NUM                          (2U)
  #define GDRIVER_AZ_PWM_SLICE_SHADOW_TRANS_ENABLE_Msk      (XMC_CCU4_SHADOW_TRANSFER_SLICE_2)
  #define GDRIVER_AZ_PWM_ALT_OUT                            (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2)

  /*********************************************************************************************************************
   * POSIF configuration macros
   ***************************************/
  #define HALL_0_PIN             											P4_1		/* POSIF1.IN0B */
  #define HALL_1_PIN             											P4_2		/* POSIF1.IN1B */
  #define HALL_2_PIN             											P4_3		/* POSIF1.IN2B */
                                    									
  #define POSIF_MODULE           											(POSIF1)
                                    									
  #define POSIF_HALL_0_INSEL      										(XMC_POSIF_INPUT_PORT_B)
  /** Refer POSIF PCONF.INSEL1 in reference manual*/
  #define POSIF_HALL_1_INSEL      										(XMC_POSIF_INPUT_PORT_B)
  /** Refer POSIF PCONF.INSEL2 in reference manual*/
  #define POSIF_HALL_2_INSEL      										(XMC_POSIF_INPUT_PORT_B)

  #define POSIF_PATTERN_UPDATE_SEL   									(XMC_POSIF_INPUT_PORT_A)
  #define POSIF_PWM_SYNC_SIGNAL_SEL  									(XMC_POSIF_INPUT_PORT_A)
                                      								
  #define CCU8_ONEMATCH_SR        										(XMC_CCU8_SLICE_SR_ID_0)
                                      								
  #define HALL_CCU4_MODULE            								CCU41
  #define HALL_CCU4_MODULE_NUM        								1U
  #define HALL_DELAY_SLICE            								CCU41_CC40
  #define HALL_DELAY_SLICE_NUM        								0U
  #define HALL_CAPTURE_SLICE          								CCU41_CC41
  #define HALL_CAPTURE_SLICE_NUM      								1U
                                      								
  /*************************** BEMF detection *************************************************************************/
  /* Make sure all are from same group. If not then scan trigger both the groups for conversion */

  /* BEMF_U VADC define*/
  #define VADC_BEMF_U_GROUP         									VADC_G0     /* P2.2 VADC group0 channel 7 */
  #define VADC_BEMF_U_CHANNEL       									(7U)
  #define VADC_BEMF_U_RESULT_REG    									(7U)
                                    									
  /* BEMF_V VADC define*/           									
  #define VADC_BEMF_V_GROUP         									VADC_G0     /* P2.1 VADC group0 channel 6 */
  #define VADC_BEMF_V_CHANNEL       									(6U)
  #define VADC_BEMF_V_RESULT_REG    									(6U)
                                    									
  /* BEMF_W VADC define*/           									
  #define VADC_BEMF_W_GROUP         									VADC_G0     /* P2.0 VADC group0 channel 5 */
  #define VADC_BEMF_W_CHANNEL       									(5U)
  #define VADC_BEMF_W_RESULT_REG    									(5U)
                                    									
  /* ********************************************************************************************************************/
  /* NVIC Interrupt Resources Configuration */
  /* ********************************************************************************************************************/
  #define CCU80_0_IRQn                                IRQ25_IRQn          /*!< CCU80 SR0 Interrupt  */
  #define PMSM_FOC_FCL_ISR                            IRQ25_Handler       /*!< Fast Control Loop(FCL) - PWM Period Match*/

  #define TRAP_IRQn                                   IRQ26_IRQn          /*!< CCU80 SR1 Interrupt  */
  #define PMSM_FOC_CTRAP_ISR                          IRQ26_Handler       /*!< CTRAP - CCU8 - PWM */

  #define POSIF_IRQn			                      IRQ27_IRQn
  #define POSIF_IRQHandler                     		  IRQ27_Handler

  #define FAULT_PIN_ERU_IRQn                          IRQ6_IRQn           /*!< ERU */
  #define PMSM_FOC_DRIVER_nFAULT_ISR                  IRQ6_Handler        /*!< ERU - 6EDL7141 nFault pin interrupt handler */

  /* NVIC ISR handler mapping */
  #define PMSM_FOC_SCL_ISR                            SysTick_Handler     /*!< Slow Control Loop(SCL) - Systick */

  /* ********************************************************************************************************************/
  /* NVIC Interrupt Priority Configuration */
  /* ********************************************************************************************************************/
  /* Interrupt priority configurations - 0 is the highest priority and 3 is the lowest */
  #define PMSM_FOC_FCL_NVIC_PRIO                      (1U)  /*!< FAST Control loop - Executed every PWM period */
  #define PMSM_FOC_SCL_NVIC_PRIO                      (2U)  /*!< Slow Control loop  - SysTick */
  #define PMSM_FOC_CTRAP_NVIC_PRIO                    (0U)  /*!< CTRAP   */
  #define PMSM_FOC_FAULT_NVIC_PRIO                    (1U)  /*!< nFault from 6EDL7141 */
  #define PMSM_FOC_POSIF_NVIC_PRIO                    (0U)

#elif(MCUTYPE == MCU_IMD_1_5kw_ref)
/*********************************************************************************************************************
 * GPIO Resources Configuration
 ********************************************************************************************************************/
  #define INVERTER_EN_PIN        											P0_7 /* Active High */
  #define nBRAKE_PIN             											P1_3 /* Active Low */
  #define DRIVER_CS_AZ_PIN       											P1_2 /* Active High */
  #define nFAULT_PIN             											P3_4 /* Active Low */
  #define TRAP_PIN               											P0_12

  #if(USER_MOTOR_BI_DIRECTION_CTRL == ENABLED)
  #define MOTOR_DIR_INPUT_PIN    											P4_10
  #endif


/*********************************************************************************************************************
   * PWM gate signal configuration macros
   *****************************	*********/
  #define PHASE_U_HS_PIN         											P3_3
  #define PHASE_U_HS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

  #define PHASE_U_LS_PIN         											P3_2
  #define PHASE_U_LS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

  #define PHASE_V_HS_PIN         											P3_1
  #define PHASE_V_HS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

  #define PHASE_V_LS_PIN         											P3_0
  #define PHASE_V_LS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

  #define PHASE_W_HS_PIN         											P1_0
  #define PHASE_W_HS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

  #define PHASE_W_LS_PIN         											P1_1
  #define PHASE_W_LS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

  #define FAULT_LED3              										P4_7   /* FAULT LED indicator */
  #define FW_ACTIVE_LED4          										P4_6   /* Flux Weakening Active LED indicator */

  #define TEST_PIN                										P4_8   /* Test pin for timing measurement */

  #define USER_INVERTER_ENABLE_LEVEL                  XMC_GPIO_OUTPUT_LEVEL_HIGH     /*!< Inverter Enable - required pin level for PWM output*/
  #define USER_INVERTER_DISABLE_LEVEL                 XMC_GPIO_OUTPUT_LEVEL_LOW      /*!< Inverter Disable - required pin level for PWM output */

  #define USER_BRAKE_ENABLE_LEVEL                     XMC_GPIO_OUTPUT_LEVEL_LOW      /*!< Brake Enable - required pin level to enable braking */
  #define USER_BRAKE_DISABLE_LEVEL                    XMC_GPIO_OUTPUT_LEVEL_HIGH     /*!< Brake Disable - required pin level to enable braking*/

  /*********************************************************************************************************************
   * CCU8 Resources Configuration
   ********************************************************************************************************************/
  #define CCU8_MODULE             										CCU80
  #define CCU8_SLICE_PHASE_U      										CCU80_CC81
  #define CCU8_SLICE_PHASE_V      										CCU80_CC82
  #define CCU8_SLICE_PHASE_W      										CCU80_CC80

  #define CCU8_SLICE_ADC_TR       										CCU80_CC83

  #define USER_CCU8_PASSIVE_LEVEL_OUT0                CCU8_PASSIVE_LOW               /*!< PWM output passive level required for driver IC for high side */
  #define USER_CCU8_PASSIVE_LEVEL_OUT1                CCU8_PASSIVE_LOW               /*!< PWM output passive level required for driver IC for low side */

  /*!< Trap signal input level selection for ctrap to occur */
  #define USER_CCU8_INPUT_TRAP_LEVEL				  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
//  #define USER_CCU8_INPUT_TRAP_LEVEL               	  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH
  /*********************************************************************************************************************
   * VADC Resources Configuration
   ********************************************************************************************************************/
  /* For simultaneous sampling */
  #define VADC_I1_GROUP         											VADC_G1
  #define VADC_I1_CHANNEL       											(0U)
  #define VADC_I1_RESULT_REG    											(0U)

  #define VADC_I2_GROUP         											VADC_G0
  #define VADC_I2_CHANNEL       											(0U)
  #define VADC_I2_RESULT_REG    											(0U)

  #define VADC_I3_GROUP         											VADC_G1
  #define VADC_I3_CHANNEL       											(1U)
  #define VADC_I3_RESULT_REG    											(1U)

  #define VADC_I4_GROUP         											VADC_G0     /* P2.8 VADC group0 channel 1 */
  #define VADC_I4_CHANNEL       											(1U)
  #define VADC_I4_RESULT_REG    											(1U)

  /* Motor Phase U VADC define */
  #define VADC_IU_G1_CHANNEL    											(3U)        /* P2.11, VADC group1 channel 3 */
  #define VADC_IU_G0_CHANNEL    											(4U)        /* P2.11, VADC group0 channel 4 */

  /* Motor Phase V VADC define */
  #define VADC_IV_G1_CHANNEL    											(2U)       /* P2.10, VADC group1 channel 2 */
  #define VADC_IV_G0_CHANNEL    											(3U)       /* P2.10, VADC group0 channel 3 */

  /* Motor Phase W VADC define */
  #define VADC_IW_G1_CHANNEL    											(4U)       /* P2.9, VADC group1 channel 4 */
  #define VADC_IW_G0_CHANNEL    											(2U)       /* P2.9, VADC group0 channel 2 */

  /* Sectors A. ADC sequences - Iv -> Iw -> Iu */
  /* VADC Group 0 Alias channel 0 and channel 1 */
  #define VADC_G0_CHANNEL_ALIAS0  										VADC_IW_G0_CHANNEL
  #define VADC_G0_CHANNEL_ALIAS1  										VADC_I4_CHANNEL

  /* VADC Group 1 Alias channel 0 and channel 1 */
  #define VADC_G1_CHANNEL_ALIAS0  										VADC_IV_G1_CHANNEL
  #define VADC_G1_CHANNEL_ALIAS1  										VADC_IU_G1_CHANNEL

  #define VADC_IU_GROUP         											VADC_G1
  #define VADC_IU_GROUP_NO      											(1U)
  #define VADC_IU_CHANNEL       											(3U)       /* P2.11, VADC group1 channel 3 */
  #define VADC_IU_RESULT_REG    											(3U)

  #define VADC_IV_GROUP         											VADC_G1
  #define VADC_IV_GROUP_NO      											(1U)
  #define VADC_IV_CHANNEL       											(2U)       /* P2.10, VADC group1 channel 2 */
  #define VADC_IV_RESULT_REG    											(2U)

  #define VADC_IW_GROUP         											VADC_G1
  #define VADC_IW_GROUP_NO      											(1U)
  #define VADC_IW_CHANNEL       											(4U)       /* P2.9, VADC group1 channel 4 */
  #define VADC_IW_RESULT_REG    											(4U)

/*E_BIKE*/
//  #define VADC_IU_GROUP         											VADC_G0
//  #define VADC_IU_GROUP_NO      											(0U)
//  #define VADC_IU_CHANNEL       											(4U)       /* P2.11, VADC group0 channel 4 */
//  #define VADC_IU_RESULT_REG    											(4U)
//
//  #define VADC_IV_GROUP         											VADC_G0
//  #define VADC_IV_GROUP_NO      											(0U)
//  #define VADC_IV_CHANNEL       											(3U)       /* P2.10, VADC group0 channel 3 */
//  #define VADC_IV_RESULT_REG    											(3U)
//
//  #define VADC_IW_GROUP         											VADC_G0
//  #define VADC_IW_GROUP_NO      											(0U)
//  #define VADC_IW_CHANNEL       											(2U)       /* P2.9, VADC group0 channel 2 */
//  #define VADC_IW_RESULT_REG    											(2U)

  /* DC link voltage VADC define */
  #define VADC_VDC_GROUP        											VADC_G1
  #define VADC_VDC_GROUP_NO     											(1U)
  #define VADC_VDC_CHANNEL      											(5U)       /* P2.3 VADC group1 channel 5 */
  #define VADC_VDC_RESULT_REG   											(5U)

//  /* T_Sense Temp sensor(Motor) VADC define*/	//added in for E-Bike
#if(E_BIKE_REF == ENABLED)
  #define VADC_TEMP_MOTOR_GROUP       										VADC_G0
  #define VADC_TEMP_MOTOR_GROUP_NO    										(0U)
  #define VADC_TEMP_MOTOR_CHANNEL     										(7U)       /* P2.2 VADC group0 channel 7 */
  #define VADC_TEMP_MOTOR_RESULT_REG  										(7U)
#endif

  /* T_Sense Temp sensor(inverter) VADC define*/
  #define VADC_TEMP_GROUP       											VADC_G1
  #define VADC_TEMP_GROUP_NO    											(1U)
  #define VADC_TEMP_CHANNEL     											(6U)       /* P2.4 VADC group1 channel 6 */
  #define VADC_TEMP_RESULT_REG  											(6U)

  /* Potentiometer VADC define*/
  #define VADC_POT_GROUP        											VADC_G1
  #define VADC_POT_GROUP_NO     											(1U)
  #define VADC_POT_CHANNEL      											(7U)       /* P2.5 VADC group1 channel 7 */
  #define VADC_POT_RESULT_REG   											(7U)
//  #define VADC_POT_GROUP        											VADC_G0
//  #define VADC_POT_GROUP_NO     											(0U)
//  #define VADC_POT_CHANNEL      											(0U)       /* P2.5 VADC group1 channel 7 */
//  #define VADC_POT_RESULT_REG   											(0U)

  /* E_BIKE Brake P2_6*/
#if(E_BIKE_REF == ENABLED)
  /*#define V_brake_switch 													P2_6*/
  #define VADC_E_BIKE_BRAKE_GROUP        									VADC_G0
  #define VADC_E_BIKE_BRAKE_GROUP_NO     									(0U)
  #define VADC_E_BIKE_BRAKE_CHANNEL      									(0U)       /* P2.6 VADC group0 channel 0 */
  #define VADC_E_BIKE_BRAKE_RESULT_REG   									(0U)
#endif

  /*************************** 6EDL7141 configuration *****************************************************************/
  #define USER_IU_ADC_BIAS                            (2048)                         /*!< Motor phase U current bias -default*/
  #define USER_IV_ADC_BIAS                            (2048)                         /*!< Motor phase V current bias -default*/
  #define USER_IW_ADC_BIAS                            (2048)                         /*!< Motor phase W current bias -default*/

  /*************************** AZ for 6EDL7141 through CCU4 ***********************************************************/
  #define GDRIVER_AZ_PWM_CCU4_MODULE                        (CCU40)
  #define GDRIVER_AZ_PWM_SLICE                              (CCU40_CC42)
  #define GDRIVER_AZ_PWM_SLICE_NUM                          (2U)
  #define GDRIVER_AZ_PWM_SLICE_SHADOW_TRANS_ENABLE_Msk      (XMC_CCU4_SHADOW_TRANSFER_SLICE_2)
  #define GDRIVER_AZ_PWM_ALT_OUT                            (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2)

  /*********************************************************************************************************************
   * POSIF configuration macros
   ***************************************/
  #define HALL_0_PIN             											P4_1		/* POSIF1.IN0B (A)*/
  #define HALL_1_PIN             											P4_2		/* POSIF1.IN1B (B)*/
  #define HALL_2_PIN             											P4_3		/* POSIF1.IN2B (C)*/

  #define POSIF_MODULE           											(POSIF1)

  #define POSIF_HALL_0_INSEL      										(XMC_POSIF_INPUT_PORT_B)
  /** Refer POSIF PCONF.INSEL1 in reference manual*/
  #define POSIF_HALL_1_INSEL      										(XMC_POSIF_INPUT_PORT_B)
  /** Refer POSIF PCONF.INSEL2 in reference manual*/
  #define POSIF_HALL_2_INSEL      										(XMC_POSIF_INPUT_PORT_B)

  #define POSIF_PATTERN_UPDATE_SEL   									(XMC_POSIF_INPUT_PORT_A)
  #define POSIF_PWM_SYNC_SIGNAL_SEL  									(XMC_POSIF_INPUT_PORT_A)

  #define CCU8_ONEMATCH_SR        										(XMC_CCU8_SLICE_SR_ID_0)

  #define HALL_CCU4_MODULE            								CCU41
  #define HALL_CCU4_MODULE_NUM        								1U
  #define HALL_DELAY_SLICE            								CCU41_CC40
  #define HALL_DELAY_SLICE_NUM        								0U
  #define HALL_CAPTURE_SLICE          								CCU41_CC41
  #define HALL_CAPTURE_SLICE_NUM      								1U

  /*************************** BEMF detection *************************************************************************/
  /* Make sure all are from same group. If not then scan trigger both the groups for conversion */
/*Not needed for E-Bike*/
  /* BEMF_U VADC define*/
  #define VADC_BEMF_U_GROUP         									VADC_G0     /* P2.2 VADC group0 channel 7 */
  #define VADC_BEMF_U_CHANNEL       									(7U)
  #define VADC_BEMF_U_RESULT_REG    									(7U)

  /* BEMF_V VADC define*/
  #define VADC_BEMF_V_GROUP         									VADC_G0     /* P2.1 VADC group0 channel 6 */
  #define VADC_BEMF_V_CHANNEL       									(6U)
  #define VADC_BEMF_V_RESULT_REG    									(6U)

  /* BEMF_W VADC define*/
  #define VADC_BEMF_W_GROUP         									VADC_G0     /* P2.0 VADC group0 channel 5 */
  #define VADC_BEMF_W_CHANNEL       									(5U)
  #define VADC_BEMF_W_RESULT_REG    									(5U)

  /* ********************************************************************************************************************/
  /* NVIC Interrupt Resources Configuration */
  /* ********************************************************************************************************************/
  #define CCU80_0_IRQn                                IRQ25_IRQn          /*!< CCU80 SR0 Interrupt  */
  #define PMSM_FOC_FCL_ISR                            IRQ25_Handler       /*!< Fast Control Loop(FCL) - PWM Period Match*/

  #define TRAP_IRQn                                   IRQ26_IRQn          /*!< CCU80 SR1 Interrupt  */
  #define PMSM_FOC_CTRAP_ISR                          IRQ26_Handler       /*!< CTRAP - CCU8 - PWM */

  #define POSIF_IRQn			                      IRQ27_IRQn
  #define POSIF_IRQHandler                     		  IRQ27_Handler
  #define POSIF_SR_INT                                XMC_SCU_IRQCTRL_POSIF1_SR0_IRQ27


  #define FAULT_PIN_ERU_IRQn                          IRQ6_IRQn           /*!< ERU */
  #define PMSM_FOC_DRIVER_nFAULT_ISR                  IRQ6_Handler        /*!< ERU - 6EDL7141 nFault pin interrupt handler */

  /* NVIC ISR handler mapping */
  #define PMSM_FOC_SCL_ISR                            SysTick_Handler     /*!< Slow Control Loop(SCL) - Systick */

  /* ********************************************************************************************************************/
  /* NVIC Interrupt Priority Configuration */
  /* ********************************************************************************************************************/
  /* Interrupt priority configurations - 0 is the highest priority and 3 is the lowest */
  #define PMSM_FOC_FCL_NVIC_PRIO                      (1U)  /*!< FAST Control loop - Executed every PWM period */
  #define PMSM_FOC_SCL_NVIC_PRIO                      (2U)  /*!< Slow Control loop  - SysTick */
  #define PMSM_FOC_CTRAP_NVIC_PRIO                    (0U)  /*!< CTRAP   */
  #define PMSM_FOC_FAULT_NVIC_PRIO                    (1U)  /*!< nFault from 6EDL7141 */
  #define PMSM_FOC_POSIF_NVIC_PRIO                    (0U)

/* =========================================== E-Bike =========================================== */
#elif(MCUTYPE == MCU_XMC1404_ebike)
/*********************************************************************************************************************
 * GPIO Resources Configuration
 ********************************************************************************************************************/
  #define INVERTER_EN_PIN        											P0_7 /* Active High */
  #define nBRAKE_PIN             											P1_3 /* Active Low */
  #define DRIVER_CS_AZ_PIN       											P1_2 /* Active High */
  #define nFAULT_PIN             											P3_4//P0_12 //P3_4/* Active Low */
  #define TRAP_PIN               											P0_12

  #if(USER_MOTOR_BI_DIRECTION_CTRL == ENABLED)
  #define MOTOR_DIR_INPUT_PIN    											P4_10
  #endif


/*********************************************************************************************************************
   * PWM gate signal configuration macros
   *****************************	*********/
  #define PHASE_U_HS_PIN         											P3_3
  #define PHASE_U_HS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

  #define PHASE_U_LS_PIN         											P3_2
  #define PHASE_U_LS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

  #define PHASE_V_HS_PIN         											P3_1
  #define PHASE_V_HS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

  #define PHASE_V_LS_PIN         											P3_0
  #define PHASE_V_LS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

  #define PHASE_W_HS_PIN         											P1_0
  #define PHASE_W_HS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

  #define PHASE_W_LS_PIN         											P1_1
  #define PHASE_W_LS_ALT_SELECT  											XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

  #define FAULT_LED3              										P4_7   /* FAULT LED indicator */
//  #define FW_ACTIVE_LED4          										P4_6   /* Flux Weakening Active LED indicator */

//  #define TEST_PIN                										P4_8   /* Test pin for timing measurement */

  #define USER_INVERTER_ENABLE_LEVEL                  XMC_GPIO_OUTPUT_LEVEL_HIGH     /*!< Inverter Enable - required pin level for PWM output*/
  #define USER_INVERTER_DISABLE_LEVEL                 XMC_GPIO_OUTPUT_LEVEL_LOW      /*!< Inverter Disable - required pin level for PWM output */

  #define USER_BRAKE_ENABLE_LEVEL                     XMC_GPIO_OUTPUT_LEVEL_LOW      /*!< Brake Enable - required pin level to enable braking */
  #define USER_BRAKE_DISABLE_LEVEL                    XMC_GPIO_OUTPUT_LEVEL_HIGH     /*!< Brake Disable - required pin level to enable braking*/

  /*********************************************************************************************************************
   * CCU8 Resources Configuration
   ********************************************************************************************************************/
  #define CCU8_MODULE             										CCU80
  #define CCU8_SLICE_PHASE_U      										CCU80_CC81
  #define CCU8_SLICE_PHASE_V      										CCU80_CC82
  #define CCU8_SLICE_PHASE_W      										CCU80_CC80

  #define CCU8_SLICE_ADC_TR       										CCU80_CC83

  #define USER_CCU8_PASSIVE_LEVEL_OUT0                CCU8_PASSIVE_LOW               /*!< PWM output passive level required for driver IC for high side */
  #define USER_CCU8_PASSIVE_LEVEL_OUT1                CCU8_PASSIVE_LOW               /*!< PWM output passive level required for driver IC for low side */

  /*!< Trap signal input level selection for ctrap to occur */
  #define USER_CCU8_INPUT_TRAP_LEVEL				  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW
//  #define USER_CCU8_INPUT_TRAP_LEVEL               	  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH
  /*********************************************************************************************************************
   * VADC Resources Configuration
   ********************************************************************************************************************/
  /* For simultaneous sampling */
  #define VADC_I1_GROUP         											VADC_G1
  #define VADC_I1_CHANNEL       											(0U)
  #define VADC_I1_RESULT_REG    											(0U)

  #define VADC_I2_GROUP         											VADC_G0
  #define VADC_I2_CHANNEL       											(0U)
  #define VADC_I2_RESULT_REG    											(0U)

  #define VADC_I3_GROUP         											VADC_G1
  #define VADC_I3_CHANNEL       											(1U)
  #define VADC_I3_RESULT_REG    											(1U)

  #define VADC_I4_GROUP         											VADC_G0     /* P2.8 VADC group0 channel 1 */
  #define VADC_I4_CHANNEL       											(1U)
  #define VADC_I4_RESULT_REG    											(1U)

  /* Motor Phase U VADC define */
  #define VADC_IU_G1_CHANNEL    											(3U)        /* P2.11, VADC group1 channel 3 */
  #define VADC_IU_G0_CHANNEL    											(4U)        /* P2.11, VADC group0 channel 4 */

  /* Motor Phase V VADC define */
  #define VADC_IV_G1_CHANNEL    											(2U)       /* P2.10, VADC group1 channel 2 */
  #define VADC_IV_G0_CHANNEL    											(3U)       /* P2.10, VADC group0 channel 3 */

  /* Motor Phase W VADC define */
  #define VADC_IW_G1_CHANNEL    											(4U)       /* P2.9, VADC group1 channel 4 */
  #define VADC_IW_G0_CHANNEL    											(2U)       /* P2.9, VADC group0 channel 2 */

  /* Sectors A. ADC sequences - Iv -> Iw -> Iu */
  /* VADC Group 0 Alias channel 0 and channel 1 */
  #define VADC_G0_CHANNEL_ALIAS0  										VADC_IW_G0_CHANNEL
  #define VADC_G0_CHANNEL_ALIAS1  										VADC_I4_CHANNEL

  /* VADC Group 1 Alias channel 0 and channel 1 */
  #define VADC_G1_CHANNEL_ALIAS0  										VADC_IV_G1_CHANNEL
  #define VADC_G1_CHANNEL_ALIAS1  										VADC_IU_G1_CHANNEL

/*E_BIKE*/
  #define VADC_IU_GROUP         											VADC_G0
  #define VADC_IU_GROUP_NO      											(0U)
  #define VADC_IU_CHANNEL       											(4U)       /* P2.11, VADC group0 channel 4 */
  #define VADC_IU_RESULT_REG    											(4U)

  #define VADC_IV_GROUP         											VADC_G0
  #define VADC_IV_GROUP_NO      											(0U)
  #define VADC_IV_CHANNEL       											(3U)       /* P2.10, VADC group0 channel 3 */
  #define VADC_IV_RESULT_REG    											(3U)

  #define VADC_IW_GROUP         											VADC_G0
  #define VADC_IW_GROUP_NO      											(0U)
  #define VADC_IW_CHANNEL       											(2U)       /* P2.9, VADC group0 channel 2 */
  #define VADC_IW_RESULT_REG    											(2U)

  /* DC link voltage VADC define */
  #define VADC_VDC_GROUP        											VADC_G1
  #define VADC_VDC_GROUP_NO     											(1U)
  #define VADC_VDC_CHANNEL      											(5U)       /* P2.3 VADC group1 channel 5 */
  #define VADC_VDC_RESULT_REG   											(5U)

  /* T_Sense Temp sensor(Motor) VADC define - For E_bike*/
  #define VADC_TEMP_MOTOR_GROUP       										VADC_G0
  #define VADC_TEMP_MOTOR_GROUP_NO    										(0U)
  #define VADC_TEMP_MOTOR_CHANNEL     										(7U)       /* P2.2 VADC group0 channel 7 */
  #define VADC_TEMP_MOTOR_RESULT_REG  										(7U)

  /* T_Sense Temp sensor(inverter) VADC define*/
  #define VADC_TEMP_GROUP       											VADC_G1
  #define VADC_TEMP_GROUP_NO    											(1U)
  #define VADC_TEMP_CHANNEL     											(6U)       /* P2.4 VADC group1 channel 6 */
  #define VADC_TEMP_RESULT_REG  											(6U)

  /* Potentiometer VADC define*/
  #define VADC_POT_GROUP        											VADC_G1
  #define VADC_POT_GROUP_NO     											(1U)
  #define VADC_POT_CHANNEL      											(7U)       /* P2.5 VADC group1 channel 7 */
  #define VADC_POT_RESULT_REG   											(7U)

  /* Brake VADC define - For E_bike */
  /*#define V_brake_switch 													P2_6*/
  #define VADC_E_BIKE_BRAKE_GROUP        									VADC_G0
  #define VADC_E_BIKE_BRAKE_GROUP_NO     									(0U)
  #define VADC_E_BIKE_BRAKE_CHANNEL      									(0U)       /* P2.6 VADC group0 channel 0 */
  #define VADC_E_BIKE_BRAKE_RESULT_REG   									(0U)

  /*************************** 6EDL7141 configuration *****************************************************************/
  #define USER_IU_ADC_BIAS                            (2048)                         /*!< Motor phase U current bias -default*/
  #define USER_IV_ADC_BIAS                            (2048)                         /*!< Motor phase V current bias -default*/
  #define USER_IW_ADC_BIAS                            (2048)                         /*!< Motor phase W current bias -default*/

  /*************************** AZ for 6EDL7141 through CCU4 ***********************************************************/
  #define GDRIVER_AZ_PWM_CCU4_MODULE                        (CCU40)
  #define GDRIVER_AZ_PWM_SLICE                              (CCU40_CC42)
  #define GDRIVER_AZ_PWM_SLICE_NUM                          (2U)
  #define GDRIVER_AZ_PWM_SLICE_SHADOW_TRANS_ENABLE_Msk      (XMC_CCU4_SHADOW_TRANSFER_SLICE_2)
  #define GDRIVER_AZ_PWM_ALT_OUT                            (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2)

  /*********************************************************************************************************************
   * POSIF configuration macros
   ***************************************/
  #define HALL_0_PIN             											P4_1		/* POSIF1.IN0B (A)*/
  #define HALL_1_PIN             											P4_2		/* POSIF1.IN1B (B)*/
  #define HALL_2_PIN             											P4_3		/* POSIF1.IN2B (C)*/

  #define POSIF_MODULE           											(POSIF1)

  #define POSIF_HALL_0_INSEL      										(XMC_POSIF_INPUT_PORT_B)
  /** Refer POSIF PCONF.INSEL1 in reference manual*/
  #define POSIF_HALL_1_INSEL      										(XMC_POSIF_INPUT_PORT_B)
  /** Refer POSIF PCONF.INSEL2 in reference manual*/
  #define POSIF_HALL_2_INSEL      										(XMC_POSIF_INPUT_PORT_B)

  #define POSIF_PATTERN_UPDATE_SEL   									(XMC_POSIF_INPUT_PORT_A)
  #define POSIF_PWM_SYNC_SIGNAL_SEL  									(XMC_POSIF_INPUT_PORT_A)

  #define CCU8_ONEMATCH_SR        										(XMC_CCU8_SLICE_SR_ID_0)

  #define HALL_CCU4_MODULE            								CCU41
  #define HALL_CCU4_MODULE_NUM        								1U
  #define HALL_DELAY_SLICE            								CCU41_CC40
  #define HALL_DELAY_SLICE_NUM        								0U
  #define HALL_CAPTURE_SLICE          								CCU41_CC41
  #define HALL_CAPTURE_SLICE_NUM      								1U

  /*************************** BEMF detection *************************************************************************/
  /* Make sure all are from same group. If not then scan trigger both the groups for conversion */
/*Not needed for E-Bike*/
//  /* BEMF_U VADC define*/
//  #define VADC_BEMF_U_GROUP         									VADC_G0     /* P2.2 VADC group0 channel 7 */
//  #define VADC_BEMF_U_CHANNEL       									(7U)
//  #define VADC_BEMF_U_RESULT_REG    									(7U)
//
//  /* BEMF_V VADC define*/
//  #define VADC_BEMF_V_GROUP         									VADC_G0     /* P2.1 VADC group0 channel 6 */
//  #define VADC_BEMF_V_CHANNEL       									(6U)
//  #define VADC_BEMF_V_RESULT_REG    									(6U)
//
//  /* BEMF_W VADC define*/
//  #define VADC_BEMF_W_GROUP         									VADC_G0     /* P2.0 VADC group0 channel 5 */
//  #define VADC_BEMF_W_CHANNEL       									(5U)
//  #define VADC_BEMF_W_RESULT_REG    									(5U)

  /* ********************************************************************************************************************/
  /* NVIC Interrupt Resources Configuration */
  /* ********************************************************************************************************************/
  #define CCU80_0_IRQn                                IRQ25_IRQn          /*!< CCU80 SR0 Interrupt  */
  #define PMSM_FOC_FCL_ISR                            IRQ25_Handler       /*!< Fast Control Loop(FCL) - PWM Period Match*/

  #define TRAP_IRQn                                   IRQ26_IRQn          /*!< CCU80 SR1 Interrupt  */
  #define PMSM_FOC_CTRAP_ISR                          IRQ26_Handler       /*!< CTRAP - CCU8 - PWM */

  #define POSIF_IRQn			                      IRQ27_IRQn
  #define POSIF_IRQHandler                     		  IRQ27_Handler
  #define POSIF_SR_INT                                XMC_SCU_IRQCTRL_POSIF1_SR0_IRQ27


  #define FAULT_PIN_ERU_IRQn                          IRQ6_IRQn           /*!< ERU */
  #define PMSM_FOC_DRIVER_nFAULT_ISR                  IRQ6_Handler        /*!< ERU - 6EDL7141 nFault pin interrupt handler */

  /* NVIC ISR handler mapping */
  #define PMSM_FOC_SCL_ISR                            SysTick_Handler     /*!< Slow Control Loop(SCL) - Systick */

  /* ********************************************************************************************************************/
  /* NVIC Interrupt Priority Configuration */
  /* ********************************************************************************************************************/
  /* Interrupt priority configurations - 0 is the highest priority and 3 is the lowest */
  #define PMSM_FOC_FCL_NVIC_PRIO                      (1U)  /*!< FAST Control loop - Executed every PWM period */
  #define PMSM_FOC_SCL_NVIC_PRIO                      (2U)  /*!< Slow Control loop  - SysTick */
  #define PMSM_FOC_CTRAP_NVIC_PRIO                    (0U)  /*!< CTRAP   */
  #define PMSM_FOC_FAULT_NVIC_PRIO                    (1U)  /*!< nFault from 6EDL7141 */
  #define PMSM_FOC_POSIF_NVIC_PRIO                    (0U)

#endif  /* end of #if(MCUTYPE == MCU_CARD_XMC1404)*/

/*********************************************************************************************************************
 * DEBUGGING ENABLE Resources Configuration
 ********************************************************************************************************************/
#define DEBUG_PWM_0_ENABLE                             (ENABLED)        /* 1 = Enable Debug PWM, 0 = Disable Debug PWM */
#define DEBUG_PWM_1_ENABLE                             (DISABLED)        /* 1 = Enable Debug PWM, 0 = Disable Debug PWM */

#define DEBUG_PWM_CCU4_MODULE                          (CCU40)

/* Debug Period Value controls the resolution of the PWM.
 * This is the value that goes into the PWM period register.
 */
#define DEBUG_PWM_PERIOD_CNTS                          (4736U)    /* tys 4800 -64 = 4736 */

/* Initial Duty Cycle of Debug PWM Channels */
#define DEBUG_PWM_50_PERCENT_DC_CNTS                   ((uint16_t)(DEBUG_PWM_PERIOD_CNTS >> 1))

///* P1.4 */
//#if (DEBUG_PWM_0_ENABLE == ENABLED)
//#define DEBUG_PWM_0_SLICE                              (CCU41_CC40)
//#define DEBUG_PWM_0_SLICE_NUM                          (0U)
//#define DEBUG_PWM_0_SLICE_SHADOW_TRANS_ENABLE_Msk      (XMC_CCU4_SHADOW_TRANSFER_SLICE_0)
//#define DEBUG_PWM_0_PORT                               (XMC_GPIO_PORT1)
//#define DEBUG_PWM_0_PIN                                (4U)
//#define DEBUG_PWM_0_ALT_OUT                            (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT9)
//#endif  /*DEBUG_PWM_0_ENABLE == 1*/

/* P4.8 tys */
#if (DEBUG_PWM_0_ENABLE == ENABLED)
#define DEBUG_PWM_0_SLICE                              (CCU40_CC40)
#define DEBUG_PWM_0_SLICE_NUM                          (0U)
#define DEBUG_PWM_0_SLICE_SHADOW_TRANS_ENABLE_Msk      (XMC_CCU4_SHADOW_TRANSFER_SLICE_0)
#define DEBUG_PWM_0_PORT                               (XMC_GPIO_PORT4)
#define DEBUG_PWM_0_PIN                                (8U)
#define DEBUG_PWM_0_ALT_OUT                            (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT6)
#endif  /*DEBUG_PWM_0_ENABLE == 1*/

/* dont use P4.2 */
#if (DEBUG_PWM_1_ENABLE == ENABLED)
#define DEBUG_PWM_1_SLICE                              (CCU40_CC42)
#define DEBUG_PWM_1_SLICE_NUM                          (2U)
#define DEBUG_PWM_1_SLICE_SHADOW_TRANS_ENABLE_Msk      (XMC_CCU4_SHADOW_TRANSFER_SLICE_1)
#define DEBUG_PWM_1_PORT                               (XMC_GPIO_PORT4)
#define DEBUG_PWM_1_PIN                                (2U)
#define DEBUG_PWM_1_ALT_OUT                            (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5)
#endif  /*DEBUG_PWM_1_ENABLE == 1 */

/* Tmp_CRS = 0 or (- Tmp_CRS) if Tmp_CRS < 0. */
#define REVERSE_CRS_OR_0  (- Tmp_CRS)

/**
 * @}
 */

/**
 * @}
 */
#endif /* PMSM_FOC_MCUCARD_PARAMETERS_H_ */
