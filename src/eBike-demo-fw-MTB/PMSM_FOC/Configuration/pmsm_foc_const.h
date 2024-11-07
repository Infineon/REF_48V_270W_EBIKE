/**
 * @file pmsm_foc_const.h
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

#ifndef PMSM_FOC_CONST_H_
#define PMSM_FOC_CONST_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <xmc_common.h>

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

/* Hardware kit. Refer to the header file description above */
#define   KIT_XMC1X_AK_MOTOR_001                    (1U)
#define   KIT_CHIMERA_XMC                           (2U)
#define   LEV_4KW_KIT                               (3U)
#define   KIT_CHIMERAXM_EVAL                        (4U)
#define	  E_BIKE_MOTOR								(5U)
#define	  LIN_MOTOR									(6U)
#define	  AA_MOTOR									(7U)
#define   CUSTOM_KIT                                (8U)
 
#define   MCUTYPE                                   MCU_XMC1404_ebike //MCU_IMD701//MCU_CARD_XMC1404//MCU_XMC1404_ebike//MCU_IMD_1_5kw_ref

#define  PMSM_FOC_HARDWARE_KIT                      E_BIKE_MOTOR
                                            											/* 1. KIT_XMC1X_AK_MOTOR_001
																						2. KIT_CHIMERA_XMC
   																						3. LEV_4KW_KIT
																						4. KIT_CHIMERAXM_EVAL
																						5. E_BIKE_MOTOR
																						6. LIN_MOTOR
																						7. AA_MOTOR
																						*/
#define   PMSM_LV15W                                (1U)
#define   LEV_4KW_INVERTER                          (2U)

/***************************************Inverter card selection *************************************************/
#if(PMSM_FOC_HARDWARE_KIT == LEV_4KW_KIT)
#define INVERTERCARD_TYPE                           LEV_4KW_INVERTER
#elif(PMSM_FOC_HARDWARE_KIT == KIT_XMC1X_AK_MOTOR_001)
#define INVERTERCARD_TYPE                           PMSM_LV15W
#endif

/***************************************MCU selection *************************************************/
#define MCU_IMD701                          1
#define MCU_CARD_XMC1404                    2
#define MCU_XMC1404_ebike					3
#define MCU_IMD_1_5kw_ref 					4

#define MOTOR_CTRL_SCHEME							PMSM_HALL_FOC					// 1. PMSM_SENSORLESS_FOC	   	   2. PMSM_HALL_FOC

#define BOTH_OFF  0
#define HIGH_ON 1
#define LOW_ON  2
#define BOTH_ON 3


#define IMD700A 1
#define SPI_LINK  0

// Device Definitions
#define DEVICE    (IMD700A)

/* 6EDL7141 device voltage levels */
#define IMD700A_DVDD_3_3_V  1
#define IMD700A_DVDD_5_0_V  2

/* SVPWM */
#define STANDARD_SVM_7_SEGMENT  1
#define STANDARD_SVM_5_SEGMENT  2
#define SVPWM_MAX_OVERMOD_RANGE 32767
#define SVPWM_MAX_LINEAR_RANGE  28377

/* CCU8 Passive LEVEL definition */
#define CCU8_PASSIVE_LOW  0
#define CCU8_PASSIVE_HIGH 1

/* UART USIC Channel Configuration */
#define USIC_DISABLED_ALL 0
#define USIC0_CH0_P1_4_P1_5 1
#define USIC0_CH1_P1_2_P1_3 2


/* Current Sensing Feedback Scheme */
#define THREE_SHUNT_ASYNC_CONV  1
#define THREE_SHUNT_SYNC_CONV 2

/* Control Scheme */
#define VQ_VOLTAGE_CTRL 0
#define TORQUE_CTRL 1
#define SPEED_INNER_CURRENT_CTRL  2
#define VF_OPEN_LOOP_CTRL 3

/* Motor startup configuration */
#define ROTOR_IPD_NONE  0
#define ROTOR_IPD_PRE_ALIGNMENT 1
#define ROTOR_IPD_INDUCTIVE_SENSING 2

#define MOTOR_STARTUP_DIRECT_FOC  0
#define MOTOR_STARTUP_VF_OPEN_LOOP  1

/* Motor stop configuration */
#define MOTOR_STOP_COASTING 1
#define MOTOR_STOP_LOW_SIDE_BRAKE 0

/* Motor status configuration */
#define PMSM_FOC_MOTOR_STATUS_STABLE  0
#define PMSM_FOC_MOTOR_STATUS_TRANSITION  1

#define ENABLED 1
#define DISABLED  0
#define TRUE  1
#define FALSE 0

/* OCP */
#define SHIFT_BIAS_LPF  2

/* OCP */
#define MOTOR_PHASE_CURRENT_SENSE 0
#define DC_LINK_CURRENT_SENSE 1

/* Reference Speed Adjustment Method */
#define MICRIUM_UC_ONLY 1
#define BY_POT_ONLY 2

/* IPD angle in q31 format */
#define PMSM_FOC_ANGLE_000_DEGREE_Q31 0
#define PMSM_FOC_ANGLE_015_DEGREE_Q31 178956971
#define PMSM_FOC_ANGLE_030_DEGREE_Q31 357913941
#define PMSM_FOC_ANGLE_060_DEGREE_Q31 715827883
#define PMSM_FOC_ANGLE_090_DEGREE_Q31 1073741824
#define PMSM_FOC_ANGLE_120_DEGREE_Q31 1431655765
#define PMSM_FOC_ANGLE_150_DEGREE_Q31 1789569707
#define PMSM_FOC_ANGLE_180_DEGREE_Q31 2147483647
#define PMSM_FOC_ANGLE_210_DEGREE_Q31 -1789569707
#define PMSM_FOC_ANGLE_240_DEGREE_Q31 -1431655765
#define PMSM_FOC_ANGLE_270_DEGREE_Q31 -1073741824
#define PMSM_FOC_ANGLE_300_DEGREE_Q31 -715827883
#define PMSM_FOC_ANGLE_330_DEGREE_Q31 -357913941
#define PMSM_FOC_ANGLE_345_DEGREE_Q31 -178956971
#define PMSM_FOC_ANGLE_INVALID  555

#define PMSM_FOC_ANGLE_N15_DEGREE_Q31 -178956970

/* CORDIC */
#define CORDIC_CIRCULAR_VECTORING_MODE  98
#define CORDIC_CIRCULAR_ROTATION_MODE 106
#define CORDIC_HYPERBOLIC_VECTORING_MODE  102
#define CORDIC_CIRCULAR_MPS_BY_K_SCALED 311
#define CORDIC_HYPERBOLIC_MPS_BY_K_SCALED 618
#define CORDIC_MPS_BY_K_SCALE 8
#define CORDIC_SHIFT  14

/* Motor direction input */
#define DIRECTION_INC 1
#define DIRECTION_DEC -1
#define DIRECTION_INVALID 0

/* GPIO input check */
#define IS_GPIO_HIGH  1
#define IS_GPIO_LOW 0

/* MOSFET on inverter board */
#define BSC007N04N04LS6_2X  1
#define BSC030N08NS5_2X 2
#define BSC007N04N04LS6_1X  3

/* Example code version*/
#define EXAMPLECODE_FIRMWARE_VERSION  (0x00000100)  /*!< MSB to LSB description MSB:Nil, Major=0x00, Minor=0x01 , Patch=0x00 */

#define RESET_BMI_ENABLE 			0
#if(RESET_BMI_ENABLE == 1)
/* defined in xmc1_scu.c but not recognized, so copy here instead */
#define ROM_BmiInstallationReq \
        (*((uint32_t (**)(uint32_t requestedBmiValue))0x00000108U)) /**< Pointer to Request BMI installation routine is
                                                                         available inside ROM. */
#define SPD0 						(0xF9C3)                 // SPD0 (P0.14) with User Mode Debug
#define SWD0 						(0xF8C3)                 // SWD0 (P0.14, P0.15) with User Mode Debug
#define PRODUCTIVE  				(0xF8C1)
#define ASC_BSL 					(0xFFC0)
#endif

#define SPI_LIB ENABLED		/*! < ENABLED | DISABLED >*/

/**
 * @}
 */

/**
 * @}
 */
#endif /* PMSM_FOC_CONST_H_ */
