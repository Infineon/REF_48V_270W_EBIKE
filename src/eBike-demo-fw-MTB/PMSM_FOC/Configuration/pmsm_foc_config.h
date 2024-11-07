/**
 * @file pmsm_foc_config.h
 * @date 15 Feb, 2023
 *
 * @cond
 *********************************************************************************************************************
 * PMSM FOC Motor Control Library
 *
 * Copyright (c) 2023, Infineon Technologies AG
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
#ifndef PMSM_FOC_CONFIG_H_
#define PMSM_FOC_CONFIG_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "../Configuration/pmsm_foc_const.h"

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
   
   
/********* Motor Parameter Configuration ************/   
#define MOTOR_LQ_SCALEDUP 8732 
#define MOTOR_LD_SCALEDUP 8732 
#define MOTOR_SCALE_OF_L 15 
#define MOTOR_R_SCALEDUP 2240 
#define MOTOR_SCALE_OF_R 14 
#define MOTOR_FLUX_LINKAGE_CONSTANT 903 
#define MOTOR_SCALE_OF_FLUX_LINKAGE_CONSTANT 13 
#define MOTOR_POLE_PAIR 3 
   
/********* Device Access ************/   
#define USER_AUTHENTICATION_ID 35 
#define USER_PASSWORD 4281791882 
   
/********* Phase Current sensing Configuration ************/   
#define USER_CURRENT_SENSING THREE_SHUNT_SYNC_CONV 
   
/********* FOC Control loop topology selection ************/   
#define USER_FOC_CTRL_SCHEME TORQUE_CTRL 
   
/********* Motor Direction control ************/   
#define USER_MOTOR_BI_DIRECTION_CTRL DISABLED 
   
/********* Motor Startup configuration ************/   
#define USER_ROTOR_IPD_METHOD ROTOR_IPD_PRE_ALIGNMENT 
#define USER_MOTOR_STARTUP_METHOD MOTOR_STARTUP_DIRECT_FOC 
   
/********* Motor Stop configuration ************/   
#define USER_MOTOR_STOP_METHOD MOTOR_STOP_COASTING 
   
/********* Micrium/MicroInspector Enable/Disable ************/   
#define USER_UCPROBE_GUI ENABLED 
#define USER_UCPROBE_OSCILLOSCOPE ENABLED 
   
/********* Voltage/speed reference setting ************/   
#define USER_REF_SETTING MICRIUM_UC_ONLY 
#define USER_TH_POT_ADC_START 20 
#define USER_TH_POT_ADC_STOP 10 
#define USER_POT_ADC_LPF 1 
   
/********* Temperature Sensor Configuration ************/   
#define TEMP_SENSOR_OFFSET 393 
#define TEMP_SENSOR_COEFFICIENT_Q15 2100 
   
   
/********* MCU Parameter Configuration ************/   
#define USER_PCLK_FREQ_MHz 96 
#define CCU8_PERIOD_REG 4799 
#define CCU8_DEADTIME_RISE 73 
#define CCU8_DEADTIME_FALL 73 
#define DEAD_TIME_STALL_FLAG_COMPENSATION ENABLED 
#define DT_COMPENSATION_V_INT 398 
   
/********* DRIVER IC Delay Configuration ************/   
#define GDRIVER_IC_DELAY 19 
   
/********* Slow Control loop(Systick Configuration) ************/   
#define PMSM_FOC_SYSTICK_COUNT 9600 
   
/********* Brake/Coasting/Bootstrap/Precharge Configuration ************/   
#define BOOTSTRAP_BRAKE_TIME 400 
#define BOOTSTRAP_PRECHARGE_TIME 400 
#define BRAKE_MIN_SPEED 2668 
   
/********* V/F Startup Configuration ************/   
#define STARTUP_VF_TRANSITION_SPEED 3048 
#define STARTUP_VF_OFFSET 512 
#define STARTUP_VF_V_PER_HZ_CONST 0 
#define STARTUP_VF_SPEED_RAMP_UP_RATE 52 
#define STARTUP_VF_STABILIZATION_COUNT 1000 
   
/********* Direct FOC Startup Configuration ************/   
#define VQ_INITIAL_VALUE 205 
#define TRANSITION_FIRST_KICK_VALUE 717 
#define TRANSITION_FIRST_KICK_COUNT 160 
   
/********* SVPWM Configuration ************/   
#define USER_SVM_SWITCHING_SCHEME STANDARD_SVM_7_SEGMENT 
#define USER_SVM_SINE_LUT_SIZE 1024 
#define USER_SVM_OVERMODULATION DISABLED 
#define SCALE_UP_SINE_LUT 2 
#define SVM_LUT_SCALE 22166 
#define SVPWM_MAX_VREF 28377 
#define  ADC_TRIGGER_DELAY 96 
#define SVPWM_T_MIN 179 
#define SVPWM_T0_THRESHOLD 794 
   
/********* Motor STALL Detection Configuration ************/   
#define USER_MOTOR_STALL_DETECTION_ENABLE DISABLED 
#define STALL_CURRENT 1049 
#define STALL_THRESHOLD_TIME 100 
#define STALL_COEFFICIENT_SCALE 13 
#define STALL_COEFFICIENT_SCALEUP 3277 
#define MOTOR_LS_X_PWMFREQ_SCALEUP 32320 
#define MOTOR_SCALE_OF_LS_X_PWMFREQ 13 
   
   
/********* Watchdog Configuration ************/   
#define USER_WATCH_DOG_TIMER DISABLED 
   
/********* Over Current Protection(OCP) Configuration ************/   
#define USER_OVERCURRENT_PROTECTION_ENABLE ENABLED 
#define OCP_LEVEL 33360 
#define OCP_LEVEL_TIME 20 
#define USER_MAP_CURRENT_TO_DCLINK DISABLED 
   
#define USER_OCP_LEVELS_PROTECTION DISABLED 
   
#if(USER_OCP_LEVELS_PROTECTION == ENABLED)   
#define USER_OCP_LEVEL_A 33360 
#define USER_OCP_LEVEL_A_TIME 20 
#define USER_OCP_LEVEL_B 14827 
#define USER_OCP_LEVEL_B_TIME 1000 
#define USER_OCP_LEVEL_C 10379 
#define USER_OCP_LEVEL_C_TIME 8000 
#endif   
   
   
/********* Rotor Pre Alignment Configuration ************/   
#define ROTOR_PRE_ALIGNMENT_CURRENT_REF 524 
#define ROTOR_PRE_ALIGNMENT_COUNT 6000 
#define ROTOR_ZERO_DEG_ID_SVPWM_REF 1024 
#define ROTOR_ZERO_DEG_ID_RAMPUP_STEP 3 
#define ROTOR_ZERO_DEG_ID_COUNT 2000 
   
/********* Initial Position Detection(IPD) Configuration ************/   
#define IPD_SENSE_PRESCALAR_VAL 5 
#define IPD_SENSE_CCU8_PULSE_WIDTH_COUNT 195 
#define IPD_SENSE_CCU8_CURRENT_DECAY_COUNT 195 
   
/********* DC Link Voltage ************/   
#define VADC_DCLINK_T 3000 
   
/********* DC Link Compensation ************/   
#define USER_VDC_VOLT_COMPENSATION DISABLED 
   
/********* Under/Over Voltage Protection ************/   
#define USER_VDC_UV_OV_PROTECTION_ENABLE ENABLED 
#define VDC_OVER_VOLTAGE_LIMIT 3600 
#define VDC_UNDER_VOLTAGE_LIMIT 2400 
#define USER_VDC_LPF 2 
   
/********* Voltage clamping during deceleration ************/   
#define USER_DECELERATION_VDC_CLAMPING_ENABLE DISABLED 
   
/********* Calibration - ADC, Phase offset ************/   
#define USER_ADC_CALIBRATION ENABLED 
#define USER_MOTOR_PH_OFFSET_CALIBRATION ENABLED 
   
/********* id/iq decoupling ************/   
#define USER_DQ_DECOUPLING DISABLED 
   
   
/********* Inverter board over temperature proetction Configuration ************/   
#define USER_BOARD_OVERTEMP_PROTECTION_ENABLE DISABLED 
#define USER_MAX_BOARD_TEMP_START_C 60 
#define USER_ABS_MAX_BOARD_TEMP_C 100 
   
/*********Max. current limit ************/   
#define USER_MAX_CURRENT_LIMIT_ENABLE DISABLED 
#define LIMIT_MAX_IS 32767 
   
/*********Max. Torque limit ************/   
#define USER_TORQUE_LIMIT_ENABLE DISABLED 
#define LIMIT_MAX_IQ 524 
#define IQ_LIMIT_BLANKING_TIME 30000 
   
/*********Flux Weakening ************/   
#define USER_FW_ENABLE DISABLED 
#define FW_MAX_VREF 28377 
#define FW_ID_REF_NEG_MAX -2621 
#define FW_ID_REF_RATIO_Q15 -19559 
#define FW_DISABLE_IQ_LIMIT 20971 
#define USER_FW_LPF 5 
   
/*********Electric Speed ************/   
#define ELECTRICAL_SPEED_LOW_LIMIT_TS 152 
#define ELECTRICAL_SPEED_HIGH_LIMIT_TS 32767 
   
/*********Rotor angle to speed conversion ************/   
#define SPEED_TO_ANGLE_CONV_FACTOR 22545 
#define SPEED_TO_ANGLE_CONV_FACTOR_SCALE 4 
   
/*********Hall FOC macro definitions ************/   
#if(MOTOR_CTRL_SCHEME == PMSM_HALL_FOC)   
#define HALL_CLK_PRE_SCALE 5 
#define HALL_RAD_PER_PWM_RES_INC_FACTOR 457467284 
#define USER_ALIGNMENT_VREF_SLEWRATE 25 
#define USER_HALL_EDGE_COUNT 36 
#define USER_STALL_COUNT 800000 
#define KpHallPLL 983 
#define KpHallPLL_SCALE 15 
#define HALL_SEC__PWM_CYCLE_CONV 150 
#endif   
   
/*********Phase Current sampling time ************/   
#define VADC_PH_CURRENT_SAMPLE_TIME 192 
   
/*********Vq Voltage Control Scheme ************/   
#define VQ_REF_HIGH_LIMIT 32767 
#define VQ_REF_LOW_LIMIT 0 
#define VQ_RAMP_UP_STEP 31 
#define VQ_RAMP_DOWN_STEP 1 
#define VQ_RAMP_STEP_TIME_FCL_COUNT 6 
   
/************* Torque Control Scheme ************************/   
#define IQ_REF_HIGH_LIMIT 20971 
#define IQ_REF_LOW_LIMIT 0 
#define IQ_RAMP_UP_STEP 16 
#define IQ_RAMP_DOWN_STEP 4 
#define IQ_RAMP_STEP_TIME_FCL_COUNT 200 
   
   
/*********Speed Control Scheme ************/   
#define SPEED_REF_HIGH_LIMIT_TS 32767 
#define SPEED_REF_LOW_LIMIT_TS 762 
#define SPEED_RAMP_UP_STEP 1 
#define SPEED_RAMP_DOWN_STEP 601 
#define SPEED_RAMP_STEP_TIME_FCL_COUNT 263 
   
/*********Torque Current PI Controller Parameters ************/   
#define PI_TORQUE_SCALE_KPKI 13 
#define PI_TORQUE_KP 5077 
#define PI_TORQUE_KI 176 
#define PI_TORQUE_UK_LIMIT_MIN -32768 
#define PI_TORQUE_UK_LIMIT_MAX 32767 
#define PI_TORQUE_IK_LIMIT_MIN -268435456 
#define PI_TORQUE_IK_LIMIT_MAX 268427264 
#define PI_TORQUE_ENABLE_ANTIWINDUP ENABLED 
   
/*********Flux Current PI Controller Parameters ************/   
#define PI_FLUX_SCALE_KPKI 13 
#define PI_FLUX_KP 5077 
#define PI_FLUX_KI 176 
#define PI_FLUX_UK_LIMIT_MIN -32768 
#define PI_FLUX_UK_LIMIT_MAX 32767 
#define PI_FLUX_IK_LIMIT_MIN -268435456 
#define PI_FLUX_IK_LIMIT_MAX 268427264 
#define PI_FLUX_ENABLE_ANTIWINDUP ENABLED 
   
   
/*********Speed PI Controller Parameters ************/   
#define PI_SPEED_KP 4588 
#define PI_SPEED_KI 1 
#define PI_SPEED_SCALE_KPKI 7 
#define PI_SPEED_UK_LIMIT_MAX 32767 
#define PI_SPEED_UK_LIMIT_MIN 0 
#define PI_SPEED_IK_LIMIT_MAX 4194176 
#define PI_SPEED_IK_LIMIT_MIN -4194304 
#define PI_SPEED_ENABLE_ANTIWINDUP ENABLED 
   
/*********Flux weakening current PI Controller Parameters ************/   
#define PI_FW_KP 102 
#define PI_FW_KI 14 
#define PI_FW_SCALE_KPKI 13 
#define PI_FW_UK_LIMIT_MAX 1 
#define PI_FW_UK_LIMIT_MIN -5243 
#define PI_FW_IK_LIMIT_MAX 8192 
#define PI_FW_IK_LIMIT_MIN -42950656 
#define PI_FW_ENABLE_ANTIWINDUP ENABLED 
   
/*********IPD Multi channel pattern update ************/   
#define WH_VL_UOFF 513 
#define WOFF_VL_UH 528 
#define WL_VOFF_UH 18 
#define WL_VH_UOFF 258 
#define WOFF_VH_UL 288 
#define WH_VOFF_UL 33 
#define WL_VL_UH 530 
#define WH_VH_UL 289 
#define WL_VH_UH 274 
#define WH_VL_UL 545 
#define WL_VH_UL 290 
#define WH_VL_UH 529 
#define WL_VL_UL 546 
#define ALL_ON 819 
#define ALL_OFF 0 
   
/*********CCU synchronous start ************/   
#define PMSM_FOC_CCU8_SYNC_START 256 
#define PMSM_FOC_CCU4_SYNC_START 1 
#define PMSM_FOC_CCU8_CCU4_SYNC_START 257 
   
/*********Critical code execution from RAM/FLASH ************/   
#define PMSM_FOC_RAM_ATTRIBUTE __RAM_FUNC 
   
/*********6EDL7141 Gate Driver Configuration ************/   
#define USER_GD_PVCC_SETPT PVCC_15V 
#define USER_CP_CLK_FREQ_kHz CP_CLK_1562_5kHz 
#define USER_CP_CLK_SS_EN_DIS CP_CLK_SS_EN 
#define USER_GD_I_PRE_EN PRECHAR_MODE_EN 
#define USER_GD_IPRE_SRC IDRIVE_PRE_40mA 
#define USER_GD_IPRE_SINK IDRIVE_PRE_60mA 
#define USER_GD_IHS_SRC IDRIVE_40mA 
#define USER_GD_IHS_SINK IDRIVE_60mA 
#define USER_GD_ILS_SRC IDRIVE_40mA 
#define USER_GD_ILS_SINK IDRIVE_60mA 
#define USER_GD_TDRIVE1 TDRIVE1_0ns 
#define USER_GD_TDRIVE2 TDRIVE2_450ns 
#define USER_GD_TDRIVE3 TDRIVE3_0ns 
#define USER_GD_TDRIVE4 TDRIVE4_880ns 
#define USER_GD_DT_RISE DT_840ns 
#define USER_GD_DT_FALL DT_840ns 
#define USER_GD_CSOPAMP_GAIN CS_GAIN_8V 
   
/*********Gate driver idle shutdown configuration************/   
#define SYSTEM_IDLE_TIME_COUNT 30000 
   
/*********GUI Parameter display ************/   
#define CONVERT_SPEED_TO_RPM 2150 
#define SPEED_TO_RPM_SCALE 14 
#define VDC_ADC_TO_mV 16383 
#define VDC_ADC_TO_mV_SCALE 10 
   
/*********E Bike ************/   
#define E_BIKE_REF DISABLED 
#define POT_MV_EN_DIS ENABLED 
#define WINDOW_SIZE 8 
#define AVG_ECYCLE_NUM 3 
#define WINDOW_SIZE_DIV_SCALE 3 
#define E_ECO_SCALE_OF_T 14 
#define E_ECO_T_SCALEDUP 11204 
#define E_NORMAL_SCALE_OF_T 14 
#define E_NORMAL_T_SCALEDUP 14005 
#define E_SPORT_SCALE_OF_T 14 
#define E_SPORT_T_SCALEDUP 28011 
   
   
#define E_BIKE_IQ_RAMP_DOWN_STEP 1 
#define E_BIKE_RAMP_STEP_TIME_FCL_COUNT 1 
#define E_BIKE_POWER_RAW_SCALE_UP 24164496 
#define MAX_POWER_LIMIT_EN_DIS DISABLED 
#define MAX_ASSIST_TORQUE_LIMIT_EN_DIS DISABLED 
   
#define  HALL_OFFSET_COMPENSATION_ANGLE_INIT 59652324 
   
#define VQ_REF_FIXED_VALUE 1000 
#define DEMO_SETUP DISABLED 
/**
 * @}
 */

/**
 * @}
 */
#endif /* PMSM_FOC_CONFIG_H_ */   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
