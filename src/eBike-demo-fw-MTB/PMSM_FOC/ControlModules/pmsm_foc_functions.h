/**
 * @file pmsm_foc_functions.h
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

#ifndef PMSM_FOC_FUNCTIONS_H
#define PMSM_FOC_FUNCTIONS_H

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup ControlModules
 * @{
 */

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

#include <PMSM_FOC/Configuration/pmsm_foc_config.h>
#include <PMSM_FOC/Configuration/pmsm_foc_const.h>

#include <PMSM_FOC/FOCLib/BMCLIB/bmclib_cart2pol.h>
#include <PMSM_FOC/FOCLib/BMCLIB/bmclib_hypermag.h>
#include <PMSM_FOC/FOCLib/BMCLIB/bmclib_park_transform.h>
#include <PMSM_FOC/FOCLib/BMCLIB/bmclib_pt1_filter.h>
#include <PMSM_FOC/FOCLib/BMCLIB/bmclib_clarke_transform.h>
#include <PMSM_FOC/FOCLib/BMCLIB/bmclib_pi.h>
#include <PMSM_FOC/FOCLib/BMCLIB/bmclib_ramp_gen.h>
#include <PMSM_FOC/FOCLib/BMCLIB/bmclib_svpwm.h>
#include <PMSM_FOC/FOCLib/BMCLIB/bmclib_v_hz_profile.h>

#include "../MCUInit/pmsm_foc_clock.h"
#include "../MCUInit/pmsm_foc_adc.h"
#include "../MCUInit/pmsm_foc_wdt.h"
#include "../MCUInit/pmsm_foc_ccu8.h"
#include "../MCUInit/pmsm_foc_cordic.h"
#include "../MCUInit/pmsm_foc_posif.h"

#include "../uCProbe/uCProbe.h"

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#define IS_CORDIC_BUSY                       (MATH->STATC & 0x01)          /*!< Returns 1 if CORDIC is busy */
#define IS_MATH_DIV_BUSY                     (MATH->DIVST & 0x01)          /*!< Returns 1 if DIVIDER is busy */
#define SWAP(x,y)                            int32_t t;t=x;x=y;y=t;        /*!< Swap two numbers */
#define SYSTEM_BE_IDLE                       ((ADC.adc_res_pot < USER_TH_POT_ADC_STOP) || (PMSM_FOC_CTRL.motor_start_flag == 0))    /*!< Reference is too low then go to IDLE state */
#define BRAKE_RELEASE						 (ADC.adc_res_pot2 < 10)

/*********************************************************************************************************************
 * ENUMS
 ********************************************************************************************************************/
/**
 * @brief This enumerates the motor control scheme options.
 */
typedef enum
{
      VQ_VOLTAGE_CTRL_SCHEME,          /*!< 00 - Vq Voltage Control  */
	  TORQUE_CTRL_SCHEME,			   /*!< 01 - Torque Control Loop */
      SPEED_INNER_CURRENT_CTRL_SCHEME, /*!< 02 - Speed Inner Current Control Loop */
      VF_OPEN_LOOP_CTRL_SCHEME         /*!< 03 - V/F Open Loop Control  */
} PMSM_FOC_CTRL_SCHEME_t;

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
typedef void (*PMSM_FOC_CTRL_SCHEME_FUNC_t)(void);


/**  @brief FOC Input structure */
typedef struct
{
    PMSM_FOC_CTRL_SCHEME_t user_ctrl_scheme; /*!< User configured control scheme */

  	int16_t i_u;                              /*!<  Motor phase current - Iu */
  	int16_t i_v;                              /*!<  Motor phase current - Iv */
  	int16_t i_w;                              /*!<  Motor phase current - Iw */

  	int16_t ref_id;                           /*!<  id reference used for Flux PI control when enabled or else from user */
  	int16_t ref_iq;                           /*!<  iq reference used for Torque PI control,generated from speed PI control or else from user */
  	int32_t ref_iq_temp;

  	int32_t ref_speed;                        /*!<  Motor reference speed in speed inner current control loop */
  	int32_t ref_vq;                           /*!<  Reference voltage in vq voltage control loop */

  	uint16_t limit_max_vref;                  /*!<  limit SVPWM amplitude */
  	int16_t limit_max_is;                     /*!<  Limit maximum stator current */
  	int16_t limit_max_neg_fw_ref_id;          /*!<  Maximum -ve flux reference current for field weakening */
  	int16_t ref_rotor_pre_align_current;      /*!< Rotor pre-alignment current limit */

  	/*e-bike functionality*/
  	int16_t assist_limit_ref_iq;
  	int16_t power_limit_ref_iq;

} PMSM_FOC_INPUT_t;

/**  @brief FOC Output Structure */
typedef struct
{
    int32_t flux_vd;                               /*!<  Flux PI controller output vd */
    int32_t torque_vq;                             /*!<  Torque PI controller output vq */

    int16_t iq_filtered;                           /*!< Iq low pass filtered */

    int16_t flux_id;                               /*!<  Park transform output - Flux current(Id) */
    int16_t torque_iq;                             /*!<  Park transform output - Torque current(Iq) */

    uint16_t current_i_mag;                         /*!< stator current magnitude */
    uint16_t current_i_mag_filtered;                /*!< Low pass stator current magnitude filtered */

    uint16_t svm_vref_16;                          /*!<  |Vref|, Magnitude (1Q15) of reference vector (for SVM) */
    uint16_t svm_angle_16;                         /*!<  Angle θ (16-bit) of reference vector. 0 ~ 2^16 represent electrical angle 0° ~ 360° */

    uint16_t pll_lock_status;                      /*!< PLL Lock status */
    int16_t  bemf;								   /* debug rejina*/

} PMSM_FOC_OUTPUT_t;

/**
 *  @brief motor control parameters
 */
typedef struct
{
    int32_t phase_inductance_lq;                /*!< Motor stator phase inductance - quadrature axis - scaled */
    int32_t phase_inductance_ld;                /*!< Motor stator phase inductance - direct axis - scaled */
    int32_t phase_resistance;                   /*!< Motor phase resistance - scaled */
    int32_t rotor_angle_q31;                    /*!< Estimated rotor angle*/
    uint16_t rotor_speed;                       /*!< Estimated rotor electric speed */
    int16_t lpf_n_bemf;                         /*!< Low pass filter coefficient for PLL internal signals */
    int16_t lpf_n_speed;                        /*!< Low pass filter coefficient for estimated speed */
    int16_t phase_inductance_scale;             /*!< Motor phase inductance scale*/
    int16_t phase_resistance_scale;             /*!< Motor phase resistance scale */
    int16_t speed_angle_conversion_factor;      /*!< Rotor speed to angle conversion factor */
    int16_t speed_angle_conversion_factor_scale; /*!< Rotor speed to angle conversion factor scale */
} MOTOR_CTRL_PARA_t;


/**  @brief multi-level OCP protection structure */
typedef struct
{
  	uint32_t ocp_level;                              /*!<  OCP current level  */
  	uint32_t ocp_level_time_count;                   /*!<  OCP current level time duration in count */
  	int32_t  ocp_level_time_counter;                 /*!<  OCP current level counter */

} PMSM_FOC_OCP_t;

/**  @brief multi-level OCP protection structure */
typedef struct
{
	#define MAX_OCP_LEVEL_NUM   3
	int16_t  OCP_level_num;
	int16_t  OCP_level[MAX_OCP_LEVEL_NUM];           /*!<  OCP current level in derived integer format */
	uint16_t OCP_level_time[MAX_OCP_LEVEL_NUM];      /*!<  OCP current level duration in derived integer format */
	int16_t  OCP_level_count[MAX_OCP_LEVEL_NUM];     /*!<  OCP current level count */
	int16_t  OCP_level_error[MAX_OCP_LEVEL_NUM];     /*!<  OCP current level error status */
} PMSM_FOC_Multi_OCP_Detect_t;

typedef struct
{
    uint32_t sum_array_data;
    uint32_t sum_ecycle_avg;
    uint16_t arraycounter;
    uint16_t arrar_full_flag;
    uint16_t hall_change_cnt;
    uint16_t hall_change_flag;
    uint32_t average_data;
    uint32_t average_ecycle;
    uint16_t throttle_adc_values[WINDOW_SIZE];
#define ECYCLE_NUM (AVG_ECYCLE_NUM*6U)
    uint16_t throttle_adc_avg[ECYCLE_NUM];
    uint16_t cycle_full_flag;
    uint16_t hall_change_once_flag;
}PMSM_FOC_THROTTLE_AVG_t;

/**  @brief Motor control structure */
typedef struct
{
  	PMSM_FOC_CTRL_SCHEME_FUNC_t ctrl_scheme_fun_ptr; /*!<  FOC Control scheme function pointer */
#if(USER_OCP_LEVELS_PROTECTION == ENABLED)
  	PMSM_FOC_Multi_OCP_Detect_t ocp;                              /*!<  Over current detection */
#else
  	PMSM_FOC_OCP_t ocp; 							 /*!<  Over current detection */
#endif

  	BMCLIB_LINEAR_RAMP_GEN_t *ramp_struct_ptr;       /*!<  Ramp structure pointer */
  	uint16_t ref_set;                                /*!<  Reference set by POT / directly by user, */
  	uint16_t ref_brake_set;

  	int32_t msm_state;                               /*!<  motor control state machine active state PMSM_FOC_MSM_t */
  	uint32_t pll_error_track_counter;                /*!<  variable for pll out-of sync tracking counter */

    uint32_t alignment_counter;                      /*!<  Counter for rotor initial positioning / alignment in V/f */
    uint32_t motor_stop_counter;                     /*!<  motor stop counter */
    uint32_t first_kick_counter;                     /*!<  Counter for first kick after rotor alignment or IPD */
    uint32_t iq_limit_blanking_counter;

  	int32_t error_status;                            /*!<  Error status to identify which error has occurred */
  	uint16_t limit_max_vq;                           /*!<  Flux weakening vq_max calculated dynamically based on vd */
  	int16_t limit_max_iq;                            /*!<  Max iq limit - set by user in torque limit mode, FW mode - runtime calculated */

  	int16_t inverter_board_temp_degrees;             /*!< inverter board temperature */

  	uint16_t motor_start_flag;                       /*!<  motor start/stop indication flag */

  	int8_t rotation_dir;                             /*!<  Rotation direction of motor (Positive/Forward - (+1), or Negative/Backward - (-1)) */
  	uint16_t speed_ctrl_vq_ref;                      /*!<  Speed ctrl startup - Vq ramp up reference */

    int8_t authentication_status;

    /*=============== E Bike Added Functionality ============================*/
    /*Mode*/
    uint16_t ebike_throttle_mode; 						/*!< E-Bike throttle mode flag */
    uint16_t ebike_throttle_mode_led1;					/*!< E-Bike throttle mode Eco led status in GUI */
    uint16_t ebike_throttle_mode_led2;					/*!< E-Bike throttle mode Normal led status in GUI */
    uint16_t ebike_throttle_mode_led3;					/*!< E-Bike throttle mode Sport led status in GUI */
//    uint16_t E_bike_mode_slope_rate;					/*!< E-Bike mode slope rate */

    /*Max Assist Torque Limit*/
    uint32_t ebike_assist_limit_iq;

    /*Power Limit*/
    uint32_t power_limit_iq;
    uint32_t limit_flag;
    /*!< E-Bike Torque in NM calculated from torque sensor voltage signal*/

    /*Startup Throttle check flag*/
    uint16_t ebike_startup_throttle_check; 				/*!< E-Bike starting throttle check flag during startup. */
    uint16_t ebike_throttle_enable;						/*!< E-Bike throttle enable flag */


    /*Moving Average*/
    uint16_t average_data;

    /*Brake*/
    uint16_t ebike_brake; /*!< E-Bike brake flag */
    uint16_t ebike_brake_ramp_down;
    uint16_t ebike_brake_iq_ref;
    uint16_t brake_counter;
    uint16_t ebike_ramp_cycle_skip_count;
    uint16_t ebike_ramp_cycle_skip_counter;
    /*===============================================================*/

    uint16_t test_flag;

} PMSM_FOC_CTRL_t;

/*********************************************************************************************************************
 * EXTERN
 ********************************************************************************************************************/
extern PMSM_FOC_INPUT_t           PMSM_FOC_INPUT;
extern PMSM_FOC_OUTPUT_t          PMSM_FOC_OUTPUT;
extern PMSM_FOC_CTRL_t            PMSM_FOC_CTRL;
extern BMCLIB_VF_OPEN_LOOP_t      PMSM_FOC_VF_OPEN_LOOP_CTRL;
extern MOTOR_CTRL_PARA_t          PMSM_FOC_MC_PARA;
extern BMCLIB_LINEAR_RAMP_GEN_t   PMSM_FOC_VQ_RAMP_GEN;
extern BMCLIB_LINEAR_RAMP_GEN_t   PMSM_FOC_IQ_RAMP_GEN;
extern BMCLIB_LINEAR_RAMP_GEN_t   PMSM_FOC_SPEED_RAMP_GEN;
extern BMCLIB_PI_CTRL_t           PMSM_FOC_TORQUE_PI;            /*!< Torque / Iq PI controller. */
extern BMCLIB_PI_CTRL_t           PMSM_FOC_FLUX_PI;              /*!< Flux /Id PI controller. */
extern BMCLIB_PI_CTRL_t           PMSM_FOC_SPEED_PI;             /*!< Speed PI controller. */
extern BMCLIB_CLARKE_TRANSFORM_t  PMSM_FOC_CLARKE_TRANSFORM;
extern BMCLIB_SVPWM_t             PMSM_FOC_SVPWM;
extern PMSM_FOC_THROTTLE_AVG_t    PMSM_FOC_THROTTLE_AVG;

#if(USER_PI_PLL_GAIN_SCHEDULING_ENABLE == ENABLED)
extern volatile uint32_t	pll_gain_scheduling_count;
extern volatile uint32_t	pll_gain_scheduling_slew_rate;
#endif

/*********************************************************************************************************************
 * API PROTOTYPES
 ********************************************************************************************************************/
void PMSM_FOC_Init(void);
void PMSM_FOC_VariablesInit(void);
void PMSM_FOC_VF_OpenLoopInit(void);
void PMSM_FOC_RefRampCtrl(void);
void PMSM_FOC_UserConfig_Init(void);

void pmsm_foc_hall_foc_varialbe_init (void);
void pmsm_foc_hall_foc_struc_init(void);
void pmsm_foc_ccu8_duty_set_zero(void);
void pmsm_foc_enable_inverter(void);
void pmsm_foc_disable_inverter(void);

void pmsm_foc_hall_pat_reinit(uint8_t hall_zero_angle);
uint8_t pmsm_foc_hall_pat_id_search(uint8_t *pattern_table, uint8_t pattern_set);
uint8_t pmsm_foc_hall_pattern_id(void);

#if(USER_OCP_LEVELS_PROTECTION == ENABLED)
void PMSM_FOC_Multi_Level_OCP_Init(void);
void PMSM_FOC_Multi_Level_OCP_Reset_Count(void);
#endif

uint32_t EXAMPLECODE_GetFirmwareVersion(void);

/*********************************************************************************************************************
 * API DEFINATION
 ********************************************************************************************************************/
__STATIC_FORCEINLINE uint32_t pmsm_foc_pin_level(XMC_GPIO_PORT_t *const port, const uint8_t pin)
{
  return (((port->IN) >> pin) & 0x1U);
}

/*
 * Get Hall input status.
 */
__STATIC_FORCEINLINE void pmsm_foc_speed_pos_get_hall_pos(uint8_t* curr_hall_pos)
{
	/* Read the Hall input GPIO pins */
	hall_sensor_data.hall[0] = pmsm_foc_pin_level(HALL_0_PIN);
	hall_sensor_data.hall[1] = pmsm_foc_pin_level(HALL_1_PIN);
	hall_sensor_data.hall[2] = pmsm_foc_pin_level(HALL_2_PIN);
	*curr_hall_pos = (uint8_t)((hall_sensor_data.hall[0] | (hall_sensor_data.hall[1] << 1) | (hall_sensor_data.hall[2] << 2)));
}

__STATIC_FORCEINLINE void pmsm_foc_divide(uint32_t dividend, uint32_t divisor)
{
	MATH->DIVCON = (0 << MATH_CON_ST_MODE_Pos | 1 << MATH_CON_X_USIGN_Pos);
	MATH->DVD = dividend;
	MATH->DVS = divisor;
}

__STATIC_FORCEINLINE void pmsm_foc_divide_getresult(uint32_t* result)
{
	while(MATH->DIVST);
	*result = MATH->QUOT;
}

__STATIC_FORCEINLINE void pmsm_foc_hall_rotor_angle(void)
{
//	hall_sensor_data.rotor_angleQ31 += (hall_sensor_data.speed);

  /* Estimate latest rotor angle (1Q31). φ[k] = φ[k-1] + ωr[k]. */
	hall_sensor_data.rotor_angleQ31 += (int32_t)((hall_sensor_data.speed *(int32_t)hall_sensor_data.speed_angle_conversion_factor) >> hall_sensor_data.speed_angle_conversion_factor_scale);
}

__STATIC_FORCEINLINE void pmsm_foc_hall_rotor_speed(void)
{
	uint32_t div_result = 0;

//	if(hall_sensor_data.correct_edge == 1U)
	if((hall_sensor_data.correct_edge == 1U) && (hall_sensor_data.event_counter > 200U))
	{
		uint16_t capture_value = hall_sensor_data.event_counter;

		pmsm_foc_divide(HALL_RAD_PER_PWM_RES_INC_FACTOR, capture_value);

		/* Calculate the time between two correct hall events */
		hall_sensor_data.event_counter = capture_value;
		pmsm_foc_divide_getresult(&div_result);

#if(HALL_PLL==ENABLED)

		/* calc the deviation from estimation angle */
		hall_sensor_data.angle_dev = hall_sensor_data.rotor_angleQ31 - hall_sensor_data.hall_angleQ31;

		/* check if angle_dev > 15deg or < -15deg, reset to hall_angleQ31 */
//		if (hall_sensor_data.angle_dev > PMSM_FOC_ANGLE_030_DEGREE_Q31 || hall_sensor_data.angle_dev < PMSM_FOC_ANGLE_330_DEGREE_Q31)
		if (hall_sensor_data.angle_dev > PMSM_FOC_ANGLE_015_DEGREE_Q31 || hall_sensor_data.angle_dev < PMSM_FOC_ANGLE_N15_DEGREE_Q31)
		{
#if(1)
			hall_sensor_data.rotor_angleQ31 = hall_sensor_data.hall_angleQ31;
#endif
			hall_sensor_data.pll_speed_adj = 0;
		}
		else
		{
			/* calc the deviation rotor speed */
			//    	  int32_t temp = (HALL_RAD_PER_PWM_RES_INC_FACTOR) /capture_value;
			hall_sensor_data.speed_dev = ((int64_t)hall_sensor_data.angle_dev * div_result) >> 32;

			/* speed adj */
			hall_sensor_data.pll_speed_adj = ((hall_sensor_data.speed_dev) * KpHallPLL)>> KpHallPLL_SCALE;
		}

#else
		hall_sensor_data.pll_speed_adj = 0;

		// Force adjust whenever cross sector x
		if(hall_sensor_data.cur_hall_pos == SEG_AFTER_SL_FOC_ZERO_FW)
		{
			hall_sensor_data.rotor_angleQ31 = PMSM_FOC_ANGLE_030_DEGREE_Q31;
		}
#endif
		hall_sensor_data.hall_cnt_sum -= hall_sensor_data.hall_cnt[hall_sensor_data.hall_cnt_pt];	/* every new start must reset all data in the array */

		hall_sensor_data.hall_cnt[hall_sensor_data.hall_cnt_pt] = capture_value;
		hall_sensor_data.hall_cnt_pt++;
		if(hall_sensor_data.hall_cnt_pt > 5)
		{
			hall_sensor_data.hall_cnt_pt = 0;
		}

		hall_sensor_data.hall_cnt_sum += capture_value;

		if(hall_sensor_data.hall_valid_cnt < USER_HALL_EDGE_COUNT)
		{
			hall_sensor_data.hall_valid_cnt++;
			uint32_t spd_res = 0;
			pmsm_foc_divide(div_result, 6U);
			pmsm_foc_divide_getresult(&spd_res);
			hall_sensor_data.speed = spd_res;		//HALL_RAD_PER_PWM_RES_INC_FACTOR/(6*capture_value);
		}
		else
		{
			uint32_t res = 0;
			pmsm_foc_divide(HALL_RAD_PER_PWM_RES_INC_FACTOR, hall_sensor_data.hall_cnt_sum);
			pmsm_foc_divide_getresult(&res);

///*Debug*/
//			if(res - hall_sensor_data.pll_speed_adj > 0)
//			{
				hall_sensor_data.speed = res - hall_sensor_data.pll_speed_adj;			//HALL_RAD_PER_PWM_RES_INC_FACTOR/hall_sensor_data.hall_cnt_sum;
//			}
		}

		hall_sensor_data.correct_edge = 0U;
	}
//	else
//	{
//		hall_sensor_data.correct_edge = 0U;
//		hall_sensor_data.event_counter = 0U;
//	}
}

#if(PMSM_FOC_HARDWARE_KIT == LEV_4KW_KIT)
__STATIC_INLINE __RAM_FUNC void pmsm_foc_get_IDCLink_current(){
	uint16_t IDCLink_adc_result;

	/* IDC link ADC LPF.  */
	IDCLink_adc_result = VADC_IDC_GROUP->RES[VADC_IDC_RESULT_REG];
	ADC.adc_res_idc = (ADC.adc_res_idc * ((1 << ADCLPF) - 1) + IDCLink_adc_result) >> ADCLPF;
}
#endif


/**
 * @}
 */

/**
 * @}
 */

#endif /* PMSM_FOC_CONTROLMODULES_PMSM_FOC_FUNCTIONS_H */

