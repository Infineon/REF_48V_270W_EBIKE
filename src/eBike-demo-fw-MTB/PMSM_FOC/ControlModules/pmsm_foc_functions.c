/**
 * @file pmsm_foc_functions.c
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
/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <xmc_common.h>                   /* SFR declarations of the selected device */

#include <PMSM_FOC/FOCLib/BMCLIB/bmclib_svpwm.h>
#include <PMSM_FOC/FOCLib/BMCLIB/bmclib_v_hz_profile.h>

#include <PMSM_FOC/Configuration/pmsm_foc_config.h>
#include <PMSM_FOC/Configuration/pmsm_foc_const.h>


#include "../MCUInit/pmsm_foc_systick.h"
#include "../MCUInit/pmsm_foc_gpio.h"
#include "../MCUInit/pmsm_foc_adc.h"
#include "../MCUInit/pmsm_foc_ccu4.h"
#include "../MCUInit/pmsm_foc_posif.h"
#include "../MCUInit/pmsm_foc_eru.h"

#include "../uCProbe/uCProbe.h"

#include "../ControlModules/pmsm_foc_error_handling.h"
#include "../ControlModules/pmsm_foc_functions.h"
#include "../ControlModules/pmsm_foc_state_machine.h"
#include "../ControlModules/pmsm_foc_functions.h"
#include "../ControlModules/pmsm_foc_vq_voltage_ctrl.h"
#include "../ControlModules/pmsm_foc_torque_ctrl.h"
#include "../ControlModules/pmsm_foc_interface.h"

#if (SPI_LIB == ENABLED)
#include "../IMD700A_SPI_LIB/utility.h"
#include "../IMD700A_SPI_LIB/ASC.h"
#include "../IMD700A_SPI_LIB/IMD700_SPI_LIB.h"
#endif

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
USER_CONFIG_t              USER_CONFIG;

PMSM_FOC_INPUT_t           PMSM_FOC_INPUT;
PMSM_FOC_OUTPUT_t          PMSM_FOC_OUTPUT;
PMSM_FOC_CTRL_t            PMSM_FOC_CTRL;

MOTOR_CTRL_PARA_t          PMSM_FOC_MC_PARA;

BMCLIB_SVPWM_t             PMSM_FOC_SVPWM;
BMCLIB_VF_OPEN_LOOP_t      PMSM_FOC_VF_OPEN_LOOP_CTRL;
BMCLIB_LINEAR_RAMP_GEN_t   PMSM_FOC_VQ_RAMP_GEN;
BMCLIB_LINEAR_RAMP_GEN_t   PMSM_FOC_IQ_RAMP_GEN;
BMCLIB_LINEAR_RAMP_GEN_t   PMSM_FOC_SPEED_RAMP_GEN;
BMCLIB_CLARKE_TRANSFORM_t  PMSM_FOC_CLARKE_TRANSFORM;

BMCLIB_PI_CTRL_t           PMSM_FOC_TORQUE_PI;            /*!< Torque / Iq PI controller. */
BMCLIB_PI_CTRL_t           PMSM_FOC_FLUX_PI;              /*!< Flux /Id PI controller. */
BMCLIB_PI_CTRL_t           PMSM_FOC_SPEED_PI;             /*!< Speed PI controller. */

PMSM_FOC_THROTTLE_AVG_t    PMSM_FOC_THROTTLE_AVG;
/*********************************************************************************************************************
 * EXTERN
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/
#if(USER_PI_PLL_GAIN_SCHEDULING_ENABLE == ENABLED)
volatile uint32_t	pll_gain_scheduling_count = 0U;
volatile uint32_t	pll_gain_scheduling_slew_rate = 0U;
#endif


/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#define PMSM_FOC_SETTLING_TIME   (0xFF)
/*********************************************************************************************************************
 * LOCAL API PROTOTYPES
 ********************************************************************************************************************/
/* Peripheral initialization of VADC module  */
static void PMSM_FOC_Measurement_Init(void);
/* Initializes motor control required peripherals */
static void PMSM_FOC_PeripheralsInit(void);
/* Initializes the user configuration & motor control variables */
static void PMSM_FOC_MotorControl_Init(void);
/* Initializes PI control structures as per user configurations */
static void PMSM_FOC_PI_Init(void);
/* Initializes the SVPWM module as per the user configurations */
static void PMSM_FOC_SVPWM_Init(void);

/*********************************************************************************************************************
 * API PROTOTYPES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * API DEFINATION
 ********************************************************************************************************************/
#if(USER_OCP_LEVELS_PROTECTION == ENABLED)
void PMSM_FOC_Multi_Level_OCP_Init(void)
{
  PMSM_FOC_CTRL.ocp.OCP_level_num = MAX_OCP_LEVEL_NUM;

  PMSM_FOC_CTRL.ocp.OCP_level[0] = USER_OCP_LEVEL_A;
  PMSM_FOC_CTRL.ocp.OCP_level_time[0] = USER_OCP_LEVEL_A_TIME;
  PMSM_FOC_CTRL.ocp.OCP_level_count[0] = 0;
  PMSM_FOC_CTRL.ocp.OCP_level_error[0] = FALSE;

  PMSM_FOC_CTRL.ocp.OCP_level[1] = USER_OCP_LEVEL_B;
  PMSM_FOC_CTRL.ocp.OCP_level_time[1] = USER_OCP_LEVEL_B_TIME;
  PMSM_FOC_CTRL.ocp.OCP_level_count[1] = 0;
  PMSM_FOC_CTRL.ocp.OCP_level_error[1] = FALSE;

  PMSM_FOC_CTRL.ocp.OCP_level[2] = USER_OCP_LEVEL_C;
  PMSM_FOC_CTRL.ocp.OCP_level_time[2] = USER_OCP_LEVEL_C_TIME;
  PMSM_FOC_CTRL.ocp.OCP_level_count[2] = 0;
  PMSM_FOC_CTRL.ocp.OCP_level_error[2] = FALSE;
}

/* Reset multi-level OCP counter */
void PMSM_FOC_Multi_Level_OCP_Reset_Count(void)
{
  PMSM_FOC_CTRL.ocp.OCP_level_count[0] = 0;
  PMSM_FOC_CTRL.ocp.OCP_level_count[1] = 0;
  PMSM_FOC_CTRL.ocp.OCP_level_count[2] = 0;
}
#endif
/*  Variables initialization before motor start */
void PMSM_FOC_VariablesInit(void)
{

    /*Init parameters (Kp / Ki, limits) of PI controllers. */
    PMSM_FOC_PI_Init();

    /* FOC Configuration initialization */
    PMSM_FOC_INPUT.ref_id = 0;
    PMSM_FOC_INPUT.ref_vq = VQ_INITIAL_VALUE;
    PMSM_FOC_INPUT.limit_max_is = LIMIT_MAX_IS;
    PMSM_FOC_INPUT.limit_max_vref = SVPWM_MAX_VREF;

    #if(USER_TORQUE_LIMIT == ENABLED)
    PMSM_FOC_INPUT.limit_max_iq = LIMIT_MAX_IQ;
    #endif
    PMSM_FOC_INPUT.ref_speed = 0;
    PMSM_FOC_INPUT.ref_iq = 0;
    PMSM_FOC_INPUT.ref_iq_temp = 0;

  	/*e-bike functionality*/
    PMSM_FOC_INPUT.assist_limit_ref_iq = 0;
    PMSM_FOC_INPUT.power_limit_ref_iq = 0;

    #if (USER_ROTOR_IPD_METHOD == ROTOR_IPD_PRE_ALIGNMENT)
    PMSM_FOC_INPUT.ref_rotor_pre_align_current = ROTOR_PRE_ALIGNMENT_CURRENT_REF;
    #endif

    #if(USER_FOC_CTRL_SCHEME == VQ_VOLTAGE_CTRL)
    PMSM_FOC_INPUT.user_ctrl_scheme = VQ_VOLTAGE_CTRL_SCHEME;
    PMSM_FOC_CTRL.ctrl_scheme_fun_ptr = &PMSM_FOC_VqVoltageCtrl;

    /* Vq ramp configuration */
    PMSM_FOC_VQ_RAMP_GEN.ramp_up_step = VQ_RAMP_UP_STEP;
    PMSM_FOC_VQ_RAMP_GEN.ramp_down_step = VQ_RAMP_DOWN_STEP;
    PMSM_FOC_VQ_RAMP_GEN.ramp_cycle_skip_count = VQ_RAMP_STEP_TIME_FCL_COUNT;
    PMSM_FOC_VQ_RAMP_GEN.ramp_cycle_skip_counter = 0;
    PMSM_FOC_VQ_RAMP_GEN.ramp_up_enable = 1;
    PMSM_FOC_VQ_RAMP_GEN.ramp_down_enable = 1;

    /* Assign vq ramp structure */
    PMSM_FOC_CTRL.ramp_struct_ptr = &PMSM_FOC_VQ_RAMP_GEN;
	#elif(USER_FOC_CTRL_SCHEME == TORQUE_CTRL)
    PMSM_FOC_INPUT.user_ctrl_scheme = TORQUE_CTRL_SCHEME;
    PMSM_FOC_CTRL.ctrl_scheme_fun_ptr = &PMSM_FOC_TorqueCtrl;

    /* Torque ramp configuration */
    PMSM_FOC_IQ_RAMP_GEN.ramp_up_step = IQ_RAMP_UP_STEP;
    PMSM_FOC_IQ_RAMP_GEN.ramp_down_step = IQ_RAMP_DOWN_STEP;
    PMSM_FOC_IQ_RAMP_GEN.ramp_cycle_skip_count = IQ_RAMP_STEP_TIME_FCL_COUNT;
    PMSM_FOC_IQ_RAMP_GEN.ramp_cycle_skip_counter = 0;
    PMSM_FOC_IQ_RAMP_GEN.ramp_up_enable = 1;
    PMSM_FOC_IQ_RAMP_GEN.ramp_down_enable = 1;

    /* Assign torque ramp structure */
    PMSM_FOC_CTRL.ramp_struct_ptr = &PMSM_FOC_IQ_RAMP_GEN;
    #elif(USER_FOC_CTRL_SCHEME == SPEED_INNER_CURRENT_CTRL)
  	/* Speed control initialization */
    PMSM_FOC_INPUT.user_ctrl_scheme = SPEED_INNER_CURRENT_CTRL_SCHEME;
    PMSM_FOC_CTRL.ctrl_scheme_fun_ptr = &PMSM_FOC_SpeedCurrentCtrl;

    /* Speed ramp configuration */
    PMSM_FOC_SPEED_RAMP_GEN.ramp_up_step = SPEED_RAMP_UP_STEP;
    PMSM_FOC_SPEED_RAMP_GEN.ramp_down_step = SPEED_RAMP_DOWN_STEP;
    PMSM_FOC_SPEED_RAMP_GEN.ramp_cycle_skip_count = SPEED_RAMP_STEP_TIME_FCL_COUNT;
    PMSM_FOC_SPEED_RAMP_GEN.ramp_cycle_skip_counter = 0;
    PMSM_FOC_SPEED_RAMP_GEN.ramp_up_enable = 1;
    PMSM_FOC_SPEED_RAMP_GEN.ramp_down_enable = 1;
    /* Assign Speed ramp structure */
    PMSM_FOC_CTRL.ramp_struct_ptr = &PMSM_FOC_SPEED_RAMP_GEN;

    #else
    PMSM_FOC_INPUT.user_ctrl_scheme = VF_OPEN_LOOP_CTRL_SCHEME;
    #endif

    PMSM_FOC_CTRL.motor_stop_counter = 0;
    PMSM_FOC_CTRL.alignment_counter = 0;
    PMSM_FOC_CTRL.error_status = 0; /* Clear error if any before start*/

    PMSM_FOC_CTRL.limit_max_vq = SVPWM_MAX_OVERMOD_RANGE;
    #if(USER_TORQUE_LIMIT_ENABLE == ENABLED)
    PMSM_FOC_CTRL.limit_max_iq = LIMIT_MAX_IQ;
    #else
    PMSM_FOC_CTRL.limit_max_iq = PMSM_FOC_INPUT.limit_max_is;
    #endif
    PMSM_FOC_CTRL.inverter_board_temp_degrees = 25;

	#if(USER_OCP_LEVELS_PROTECTION == ENABLED)
    /* Multi-level OCP init */
    PMSM_FOC_Multi_Level_OCP_Init();
	#else
	PMSM_FOC_CTRL.iq_limit_blanking_counter = 0;
	PMSM_FOC_CTRL.pll_error_track_counter = 0;
    /* OCP Initialization */
    PMSM_FOC_CTRL.ocp.ocp_level = OCP_LEVEL;
    PMSM_FOC_CTRL.ocp.ocp_level_time_count = OCP_LEVEL_TIME;
    PMSM_FOC_CTRL.ocp.ocp_level_time_counter = 0;
	#endif

    PMSM_FOC_MC_PARA.rotor_angle_q31 = 0;
    PMSM_FOC_MC_PARA.rotor_speed = 0;

    PMSM_FOC_OUTPUT.current_i_mag = 0;
    PMSM_FOC_OUTPUT.current_i_mag_filtered = 0;
    PMSM_FOC_OUTPUT.iq_filtered = 0;

    PMSM_FOC_OUTPUT.svm_vref_16 = 0;
    PMSM_FOC_OUTPUT.svm_angle_16 = 0;		// Init Vref angle θ = X°.

    PMSM_FOC_OUTPUT.flux_id = 0;
    PMSM_FOC_OUTPUT.torque_iq = 0;
    PMSM_FOC_OUTPUT.torque_vq = VQ_INITIAL_VALUE;
    PMSM_FOC_OUTPUT.flux_vd = 0;

    ADC.adc_res_vdc = VADC_DCLINK_T;
    ADC.adc_res_vdc_filtered = VADC_DCLINK_T;
    ADC.adc_res_idc = 0;

    /* PLL observer initialization */
    PMSM_FOC_MC_PARA.phase_inductance_lq    = (int32_t)MOTOR_LQ_SCALEDUP;
    PMSM_FOC_MC_PARA.phase_inductance_ld    = (int32_t)MOTOR_LD_SCALEDUP;
    PMSM_FOC_MC_PARA.phase_inductance_scale = (int16_t)MOTOR_SCALE_OF_L;

    PMSM_FOC_MC_PARA.phase_resistance       = (int32_t)MOTOR_R_SCALEDUP;
    PMSM_FOC_MC_PARA.phase_resistance_scale = (int16_t)MOTOR_SCALE_OF_R;

#if(MOTOR_CTRL_SCHEME != PMSM_HALL_FOC)
    PMSM_FOC_MC_PARA.lpf_n_speed = USER_PLL_SPEED_LPF;
#endif

    /* Reset PLL observer parameters */
    PMSM_FOC_MC_PARA.rotor_angle_q31 = 0;
    PMSM_FOC_MC_PARA.rotor_speed = 0;

    PMSM_FOC_MC_PARA.speed_angle_conversion_factor = SPEED_TO_ANGLE_CONV_FACTOR;
    PMSM_FOC_MC_PARA.speed_angle_conversion_factor_scale = SPEED_TO_ANGLE_CONV_FACTOR_SCALE;

    /*=============== E Bike Added Functionality ============================*/
    PMSM_FOC_CTRL.ebike_throttle_mode_led1 = 0;
    PMSM_FOC_CTRL.ebike_throttle_mode_led2 = 0;
    PMSM_FOC_CTRL.ebike_throttle_mode_led3 = 0;

//    PMSM_FOC_CTRL.Real_Speed_in_rpm = 0;

    PMSM_FOC_THROTTLE_AVG.arraycounter = 0;
    PMSM_FOC_THROTTLE_AVG.arrar_full_flag = 0;
    PMSM_FOC_THROTTLE_AVG.hall_change_cnt = 0;
    PMSM_FOC_THROTTLE_AVG.hall_change_flag = 0;
    PMSM_FOC_THROTTLE_AVG.sum_array_data = 0;
    PMSM_FOC_THROTTLE_AVG.average_data = 0;
    PMSM_FOC_THROTTLE_AVG.average_ecycle = 0;
    PMSM_FOC_THROTTLE_AVG.sum_ecycle_avg = 0;
    PMSM_FOC_THROTTLE_AVG.cycle_full_flag = 0;
    PMSM_FOC_THROTTLE_AVG.hall_change_once_flag = 0;

    for(uint8_t i=0; i<WINDOW_SIZE; i++)
    {
      PMSM_FOC_THROTTLE_AVG.throttle_adc_values[i] = 0;
    }
    for(uint8_t i=0; i<ECYCLE_NUM; i++)
    {
      PMSM_FOC_THROTTLE_AVG.throttle_adc_avg[i] = 0;
    }

    PMSM_FOC_CTRL.average_data = 0;

    /*Brake*/
    PMSM_FOC_CTRL.ebike_brake = 0;
    PMSM_FOC_CTRL.ebike_brake_ramp_down = 0;
    PMSM_FOC_CTRL.ebike_brake_iq_ref = 0;
    PMSM_FOC_CTRL.brake_counter = 0;
    PMSM_FOC_CTRL.ebike_ramp_cycle_skip_count = E_BIKE_RAMP_STEP_TIME_FCL_COUNT;
    PMSM_FOC_CTRL.ebike_ramp_cycle_skip_counter = 0;

    /*Power Limit*/
    PMSM_FOC_CTRL.power_limit_iq = 0;
    PMSM_FOC_CTRL.limit_flag = 0;

    /*Max Assist Torque limit*/
    PMSM_FOC_CTRL.ebike_assist_limit_iq = 0;
    /*=======================================================================*/

    PMSM_FOC_CTRL.test_flag = 0;

    /* SVPWM module initialization */
    PMSM_FOC_SVPWM_Init();

}

/* API to initialize PI Controller parameters */
void PMSM_FOC_PI_Init(void)
{
    /************** Speed PI controller  *********************************************/
    PMSM_FOC_SPEED_PI.kp = USER_CONFIG.pi_speed_kp;
    PMSM_FOC_SPEED_PI.ki = USER_CONFIG.pi_speed_ki;
    PMSM_FOC_SPEED_PI.scale_kpki = USER_CONFIG.pi_speed_scale_kpki;
    PMSM_FOC_SPEED_PI.enable_antiwindup = USER_CONFIG.pi_speed_antiwindup_enable;

    PMSM_FOC_SPEED_PI.ik_limit_min = USER_CONFIG.pi_speed_ik_limit_min;
    PMSM_FOC_SPEED_PI.ik_limit_max = USER_CONFIG.pi_speed_ik_limit_max;
    PMSM_FOC_SPEED_PI.uk_limit_min = USER_CONFIG.pi_speed_uk_limit_min;
    PMSM_FOC_SPEED_PI.uk_limit_max = USER_CONFIG.pi_speed_uk_limit_max;
    PMSM_FOC_SPEED_PI.uk_limit_max_scaled = (int32_t)(PMSM_FOC_SPEED_PI.uk_limit_max << PMSM_FOC_SPEED_PI.scale_kpki);
    PMSM_FOC_SPEED_PI.uk_limit_min_scaled = (int32_t)(PMSM_FOC_SPEED_PI.uk_limit_min << PMSM_FOC_SPEED_PI.scale_kpki);

    PMSM_FOC_SPEED_PI.uk = 0;
    PMSM_FOC_SPEED_PI.ik = 0;
    PMSM_FOC_SPEED_PI.error = 0;

    /*************** Torque(Iq) PI controller  ****************************************/
    PMSM_FOC_TORQUE_PI.kp = USER_CONFIG.pi_torque_kp;
    PMSM_FOC_TORQUE_PI.ki = USER_CONFIG.pi_torque_ki;
    PMSM_FOC_TORQUE_PI.scale_kpki = USER_CONFIG.pi_torque_scale_kpki;
    PMSM_FOC_TORQUE_PI.enable_antiwindup = USER_CONFIG.pi_torque_antiwindup_enable;

    PMSM_FOC_TORQUE_PI.ik_limit_min = USER_CONFIG.pi_torque_ik_limit_min;
    PMSM_FOC_TORQUE_PI.ik_limit_max = USER_CONFIG.pi_torque_ik_limit_max;
    PMSM_FOC_TORQUE_PI.uk_limit_min = USER_CONFIG.pi_torque_uk_limit_min;
    PMSM_FOC_TORQUE_PI.uk_limit_max = USER_CONFIG.pi_torque_uk_limit_max;
    PMSM_FOC_TORQUE_PI.uk_limit_max_scaled = (int32_t)(PMSM_FOC_TORQUE_PI.uk_limit_max << PMSM_FOC_TORQUE_PI.scale_kpki);
    PMSM_FOC_TORQUE_PI.uk_limit_min_scaled = (int32_t)(PMSM_FOC_TORQUE_PI.uk_limit_min << PMSM_FOC_TORQUE_PI.scale_kpki);

    PMSM_FOC_TORQUE_PI.uk = 0;
    PMSM_FOC_TORQUE_PI.ik = 0;
    PMSM_FOC_TORQUE_PI.error = 0;

    /**************** Flux(Id) PI controller  ******************************************/
    PMSM_FOC_FLUX_PI.kp = USER_CONFIG.pi_flux_kp;
    PMSM_FOC_FLUX_PI.ki = USER_CONFIG.pi_flux_ki;
    PMSM_FOC_FLUX_PI.scale_kpki = USER_CONFIG.pi_flux_scale_kpki;
    PMSM_FOC_FLUX_PI.enable_antiwindup = USER_CONFIG.pi_flux_antiwindup_enable;

    PMSM_FOC_FLUX_PI.ik_limit_min = USER_CONFIG.pi_flux_ik_limit_min;
    PMSM_FOC_FLUX_PI.ik_limit_max = USER_CONFIG.pi_flux_ik_limit_max;
    PMSM_FOC_FLUX_PI.uk_limit_min = USER_CONFIG.pi_flux_uk_limit_min;
    PMSM_FOC_FLUX_PI.uk_limit_max = USER_CONFIG.pi_flux_uk_limit_max;
    PMSM_FOC_FLUX_PI.uk_limit_max_scaled = (int32_t)(PMSM_FOC_FLUX_PI.uk_limit_max << PMSM_FOC_FLUX_PI.scale_kpki);
    PMSM_FOC_FLUX_PI.uk_limit_min_scaled = (int32_t)(PMSM_FOC_FLUX_PI.uk_limit_min << PMSM_FOC_FLUX_PI.scale_kpki);

    PMSM_FOC_FLUX_PI.uk = 0;
    PMSM_FOC_FLUX_PI.ik = 0;
    PMSM_FOC_FLUX_PI.error = 0;
} /* End of pmsm_foc_pi_controller_init () */

/*  Initialization before motor start */
void PMSM_FOC_MotorControl_Init(void)
{
    /* load user configuration */
    // PMSM_FOC_UserConfig_Init();    /* This function will be called in the PMSM_FOC_uCProbe_Init() when there is no user config data written to the flash yet  */

    /* Init variables. */
    PMSM_FOC_VariablesInit();

    /*Only wants variable to initialize once*/
    /*=============== E Bike Added Functionality ============================*/
    PMSM_FOC_CTRL.ebike_throttle_mode = 2; /*Throttle mode default is normal (2), Eco(1), Sport(3)*/

    /*E Bike startup throttle check variable. Only to init at startup.*/
    PMSM_FOC_CTRL.ebike_startup_throttle_check = 0; 	/* 0 - Startup throttle safety check. 1 - Startup throttle safety check off once throttle in zero position.*/
    PMSM_FOC_CTRL.ebike_throttle_enable = 0;			/* 0 - throttle/slider not in zero position. 1 - throttle/slider back/in zero position.*/

//    ADC.adc_res_pot = 2500; /*E-bike startup Testing*/
    /*=======================================================================*/

    /* Next go to stop motor state and wait for the motor start command */
    PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_STOP_MOTOR;
}

/* API to initialize MCU and peripherals for motor control */
void PMSM_FOC_Init(void)
{
    /* Initializes motor control peripherals */
    PMSM_FOC_PeripheralsInit();

#if(MOTOR_CTRL_SCHEME == PMSM_HALL_FOC)
    pmsm_foc_ccu4_hall_init();
    /* Configure default hall pattern angle */
    memcpy(hall_pat_angle, hall_pat_angle_fw[(HALL_SEG_SL_FOC_ZERO-1)], sizeof(hall_pat_angle));
    pmsm_foc_set_default_hall_pattern();

    /*Change the DIRECTION_INC if (1) else DIRECTION_DEC (-1)*/
    hall_sensor_data.hall_pat_dir = DIRECTION_INC;
    pmsm_foc_reset_hall_pattern();


    pmsm_foc_posif_hall_init();
    pmsm_foc_gpio_hall_init();
    pmsm_foc_speed_pos_get_hall_pos(&hall_sensor_data.cur_hall_pos);
    /* Configure current and expected hall patterns */
    XMC_POSIF_HSC_SetHallPatterns(POSIF_MODULE, pmsm_foc_hall_pattern.hall_pattern_pos[hall_sensor_data.cur_hall_pos]);
    /* Update hall pattern */
    XMC_POSIF_HSC_UpdateHallPattern(POSIF_MODULE);
    hall_sensor_data.run_zero_angle_id = DISABLED;
#endif

    /* Read from flash the user configuration data which will be used to load some motor control parameters */
    /* So, this function, which read the user configuration data from flash, should be called here before the motor control parameter initialization */
    #if(USER_UCPROBE_GUI == ENABLED)
    PMSM_FOC_uCProbe_Init();
    #endif

#if (SPI_LIB == ENABLED)
    /* Configure 6EDL7141 driver IC */
//    XMC_GPIO_ToggleOutput(TEST_PIN);
//    XMC_GPIO_SetOutputHigh(TEST_PIN);
    ASC_Init();
    SPI_Master_Init(SPI_MAS_CH);
    Config_Driver_6EDL7141();

    /* Clears the Fault on 6EDL7141 */
    Driver_Faults_Clear();
#endif

    /* Initialization for motor control. */
    PMSM_FOC_MotorControl_Init();
    PMSM_FOC_CTRL.rotation_dir = DIRECTION_INC;

    /* uC Probe oscilloscope initialization */
    #if (USER_UCPROBE_OSCILLOSCOPE == ENABLED)
    ProbeScope_Init(PROBE_SCOPE_SAMPLING_CLK_HZ);
    #endif

//#if (SPI_LIB == DISABLED)
    /* Clear trap event before start */
    XMC_CCU8_SLICE_ClearEvent(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_EVENT2);
    XMC_CCU8_SLICE_ClearEvent(CCU8_SLICE_PHASE_V, XMC_CCU8_SLICE_IRQ_ID_EVENT2);
    XMC_CCU8_SLICE_ClearEvent(CCU8_SLICE_PHASE_W, XMC_CCU8_SLICE_IRQ_ID_EVENT2);
    NVIC_ClearPendingIRQ(TRAP_IRQn);
//#endif
    
    /* Synchronous start of CAPCOM modules, e.g.: CCU8x, and or CCU4x */
    PMSM_FOC_CCUx_SyncStart();      
}

static void PMSM_FOC_PeripheralsInit(void)
{
    /* Reset configuration, clock configuration */
    PMSM_FOC_Clock_Init();

    /* Enable prefetch unit */
    XMC_SCU_EnablePrefetchUnit();

    /* Init GPIOs */
    PMSM_FOC_GPIO_Init();

    /* Init CCU8 */
    PMSM_FOC_CCU8_Init();

    /* Init VADC */
    PMSM_FOC_Measurement_Init();

#if ((DEBUG_PWM_0_ENABLE == ENABLED) || (DEBUG_PWM_1_ENABLE == ENABLED))
    /* Init CCU4  */
    PMSM_FOC_CCU4_Init();
#endif

    /* Init MATH Unit (i.e.: CORDIC Co-processor and Divider Unit DIV) */
    PMSM_FOC_MATH_Init();

    #if(USER_WATCH_DOG_TIMER == ENABLED)
    /* Init WDT */
    PMSM_FOC_WDT_Init();
    #endif
    
    /* Init ERU - Driver interrupt*/
    PMSM_FOC_PIN_INTERRUPT_Init();

    /* Initializes systick - this will start the state machine interrupt */
    PMSM_FOC_SYSTICK_TimerInit();    
}

/* Initializes measurement module along with the peripherals - VADC */
static void PMSM_FOC_Measurement_Init(void)
{
    /* Initialize VADC peripheral */
    PMSM_FOC_VADC_ModuleInit();

    /* Motor phase current measurement VADC peripheral initialization */
    PMSM_FOC_VADC_PhCurrentInit();

    /* Initialize offset for phase current sensing */
    ADC.adc_bias_iu = USER_IU_ADC_BIAS;
    ADC.adc_bias_iv = USER_IV_ADC_BIAS;
    ADC.adc_bias_iw = USER_IW_ADC_BIAS;

    /* DC bus voltage measurement VADC peripheral initialization */
    PMSM_FOC_VADC_VDCInit();

    /* Potentiometer measurement VADC peripheral initialization */
    PMSM_FOC_VADC_PotInit();

    /* Board temperature monitoring */
    PMSM_FOC_VADC_BoardTempSensor_Init();

	#if(PMSM_FOC_HARDWARE_KIT == LEV_4KW_KIT)
    pmsm_adc_idc_init();
    pmsm_adc_vref_init();
	#endif
}

/* SVPWM module initialization */
void PMSM_FOC_SVPWM_Init(void)
{
    PMSM_FOC_SVPWM.previous_sector_num = 0;
    PMSM_FOC_SVPWM.flag_3or2_adc = ADC_SAMPLING_USE_ALL;// Init to use all (e.g.: three) ADC samplings for current reconstruction, for 2or3-shunt.
    PMSM_FOC_SVPWM.pwm_period_reg_val = CCU8_PERIOD_REG;
    PMSM_FOC_SVPWM.sine_table_scale_up = SCALE_UP_SINE_LUT;

    #if(USER_CURRENT_SENSING == THREE_SHUNT_SYNC_CONV)
    /*Sync adc sampling is configured */
    PMSM_FOC_SVPWM.t0_threshold = SVPWM_T0_THRESHOLD;
    PMSM_FOC_SVPWM.t_min = SVPWM_T_MIN;
    #else
    /*Async adc sampling is configured */
    PMSM_FOC_SVPWM.t0_threshold = SVPWM_T0_THRESHOLD;
    PMSM_FOC_SVPWM.t_min = SVPWM_T_MIN;
    #endif

    PMSM_FOC_SVPWM.t_max = CCU8_PERIOD_REG-PMSM_FOC_SVPWM.t_min;

    #if(USER_SVM_SWITCHING_SCHEME == STANDARD_SVM_5_SEGMENT)
    PMSM_FOC_SVPWM.modulation_func_ptr = BMCLIB_SVPWM_FiveSeg;
    #else
    PMSM_FOC_SVPWM.modulation_func_ptr = BMCLIB_SVPWM_SevenSeg;
    #endif

    PMSM_FOC_SVPWM.ccu8_module_ptr = CCU8_MODULE;
    if (PMSM_FOC_CTRL.rotation_dir == DIRECTION_INC)
    {
        /* motor rotation direction - positive. */
        PMSM_FOC_SVPWM.ccu8_phu_module_ptr = CCU8_SLICE_PHASE_U;
        PMSM_FOC_SVPWM.ccu8_phv_module_ptr = CCU8_SLICE_PHASE_V;
        PMSM_FOC_SVPWM.ccu8_phw_module_ptr = CCU8_SLICE_PHASE_W;
    }
    else
    {
        /* motor rotation direction - negative */
        PMSM_FOC_SVPWM.ccu8_phu_module_ptr = CCU8_SLICE_PHASE_U;
        PMSM_FOC_SVPWM.ccu8_phv_module_ptr = CCU8_SLICE_PHASE_W;
        PMSM_FOC_SVPWM.ccu8_phw_module_ptr = CCU8_SLICE_PHASE_V;
    }
}

/** Input reference to FOC controlled through Ramp function.
 * -----------------------------------------------------------------------------------*/
PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_RefRampCtrl(void)
{
    /* If motor drawing higher current than the set limit then avoid ramp up */
    if(PMSM_FOC_OUTPUT.current_i_mag_filtered > LIMIT_MAX_IS)
    {
        /* Disable rampup for time being */
        PMSM_FOC_SPEED_RAMP_GEN.ramp_up_enable = 0;
        PMSM_FOC_VQ_RAMP_GEN.ramp_up_enable = 0;
        PMSM_FOC_IQ_RAMP_GEN.ramp_up_enable = 0;
    }
    else
    {
        /* Enable ramp up */
        PMSM_FOC_SPEED_RAMP_GEN.ramp_up_enable = 1;
        PMSM_FOC_VQ_RAMP_GEN.ramp_up_enable = 1;
        PMSM_FOC_IQ_RAMP_GEN.ramp_up_enable =1;

        /* E_Bike brake */
#if (E_BIKE_REF == ENABLED)
        if(PMSM_FOC_CTRL.ebike_brake == 1)
        {
            /* Disable rampup for time being */
            PMSM_FOC_SPEED_RAMP_GEN.ramp_up_enable = 0;
            PMSM_FOC_VQ_RAMP_GEN.ramp_up_enable = 0;
            PMSM_FOC_IQ_RAMP_GEN.ramp_up_enable = 0;
        }
#endif
    }

    /* If DC bus voltage is increasing and exceeding the max. configured limit then avoid ramp down */
    #if(USER_DECELERATION_VDC_CLAMPING_ENABLE == ENABLED)
    if(ADC.adc_res_vdc_filtered > VDC_OVER_VOLTAGE_LIMIT)
    {
        /* Disable ramp down for time being */
        PMSM_FOC_SPEED_RAMP_GEN.ramp_down_enable = 0;
        PMSM_FOC_VQ_RAMP_GEN.ramp_down_enable = 0;
    }
    else
    {
        /* Enable ramp down  */
        PMSM_FOC_SPEED_RAMP_GEN.ramp_down_enable = 1;
        PMSM_FOC_VQ_RAMP_GEN.ramp_down_enable = 1;
    }
    #endif

    /*=========================== Max Assist Torque limit calculation =====================================*/
#if((E_BIKE_REF == ENABLED)&& (MAX_ASSIST_TORQUE_LIMIT_EN_DIS == ENABLED))
    /*Set Max Assist Torque limit according to Bike speed - Higher the Speed, lower the assist torque*/
    /*2^15-1 = 32767*/

    if(PMSM_FOC_MC_PARA.rotor_speed < 32767)
    {
    	PMSM_FOC_CTRL.ebike_assist_limit_iq = 32767 - PMSM_FOC_MC_PARA.rotor_speed;
    }
    else
    {
    	PMSM_FOC_CTRL.ebike_assist_limit_iq = 0;
    }
#else
    PMSM_FOC_CTRL.ebike_assist_limit_iq = LIMIT_MAX_IS; /*Default 32767*/
#endif/*===============================================================================*/
    /* ============================== Power Limit Calculation ==========================*/
#if(MAX_POWER_LIMIT_EN_DIS == ENABLED)
    pmsm_foc_divide(E_BIKE_POWER_RAW_SCALE_UP, PMSM_FOC_MC_PARA.rotor_speed);
    pmsm_foc_divide_getresult(&PMSM_FOC_CTRL.power_limit_iq);

    /*Power limit check after having certain speed. Rotor speed zero result in undefined/data type max value for power limit iq.*/
    if(PMSM_FOC_MC_PARA.rotor_speed <= BRAKE_MIN_SPEED)
    {
    	PMSM_FOC_CTRL.power_limit_iq = LIMIT_MAX_IS; /*Default 32767*/
    }

    if(PMSM_FOC_CTRL.power_limit_iq > LIMIT_MAX_IS)
    {
    	PMSM_FOC_CTRL.power_limit_iq = LIMIT_MAX_IS; /*Default 32767*/
    }
#else
    PMSM_FOC_CTRL.power_limit_iq = LIMIT_MAX_IS; /*Default 32767*/
#endif /*===============================================================================*/

  	/**************************************************************************************************************
  	* Speed Control Ramp
  	* *************************************************************************************************************/
    if (PMSM_FOC_CTRL.ctrl_scheme_fun_ptr == &PMSM_FOC_SpeedCurrentCtrl)
    {
      /* POT ADC values 0 ~ 2^12 represent Motor target speed of ELECTRICAL_SPEED_LOW_LIMIT_TS ~ ELECTRICAL_SPEED_HIGH_LIMIT_TS:*/
        PMSM_FOC_CTRL.ref_set = SPEED_REF_LOW_LIMIT_TS + ((uint32_t) ((SPEED_REF_HIGH_LIMIT_TS - SPEED_REF_LOW_LIMIT_TS ) * ADC.adc_res_pot) >> 12);
        /* Limit speed, in case ADC values not 0 ~ 2^12.*/
        PMSM_FOC_CTRL.ref_set = MIN_MAX_LIMIT(PMSM_FOC_CTRL.ref_set, SPEED_REF_HIGH_LIMIT_TS, SPEED_REF_LOW_LIMIT_TS);
        /* Ramp generates the Motor reference speed */
        BMCLIB_LinearRampGenerator((int32_t)PMSM_FOC_CTRL.ref_set, &PMSM_FOC_INPUT.ref_speed, &PMSM_FOC_SPEED_RAMP_GEN);
    }

  	/**************************************************************************************************************
  	* VQ Control Ramp
  	* *************************************************************************************************************/
    else if (PMSM_FOC_CTRL.ctrl_scheme_fun_ptr == &PMSM_FOC_VqVoltageCtrl)
    {
#if(POT_MV_EN_DIS == ENABLED)
    	/*Potentiometer moving average*/
    	PMSM_FOC_CTRL.ref_set = VQ_REF_LOW_LIMIT + ((uint32_t) ((VQ_REF_HIGH_LIMIT - VQ_REF_LOW_LIMIT ) * PMSM_FOC_THROTTLE_AVG.average_data) >> 12);
#else
        /* POT ADC values 0 ~ 2^12 represent VQ_REF_HIGH_LIMIT */
        PMSM_FOC_CTRL.ref_set = VQ_REF_LOW_LIMIT + ((uint32_t) ((VQ_REF_HIGH_LIMIT - VQ_REF_LOW_LIMIT ) * ADC.adc_res_pot) >> 12);
#endif
        /* Limit speed, in case ADC values not 0 ~ 2^12.*/
        PMSM_FOC_CTRL.ref_set = MIN_MAX_LIMIT(PMSM_FOC_CTRL.ref_set, VQ_REF_HIGH_LIMIT, VQ_REF_LOW_LIMIT);

        /* Ramp generates the Motor reference voltage */
#if((MAX_ASSIST_TORQUE_LIMIT_EN_DIS == ENABLED) || (MAX_POWER_LIMIT_EN_DIS == ENABLED))
        if(PMSM_FOC_CTRL.limit_flag == 0)
        {
        	BMCLIB_LinearRampGenerator((int32_t)PMSM_FOC_CTRL.ref_set, &PMSM_FOC_INPUT.ref_vq, &PMSM_FOC_VQ_RAMP_GEN);
        }
#else
        BMCLIB_LinearRampGenerator((int32_t)PMSM_FOC_CTRL.ref_set, &PMSM_FOC_INPUT.ref_vq, &PMSM_FOC_VQ_RAMP_GEN);
#endif

//        /*====================== Max assist torque limit & Power LIMIT CHECK (V_q Control) ======================*/
#if(MAX_ASSIST_TORQUE_LIMIT_EN_DIS == ENABLED)
        if((PMSM_FOC_OUTPUT.iq_filtered > PMSM_FOC_CTRL.ebike_assist_limit_iq) && (PMSM_FOC_OUTPUT.iq_filtered > 0)) /*iq filtered might go negative*/
		{
			PMSM_FOC_INPUT.ref_vq = PMSM_FOC_INPUT.ref_vq - VQ_RAMP_DOWN_STEP;

			if(PMSM_FOC_INPUT.ref_vq <= 0)
			{
				PMSM_FOC_INPUT.ref_vq = 0;
			}

			PMSM_FOC_CTRL.limit_flag = 1;

		}
		else
		{
			PMSM_FOC_CTRL.limit_flag = 0;
		}
#endif

#if(MAX_POWER_LIMIT_EN_DIS == ENABLED)
        if((PMSM_FOC_OUTPUT.iq_filtered > PMSM_FOC_CTRL.power_limit_iq) && (PMSM_FOC_OUTPUT.iq_filtered > 0)) /*iq filtered might go negative*/
		{
			PMSM_FOC_INPUT.ref_vq = PMSM_FOC_INPUT.ref_vq - VQ_RAMP_DOWN_STEP;

			if(PMSM_FOC_INPUT.ref_vq <= 0)
			{
				PMSM_FOC_INPUT.ref_vq = 0;
			}

			PMSM_FOC_CTRL.limit_flag = 1;

		}
		else
		{
			PMSM_FOC_CTRL.limit_flag = 0;
		}
#endif
        /*=========================================================================================================*/

    }

  	/**************************************************************************************************************
  	* Torque Control Ramp
  	* *************************************************************************************************************/
    else if (PMSM_FOC_CTRL.ctrl_scheme_fun_ptr == &PMSM_FOC_TorqueCtrl)
    {
    	/*====================== E Bike 3 Mode throttle Gain, startup throttle check and E bike brake ramp down ======================*/
#if (E_BIKE_REF == ENABLED)
    	/*E bike will not ramp up if enable is zero*/
    	if(PMSM_FOC_CTRL.ebike_throttle_enable == 1) /*1 - throttle/slider in zero position at startup, ramp up enable. */
    	{
    		if ((PMSM_FOC_IQ_RAMP_GEN.ramp_up_enable == 1) && (PMSM_FOC_CTRL.ebike_brake == 0))
    		{
    			/*================= Different ramp up iq_ref value of different mode from user pedaling ======================================*/
				if(PMSM_FOC_CTRL.ebike_throttle_mode== E_BIKE_THROTTLE_ECO_MODE)
				{
					#if(POT_MV_EN_DIS == ENABLED)
					/*Potentiometer moving average*/
					PMSM_FOC_INPUT.ref_iq_temp = (E_ECO_T_SCALEDUP * PMSM_FOC_CTRL.average_data)>>E_ECO_SCALE_OF_T;
					#else
					PMSM_FOC_INPUT.ref_iq_temp = (E_ECO_T_SCALEDUP * ADC.adc_res_pot)>>E_ECO_SCALE_OF_T;
					#endif
				}
				else if(PMSM_FOC_CTRL.ebike_throttle_mode== E_BIKE_THROTTLE_NORMAL_MODE)
				{
					#if(POT_MV_EN_DIS == ENABLED)
					/*Potentiometer moving average*/
					PMSM_FOC_INPUT.ref_iq_temp = (E_NORMAL_T_SCALEDUP * PMSM_FOC_CTRL.average_data)>>E_NORMAL_SCALE_OF_T;
					#else
					PMSM_FOC_INPUT.ref_iq_temp = (E_NORMAL_T_SCALEDUP * ADC.adc_res_pot)>>E_NORMAL_SCALE_OF_T;
					#endif
				}
				else if(PMSM_FOC_CTRL.ebike_throttle_mode== E_BIKE_THROTTLE_SPORT_MODE)
				{
					#if(POT_MV_EN_DIS == ENABLED)
					/*Potentiometer moving average*/
					PMSM_FOC_INPUT.ref_iq_temp = (E_SPORT_T_SCALEDUP * PMSM_FOC_CTRL.average_data)>>E_SPORT_SCALE_OF_T;
					#else
					PMSM_FOC_INPUT.ref_iq_temp = (E_SPORT_T_SCALEDUP * ADC.adc_res_pot)>>E_SPORT_SCALE_OF_T;
					#endif
				}
			}

    		/*Make sure the ref_iq temp does not exceed the max iq.*/
			if(PMSM_FOC_INPUT.ref_iq_temp >= LIMIT_MAX_IS)
			{
				PMSM_FOC_INPUT.ref_iq_temp = LIMIT_MAX_IS;
			}

			/*======================================== E bike Brake ==================================*/
			else if((PMSM_FOC_IQ_RAMP_GEN.ramp_up_enable == 0) || (PMSM_FOC_CTRL.ebike_brake == 1))
			{
				PMSM_FOC_CTRL.ebike_ramp_cycle_skip_counter++;
				if(PMSM_FOC_CTRL.ebike_ramp_cycle_skip_counter >= PMSM_FOC_CTRL.ebike_ramp_cycle_skip_count)
				{
					PMSM_FOC_CTRL.ebike_ramp_cycle_skip_counter = 0;
					PMSM_FOC_INPUT.ref_iq_temp = PMSM_FOC_INPUT.ref_iq_temp - E_BIKE_IQ_RAMP_DOWN_STEP;

					if(PMSM_FOC_INPUT.ref_iq_temp < 1)
					{
						PMSM_FOC_INPUT.ref_iq_temp = 0;
						ADC.adc_res_pot = 0;

						/* Next, go to Motor Stop. */
	//		        	PMSM_FOC_CTRL.motor_stop_counter = 0;		/* For avoid the pmsm_foc_ccu8_duty_set_zero */

						/* Next go to Motor coasting function and wait for motor start command */
						PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_STOP_MOTOR;
	//		        	PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_MOTOR_COASTING;
					}
				}
			}
    	}
    	else if(PMSM_FOC_CTRL.ebike_throttle_enable == 0) /*1 - throttle/slider back/in zero position.*/
    	{
    		PMSM_FOC_INPUT.ref_iq_temp = 0;
    	}

#else

#if(POT_MV_EN_DIS == ENABLED)
    	/*Potentiometer moving average*/
    	PMSM_FOC_CTRL.ref_set = IQ_REF_LOW_LIMIT + ((uint32_t) ((IQ_REF_HIGH_LIMIT - IQ_REF_LOW_LIMIT) * PMSM_FOC_CTRL.average_data) >> 12);
#else
    	PMSM_FOC_CTRL.ref_set = IQ_REF_LOW_LIMIT + ((uint32_t) ((IQ_REF_HIGH_LIMIT - IQ_REF_LOW_LIMIT) * ADC.adc_res_pot) >> 12);
#endif
    	PMSM_FOC_CTRL.ref_set = MIN_MAX_LIMIT(PMSM_FOC_CTRL.ref_set, IQ_REF_HIGH_LIMIT, IQ_REF_LOW_LIMIT);
    	BMCLIB_LinearRampGenerator((int32_t)PMSM_FOC_CTRL.ref_set, &PMSM_FOC_INPUT.ref_iq_temp, &PMSM_FOC_IQ_RAMP_GEN);
#endif /*#if (E_BIKE_REF == ENABLED)*/
    }
    else
    {
        /* V/F open loop */
    }
}

/* ***********************************************************************************************************************************
 * STATIC API IMPLEMENTATION
 * ***********************************************************************************************************************************/


/* Initialize V/F open loop parameters */
void PMSM_FOC_VF_OpenLoopInit(void)
{
    /* V/F configuration */
    PMSM_FOC_VF_OPEN_LOOP_CTRL.vf_offset = STARTUP_VF_OFFSET;
    PMSM_FOC_VF_OPEN_LOOP_CTRL.vf_constant = STARTUP_VF_V_PER_HZ_CONST;
    PMSM_FOC_VF_OPEN_LOOP_CTRL.vf_speed_to_angle_conv_factor = SPEED_TO_ANGLE_CONV_FACTOR;
    PMSM_FOC_VF_OPEN_LOOP_CTRL.vf_speed_to_angle_conv_factor_scale = SPEED_TO_ANGLE_CONV_FACTOR_SCALE;
    PMSM_FOC_VF_OPEN_LOOP_CTRL.vf_transition_speed = STARTUP_VF_TRANSITION_SPEED;
    PMSM_FOC_VF_OPEN_LOOP_CTRL.vf_speed_ramp_up_rate = STARTUP_VF_SPEED_RAMP_UP_RATE;
    PMSM_FOC_VF_OPEN_LOOP_CTRL.stablization_count = STARTUP_VF_STABILIZATION_COUNT;

    #if(USER_FOC_CTRL_SCHEME == VF_OPEN_LOOP_CTRL)
    PMSM_FOC_VF_OPEN_LOOP_CTRL.dont_exit_open_loop_flag = TRUE;
    #else
    PMSM_FOC_VF_OPEN_LOOP_CTRL.dont_exit_open_loop_flag = FALSE;
    #endif

    /* Reset V/F control variables  */
    PMSM_FOC_VF_OPEN_LOOP_CTRL.vf_motor_speed = 0;
    PMSM_FOC_VF_OPEN_LOOP_CTRL.stablization_counter = 0;
    PMSM_FOC_VF_OPEN_LOOP_CTRL.vref_angle = 0;
    PMSM_FOC_VF_OPEN_LOOP_CTRL.vref_mag = 0;

    /* Change the motor control state and status */
    PMSM_FOC_VF_OPEN_LOOP_CTRL.status = V_HZ_PROFILE_GEN_STATUS_IN_PROGRESS;
    PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_VF_OPENLOOP;
}

/*************************************************************************************************************************************
 *  User configuration initialization before motor start
 * ***********************************************************************************************************************************/
void PMSM_FOC_UserConfig_Init(void)
{
    /* Speed PI Controller */
    USER_CONFIG.pi_speed_kp = PI_SPEED_KP;
    USER_CONFIG.pi_speed_ki = PI_SPEED_KI;
  	USER_CONFIG.pi_speed_scale_kpki = PI_SPEED_SCALE_KPKI;
    USER_CONFIG.pi_speed_ik_limit_max = PI_SPEED_IK_LIMIT_MAX;
    USER_CONFIG.pi_speed_ik_limit_min = PI_SPEED_IK_LIMIT_MIN;
    USER_CONFIG.pi_speed_uk_limit_max = PI_SPEED_UK_LIMIT_MAX;
    USER_CONFIG.pi_speed_uk_limit_min = PI_SPEED_UK_LIMIT_MIN;
    USER_CONFIG.pi_speed_antiwindup_enable = PI_SPEED_ENABLE_ANTIWINDUP;

    /* Flux PI Controller */
    USER_CONFIG.pi_flux_kp = PI_FLUX_KP;
    USER_CONFIG.pi_flux_ki = PI_FLUX_KI;
  	USER_CONFIG.pi_flux_scale_kpki = PI_FLUX_SCALE_KPKI;
    USER_CONFIG.pi_flux_ik_limit_max = PI_FLUX_IK_LIMIT_MAX;
    USER_CONFIG.pi_flux_ik_limit_min = PI_FLUX_IK_LIMIT_MIN;
    USER_CONFIG.pi_flux_uk_limit_max = PI_FLUX_UK_LIMIT_MAX;
    USER_CONFIG.pi_flux_uk_limit_min = PI_FLUX_UK_LIMIT_MIN;
    USER_CONFIG.pi_flux_antiwindup_enable = PI_FLUX_ENABLE_ANTIWINDUP;

    /* Torque PI Controller */
    USER_CONFIG.pi_torque_kp = PI_TORQUE_KP;
    USER_CONFIG.pi_torque_ki = PI_TORQUE_KI;
  	USER_CONFIG.pi_torque_scale_kpki = PI_TORQUE_SCALE_KPKI;
    USER_CONFIG.pi_torque_ik_limit_max = PI_TORQUE_IK_LIMIT_MAX;
    USER_CONFIG.pi_torque_ik_limit_min = PI_TORQUE_IK_LIMIT_MIN;
    USER_CONFIG.pi_torque_uk_limit_max = PI_TORQUE_UK_LIMIT_MAX;
    USER_CONFIG.pi_torque_uk_limit_min = PI_TORQUE_UK_LIMIT_MIN;
    USER_CONFIG.pi_torque_antiwindup_enable = PI_TORQUE_ENABLE_ANTIWINDUP;

#if(MOTOR_CTRL_SCHEME != PMSM_HALL_FOC)
    /* PLL Observer PI Controller */
    USER_CONFIG.pi_pll_kp = PI_PLL_KP;
    USER_CONFIG.pi_pll_ki = PI_PLL_KI;
  	USER_CONFIG.pi_pll_scale_kpki = PI_PLL_SCALE_KPKI;
    USER_CONFIG.pi_pll_ik_limit_max = PI_PLL_IK_LIMIT_MAX;
    USER_CONFIG.pi_flux_ik_limit_min = PI_PLL_IK_LIMIT_MIN;
    USER_CONFIG.pi_pll_uk_limit_max = PI_PLL_UK_LIMIT_MAX;
    USER_CONFIG.pi_pll_uk_limit_min = PI_PLL_UK_LIMIT_MIN;
#endif
}

uint32_t EXAMPLECODE_GetFirmwareVersion(void)
{
    uint32_t temp;
    temp = (uint32_t)EXAMPLECODE_FIRMWARE_VERSION;
    return temp;
}


void pmsm_foc_hall_foc_varialbe_init (void)
{
  hall_sensor_data.che_flag = 0;
  hall_sensor_data.whe_flag = 0;
  hall_sensor_data.prescaler = 32;

  pmsm_foc_hall_foc_struc_init();

  pmsm_foc_speed_pos_get_hall_pos(&hall_sensor_data.cur_hall_pos);
  hall_sensor_data.hall_angleQ31 = hall_pat_angle[hall_sensor_data.cur_hall_pos];
}

void pmsm_foc_hall_foc_struc_init(void)
{
  hall_sensor_data.hall_cnt_pt = 0;
  hall_sensor_data.hall_valid_cnt = 0;
  hall_sensor_data.hall_cnt_sum = 0;
  hall_sensor_data.first_edge_flag = 0;
  hall_sensor_data.speed = 0;
  hall_sensor_data.speed_rpm = 0;
  hall_sensor_data.stall_counter = 0;
  hall_sensor_data.zero_deg_id_counter = 0;
  hall_sensor_data.open_loop_stable_counter = 0;

  hall_sensor_data.speed_angle_conversion_factor = SPEED_TO_ANGLE_CONV_FACTOR;
  hall_sensor_data.speed_angle_conversion_factor_scale = SPEED_TO_ANGLE_CONV_FACTOR_SCALE;

  for(int8_t i=0; i<6; i++)
  {
    hall_sensor_data.hall_cnt[i] = 0;
  }
}

void pmsm_foc_disable_inverter(void)
{

  XMC_GPIO_SetMode(PHASE_U_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
  XMC_GPIO_SetMode(PHASE_U_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
  XMC_GPIO_SetMode(PHASE_V_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
  XMC_GPIO_SetMode(PHASE_V_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
  XMC_GPIO_SetMode(PHASE_W_HS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);
  XMC_GPIO_SetMode(PHASE_W_LS_PIN, XMC_GPIO_MODE_INPUT_TRISTATE);

#if(MCUTYPE == MCU_CARD_XMC1404)
  XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, USER_INVERTER_DISABLE_LEVEL); /* Disable gate driver. */
#endif
//  XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, USER_INVERTER_DISABLE_LEVEL); /* Disable gate driver. */

  PMSM_FOC_CCU8_SetDutyZero();

}

void pmsm_foc_enable_inverter(void)
{

  PMSM_FOC_CCU8_SetDutyZero();

  XMC_GPIO_SetMode(PHASE_U_HS_PIN, PHASE_U_HS_ALT_SELECT);
  XMC_GPIO_SetMode(PHASE_U_LS_PIN, PHASE_U_LS_ALT_SELECT);
  XMC_GPIO_SetMode(PHASE_V_HS_PIN, PHASE_V_HS_ALT_SELECT);
  XMC_GPIO_SetMode(PHASE_V_LS_PIN, PHASE_V_LS_ALT_SELECT);
  XMC_GPIO_SetMode(PHASE_W_HS_PIN, PHASE_W_HS_ALT_SELECT);
  XMC_GPIO_SetMode(PHASE_W_LS_PIN, PHASE_W_LS_ALT_SELECT);

#if(MCUTYPE == MCU_CARD_XMC1404)
  XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, USER_INVERTER_ENABLE_LEVEL); /* Enable gate driver. */
#endif
//  XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, USER_INVERTER_ENABLE_LEVEL); /* Enable gate driver. */
}

void pmsm_foc_hall_pat_reinit(uint8_t hall_zero_angle)
{
	if (hall_sensor_data.hall_pat_dir == DIRECTION_INC)
	{
		memcpy(hall_pat_angle, hall_pat_angle_fw[hall_zero_angle-1], sizeof(hall_pat_angle));
	}
	else
	{
		memcpy(hall_pat_angle, hall_pat_angle_rev[hall_zero_angle-1], sizeof(hall_pat_angle));
	}
}

uint8_t pmsm_foc_hall_pat_id_search(uint8_t *pattern_table, uint8_t pattern_set)
{
	uint8_t count =0;
	uint8_t found =0;

	/* check acquired hall pattern seq against pos&neg pattern seq */
	while ((count < 6) && (found == 0U))
	{
		if(pattern_table[count] == pattern_set)
		{
			found = 1U;
		} else
		{
			count++;
		}
	}

	return count;
}

uint8_t pmsm_foc_hall_pattern_id(void)
{
	uint8_t pos_tbl_idx = 0;
	uint8_t neg_tbl_idx = 0;
	uint8_t pos_match_count = 0;
	uint8_t neg_match_count = 0;

	for(int8_t x=0; x<15; x++)
	{
		uint8_t curr_pattern = hall_pat_open_run[x];
		pos_tbl_idx = pmsm_foc_hall_pat_id_search(hall_pat_seq_pos, curr_pattern);
		neg_tbl_idx = pmsm_foc_hall_pat_id_search(hall_pat_seq_neg, curr_pattern);

		pos_tbl_idx = (pos_tbl_idx+1)%6U;
		neg_tbl_idx = (neg_tbl_idx+1)%6U;

		if (hall_pat_open_run[x+1] == hall_pat_seq_pos[pos_tbl_idx])
		{
			pos_match_count += 1U;
		}
		else if (hall_pat_open_run[x+1] == hall_pat_seq_neg[neg_tbl_idx])
		{
			neg_match_count += 1U;
		}
	}

	/* Highest match count decide the dir table used */
	if (pos_match_count == 15U)
	{
		hall_sensor_data.hall_pat_dir = DIRECTION_INC;
	}
	else if (neg_match_count == 15U)
	{
		hall_sensor_data.hall_pat_dir = DIRECTION_DEC;
	}
	else {
		hall_sensor_data.hall_pat_dir = DIRECTION_INVALID;
		/* missing pattern in open loop run array or invalid pattern*/
		PMSM_FOC_CTRL.error_status |= PMSM_FOC_EID_INVALID_HALL_PAT;
	}

	return hall_sensor_data.hall_pat_dir;
}


