/**
 * @file pmsm_foc_posif.h
 * @date 09 May, 2019
 *
 * @cond
 *********************************************************************************************************************
 * PMSM FOC motor Control Library
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

#ifndef MCUINIT_POSIF_H_
#define MCUINIT_POSIF_H_
/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "../Configuration/pmsm_foc_config.h"
#include "../Configuration/pmsm_foc_mcuhw_params.h"
#include "xmc_posif.h"

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup MCUInit
 * @{
 */

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#if(MOTOR_CTRL_SCHEME == PMSM_HALL_FOC)
#define HALL_PLL                        ENABLED

/* ********************************************* Scaling for Hall-FOC algo *****************************************************************************/
#if(PMSM_FOC_HARDWARE_KIT == LEV_4KW_KIT)
#define HALL_SEG_SL_FOC_ZERO                  (3U)      /* in which Hall seg does SL FOC angle becomes zero */
#define SEG_AFTER_SL_FOC_ZERO_FW              (1U)      /* the Hall seg right after HALL_SEG_SL_FOC_ZERO in forward rotation */
#define SEG_AFTER_SL_FOC_ZERO_BW              (2U)      /* the Hall seg right after HALL_SEG_SL_FOC_ZERO in backward rotation */
#elif(PMSM_FOC_HARDWARE_KIT == KIT_XMC1X_AK_MOTOR_001)
//#define HALL_SEG_SL_FOC_ZERO                  (6U)      /* in which Hall seg does SL FOC angle becomes zero */
//#define SEG_AFTER_SL_FOC_ZERO_FW              (4U)      /* the Hall seg right after HALL_SEG_SL_FOC_ZERO in forward rotation */
//#define SEG_AFTER_SL_FOC_ZERO_BW              (2U)      /* the Hall seg right after HALL_SEG_SL_FOC_ZERO in backward rotation */

/*LIN MOTOR*/
#elif(PMSM_FOC_HARDWARE_KIT == LIN_MOTOR)
#define HALL_SEG_SL_FOC_ZERO                  (4U)      /* in which Hall seg does SL FOC angle becomes zero */
#define SEG_AFTER_SL_FOC_ZERO_FW              (5U)      /* the Hall seg right after HALL_SEG_SL_FOC_ZERO in forward rotation */
#define SEG_AFTER_SL_FOC_ZERO_BW              (6U)      /* the Hall seg right after HALL_SEG_SL_FOC_ZERO in backward rotation */

#elif(PMSM_FOC_HARDWARE_KIT == E_BIKE_MOTOR)
#define HALL_SEG_SL_FOC_ZERO                  (2U)      /* in which Hall seg does SL FOC angle becomes zero */
#define SEG_AFTER_SL_FOC_ZERO_FW              (6U)      /* the Hall seg right after HALL_SEG_SL_FOC_ZERO in forward rotation */
#define SEG_AFTER_SL_FOC_ZERO_BW              (3U)      /* the Hall seg right after HALL_SEG_SL_FOC_ZERO in backward rotation */

#elif(PMSM_FOC_HARDWARE_KIT == AA_MOTOR)
#define HALL_SEG_SL_FOC_ZERO                  (3U)      /* in which Hall seg does SL FOC angle becomes zero */
#define SEG_AFTER_SL_FOC_ZERO_FW              (2U)      /* the Hall seg right after HALL_SEG_SL_FOC_ZERO in forward rotation */
#define SEG_AFTER_SL_FOC_ZERO_BW              (1U)      /* the Hall seg right after HALL_SEG_SL_FOC_ZERO in backward rotation */

//#define HALL_SEG_SL_FOC_ZERO                  (2U)      /* in which Hall seg does SL FOC angle becomes zero */
//#define SEG_AFTER_SL_FOC_ZERO_FW              (6U)      /* the Hall seg right after HALL_SEG_SL_FOC_ZERO in forward rotation */
//#define SEG_AFTER_SL_FOC_ZERO_BW              (3U)      /* the Hall seg right after HALL_SEG_SL_FOC_ZERO in backward rotation */

#elif(PMSM_FOC_HARDWARE_KIT == KIT_CHIMERA_XMC)
#define HALL_SEG_SL_FOC_ZERO                  (4U)      /* in which Hall seg does SL FOC angle becomes zero */
#define SEG_AFTER_SL_FOC_ZERO_FW              (5U)      /* the Hall seg right after HALL_SEG_SL_FOC_ZERO in forward rotation */
#define SEG_AFTER_SL_FOC_ZERO_BW              (6U)      /* the Hall seg right after HALL_SEG_SL_FOC_ZERO in backward rotation */
#elif(PMSM_FOC_HARDWARE_KIT == KIT_CHIMERAXM_EVAL)
#define HALL_SEG_SL_FOC_ZERO                  (3U)      /* in which Hall seg does SL FOC angle becomes zero */
#define SEG_AFTER_SL_FOC_ZERO_FW              (2U)      /* the Hall seg right after HALL_SEG_SL_FOC_ZERO in forward rotation */
#define SEG_AFTER_SL_FOC_ZERO_BW              (1U)      /* the Hall seg right after HALL_SEG_SL_FOC_ZERO in backward rotation */
#endif
#endif      /* end of if(MOTOR_CTRL_SCHEME == PMSM_HALL_FOC) */

/**********************************************************************************************************************
* DATA STRUCTURES
**********************************************************************************************************************/
typedef enum HALL_CALIB_DIR_ID_STATUS
{
	HALL_CALIB_DIR_ID_STATUS_COMPLETED,
	HALL_CALIB_DIR_ID_STATUS_IN_PROGRESS
} HALL_CALIB_DIR_ID_STATUS_t;

typedef struct Hall_Sensor_Data   /* For Hall signal processing */
{
  int32_t speed;                  /* Rotor speed obtained from Hall */
  int32_t speed_rpm;              /* Rotor speed obtained from Hall, in rpm */
  int32_t hall_angleQ31;          /* Coarse rotor angle based on Hall pattern edges (1Q23 << 8) */
  int32_t rotor_angleQ31;         /* Estimated fine rotor angle (1Q23 << 8) from Hall */

  uint32_t zero_deg_id_counter;
  uint32_t event_counter;         /* Counter for Hall events. */


  uint32_t prescaler;             /* Prescaler variable */
  uint8_t che_flag;               /* Correct hall event flag variable */
  uint8_t whe_flag;               /* Wrong hall event flag variable */

  uint32_t stall_counter;         /* Counter for Hall stall detection */
  uint32_t restart_counter;       /* Counter for retry times to start motor if stall has been detected by Hall */
  uint32_t Rst_Restart_Counter;   /* To reset Hall_Restart_Counter if no motor stall for certain time (e.g.: 20s) */

  int32_t pll_speed_adj;
  int32_t angle_dev;
  int32_t speed_dev;

  uint16_t hall_cnt[6];           /* Hall seg data in one electric cycle */
  uint32_t hall_cnt_sum;          /* this variable must be reset before every start */
  uint8_t  cur_hall_pos;          /* Hall position variable */
  uint8_t  prev_hall_pos;         /*!< to store previous captured hall pattern - used in wrong hall event to identify hall failure */
  uint8_t  hall[3];               /* Hall signal */
  uint8_t  hall_cnt_pt;           /* this variable must be reset before every start */
  uint8_t  hall_valid_cnt;        /* this variable must be reset before every start */
  uint8_t  first_edge_flag;       /* first Hall edge detected */
  uint8_t  correct_edge;		  /* Detected a correct hall edge change in interrupt */
  uint8_t  zero_hall_pos;
  uint8_t  run_zero_angle_id;
  uint8_t  run_hall_pat_dir_id;
  int8_t   hall_pat_dir; 		  /* hall pattern pos/neg for direction of uvw positive */
  uint8_t  open_loop_stable_counter;          /* Counter to determine start the hall pattern read */

  HALL_CALIB_DIR_ID_STATUS_t  status;

  int16_t speed_angle_conversion_factor;      /*!< Rotor speed to angle conversion factor */
  int16_t speed_angle_conversion_factor_scale; /*!< Rotor speed to angle conversion factor scale */

} Hall_Sensor_Data_t;

typedef struct PMSM_FOC_Hall_Pattern
{
  uint8_t hall_pattern_pos[8];            /*!< Hall pattern for positive direction */
  uint8_t hall_pattern_neg[8];            /*!< Hall pattern for negative direction */
} PMSM_FOC_Hall_Pattern_t;


/**********************************************************************************************************************
* Extern Variables
**********************************************************************************************************************/
extern int32_t hall_pat_angle[8];   /* angle decimal value of corresponding Hall pattern according to pos/neg direction */
extern const int32_t hall_pat_angle_fw[6][8];
extern const int32_t hall_pat_angle_rev[6][8];
extern Hall_Sensor_Data_t hall_sensor_data;
extern PMSM_FOC_Hall_Pattern_t pmsm_foc_hall_pattern;
extern uint8_t hall_pat_seq_pos[6];
extern uint8_t hall_pat_seq_neg[6];
extern volatile uint8_t hall_pat_open_run[16];
extern volatile uint8_t hall_pat_open_run_previous;

#ifdef __cplusplus
   extern "C" {
#endif
/***********************************************************************************************************************
 * API Prototypes
 **********************************************************************************************************************/
/*
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * This function initializes the POSIF peripheral in multichannel mode.
 * Enables below Multi-channel pattern shadow transfer event and connects to interrupt node
 */
void PMSM_FOC_POSIF_Init(void);
void pmsm_foc_set_default_hall_pattern(void);
void pmsm_foc_reset_hall_pattern(void);
void pmsm_foc_posif_hall_init(void);

/**
 * @}
 */

/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif /* MCUINIT_POSIF_H_ */
