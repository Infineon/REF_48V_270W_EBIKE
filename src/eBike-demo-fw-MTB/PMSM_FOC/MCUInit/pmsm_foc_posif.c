/**
 * @file pmsm_foc_posif.c
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
/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "../Configuration/pmsm_foc_config.h"
#include "../PMSM_FOC/MCUInit/pmsm_foc_posif.h"
/***********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/
/* POSIF module configuration */
const XMC_POSIF_CONFIG_t posif_config =
{
  .mode   = XMC_POSIF_MODE_HALL_SENSOR,         /* POSIF Operational mode */
  .input0 = POSIF_HALL_0_INSEL,             /* Choice of input for Input-0 P0.13*/
  .input1 = POSIF_HALL_1_INSEL,             /* Choice of input for Input-1 P1.1*/
  .input2 = POSIF_HALL_2_INSEL,             /* Choice of input for Input-2 P0.15*/
  .filter = XMC_POSIF_FILTER_16_CLOCK_CYCLE,    /* Input filter configuration */
};

/*******************************************************************************
* Global variables
*******************************************************************************/
#define HALL_POSIF_MCM(EP,CP)   (((uint32_t)EP<< 3)|(uint32_t)CP)

const PMSM_FOC_Hall_Pattern_t pmsm_foc_hall_pattern_constant =
{
  .hall_pattern_pos = {  /* CW Hall pattern */
            (uint8_t)HALL_POSIF_MCM(0,0),(uint8_t)HALL_POSIF_MCM(3,1),
            (uint8_t)HALL_POSIF_MCM(6,2),(uint8_t)HALL_POSIF_MCM(2,3),
            (uint8_t)HALL_POSIF_MCM(5,4),(uint8_t)HALL_POSIF_MCM(1,5),
            (uint8_t)HALL_POSIF_MCM(4,6),(uint8_t)HALL_POSIF_MCM(0,0)
                      },

  .hall_pattern_neg = {  /* CCW Hall pattern */
            (uint8_t)HALL_POSIF_MCM(0,0),(uint8_t)HALL_POSIF_MCM(5,1),
            (uint8_t)HALL_POSIF_MCM(3,2),(uint8_t)HALL_POSIF_MCM(1,3),
            (uint8_t)HALL_POSIF_MCM(6,4),(uint8_t)HALL_POSIF_MCM(4,5),
            (uint8_t)HALL_POSIF_MCM(2,6),(uint8_t)HALL_POSIF_MCM(0,0)
                      },
};

PMSM_FOC_Hall_Pattern_t pmsm_foc_hall_pattern;
Hall_Sensor_Data_t hall_sensor_data;
int32_t hall_pat_angle[8] = {};

uint8_t hall_pat_seq_pos[6] = {1U, 3U, 2U, 6U, 4U, 5U};
uint8_t hall_pat_seq_neg[6] = {1U, 5U, 4U, 6U, 2U, 3U};

volatile uint8_t hall_pat_open_run[16] ={};
volatile uint8_t hall_pat_open_run_previous;
/*
 * Each edge of Hall pattern represents an angle after the SL FOC zero deg sector is decided.
 * Hall pos pattern: 1, 3, 2, 6, 4, 5, neg pattern: 1, 5, 4, 6, 2, 3
 */

const int32_t hall_pat_angle_fw[6][8] = {
		/* =============================== 1 =============================== */
				/*Center Degree*/
//		{PMSM_FOC_ANGLE_INVALID,
//				PMSM_FOC_ANGLE_000_DEGREE_Q31, PMSM_FOC_ANGLE_120_DEGREE_Q31,
//				PMSM_FOC_ANGLE_060_DEGREE_Q31, PMSM_FOC_ANGLE_240_DEGREE_Q31,
//				PMSM_FOC_ANGLE_300_DEGREE_Q31, PMSM_FOC_ANGLE_180_DEGREE_Q31,
//				PMSM_FOC_ANGLE_INVALID},

				/*Edge Degree(-30 degree)*/
		{PMSM_FOC_ANGLE_INVALID,
				PMSM_FOC_ANGLE_330_DEGREE_Q31, PMSM_FOC_ANGLE_090_DEGREE_Q31,
				PMSM_FOC_ANGLE_030_DEGREE_Q31, PMSM_FOC_ANGLE_210_DEGREE_Q31,
				PMSM_FOC_ANGLE_270_DEGREE_Q31, PMSM_FOC_ANGLE_150_DEGREE_Q31,
				PMSM_FOC_ANGLE_INVALID},

		/* =============================== 2 =============================== */
#if(PMSM_FOC_HARDWARE_KIT == E_BIKE_MOTOR)
				{PMSM_FOC_ANGLE_INVALID,
					PMSM_FOC_ANGLE_210_DEGREE_Q31, PMSM_FOC_ANGLE_330_DEGREE_Q31,
					PMSM_FOC_ANGLE_270_DEGREE_Q31, PMSM_FOC_ANGLE_090_DEGREE_Q31,
					PMSM_FOC_ANGLE_150_DEGREE_Q31, PMSM_FOC_ANGLE_030_DEGREE_Q31,
					PMSM_FOC_ANGLE_INVALID},

#else	/*Center*/
//		{PMSM_FOC_ANGLE_INVALID,
//				PMSM_FOC_ANGLE_240_DEGREE_Q31, PMSM_FOC_ANGLE_000_DEGREE_Q31,
//				PMSM_FOC_ANGLE_300_DEGREE_Q31, PMSM_FOC_ANGLE_120_DEGREE_Q31,
//				PMSM_FOC_ANGLE_180_DEGREE_Q31, PMSM_FOC_ANGLE_060_DEGREE_Q31,
//				PMSM_FOC_ANGLE_INVALID},


				/*Edge Degree (-30 degree)*/
		{PMSM_FOC_ANGLE_INVALID,
				PMSM_FOC_ANGLE_210_DEGREE_Q31, PMSM_FOC_ANGLE_330_DEGREE_Q31,
				PMSM_FOC_ANGLE_270_DEGREE_Q31, PMSM_FOC_ANGLE_090_DEGREE_Q31,
				PMSM_FOC_ANGLE_150_DEGREE_Q31, PMSM_FOC_ANGLE_030_DEGREE_Q31,
				PMSM_FOC_ANGLE_INVALID},
#endif

		/* =============================== 3 =============================== */
#if(PMSM_FOC_HARDWARE_KIT == AA_MOTOR)
				/*Center Degree*/
//		{PMSM_FOC_ANGLE_INVALID,
//				PMSM_FOC_ANGLE_300_DEGREE_Q31, PMSM_FOC_ANGLE_060_DEGREE_Q31,
//				PMSM_FOC_ANGLE_000_DEGREE_Q31, PMSM_FOC_ANGLE_180_DEGREE_Q31,
//				PMSM_FOC_ANGLE_240_DEGREE_Q31, PMSM_FOC_ANGLE_120_DEGREE_Q31,
//				PMSM_FOC_ANGLE_INVALID},

				/*Edge Degree(-30 degree)*/
//	{PMSM_FOC_ANGLE_INVALID,
//			PMSM_FOC_ANGLE_270_DEGREE_Q31, PMSM_FOC_ANGLE_030_DEGREE_Q31,
//			PMSM_FOC_ANGLE_330_DEGREE_Q31, PMSM_FOC_ANGLE_150_DEGREE_Q31,
//			PMSM_FOC_ANGLE_210_DEGREE_Q31, PMSM_FOC_ANGLE_090_DEGREE_Q31,
//			PMSM_FOC_ANGLE_INVALID},

				/*SL calibration*/
		{PMSM_FOC_ANGLE_INVALID,
				-1343422464, 35979264,
				-823853056, 1417150464,
				-1798307840, 658374656,
			PMSM_FOC_ANGLE_INVALID},
#else
					/*Center Degree*/
//		{PMSM_FOC_ANGLE_INVALID,
//				PMSM_FOC_ANGLE_300_DEGREE_Q31, PMSM_FOC_ANGLE_060_DEGREE_Q31,
//				PMSM_FOC_ANGLE_000_DEGREE_Q31, PMSM_FOC_ANGLE_180_DEGREE_Q31,
//				PMSM_FOC_ANGLE_240_DEGREE_Q31, PMSM_FOC_ANGLE_120_DEGREE_Q31,
//				PMSM_FOC_ANGLE_INVALID},

					/*Edge Degree(-30 degree)*/
		{PMSM_FOC_ANGLE_INVALID,
				PMSM_FOC_ANGLE_270_DEGREE_Q31, PMSM_FOC_ANGLE_030_DEGREE_Q31,
				PMSM_FOC_ANGLE_330_DEGREE_Q31, PMSM_FOC_ANGLE_150_DEGREE_Q31,
				PMSM_FOC_ANGLE_210_DEGREE_Q31, PMSM_FOC_ANGLE_090_DEGREE_Q31,
				PMSM_FOC_ANGLE_INVALID},
#endif

		/* =============================== 4 =============================== */
				/*Center*/
//		{PMSM_FOC_ANGLE_INVALID,
//				PMSM_FOC_ANGLE_120_DEGREE_Q31, PMSM_FOC_ANGLE_240_DEGREE_Q31,
//				PMSM_FOC_ANGLE_180_DEGREE_Q31, PMSM_FOC_ANGLE_000_DEGREE_Q31,
//				PMSM_FOC_ANGLE_060_DEGREE_Q31, PMSM_FOC_ANGLE_300_DEGREE_Q31,
//				PMSM_FOC_ANGLE_INVALID},

				/*Edge Degree(-30 degree)*/
		{PMSM_FOC_ANGLE_INVALID,
				PMSM_FOC_ANGLE_090_DEGREE_Q31, PMSM_FOC_ANGLE_210_DEGREE_Q31,
				PMSM_FOC_ANGLE_150_DEGREE_Q31, PMSM_FOC_ANGLE_330_DEGREE_Q31,
				PMSM_FOC_ANGLE_030_DEGREE_Q31, PMSM_FOC_ANGLE_270_DEGREE_Q31,
				PMSM_FOC_ANGLE_INVALID},

		/* =============================== 5 =============================== */
				/*Center*/
//		{PMSM_FOC_ANGLE_INVALID,
//				PMSM_FOC_ANGLE_060_DEGREE_Q31, PMSM_FOC_ANGLE_180_DEGREE_Q31,
//				PMSM_FOC_ANGLE_120_DEGREE_Q31, PMSM_FOC_ANGLE_300_DEGREE_Q31,
//				PMSM_FOC_ANGLE_000_DEGREE_Q31, PMSM_FOC_ANGLE_240_DEGREE_Q31,
//				PMSM_FOC_ANGLE_INVALID},

				/*Edge Degree(-30 degree)*/
		{PMSM_FOC_ANGLE_INVALID,
				PMSM_FOC_ANGLE_030_DEGREE_Q31, PMSM_FOC_ANGLE_150_DEGREE_Q31,
				PMSM_FOC_ANGLE_090_DEGREE_Q31, PMSM_FOC_ANGLE_270_DEGREE_Q31,
				PMSM_FOC_ANGLE_330_DEGREE_Q31, PMSM_FOC_ANGLE_210_DEGREE_Q31,
				PMSM_FOC_ANGLE_INVALID},

		/* =============================== 6 =============================== */
				/*Center*/
//		{PMSM_FOC_ANGLE_INVALID,
//				PMSM_FOC_ANGLE_180_DEGREE_Q31, PMSM_FOC_ANGLE_300_DEGREE_Q31,
//				PMSM_FOC_ANGLE_240_DEGREE_Q31, PMSM_FOC_ANGLE_060_DEGREE_Q31,
//				PMSM_FOC_ANGLE_120_DEGREE_Q31, PMSM_FOC_ANGLE_000_DEGREE_Q31,
//				PMSM_FOC_ANGLE_INVALID}

				/*Edge Degree(-30 degree)*/
		{PMSM_FOC_ANGLE_INVALID,
				PMSM_FOC_ANGLE_150_DEGREE_Q31, PMSM_FOC_ANGLE_270_DEGREE_Q31,
				PMSM_FOC_ANGLE_210_DEGREE_Q31, PMSM_FOC_ANGLE_030_DEGREE_Q31,
				PMSM_FOC_ANGLE_090_DEGREE_Q31, PMSM_FOC_ANGLE_330_DEGREE_Q31,
				PMSM_FOC_ANGLE_INVALID}
};

const int32_t hall_pat_angle_rev[6][8] = {
		/* =============================== 1 =============================== */
				/*Center*/
//		{PMSM_FOC_ANGLE_INVALID,
//				PMSM_FOC_ANGLE_000_DEGREE_Q31, PMSM_FOC_ANGLE_240_DEGREE_Q31,
//				PMSM_FOC_ANGLE_300_DEGREE_Q31, PMSM_FOC_ANGLE_120_DEGREE_Q31,
//				PMSM_FOC_ANGLE_060_DEGREE_Q31, PMSM_FOC_ANGLE_180_DEGREE_Q31,
//				PMSM_FOC_ANGLE_INVALID},

				/*Edge Degree(-30 degree)*/
		{PMSM_FOC_ANGLE_INVALID,
				PMSM_FOC_ANGLE_330_DEGREE_Q31, PMSM_FOC_ANGLE_210_DEGREE_Q31,
				PMSM_FOC_ANGLE_270_DEGREE_Q31, PMSM_FOC_ANGLE_090_DEGREE_Q31,
				PMSM_FOC_ANGLE_030_DEGREE_Q31, PMSM_FOC_ANGLE_150_DEGREE_Q31,
				PMSM_FOC_ANGLE_INVALID},

		/* =============================== 2 =============================== */
				/*Center*/
//		{PMSM_FOC_ANGLE_INVALID,
//				PMSM_FOC_ANGLE_120_DEGREE_Q31, PMSM_FOC_ANGLE_000_DEGREE_Q31,
//				PMSM_FOC_ANGLE_060_DEGREE_Q31, PMSM_FOC_ANGLE_240_DEGREE_Q31,
//				PMSM_FOC_ANGLE_180_DEGREE_Q31, PMSM_FOC_ANGLE_300_DEGREE_Q31,
//				PMSM_FOC_ANGLE_INVALID},

				/*Edge Degree(-30 degree)*/
		{PMSM_FOC_ANGLE_INVALID,
				PMSM_FOC_ANGLE_090_DEGREE_Q31, PMSM_FOC_ANGLE_330_DEGREE_Q31,
				PMSM_FOC_ANGLE_030_DEGREE_Q31, PMSM_FOC_ANGLE_210_DEGREE_Q31,
				PMSM_FOC_ANGLE_150_DEGREE_Q31, PMSM_FOC_ANGLE_270_DEGREE_Q31,
				PMSM_FOC_ANGLE_INVALID},
		/* =============================== 3 =============================== */
				/*Center*/
//		{PMSM_FOC_ANGLE_INVALID,
//				PMSM_FOC_ANGLE_060_DEGREE_Q31, PMSM_FOC_ANGLE_300_DEGREE_Q31,
//				PMSM_FOC_ANGLE_000_DEGREE_Q31, PMSM_FOC_ANGLE_180_DEGREE_Q31,
//				PMSM_FOC_ANGLE_120_DEGREE_Q31, PMSM_FOC_ANGLE_240_DEGREE_Q31,
//				PMSM_FOC_ANGLE_INVALID},

				/*Edge Degree(-30 degree)*/
		{PMSM_FOC_ANGLE_INVALID,
				PMSM_FOC_ANGLE_030_DEGREE_Q31, PMSM_FOC_ANGLE_270_DEGREE_Q31,
				PMSM_FOC_ANGLE_330_DEGREE_Q31, PMSM_FOC_ANGLE_150_DEGREE_Q31,
				PMSM_FOC_ANGLE_090_DEGREE_Q31, PMSM_FOC_ANGLE_210_DEGREE_Q31,
				PMSM_FOC_ANGLE_INVALID},
		/* =============================== 4 =============================== */
				/*Center*/
//		{PMSM_FOC_ANGLE_INVALID,
//				PMSM_FOC_ANGLE_240_DEGREE_Q31, PMSM_FOC_ANGLE_120_DEGREE_Q31,
//				PMSM_FOC_ANGLE_180_DEGREE_Q31, PMSM_FOC_ANGLE_000_DEGREE_Q31,
//				PMSM_FOC_ANGLE_300_DEGREE_Q31, PMSM_FOC_ANGLE_060_DEGREE_Q31,
//				PMSM_FOC_ANGLE_INVALID},

				/*Edge Degree(-30 degree)*/
		{PMSM_FOC_ANGLE_INVALID,
				PMSM_FOC_ANGLE_210_DEGREE_Q31, PMSM_FOC_ANGLE_090_DEGREE_Q31,
				PMSM_FOC_ANGLE_150_DEGREE_Q31, PMSM_FOC_ANGLE_330_DEGREE_Q31,
				PMSM_FOC_ANGLE_270_DEGREE_Q31, PMSM_FOC_ANGLE_030_DEGREE_Q31,
				PMSM_FOC_ANGLE_INVALID},
		/* =============================== 5 =============================== */
//		{PMSM_FOC_ANGLE_INVALID,
//				PMSM_FOC_ANGLE_300_DEGREE_Q31, PMSM_FOC_ANGLE_180_DEGREE_Q31,
//				PMSM_FOC_ANGLE_240_DEGREE_Q31, PMSM_FOC_ANGLE_060_DEGREE_Q31,
//				PMSM_FOC_ANGLE_000_DEGREE_Q31, PMSM_FOC_ANGLE_120_DEGREE_Q31,
//				PMSM_FOC_ANGLE_INVALID},

				/*Edge Degree(-30 degree)*/
		{PMSM_FOC_ANGLE_INVALID,
				PMSM_FOC_ANGLE_270_DEGREE_Q31, PMSM_FOC_ANGLE_150_DEGREE_Q31,
				PMSM_FOC_ANGLE_210_DEGREE_Q31, PMSM_FOC_ANGLE_030_DEGREE_Q31,
				PMSM_FOC_ANGLE_330_DEGREE_Q31, PMSM_FOC_ANGLE_090_DEGREE_Q31,
				PMSM_FOC_ANGLE_INVALID},
		/* =============================== 6 =============================== */
				/*Center*/
//		{PMSM_FOC_ANGLE_INVALID,
//				PMSM_FOC_ANGLE_180_DEGREE_Q31, PMSM_FOC_ANGLE_060_DEGREE_Q31,
//				PMSM_FOC_ANGLE_120_DEGREE_Q31, PMSM_FOC_ANGLE_300_DEGREE_Q31,
//				PMSM_FOC_ANGLE_240_DEGREE_Q31, PMSM_FOC_ANGLE_000_DEGREE_Q31,
//				PMSM_FOC_ANGLE_INVALID}

				/*Edge Degree(-30 degree)*/
		{PMSM_FOC_ANGLE_INVALID,
				PMSM_FOC_ANGLE_150_DEGREE_Q31, PMSM_FOC_ANGLE_030_DEGREE_Q31,
				PMSM_FOC_ANGLE_090_DEGREE_Q31, PMSM_FOC_ANGLE_270_DEGREE_Q31,
				PMSM_FOC_ANGLE_210_DEGREE_Q31, PMSM_FOC_ANGLE_330_DEGREE_Q31,
				PMSM_FOC_ANGLE_INVALID}
};


/**
 * Hall sensor mode configurations
 * Blanking signal is connected to CCU4 blanking slice compare match
 */
const XMC_POSIF_HSC_CONFIG_t posif_hall_config =
{
  .disable_idle_signal   = 1,  /* Disable idle signal upon wrong hall event */
  .sampling_trigger      = XMC_POSIF_INPUT_PORT_A,  /* HSDA is used to trigger POSIF to sample hall pattern */
  .sampling_trigger_edge = XMC_POSIF_HSC_TRIGGER_EDGE_RISING,   /* Rising edge */

//  .external_error_port    = XMC_POSIF_INPUT_PORT_A,
//  .external_error_enable  = 0U,
//  .external_error_level   = XMC_POSIF_INPUT_ACTIVE_LEVEL_HIGH
};

void pmsm_foc_set_default_hall_pattern()
{
	memcpy(&pmsm_foc_hall_pattern, &pmsm_foc_hall_pattern_constant, sizeof(pmsm_foc_hall_pattern_constant));
}

void pmsm_foc_reset_hall_pattern()
{
	uint8_t hall_pattern_temp[8];
	uint8_t arraySize = sizeof(hall_pattern_temp);

	if (hall_sensor_data.hall_pat_dir == DIRECTION_DEC)
	{
		memcpy(hall_pattern_temp, pmsm_foc_hall_pattern_constant.hall_pattern_pos, arraySize);
		memcpy(pmsm_foc_hall_pattern.hall_pattern_pos, pmsm_foc_hall_pattern_constant.hall_pattern_neg, arraySize);
		memcpy(pmsm_foc_hall_pattern.hall_pattern_neg, hall_pattern_temp, arraySize);
	}
}

void pmsm_foc_posif_hall_init()
{
  /* Initialize POSIF module */
  XMC_POSIF_Init(POSIF_MODULE, &posif_config);

  /* Initialize hall sensor mode */
  XMC_POSIF_HSC_Init(POSIF_MODULE, &posif_hall_config);

  /* Enables event generation */
  XMC_POSIF_EnableEvent(POSIF_MODULE, XMC_POSIF_IRQ_EVENT_CHE);
  XMC_POSIF_EnableEvent(POSIF_MODULE, XMC_POSIF_IRQ_EVENT_WHE);

  /* Connect correct hall event to SR0 and wrong hall event to SR1 */
  XMC_POSIF_SetInterruptNode(POSIF_MODULE, XMC_POSIF_IRQ_EVENT_CHE, XMC_POSIF_SR_ID_0);
  /* in BLDC_SCALAR_HALL_SMC14 project, XMC_POSIF_IRQ_EVENT_WHE is also connected to SR0*/
  XMC_POSIF_SetInterruptNode(POSIF_MODULE, XMC_POSIF_IRQ_EVENT_WHE, XMC_POSIF_SR_ID_0);

  /* Selects SR source for NVIC interrupt node */
  XMC_SCU_SetInterruptControl(POSIF_IRQn, XMC_SCU_IRQCTRL_POSIF0_SR0_IRQ27);

  /* Selects SR source for NVIC interrupt node */
  XMC_SCU_SetInterruptControl(POSIF_IRQn, POSIF_SR_INT);

  /* Set priority */
  NVIC_SetPriority(POSIF_IRQn, PMSM_FOC_POSIF_NVIC_PRIO);

  /* Enable IRQ */
  NVIC_EnableIRQ(POSIF_IRQn);

  /* Start the POSIF module */
  XMC_POSIF_Start(POSIF_MODULE);

  /* Start CCU40_CC40 and CCU40_CC41 Timers */
  XMC_CCU4_SLICE_StartTimer(HALL_DELAY_SLICE);
  XMC_CCU4_SLICE_StartTimer(HALL_CAPTURE_SLICE);
}
