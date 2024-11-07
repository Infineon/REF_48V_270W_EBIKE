/**
 * @file pmsm_foc_scl_systick_isr.c
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
#include "../ControlModules/pmsm_foc_error_handling.h"
#include "../Configuration/pmsm_foc_config.h"
#include "../ControlModules/pmsm_foc_functions.h"
#include "../ControlModules/pmsm_foc_interface.h"
#include "../uCProbe/uCProbe.h"

#if (SPI_LIB == ENABLED)
#include "../IMD700A_SPI_LIB/IMD700_SPI_LIB.h"
#endif
/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup Interrupts
 * @{
 */

/*********************************************************************************************************************
 * EXTERN
 ********************************************************************************************************************/
/*********************************************************************************************************************
 * STATIC INLINE
 ********************************************************************************************************************/
static volatile uint16_t vref_1_65_adc;

/**********************************************************************************************************************
 * API IMPLEMENTATION
 **********************************************************************************************************************/
/**
 * @brief Systick event handler for Slow Control Loop(SCL) \n
 * This is the lowest priority interrupt which performs less time critical tasks like ramp, potentiometer reading,etc.\n
 *
 * @param None
 *
 * @retval None
 */
PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_SCL_ISR(void)
{
    if (PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_CLOSED_LOOP)
    {
        if(PMSM_FOC_INPUT.user_ctrl_scheme == SPEED_INNER_CURRENT_CTRL_SCHEME)
        {
            /* Speed PI Controller */
            BMCLIB_PI_Controller(PMSM_FOC_INPUT.ref_speed, hall_sensor_data.speed, 0, &PMSM_FOC_SPEED_PI);
        }

        /* Check if system is idle then stop the Motor as per use configuration */
        if (SYSTEM_BE_IDLE)
        {
            /* Clear counters. */
//            PMSM_FOC_MC_PARA.rotor_speed = 0;
//            PMSM_FOC_MC_PARA.rotor_angle_q31 = 0;
//            hall_sensor_data.rotor_speed = 0;
//            hall_sensor_data.rotor_angle_q31 = 0;

        	PMSM_FOC_CTRL.motor_stop_counter = 0;
        	/* Next go to Motor coast state and wait for motor start command */
        	PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_MOTOR_COASTING;
        }
    }

  	/**************************************************************************************************************
  	* Slow control error monitoring - START *
  	* *************************************************************************************************************/

  	#if(USER_BOARD_OVERTEMP_PROTECTION_ENABLE == ENABLED)
  	/***************** Over temperature Protection ***************************************
  	* Read analog input from MCP9700 temperature sensor
        *     0°C = 500mV
        *     sensitivity = 10mV/°C
        * For DVDD=3.3V, 10mV = 12.41212 (ADC counts)
        * For DVDD=5.0V, 10mV = 8.192 (ADC counts)
        * t_sensor is °C in Q4, conversion from ADC counts to t_sensor:
        *     DVDD=3.3V, 1/12.41212 = 0.08056 ~= 330 >> 12
        *     DVDD=5.0V, 1/8.192 = 1.953125 = 500 >> 12
        ****************************************************/
  	/* Over temperature protection */
  	int32_t adc_res_tempSense_degreeC;

  	ADC.adc_res_temp = XMC_VADC_GROUP_GetResult(VADC_TEMP_GROUP, VADC_TEMP_CHANNEL);

  	/* Convert voltage to temperature output */
  	adc_res_tempSense_degreeC = (int32_t)(((ADC.adc_res_temp - TEMP_SENSOR_OFFSET) * TEMP_SENSOR_COEFFICIENT_Q15) >> 15);

  	/* LPF */
   	PMSM_FOC_CTRL.inverter_board_temp_degrees += (int16_t)((adc_res_tempSense_degreeC - PMSM_FOC_CTRL.inverter_board_temp_degrees) >> 2);

   	if(PMSM_FOC_CTRL.inverter_board_temp_degrees > (int16_t)USER_ABS_MAX_BOARD_TEMP_C)
   	{
     	/* Next Go to error state */
     	PMSM_FOC_CTRL.error_status |= PMSM_FOC_EID_INVERTER_OVER_TEMP;
   	}

   	if(PMSM_FOC_CTRL.inverter_board_temp_degrees > (int16_t)USER_MAX_BOARD_TEMP_START_C)
   	{
            if((PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_STOP_MOTOR) || (PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_MOTOR_COASTING))
            {
                /* Wait in error state to reduce the board temperature */
                PMSM_FOC_CTRL.error_status |= PMSM_FOC_EID_INVERTER_OVER_TEMP;
            }
   	}
  	#endif //End of #if(USER_BOARD_OVERTEMP_PROTECTION == ENABLED)

    /***************** Over/Under voltage protection ************************************/
    #if(USER_VDC_UV_OV_PROTECTION_ENABLE == ENABLED)
    /* Check VDC bus voltage crossing limit and ramp down is not active */
    if (ADC.adc_res_vdc_filtered > VDC_OVER_VOLTAGE_LIMIT)
    {
      /* Next Go to error state */
      PMSM_FOC_CTRL.error_status |= PMSM_FOC_EID_OVER_VOLT;
    }
    else if (ADC.adc_res_vdc_filtered < VDC_UNDER_VOLTAGE_LIMIT)
    {
      /* Next Go to error state */
      PMSM_FOC_CTRL.error_status |= PMSM_FOC_EID_UNDER_VOLT;
    }
    #endif // End of #if(USER_VDC_UV_OV_PROTECTION_ENABLE == ENABLED)


	#if(PMSM_FOC_HARDWARE_KIT == LEV_4KW_KIT)
    vref_1_65_adc = VADC_VDC_GROUP->RES[VADC_VREF_RESULT_REG];
	#endif

    /**************************************************************************************************************
    * Slow control error monitoring - END *
    * *************************************************************************************************************/

    #ifdef FW_ACTIVE_LED4
    /* Flux Weakening Active Indicator */
    if(PMSM_FOC_INPUT.ref_id != 0)
    {
        /* Flux Weakening Active - Turn on the LED */
        XMC_GPIO_SetOutputLow(FW_ACTIVE_LED4);
    }
    else
    {
        /* Flux Weakening not active - Turn off the LED */
        XMC_GPIO_SetOutputHigh(FW_ACTIVE_LED4);
    }
    #endif

    /* LED 3 - Fault indicator */
    #ifdef FAULT_LED3
    if (PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_ERROR)
    {
        /* Set the Fault LED indicator */
        XMC_GPIO_SetOutputLow(FAULT_LED3);
    }
    else
    {
        /* Clear the Fault LED indicator */
        XMC_GPIO_SetOutputHigh(FAULT_LED3);
    }
    #endif
    /***************************************************************************************************/
    /* If ucProbe/Micro-inspector GUI is enabled then this function process the cmd received from GUI */
    #if(USER_UCPROBE_GUI == ENABLED)
    PMSM_FOC_ucProbe_CmdProcessing();
    #endif

    /**************************************************************************************************************
    *                                      E-Bike Control                                   *
    * *************************************************************************************************************/
    #if(E_BIKE_REF == ENABLED)
    E_BIKE_THROTTLE_mode_detection();
    #endif
}

///****************E_bike brake on pot ramp down************************/
//if (PMSM_FOC_CTRL.E_bike_user_brake_flag == 1)
//{
//	PMSM_FOC_CTRL.E_bike_pot_ramp_down = 1;
//
//	if(PMSM_FOC_CTRL.E_bike_user_brake_counter > 64)
//	{
//		PMSM_FOC_CTRL.E_bike_user_brake_counter = 0;
//		ADC.adc_res_pot = ADC.adc_res_pot - E_bike_brake_rate;
//
//    	if(SYSTEM_BE_IDLE)
//    	{
//            /* Next go to Motor stop state and wait for motor start command */
////            		ADC.adc_res_pot = 0;
//    		PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_STOP_MOTOR;
//    	}
//	}
//}//PMSM_FOC_CTRL.E_bike_user_brake_flag == 1
//
///*------------------------- Bike Speed Detection --------------------------*/
//






/**
 * @}
 */
/**
 * @}
 */
