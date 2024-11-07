/**
 * @file pmsm_foc_interface.h
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
#ifndef PMSM_FOC_INTERFACE_H_
#define PMSM_FOC_INTERFACE_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <PMSM_FOC/FOCLib/BMCLIB/bmclib_ramp_gen.h>
#include <PMSM_FOC/FOCLib/BMCLIB/bmclib_svpwm.h>
#include <PMSM_FOC/FOCLib/BMCLIB/bmclib_pi.h>
#include <PMSM_FOC/Configuration/pmsm_foc_config.h>
#include "../MIDSys/pmsm_foc_current_sense_3S.h"
#include "../MIDSys/pmsm_foc_debug.h"
#include "../ControlModules/pmsm_foc_error_handling.h"
#include "../ControlModules/pmsm_foc_functions.h"
#include "../ControlModules/pmsm_foc_state_machine.h"

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup ControlModules
 * @{
 */

/*********************************************************************************************************************
 * ENUMS
 ********************************************************************************************************************/
/**
 * @brief This enumerates the E-Bike throttle mode user selection.
 */
typedef enum
{
    E_BIKE_THROTTLE_ECO_MODE			= 1,
	E_BIKE_THROTTLE_NORMAL_MODE   		= 2,
	E_BIKE_THROTTLE_SPORT_MODE			= 3,
} E_BIKE_THROTTLE_MODE_t;
/*********************************************************************************************************************
 * EXTERN's
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * API PROTOTYPES
 ********************************************************************************************************************/
void PMSM_FOC_MotorStart(void);
void PMSM_FOC_MotorStop(void);
void PMSM_FOC_SetMotorDirection(void);

#if(E_BIKE_REF == ENABLED)
void E_BIKE_THROTTLE_mode_detection(void);
#endif
/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/**
 * @brief This function monitors the system periodically
 *
 * @param None
 *
 * @retval None
 */
__STATIC_FORCEINLINE void PMSM_FOC_SysMonitoring(void)
{
    /***************** Over Current protection *****************************************/
    #if(USER_OVERCURRENT_PROTECTION_ENABLE == ENABLED)
    uint32_t current_mapped_peak;

    #if(USER_MAP_CURRENT_TO_DCLINK == ENABLED)
    /* scale phase current peak to equivalent DC link current peak by multiplying modulation index*/
    current_mapped_peak = (uint32_t)((PMSM_FOC_OUTPUT.current_i_mag_filtered * PMSM_FOC_OUTPUT.svm_vref_16) >> 15);
    #else
    /* Used filtered phase current for OCP */
    current_mapped_peak = PMSM_FOC_OUTPUT.current_i_mag_filtered;
    #endif

#if(USER_OCP_LEVELS_PROTECTION == ENABLED)
    for(int i=0; i< PMSM_FOC_CTRL.ocp.OCP_level_num; i++)
    { /* scale phase current peak to equivalent DC link current peak by multiplying modulation index*/
      if(current_mapped_peak > PMSM_FOC_CTRL.ocp.OCP_level[i])
      {
    	  PMSM_FOC_CTRL.ocp.OCP_level_count[i]++;
      }
      else
      {
        if(PMSM_FOC_CTRL.ocp.OCP_level_count[i] > 0)
        {
        	PMSM_FOC_CTRL.ocp.OCP_level_count[i]--;
        }
      }

      if(PMSM_FOC_CTRL.ocp.OCP_level_count[i] > PMSM_FOC_CTRL.ocp.OCP_level_time[i])
      {
        PMSM_FOC_CTRL.error_status |= PMSM_FOC_EID_OVER_CURRENT;
        PMSM_FOC_CTRL.ocp.OCP_level_error[i] = TRUE;
        /* when one OCP happens, no need to check others, so reset all counts */
        PMSM_FOC_Multi_Level_OCP_Reset_Count();
        break;
      }
    }
#else
//    XMC_GPIO_ToggleOutput(TEST_PIN);
    if(current_mapped_peak > PMSM_FOC_CTRL.ocp.ocp_level)
    {
        PMSM_FOC_CTRL.ocp.ocp_level_time_counter++;
    }
    else
    {
        if(PMSM_FOC_CTRL.ocp.ocp_level_time_counter > 0)
        {
          PMSM_FOC_CTRL.ocp.ocp_level_time_counter--;
        }
    }

    if(PMSM_FOC_CTRL.ocp.ocp_level_time_counter > PMSM_FOC_CTRL.ocp.ocp_level_time_count)
    {
      /* Next go to error state with over current fault */
      PMSM_FOC_CTRL.error_status |= PMSM_FOC_EID_OVER_CURRENT;
      /* Reset OCP detection counter for next iteration */
      PMSM_FOC_CTRL.ocp.ocp_level_time_counter = 0;
    }
#endif //#if(USER_OCP_LEVELS_PROTECTION == ENABLED)

    #endif //End of #if(USER_OVERCURRENT_PROTECTION_ENABLE == ENABLED)

#if 0		//(PMSM_FOC_HARDWARE_KIT == LEV_4KW_KIT)
   pmsm_foc_get_IDCLink_current();

  /* ------------------------------------ SW Over Current Protection -------------------------------------- */
  /* ------------------------------------ IDC Over Current Protection -------------------------------------- */
  if((ADC.adc_res_idc) > IDC_MAX_LIMIT)
  {
	  PMSM_FOC_CTRL.error_status |= PMSM_FOC_EID_IDC_OVERCURRENT;
  }
  /* ------------------------------------ IDC Over Current Protection -------------------------------------- */
#endif

    /* Below Monitoring is done only in the close loop */
    if(PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_CLOSED_LOOP)
    {
        /***************** Torque Limiting  *****************************************/
      #if(USER_TORQUE_LIMIT_ENABLE == ENABLED)
      /* Check if iq limit exceeded even after blanking time. If yes then go to error state */
      if (PMSM_FOC_OUTPUT.iq_filtered >= PMSM_FOC_CTRL.limit_max_iq)
      {
          if (PMSM_FOC_CTRL.iq_limit_blanking_counter >= IQ_LIMIT_BLANKING_TIME)
          {
              /* Even after motor speed reduced due to current limit, motor still drawing higher current */
              /* Next go to error state */
              PMSM_FOC_CTRL.error_status |= PMSM_FOC_EID_TORQUE_LIMIT_EXCEED;
          }
          else
          {
              PMSM_FOC_CTRL.iq_limit_blanking_counter++;
          }
      }
      else
      {
         if(PMSM_FOC_CTRL.iq_limit_blanking_counter > 0)
         {
            PMSM_FOC_CTRL.iq_limit_blanking_counter--;
         }
      }
      #endif  // End of #if(USER_TORQUE_LIMIT_ENABLE == ENABLED)

	  #if(USER_MOTOR_STALL_DETECTION_ENABLE == ENABLED)
      /* stall protection */
      hall_sensor_data.stall_counter ++;

      if(hall_sensor_data.stall_counter > USER_STALL_COUNT)
      {
    	  PMSM_FOC_CTRL.error_status |= PMSM_FOC_EID_STALL;
    	  hall_sensor_data.stall_counter = 0;
      }
	  #endif
    }

    /************************************************************************************/
    if (PMSM_FOC_CTRL.error_status != PMSM_FOC_EID_NO_ERROR)
    {
#if (SPI_LIB == ENABLED)
       /* Activate brake */
        XMC_GPIO_SetOutputLow(nBRAKE_PIN);
#else
        pmsm_foc_disable_inverter();
#endif //SPI=Enabled

        PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_ERROR;
    }
}

#if(POT_MV_EN_DIS == ENABLED)
__STATIC_FORCEINLINE void PMSM_FOC_POT_Avg_Process(void)
{
  /* continuous moving average regardless of Hall sector or electrical cycle changes */
  if(PMSM_FOC_THROTTLE_AVG.arrar_full_flag == 0)
  {
    PMSM_FOC_THROTTLE_AVG.throttle_adc_values[PMSM_FOC_THROTTLE_AVG.arraycounter] = ADC.adc_res_pot;
    /*Total sum: Subtract oldest number and replace with new number from ADC reading*/
    PMSM_FOC_THROTTLE_AVG.sum_array_data += ADC.adc_res_pot;

    pmsm_foc_divide(PMSM_FOC_THROTTLE_AVG.sum_array_data, (uint32_t)(PMSM_FOC_THROTTLE_AVG.arraycounter + 1));
    pmsm_foc_divide_getresult(&PMSM_FOC_THROTTLE_AVG.average_data);
  }
  else
  {
//    uint16_t head = 0;
//    if(PMSM_FOC_THROTTLE_AVG.arraycounter == WINDOW_SIZE - 1)
//      head = 0;
//    else
//      head = PMSM_FOC_THROTTLE_AVG.arraycounter + 1;

//    PMSM_FOC_THROTTLE_AVG.throttle_adc_values[PMSM_FOC_THROTTLE_AVG.arraycounter] = ADC.adc_res_pot;
//    PMSM_FOC_THROTTLE_AVG.sum_array_data += ADC.adc_res_pot;
//    PMSM_FOC_THROTTLE_AVG.sum_array_data -= PMSM_FOC_THROTTLE_AVG.throttle_adc_values[head];

    PMSM_FOC_THROTTLE_AVG.sum_array_data += ADC.adc_res_pot;
    PMSM_FOC_THROTTLE_AVG.sum_array_data -= PMSM_FOC_THROTTLE_AVG.throttle_adc_values[PMSM_FOC_THROTTLE_AVG.arraycounter];
    PMSM_FOC_THROTTLE_AVG.throttle_adc_values[PMSM_FOC_THROTTLE_AVG.arraycounter] = ADC.adc_res_pot;

    PMSM_FOC_THROTTLE_AVG.average_data = PMSM_FOC_THROTTLE_AVG.sum_array_data >> WINDOW_SIZE_DIV_SCALE;
  }

  PMSM_FOC_THROTTLE_AVG.arraycounter++;
  if(PMSM_FOC_THROTTLE_AVG.arraycounter >= WINDOW_SIZE)
  {
    PMSM_FOC_THROTTLE_AVG.arrar_full_flag = 1;
    PMSM_FOC_THROTTLE_AVG.arraycounter = 0;
  }

  /* electrical cycle moving average */
  if(PMSM_FOC_THROTTLE_AVG.hall_change_flag)
  {
    PMSM_FOC_THROTTLE_AVG.hall_change_once_flag = 1;

    if(PMSM_FOC_THROTTLE_AVG.cycle_full_flag == 0)
    {
      // not one electrical cycle yet
      PMSM_FOC_THROTTLE_AVG.throttle_adc_avg[PMSM_FOC_THROTTLE_AVG.hall_change_cnt] = PMSM_FOC_THROTTLE_AVG.average_data;
      PMSM_FOC_THROTTLE_AVG.sum_ecycle_avg += PMSM_FOC_THROTTLE_AVG.average_data;

      pmsm_foc_divide(PMSM_FOC_THROTTLE_AVG.sum_ecycle_avg, (uint32_t)(PMSM_FOC_THROTTLE_AVG.hall_change_cnt + 1));
    }
    else
    {
      //more than one electrical cycle
//      uint16_t head = 0;
//      if(PMSM_FOC_THROTTLE_AVG.hall_change_cnt == ECYCLE_NUM-1)
//        head = 0;
//      else
//        head = PMSM_FOC_THROTTLE_AVG.hall_change_cnt + 1;

//      PMSM_FOC_THROTTLE_AVG.throttle_adc_avg[PMSM_FOC_THROTTLE_AVG.hall_change_cnt] = PMSM_FOC_THROTTLE_AVG.average_data;
//      PMSM_FOC_THROTTLE_AVG.sum_ecycle_avg += PMSM_FOC_THROTTLE_AVG.average_data;
//      PMSM_FOC_THROTTLE_AVG.sum_ecycle_avg -= PMSM_FOC_THROTTLE_AVG.throttle_adc_avg[head];

      PMSM_FOC_THROTTLE_AVG.sum_ecycle_avg += PMSM_FOC_THROTTLE_AVG.average_data;
      PMSM_FOC_THROTTLE_AVG.sum_ecycle_avg -= PMSM_FOC_THROTTLE_AVG.throttle_adc_avg[PMSM_FOC_THROTTLE_AVG.hall_change_cnt];
      PMSM_FOC_THROTTLE_AVG.throttle_adc_avg[PMSM_FOC_THROTTLE_AVG.hall_change_cnt] = PMSM_FOC_THROTTLE_AVG.average_data;

      pmsm_foc_divide(PMSM_FOC_THROTTLE_AVG.sum_ecycle_avg, ECYCLE_NUM);
    }

    PMSM_FOC_THROTTLE_AVG.hall_change_cnt++;
    if(PMSM_FOC_THROTTLE_AVG.hall_change_cnt >= ECYCLE_NUM) /*without (num -1), will have cnt [18] result sum overflow.*/
    {
      PMSM_FOC_THROTTLE_AVG.hall_change_cnt = 0;
      PMSM_FOC_THROTTLE_AVG.cycle_full_flag = 1;
    }

    pmsm_foc_divide_getresult(&PMSM_FOC_THROTTLE_AVG.average_ecycle);

    PMSM_FOC_THROTTLE_AVG.hall_change_flag = 0;
  }

  if(PMSM_FOC_THROTTLE_AVG.hall_change_once_flag)
    PMSM_FOC_CTRL.average_data = PMSM_FOC_THROTTLE_AVG.average_ecycle;
  else
    PMSM_FOC_CTRL.average_data = PMSM_FOC_THROTTLE_AVG.average_data;

}
#endif/*#if(POT_MV_EN_DIS == ENABLED)*/

__STATIC_FORCEINLINE void PMSM_FOC_MiscWorks(void)
{
    #if(USER_REF_SETTING == BY_POT_ONLY)
    uint16_t pot_adc_result;
    pot_adc_result = XMC_VADC_GROUP_GetResult(VADC_POT_GROUP, VADC_POT_RESULT_REG);
    /* POT ADC LPF */
    ADC.adc_res_pot += (int32_t) ((pot_adc_result - ADC.adc_res_pot) >> USER_POT_ADC_LPF);
    #endif

    #if(USER_WATCH_DOG_TIMER == ENABLED)
    /* Service watchdog. Without WDT service regularly , it will reset system.*/
    XMC_WDT_Service();
    #endif

    /* Read ADC result for DC link voltage */
    ADC.adc_res_vdc = XMC_VADC_GROUP_GetResult(VADC_VDC_GROUP, VADC_VDC_RESULT_REG);
    /* DC link ADC LPF. */
    ADC.adc_res_vdc_filtered += (int32_t)((ADC.adc_res_vdc - ADC.adc_res_vdc_filtered) >> USER_VDC_LPF);

  	/**************************************************************************************************************
  	* E-Bike Throttle reading and Brake reading *
  	* *************************************************************************************************************/
#if (E_BIKE_REF == ENABLED)
    /*First Torque/Throttle zero check before able to ramp up*/
    /*GUI Slider*/
    if(PMSM_FOC_CTRL.ebike_startup_throttle_check == 0) /*Start check*/
    {
        if(ADC.adc_res_pot > USER_TH_POT_ADC_STOP)
        {
        	PMSM_FOC_CTRL.ebike_throttle_enable = 0; 	/*0 - throttle/slider not in zero position. Disable ramp up.*/
        }
        else if(ADC.adc_res_pot <= USER_TH_POT_ADC_STOP)
        {
        	PMSM_FOC_CTRL.ebike_throttle_enable = 1;	/*1 - throttle/slider back/in zero position.*/
        	PMSM_FOC_CTRL.ebike_startup_throttle_check = 1; /*1- stop check.*/
        }
    }
#endif

    /*Potentiometer Input Moving Average*/
#if(POT_MV_EN_DIS == ENABLED)
    PMSM_FOC_POT_Avg_Process();
#endif


}

/**
 * @}
 */

/**
 * @}
 */
#endif /* PMSM_FOC_INTERFACE_H_ */


