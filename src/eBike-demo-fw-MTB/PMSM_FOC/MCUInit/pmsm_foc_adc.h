/**
 * @file pmsm_foc_adc.h
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
#ifndef PMSM_FOC_ADC_H_
#define PMSM_FOC_ADC_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <PMSM_FOC/Configuration/pmsm_foc_config.h>

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup MCUInit
 * @{
 */
/*********************************************************************************************************************
 * MACRO
 ********************************************************************************************************************/
#define SHS0_CALOC0 ((uint32_t *)0x480340E0)
#define SHS0_CALOC1 ((uint32_t *)0x480340E4)
#define SHS0_CALCTR ((uint32_t *)0x480340BC)

#define SHS_CALLOC0_CLEAR_OFFSET (0x8000U)
#define REG_RESET (0x00U)
#define GLOBCFG_CLEAR (0x80030000U)
#define CLEAR_OFFSET_CALIB_VALUES         *SHS0_CALOC0 = SHS_CALLOC0_CLEAR_OFFSET;\
                                          *SHS0_CALOC1 = SHS_CALLOC0_CLEAR_OFFSET
/** Delay cycles to complete startup calibration */
#define VADC_CALIBRATION_CYCLE_DELAY  (20U)
/** Trigger dummy conversion for 9* 2000 times.*/
#define VADC_DUMMY_CONVERSION_COUNTER (18000U)


/*********************************************************************************************************************
 * ENUMS
 ********************************************************************************************************************/
/* Enumeration for MCU with internal opamp gain selection */
typedef enum SHS_GAIN_FACTOR
{
    SHS_GAIN_FACTOR_1 = 0,   /**< Select gain factor 1 */
    SHS_GAIN_FACTOR_3,       /**< Select gain factor 3 */
    SHS_GAIN_FACTOR_6,       /**< Select gain factor 6 */
    SHS_GAIN_FACTOR_12       /**< Select gain factor 12 */
}SHS_GAIN_FACTOR_t;


/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
typedef struct ADCType
{
    int32_t adc_bias_iu;			/* Bias of ADC Iu. */
    int32_t adc_bias_iv;			/* Bias of ADC Iv. */
    int32_t adc_bias_iw;			/* Bias of ADC Iw. */
    int32_t adc_bias_idc;		  /* Bias of dc-link current amplifier, or on-chip gain */

  	int32_t adc_res_pot;			/* ADC Value of potentiometer (POT) */
  	int32_t adc_res_pot2;
  	int32_t adc_res_vdc;			/* ADC Value of inverter DC link voltage Vdc */
  	int32_t adc_res_vdc_filtered;      /* ADC Value of inverter DC link voltage Vdc */
  	int32_t adc_res_idc;          /* ADC Value of inverter DC link current Idc*/

    uint16_t adc_res_temp;      /* ADC Value of temperature sensor */
    uint16_t adc_res_iu;		    /* ADC result Motor Ph-U */
    uint16_t adc_res_iv;        /* ADC result Motor Ph-V */
    uint16_t adc_res_iw;        /* ADC result Motor Ph-W */

    /*E-Bike Functionality*/
    uint16_t adc_res_e_bike_brake; 			/*ADC value of e_bike brake*/
    uint16_t adc_res_e_bike_brake_filtered; /*ADC value of e_bike brake LPF*/
    int32_t adc_res_tor_sense;				/*ADC value of torque sensor output*/

}ADC_t;

/*********************************************************************************************************************
 * EXTETRN
 ********************************************************************************************************************/
extern ADC_t ADC;

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC module with the associated configuration structure for 3-shunt phase current sensing. <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void PMSM_FOC_VADC_PhCurrentInit(void);

/**
 * @param current_sector_num - Present SVPWM sector number
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Configures the phase current VADC channels in synchronous mode as per the SVPWM present sector number <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
#if(USER_CURRENT_SENSING == THREE_SHUNT_SYNC_CONV)
void PMSM_FOC_VADC_PhCurrentSyncConfig(uint16_t current_sector_num);
#endif //End of #if(USER_CURRENT_SENSING == THREE_SHUNT_SYNC_CONV)


void PMSM_FOC_VADC_ModuleInit(void);
/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC channel for potentiameter voltage sensing.  <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void PMSM_FOC_VADC_PotInit(void);

/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC channel for DC Link voltage sensing.  <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void PMSM_FOC_VADC_VDCInit(void);

void PMSM_FOC_VADC_BoardTempSensor_Init(void);

/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC channel for BEMF voltage sensing.  <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */

#if(E_BIKE_REF == ENABLED)
void PMSM_FOC_VADC_MOTORTempSensor_Init(void);

/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC channel for Motor temperature sensing.  <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */


void PMSM_FOC_VADC_E_BIKE_BRAKE_Init(void);

/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC channel for E_BIKE_BRAKE sensing.  <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
#endif

void PMSM_FOC_VADC_BEMF_Init(void);
/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Removes the BEMF sensing VADC channel from the SCAN request source.  <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void PMSM_FOC_VADC_BEMF_Disable(void);

void PMSM_FOC_VADC_GainCalib(void);
void PMSM_FOC_VADC_StartupCalib(void);

#if(PMSM_FOC_HARDWARE_KIT == LEV_4KW_KIT)
void pmsm_adc_idc_init(void);
void pmsm_adc_vref_init(void);
#endif

/**
 * @}
 */

/**
 * @}
 */

#endif /* MCUINIT_ADC_H_ */
