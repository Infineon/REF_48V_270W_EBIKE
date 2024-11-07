/**
 * @file bmclib_svpwm.h
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

#ifndef BMCLIB_SVPWM_H_
#define BMCLIB_SVPWM_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

#include "../PMSM_FOC/Configuration/pmsm_foc_config.h"
#include "xmc_ccu8.h"
/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#if(USER_SVM_SINE_LUT_SIZE == 256U)  /* Look-Up Table (LUT) array size 256 */
#define ANGLETEMP_SHIFT   (8U)       /* For calculation of Angle_Temp */
#define SECTOR_ANGLE_AND  (0x00FFU)  /* For calculation of Sector_Angle */
#define SECTOR_NO_SHIFT   (8U)       /* For calculation of sector number */
#define MAX_LUT_INDEX     (255U)     /* Maximum angle index in LUT */
#else                                /* Look-Up Table (LUT) array size 1024 */
#define ANGLETEMP_SHIFT   (6U)       /* For calculation of Angle_Temp */
#define SECTOR_ANGLE_AND  (0x03FFU)  /* For calculation of Sector_Angle */
#define SECTOR_NO_SHIFT   (10U)      /* For calculation of sector number */
#define MAX_LUT_INDEX     (1023U)    /* Maximum angle index in LUT */
#endif

#define RATIO_T0_111               (2U)                       /* = 2 for standard SVM. */
#define SHIFT_OVERMODULATION       (6U)                       /* Over-modulation resolution increase for calculations */

/*********************************************************************************************************************
 * EXTERN
 ********************************************************************************************************************/
extern const uint16_t SIN60_TAB[];  /* Sine LUT used for SVM calculations, array size 256 or 1024. */

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
typedef void (*SVPWM_MODULATION_t)(uint32_t svpwm_struct_address);

typedef enum
{
    ADC_SAMPLING_USE_ALL,                       /*!< Use three ADC for sampling shunt current */
    ADC_SAMPLING_USE_TWO                        /*!< Use two ADC for sampling shunt current */
} ADC_SAMPLING_FLAG_t;

typedef struct BMCLIB_SVPWMType
{
    CCU8_GLOBAL_TypeDef * ccu8_module_ptr;      /*!< CCU8 module pointer, CCU80,CCU81*/
    CCU8_CC8_TypeDef *ccu8_phu_module_ptr;      /*!< CCU8 module pointer for Phase U */
    CCU8_CC8_TypeDef *ccu8_phv_module_ptr;      /*!< CCU8 module pointer for Phase V */
    CCU8_CC8_TypeDef *ccu8_phw_module_ptr;      /*!< CCU8 module pointer for Phase W */

    SVPWM_MODULATION_t modulation_func_ptr;     /*!< Function pointer for the svpwm modulation - 7 Segment, 5 Segment */
    ADC_SAMPLING_FLAG_t flag_3or2_adc;          /*!< used for dynamic switching between current sampling */

    uint16_t pwm_period_reg_val;                /*!< PWM period register value */
    uint16_t t_min;                             /*!< Minimum SVPWM time vector duration in which current sensing of 3 phase is possible */
    uint16_t t_max;                             /*!< Maximum SVPWM time vector duration */
    uint16_t t0_threshold;                      /*!< If T0 > T0 threshold 3 current sampling else, two current sampling */
    uint16_t current_sector_num;                /*!< Current new sector number: 0 ~ 5 (represent Sector A ~ F) in SVM space vector hexagon */
    uint16_t previous_sector_num;               /*!< SVM sector number of last PWM cycle, for 3-phase current reconstruction */

    uint16_t invalid_current_sample_flag;       /*!< indicates invalid current sample */
    uint16_t sine_table_scale_up;               /*!< Sine table scale up for better resolution  - 0,1,2 */

    uint16_t sampling_critical_vector;          /*!< Internal variable */
    uint16_t t1;                                /*!< Internal variable */
    uint16_t t2;                                /*!< Internal variable */
    uint16_t t1nt2; /* Time (T1+T2). */         /*!< Internal variable */
    uint16_t t0;                                /*!< Internal variable */
} BMCLIB_SVPWM_t;

/*********************************************************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * LOCAL VARIABLES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
/**
 * @param Amplitude   \n
 * @param Angle Rotor Angle \n
 * @param svpwm_ptr BMCLIB_SVPWM_t structure pointer (pass by reference)
 *
 * @return None<BR>
 *
 * \par Description:
 * To update SVPWM CCU8 duty cycles, based upon the configuration <BR>\n
 *
 * \par Related APIs:
 * None.
 */
/* API to update SVPWM CCU8 duty cycles based upon modulation scheme configured  */
__STATIC_FORCEINLINE void BMCLIB_SVPWM_Update(uint16_t amplitude, uint16_t angle, BMCLIB_SVPWM_t * const svpwm_ptr)
{
    /* SVM sector angle. */
    uint16_t angle_temp;
    uint16_t sector_angle_index;

    svpwm_ptr->previous_sector_num = svpwm_ptr->current_sector_num; /* Record sector information of last PWM cycle. */

    /* Angle: 0 ~ 2^16 represent electrical angle 0° ~ 360°. */
    angle_temp = (uint16_t)((angle * 6U) >> ANGLETEMP_SHIFT);
    sector_angle_index = (uint16_t)(angle_temp & SECTOR_ANGLE_AND); /* Relative angle θrel in each sector. */
    svpwm_ptr->current_sector_num = (uint16_t)(angle_temp >> SECTOR_NO_SHIFT); /* Update new SVM sector number. */

    /* Calculate T1 / T2 by LUT. */  
    svpwm_ptr->t1 = (uint16_t)(((uint32_t)((amplitude * SIN60_TAB[MAX_LUT_INDEX - sector_angle_index]) >> 15) * SVM_LUT_SCALE) >> (15 + svpwm_ptr->sine_table_scale_up));
    svpwm_ptr->t2 = (uint16_t)(((uint32_t)((amplitude * SIN60_TAB[sector_angle_index]) >> 15) * SVM_LUT_SCALE) >> (15 + svpwm_ptr->sine_table_scale_up));

    svpwm_ptr->t1nt2 = svpwm_ptr->t1 + svpwm_ptr->t2; 

     /* Temp variable for (T1+T2) <= CCU8_PERIOD_REG. */
    if (svpwm_ptr->t1nt2 > svpwm_ptr->pwm_period_reg_val)
    {
        MATH->DIVCON = (0x00008004 | (SHIFT_OVERMODULATION << 16UL) | (SHIFT_OVERMODULATION << 8UL));
        MATH->DVD = svpwm_ptr->t1 * svpwm_ptr->pwm_period_reg_val;
        MATH->DVS = svpwm_ptr->t1nt2;
        svpwm_ptr->t1nt2 = svpwm_ptr->pwm_period_reg_val;

        /* CPU wait */
        while (MATH->DIVST);

        /* Read CORDIC result */
        svpwm_ptr->t1 = MATH->QUOT;
        svpwm_ptr->t2 = svpwm_ptr->pwm_period_reg_val - svpwm_ptr->t1;
    }

    switch (svpwm_ptr->current_sector_num)
    {
    case 0:                           /* Sector A */
    case 2:                           /* Sector C */
    case 4:                           /* Sector E */
        if(svpwm_ptr->t2 > svpwm_ptr->t_max)
        {
        	svpwm_ptr->t2 = svpwm_ptr->t_max;
        	svpwm_ptr->t1 = svpwm_ptr->t1nt2 - svpwm_ptr->t2;
        }
        svpwm_ptr->sampling_critical_vector = svpwm_ptr->t2;
        break;

    case 1:                           /* Sector B */
    case 3:                           /* Sector D */
    case 5:                           /* Sector F */
        if(svpwm_ptr->t1 > svpwm_ptr->t_max)
        {
        	svpwm_ptr->t1 = svpwm_ptr->t_max;
        	svpwm_ptr->t2 = svpwm_ptr->t1nt2 - svpwm_ptr->t2;
        }
        svpwm_ptr->sampling_critical_vector = svpwm_ptr->t1;
        break;
    default:
        break;
    }

    svpwm_ptr->t0 = svpwm_ptr->pwm_period_reg_val - svpwm_ptr->t1nt2;

    /* Calls modulator function according to the user configuration */
    svpwm_ptr->modulation_func_ptr((uint32_t)svpwm_ptr);

    /* Check if 3 or 2 Phase current sampling is possible */
    if (svpwm_ptr->t0 > svpwm_ptr->t0_threshold)
    {
      /* To use all (e.g.: three) ADC samplings for current reconstruction. */
      svpwm_ptr->flag_3or2_adc = ADC_SAMPLING_USE_ALL;
    }
    else
    {
       /* To use two ADC samplings for current reconstruction. */
       svpwm_ptr->flag_3or2_adc = ADC_SAMPLING_USE_TWO;
    }

    /* Check if Phase current sampling is possible */
    if(svpwm_ptr->sampling_critical_vector > svpwm_ptr->t_max)
    {
      svpwm_ptr->invalid_current_sample_flag = 1;
    }
    else
    {
      svpwm_ptr->invalid_current_sample_flag = 0;
    }

    /* Enable shadow transfer for slice 0,1,2 for CCU80 Kernel. */
    svpwm_ptr->ccu8_module_ptr->GCSS |= (uint32_t) (XMC_CCU8_SHADOW_TRANSFER_SLICE_0 | XMC_CCU8_SHADOW_TRANSFER_SLICE_1
        | XMC_CCU8_SHADOW_TRANSFER_SLICE_2);
}

/**
 * @param svpwm_struct_address BMCLIB_SVPWM_t structure address
 *
 * @return None<BR>
 *
 * \par Description:
 * To calculate the duty cycle for each phase as per 7 Segment(Continuous) SVPWM modulation, based upon BMCLIB_SVPWM value <BR>\n
 *
 * \par Related APIs:
 * None.
 */
/* This function calculates the duty cycle for each phase as per 7 Segment(Continuous) SVPWM modulation */
__STATIC_FORCEINLINE void BMCLIB_SVPWM_SevenSeg(uint32_t svpwm_struct_address)
{
    /* 7-segment SVM, T0, T0_111 for first [111], T0_111 + T1/2, T0_111 + T2/2, T0_111 + (T1+T2)/2. */
    uint16_t t0_111;
    uint16_t t0nhalft1;
    uint16_t t0nhalft2;
    uint16_t t0nhalft1nt2;
    uint16_t pwm_period;
    BMCLIB_SVPWM_t * svpwm_ptr;
    svpwm_ptr = (BMCLIB_SVPWM_t *)svpwm_struct_address;

    pwm_period = svpwm_ptr->pwm_period_reg_val;
    t0_111 = svpwm_ptr->t0 >> RATIO_T0_111;                                                    /* T0_111, time of first [111]. */
    t0nhalft1 = (svpwm_ptr->t0 + (uint16_t)(svpwm_ptr->t1 << (RATIO_T0_111 - 1U))) >> RATIO_T0_111;       /* T0_111 + T1/2. */
    t0nhalft2 = (svpwm_ptr->t0 + (uint16_t)(svpwm_ptr->t2 << (RATIO_T0_111 - 1U))) >> RATIO_T0_111;       /* T0_111 + T2/2. */
    t0nhalft1nt2 = (svpwm_ptr->t0 + (uint16_t)((svpwm_ptr->t1 + svpwm_ptr->t2) << (RATIO_T0_111 - 1U))) >> RATIO_T0_111;     /* T0_111 + (T1+T2)/2. */

    /* Standard 7-segment symmetric PWM: */
    switch (svpwm_ptr->current_sector_num)
    {
    case 0:                           /* Sector A */
        svpwm_ptr->ccu8_phu_module_ptr->CR1S = (uint32_t) t0nhalft1nt2;
        svpwm_ptr->ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1nt2);

        svpwm_ptr->ccu8_phv_module_ptr->CR1S = (uint32_t) t0nhalft2;
        svpwm_ptr->ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft2);

        svpwm_ptr->ccu8_phw_module_ptr->CR1S = (uint32_t) t0_111;
        svpwm_ptr->ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - t0_111);
        svpwm_ptr->sampling_critical_vector = svpwm_ptr->t2;
        break;

    case 1:                           /* Sector B */
        svpwm_ptr->ccu8_phu_module_ptr->CR1S = (uint32_t) t0nhalft1;
        svpwm_ptr->ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1);

        svpwm_ptr->ccu8_phv_module_ptr->CR1S = (uint32_t) t0nhalft1nt2;
        svpwm_ptr->ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1nt2);

        svpwm_ptr->ccu8_phw_module_ptr->CR1S = (uint32_t) t0_111;
        svpwm_ptr->ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - t0_111);
        svpwm_ptr->sampling_critical_vector = svpwm_ptr->t1;
        break;

    case 2:                         /* Sector C */
        svpwm_ptr->ccu8_phu_module_ptr->CR1S = (uint32_t) t0_111;
        svpwm_ptr->ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - t0_111);

        svpwm_ptr->ccu8_phv_module_ptr->CR1S = (uint32_t) t0nhalft1nt2;
        svpwm_ptr->ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1nt2);

        svpwm_ptr->ccu8_phw_module_ptr->CR1S = (uint32_t) t0nhalft2;
        svpwm_ptr->ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft2);
        svpwm_ptr->sampling_critical_vector = svpwm_ptr->t2;
        break;

    case 3:                           /* Sector D */
        svpwm_ptr->ccu8_phu_module_ptr->CR1S = (uint32_t) t0_111;
        svpwm_ptr->ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - t0_111);

        svpwm_ptr->ccu8_phv_module_ptr->CR1S = (uint32_t) t0nhalft1;
        svpwm_ptr->ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1);

        svpwm_ptr->ccu8_phw_module_ptr->CR1S = (uint32_t) t0nhalft1nt2;
        svpwm_ptr->ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1nt2);
        svpwm_ptr->sampling_critical_vector = svpwm_ptr->t1;
        break;

    case 4:                           /* Sector E */
        svpwm_ptr->ccu8_phu_module_ptr->CR1S = (uint32_t) t0nhalft2;
        svpwm_ptr->ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft2);

        svpwm_ptr->ccu8_phv_module_ptr->CR1S = (uint32_t) t0_111;
        svpwm_ptr->ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - t0_111);

        svpwm_ptr->ccu8_phw_module_ptr->CR1S = (uint32_t) t0nhalft1nt2;
        svpwm_ptr->ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1nt2);
        svpwm_ptr->sampling_critical_vector = svpwm_ptr->t2;
        break;

    case 5:                           /* Sector F */
        svpwm_ptr->ccu8_phu_module_ptr->CR1S = (uint32_t) t0nhalft1nt2;
        svpwm_ptr->ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1nt2);

        svpwm_ptr->ccu8_phv_module_ptr->CR1S = (uint32_t) t0_111;
        svpwm_ptr->ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - t0_111);

        svpwm_ptr->ccu8_phw_module_ptr->CR1S = (uint32_t) t0nhalft1;
        svpwm_ptr->ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1);
        svpwm_ptr->sampling_critical_vector = svpwm_ptr->t1;
        break;

    default:
        // Control should not come here.
        break;
    }
}

/**
 * @param svpwm_struct_address BMCLIB_SVPWM_t structure address
 *
 * @return None<BR>
 *
 * \par Description:
 * To calculate the duty cycle for each phase as per 5 Segment(Discontinuous) SVPWM modulation, based upon BMCLIB_SVPWM value <BR>\n
 *
 * \par Related APIs:
 * None.
 */

/* This function calculates the duty cycle for each phase as per 5 Segment(Discontinuous) SVPWM modulation */
__STATIC_FORCEINLINE void BMCLIB_SVPWM_FiveSeg(uint32_t svpwm_struct_address)
{
    /* 5-segment SVM, (T1+T2)/2, T2/2, T1/2 */
    uint16_t half_t1nt2;
    uint16_t half_t2;
    uint16_t half_t1;
    uint16_t pwm_period;
    BMCLIB_SVPWM_t * svpwm_ptr;
    svpwm_ptr = (BMCLIB_SVPWM_t *)svpwm_struct_address;

    pwm_period = svpwm_ptr->pwm_period_reg_val;

    half_t2 = (uint16_t)(svpwm_ptr->t2 >> 1U);                    // T2/2.
    half_t1 = (uint16_t)(svpwm_ptr->t1 >> 1U);                    // T1/2.
    half_t1nt2 = (uint16_t)(svpwm_ptr->t1nt2 >> 1U);              // (T1+T2)/2.

    /* Standard 5-segment symmetric PWM: */
    switch (svpwm_ptr->current_sector_num)
    {
    case 0:                        // Sector A
        svpwm_ptr->ccu8_phu_module_ptr->CR1S = (uint32_t) half_t1nt2;
        svpwm_ptr->ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1nt2);

        svpwm_ptr->ccu8_phv_module_ptr->CR1S = (uint32_t) half_t2;
        svpwm_ptr->ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - half_t2);

        svpwm_ptr->ccu8_phw_module_ptr->CR1S = (uint32_t) 0;
        svpwm_ptr->ccu8_phw_module_ptr->CR2S = (uint32_t)(pwm_period+1);

        svpwm_ptr->sampling_critical_vector = svpwm_ptr->t2;
        break;

    case 1:                       // Sector B
        svpwm_ptr->ccu8_phu_module_ptr->CR1S = (uint32_t) half_t1;
        svpwm_ptr->ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1);

        svpwm_ptr->ccu8_phv_module_ptr->CR1S = (uint32_t) half_t1nt2;
        svpwm_ptr->ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1nt2);

        svpwm_ptr->ccu8_phw_module_ptr->CR1S = (uint32_t) 0;
        svpwm_ptr->ccu8_phw_module_ptr->CR2S = (uint32_t)(pwm_period+1);

        svpwm_ptr->sampling_critical_vector = svpwm_ptr->t1;
        break;

    case 2:                       // Sector C
        svpwm_ptr->ccu8_phu_module_ptr->CR1S = (uint32_t) 0;
        svpwm_ptr->ccu8_phu_module_ptr->CR2S = (uint32_t)(pwm_period+1);

        svpwm_ptr->ccu8_phv_module_ptr->CR1S = (uint32_t) half_t1nt2;
        svpwm_ptr->ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1nt2);

        svpwm_ptr->ccu8_phw_module_ptr->CR1S = (uint32_t) half_t2;
        svpwm_ptr->ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - half_t2);

        svpwm_ptr->sampling_critical_vector = svpwm_ptr->t2;
        break;

    case 3:                       // Sector D
        svpwm_ptr->ccu8_phu_module_ptr->CR1S = (uint32_t) 0;
        svpwm_ptr->ccu8_phu_module_ptr->CR2S = (uint32_t)(pwm_period+1);

        svpwm_ptr->ccu8_phv_module_ptr->CR1S = (uint32_t) half_t1;
        svpwm_ptr->ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1);

        svpwm_ptr->ccu8_phw_module_ptr->CR1S = (uint32_t) half_t1nt2;
        svpwm_ptr->ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1nt2);

        svpwm_ptr->sampling_critical_vector = svpwm_ptr->t1;
        break;

    case 4:                       // Sector E
        svpwm_ptr->ccu8_phu_module_ptr->CR1S = (uint32_t) half_t2;
        svpwm_ptr->ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - half_t2);

        svpwm_ptr->ccu8_phv_module_ptr->CR1S = (uint32_t) 0;
        svpwm_ptr->ccu8_phv_module_ptr->CR2S = (uint32_t)(pwm_period+1);

        svpwm_ptr->ccu8_phw_module_ptr->CR1S = (uint32_t) half_t1nt2;
        svpwm_ptr->ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1nt2);

        svpwm_ptr->sampling_critical_vector = svpwm_ptr->t2;
        break;

    case 5:                       // Sector F
        svpwm_ptr->ccu8_phu_module_ptr->CR1S = (uint32_t) half_t1nt2;
        svpwm_ptr->ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1nt2);

        svpwm_ptr->ccu8_phv_module_ptr->CR1S = (uint32_t) 0;
        svpwm_ptr->ccu8_phv_module_ptr->CR2S = (uint32_t)(pwm_period+1);

        svpwm_ptr->ccu8_phw_module_ptr->CR1S = (uint32_t) half_t1;
        svpwm_ptr->ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1);

        svpwm_ptr->sampling_critical_vector = svpwm_ptr->t1;
        break;

    default:
        // Control should not come here.
        break;
    }
}


#endif /* BMCLIB_SVPWM_H_ */
