/**
 * @file pmsm_foc_adc.c
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

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <PMSM_FOC/Configuration/pmsm_foc_mcuhw_params.h>
#include <PMSM_FOC/Configuration/pmsm_foc_config.h>
#include <PMSM_FOC/ControlModules/pmsm_foc_functions.h>
#include "../MCUInit/pmsm_foc_adc.h"

/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/
ADC_t ADC;

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/**
 *  Data Structure initialization - VADC global Configuration.
 */
XMC_VADC_GLOBAL_CONFIG_t VADC_GlobalConfig =
{
    .module_disable                  = 0U,
    .disable_sleep_mode_control      = 1U,
    .clock_config =
    {
        .analog_clock_divider          = 0U,       /*Divider Factor for the Analog Internal Clock*/
        .arbiter_clock_divider         = 0U,
        .msb_conversion_clock          = 0U,
    },
    .data_reduction_control          = 0U, /* Data Reduction disabled*/
    .wait_for_read_mode              = 0U, /* GLOBRES Register will not be overwrite until the previous value is read*/
    .event_gen_enable                = 0U, /* Result Event from GLOBRES is disabled*/

    .boundary0                       = 0U, /* Lower boundary value for Normal comparison mode*/
    .boundary1                       = 0U  /* Upper boundary value for Normal comparison mode*/
};

/**
 *  Data Structure initialization - VADC group 0 Configuration.
 */
XMC_VADC_GROUP_CONFIG_t VADC_Group0_Init =
{
    .emux_config =
    {
        .stce_usage                      = 0U,                             /*Use STCE when the setting changes*/
        .emux_mode                       = XMC_VADC_GROUP_EMUXMODE_SWCTRL, /* Mode for Emux conversion*/
        .emux_coding                     = XMC_VADC_GROUP_EMUXCODE_BINARY, /*Channel progression - binary format*/
        .starting_external_channel       = (uint32_t) 0U,                  /* Channel starts at 0 for EMUX*/
        .connected_channel               = (uint32_t) 0U                   /* Channel connected to EMUX*/
    },
    .class0 =
    {
    	  .sample_time_std_conv            = 0U,                          /*The Sample time is (2*tadci)*/
        .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
        .sampling_phase_emux_channel     = (uint32_t) 1U,               /*The Sample time is (3*tadci)*/
        .conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
    },  /* !<ICLASS-0 */
    .class1 =
    {
    	  .sample_time_std_conv            = 0U,                          /*The Sample time is (2*tadci)*/
        .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
        .sampling_phase_emux_channel     = (uint32_t) 1U,               /*The Sample time is (3*tadci)*/
        .conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
    }, /* !< ICLASS-1 */
    .boundary0                         = 0U,          /* Lower boundary value for Normal comparison mode*/
    .boundary1                         = 0U,          /* Upper boundary value for Normal comparison mode*/
    .arbitration_round_length          = 0U,        /* 4 arbitration slots per round selected (tarb = 4*tadcd) */
    .arbiter_mode                      = (uint32_t) XMC_VADC_GROUP_ARBMODE_ALWAYS, /*Determines when the arbiter should run.*/
};

/**
 *  Data Structure initialization - VADC group 1 Configuration.
 */
XMC_VADC_GROUP_CONFIG_t VADC_Group1_Init =
{
    .emux_config =
    {
        .stce_usage                      = 0U,                               /*Use STCE when the setting changes*/
        .emux_mode                       = XMC_VADC_GROUP_EMUXMODE_SWCTRL,   /* Mode for Emux conversion*/
        .emux_coding                     = XMC_VADC_GROUP_EMUXCODE_BINARY,   /*Channel progression - binary format*/
        .starting_external_channel       = (uint32_t) 0U,                   /* Channel starts at 0 for EMUX*/
        .connected_channel               = (uint32_t) 0U                    /* Channel connected to EMUX*/
    },
    .class0 =
    {
    	  .sample_time_std_conv            = 0U,                          /*The Sample time is (2*tadci)*/
        .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
        .sampling_phase_emux_channel     = (uint32_t) 0U,               /*The Sample time is (2*tadci)*/
        .conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
    },  /* !<ICLASS-0 */
    .class1 =
    {
    	  .sample_time_std_conv            = 0U,                          /*The Sample time is (2*tadci)*/
        .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
        .sampling_phase_emux_channel     = (uint32_t) 0U,               /*The Sample time is (2*tadci)*/
        .conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
    }, /* !< ICLASS-1 */
    .boundary0                         = 0U,  /* Lower boundary value for Normal comparison mode*/
    .boundary1                         = 0U,  /* Upper boundary value for Normal comparison mode*/
    .arbitration_round_length          = (uint32_t) 0U,  /* 4 arbitration slots per round selected (tarb = 4*tadcd) */
    .arbiter_mode                      = (uint32_t) XMC_VADC_GROUP_ARBMODE_ALWAYS, /*Determines when the arbiter should run.*/
};

/**
 *  Data Structure initialization - VADC group scan request source.
 */

XMC_VADC_BACKGROUND_CONFIG_t VADC_Group_ScanConfig =
{
    .conv_start_mode  = (uint8_t) XMC_VADC_STARTMODE_WFS,
    .req_src_priority = (uint8_t) XMC_VADC_GROUP_RS_PRIORITY_0,
    .trigger_signal   = (uint8_t) XMC_VADC_REQ_TR_P,            /*If trigger needed the signal input*/
    .trigger_edge     = (uint8_t) XMC_VADC_TRIGGER_EDGE_NONE,   /*Trigger edge needed if trigger enabled*/
    .gate_signal      = (uint32_t) XMC_VADC_REQ_GT_P,            /*If gating needed the signal input*/
    .timer_mode       = (uint32_t) 0,                            /* Disabled equidistant sampling*/
    .external_trigger = (uint32_t) 0,
    .load_mode        = (uint32_t) XMC_VADC_SCAN_LOAD_COMBINE,   /*Response from SCAN when a Load event occours.*/
};


/* Potentiometer ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_Pot_Init =
{
    .alias_channel       = -1,
    .result_reg_number   = VADC_POT_RESULT_REG,
    .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority    = 0,
    .sync_conversion     = false
};

/* DC voltage ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_VDC_Init =
{
    .alias_channel         = -1,
    .result_reg_number     = VADC_VDC_RESULT_REG,
    .alternate_reference   = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority      = 0,
    .sync_conversion       = false,
    #if(USER_VDC_UV_OV_PROTECTION_ENABLE == ENABLED)
    .lower_boundary_select = XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
    .upper_boundary_select = XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND1,
    .event_gen_criteria    = XMC_VADC_CHANNEL_EVGEN_OUTBOUND,
    #endif
};

/* Board temperature sensing VADC configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_TEMP_Init =
{
    .alias_channel         = -1,
    .result_reg_number     = VADC_TEMP_RESULT_REG,
    .alternate_reference   = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority      = 0,
    .sync_conversion       = false,
};

#if(E_BIKE_REF == ENABLED)
/* T_Sense Temp sensor(Motor) - Board temperature sensing VADC configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_TEMP_MOTOR_Init =
{
    .alias_channel         = -1,
    .result_reg_number     = VADC_TEMP_MOTOR_RESULT_REG,
    .alternate_reference   = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority      = 0,
    .sync_conversion       = false,
};

/* E_BIKE BRAKE ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_E_BIKE_BRAKE_Init =
{
    .alias_channel       = -1,
    .result_reg_number   = VADC_E_BIKE_BRAKE_RESULT_REG,
    .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority    = 0,
    .sync_conversion     = false
};
#endif

/* E-clutch ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_ECLUTCH_Init =
{
    .alias_channel         = -1,
    .result_reg_number     = VADC_I4_RESULT_REG,
    .alternate_reference   = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority      = 0,
    .sync_conversion       = true,
};

#if(PMSM_FOC_HARDWARE_KIT == LEV_4KW_KIT)
/* DC current ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_channel_idc_init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_IDC_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false
};

/* VREF voltage ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_channel_vref_init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_VREF_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false,

};
#endif

#if(MCUTYPE == MCU_IMD701)
/* BEMF-U ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_BEMFU_Init =
{
    .alias_channel        = -1,
    .result_reg_number    = VADC_BEMF_U_RESULT_REG,
    .alternate_reference  = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority     = 0,
    .sync_conversion      = false,
};

/* BEMF-V ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_BEMFV_Init =
{
    .alias_channel = -1,
    .result_reg_number = VADC_BEMF_V_RESULT_REG,
    .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority = 0,
    .sync_conversion = false,
};

/* BEMF-W ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_BEMFW_Init =
{
    .alias_channel = -1,
    .result_reg_number = VADC_BEMF_W_RESULT_REG,
    .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority = 0,
    .sync_conversion = false,
};
#endif

/**
 *  Data Structure initialization - VADC group queue request source.
 */
XMC_VADC_QUEUE_CONFIG_t VADC_Group_QueueConfig =
{
    .conv_start_mode  = (uint8_t) XMC_VADC_STARTMODE_WFS,       /* Conversion start mode WFS/CIR/CNR*/
    .req_src_priority = (uint8_t) XMC_VADC_GROUP_RS_PRIORITY_1, /*The queue request source priority */
    .trigger_signal   = (uint8_t) XMC_VADC_REQ_TR_P,            /* gate input as a trigger */
    .trigger_edge     = (uint8_t) XMC_VADC_TRIGGER_EDGE_RISING,  /*Trigger edge needed if trigger enabled*/
    .gate_signal      = (uint32_t) XMC_VADC_REQ_GT_F,            /*If gating needed the signal input - CCU80.ST3*/
    .timer_mode       = (uint32_t) 0,                            /* Disabled equidistant sampling*/
    .external_trigger = (uint32_t) 1                             /*External trigger Enabled/Disabled*/
};

#if (USER_CURRENT_SENSING ==  THREE_SHUNT_SYNC_CONV)
/**
 *  Data Structure initialization - VADC group queue entries.
 */
XMC_VADC_QUEUE_ENTRY_t VADC_Group1_QueueEntry_AliasCH0 =
{
    .channel_num        = VADC_I1_CHANNEL,
    .external_trigger   = true,
    .generate_interrupt = false,
    .refill_needed      = true
};

XMC_VADC_QUEUE_ENTRY_t VADC_Group1_QueueEntry_AliasCH1 =
{
    .channel_num        = VADC_I3_CHANNEL,
    .external_trigger   = false,
    .generate_interrupt = false,
    .refill_needed      = true
};

/**
 *  Data Structure initialization - VADC group channels.
 */
XMC_VADC_CHANNEL_CONFIG_t VADC_G1_CH0_Init =
{
    .alias_channel       = (int8_t)VADC_G1_CHANNEL_ALIAS0,
    .result_reg_number   = 0,
    .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority    = 0,
    .sync_conversion     = true
};

XMC_VADC_CHANNEL_CONFIG_t VADC_G1_CH1_Init =
{
    .alias_channel       = (int8_t)VADC_G1_CHANNEL_ALIAS1,
    .result_reg_number   = 1,
    .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority    = 0,
    .sync_conversion     = true
};

XMC_VADC_CHANNEL_CONFIG_t VADC_G0_CH0_Init =
{
    .alias_channel       = (int8_t)VADC_G0_CHANNEL_ALIAS0,
    .result_reg_number   = 0,
    .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority    = 0,
    .sync_conversion     = true
};

XMC_VADC_CHANNEL_CONFIG_t VADC_G0_CH1_Init =
{
    .alias_channel       = (int8_t)VADC_G0_CHANNEL_ALIAS1,
    .result_reg_number   = 1,
    .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority    = 0,
    .sync_conversion     = true
};

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/* API to initialize VADC module for 3-shunt phase current sensing */
void PMSM_FOC_VADC_PhCurrentInit(void)
{
    /* Configuration of VADC_G1 - Q source */
    /* External trigger 1,  refill 1 */
    XMC_VADC_GROUP_QueueInit(VADC_G1, &VADC_Group_QueueConfig);

    XMC_VADC_GROUP_QueueFlushEntries(VADC_G1);
    XMC_VADC_GROUP_QueueFlushEntries(VADC_G0);

     /* Configure the gating mode for queue*/
    XMC_VADC_GROUP_QueueSetGatingMode(VADC_G1, XMC_VADC_GATEMODE_IGNORE);

    /* Request the LLD to insert the channel */
    XMC_VADC_GROUP_QueueInsertChannel(VADC_G1, VADC_Group1_QueueEntry_AliasCH0);

    XMC_VADC_GROUP_QueueInsertChannel(VADC_G1, VADC_Group1_QueueEntry_AliasCH1);

    /* Master group - G1 for Synchronous ADC */
    /* I1, Result Register RES0 */
    XMC_VADC_GROUP_ChannelInit(VADC_I1_GROUP, VADC_I1_CHANNEL, &VADC_G1_CH0_Init);

    /* I3, Result Register RES1 */
    XMC_VADC_GROUP_ChannelInit(VADC_I3_GROUP, VADC_I3_CHANNEL, &VADC_G1_CH1_Init);

    /* I2, Result Register RES0 */
    XMC_VADC_GROUP_ChannelInit(VADC_I2_GROUP, VADC_I2_CHANNEL, &VADC_G0_CH0_Init);

    /* G0CH1, Result Register RES1 */
    XMC_VADC_GROUP_ChannelInit(VADC_I4_GROUP, VADC_I4_CHANNEL, &VADC_G0_CH1_Init);

    /* G0: synchronization slave */
    XMC_VADC_GROUP_SetSyncSlave(VADC_G0, 1U, 0U);

    /* Ready input R1 is considered */
    XMC_VADC_GROUP_CheckSlaveReadiness(VADC_G0, 0U);

    /* G1: synchronization master */
    XMC_VADC_GROUP_SetSyncMaster(VADC_G1);
}

#elif (USER_CURRENT_SENSING ==  THREE_SHUNT_ASYNC_CONV)
XMC_VADC_QUEUE_ENTRY_t VADC_QueueEntry_Iu =
{
    .channel_num        = VADC_IU_CHANNEL,
    .external_trigger   = true,
    .generate_interrupt = false,
    .refill_needed      = true
};

XMC_VADC_QUEUE_ENTRY_t VADC_QueueEntry_Iv =
{
    .channel_num        = VADC_IV_CHANNEL,
    .external_trigger   = false,
    .generate_interrupt = false,
    .refill_needed      = true
};

XMC_VADC_QUEUE_ENTRY_t VADC_QueueEntry_Iw =
{
    .channel_num        = VADC_IW_CHANNEL,
    .external_trigger   = false,
    .generate_interrupt = false,
    .refill_needed      = true
};

/**
 *  Data Structure initialization - motor Phases VADC channels.
 */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_Iu_Init =
{
    .alias_channel       = -1,
    .result_reg_number   = VADC_IU_RESULT_REG,
    .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority    = 0,
    .sync_conversion     = false
};

XMC_VADC_CHANNEL_CONFIG_t VADC_CH_Iv_Init =
{
    .alias_channel       = -1,
    .result_reg_number   = VADC_IV_RESULT_REG,
    .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority    = 0,
    .sync_conversion     = false
};

XMC_VADC_CHANNEL_CONFIG_t VADC_CH_Iw_Init =
{
    .alias_channel       = -1,
    .result_reg_number   = VADC_IW_RESULT_REG,
    .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority    = 0,
    .sync_conversion     = false
};

void PMSM_FOC_VADC_PhCurrentInit(void)
{
    /* Configuration of VADC_G1 - Q source */
    /* External trigger 1,  refill 1 */
    XMC_VADC_GROUP_QueueInit(VADC_G0, &VADC_Group_QueueConfig);
    XMC_VADC_GROUP_QueueInit(VADC_G1, &VADC_Group_QueueConfig);

    XMC_VADC_GROUP_QueueFlushEntries(VADC_G0);
    XMC_VADC_GROUP_QueueFlushEntries(VADC_G1);

     /* Configure the gating mode for queue*/
    XMC_VADC_GROUP_QueueSetGatingMode(VADC_G0, XMC_VADC_GATEMODE_IGNORE);
    XMC_VADC_GROUP_QueueSetGatingMode(VADC_G1, XMC_VADC_GATEMODE_IGNORE);

    /* Channel initialization - Result Register / Gain factor setting  */
    XMC_VADC_GROUP_ChannelInit(VADC_IU_GROUP, VADC_IU_CHANNEL, &VADC_CH_Iu_Init);
    XMC_VADC_GROUP_ChannelInit(VADC_IV_GROUP, VADC_IV_CHANNEL, &VADC_CH_Iv_Init);
    XMC_VADC_GROUP_ChannelInit(VADC_IW_GROUP, VADC_IW_CHANNEL, &VADC_CH_Iw_Init);

    /* Request the LLD to insert the channel */
    XMC_VADC_GROUP_QueueInsertChannel(VADC_IU_GROUP, VADC_QueueEntry_Iu);
    XMC_VADC_GROUP_QueueInsertChannel(VADC_IV_GROUP, VADC_QueueEntry_Iv);
    XMC_VADC_GROUP_QueueInsertChannel(VADC_IW_GROUP, VADC_QueueEntry_Iw);
}
#endif

void PMSM_FOC_VADC_ModuleInit(void)
{
    #if(INTERNAL_OP_GAIN == ENABLED)
    uint32_t gain_factor;
    #endif

    XMC_VADC_GLOBAL_Init(VADC, &VADC_GlobalConfig);

    /* Short sample time(SST) set to 5, sample time = SST * tadc (Ex: fADC = 48 MHz, i.e. tADC = 20.84 ns, sample time = 20.84 ns */
    XMC_VADC_GLOBAL_SHS_SetShortSampleTime(SHS0,XMC_VADC_GROUP_INDEX_0,5);
    XMC_VADC_GLOBAL_SHS_SetShortSampleTime(SHS0,XMC_VADC_GROUP_INDEX_1,5);

    XMC_VADC_GROUP_Init(VADC_G0, &VADC_Group0_Init);
    XMC_VADC_GROUP_Init(VADC_G1, &VADC_Group1_Init);

    /* Configuration of VADC_G1/0,  Turn on ADC modules */
    XMC_VADC_GROUP_SetPowerMode(VADC_G0, XMC_VADC_GROUP_POWERMODE_NORMAL);
    XMC_VADC_GROUP_SetPowerMode(VADC_G1, XMC_VADC_GROUP_POWERMODE_NORMAL);

    #if(USER_ADC_CALIBRATION == ENABLED)
    #if (UC_SERIES == XMC13)
    /* XMC13 AA STEP micro-controllers as per ADC_AI.004 errata.*/
    PMSM_FOC_VADC_StartupCalib();
    #else
    /* Trigger Start-up Calibration - Takes around 40uSec */
    /*  */
    XMC_VADC_GLOBAL_StartupCalibration(VADC);
    #endif
    #endif

    XMC_VADC_GROUP_ScanInit(VADC_G0, &VADC_Group_ScanConfig);
    /* Configure the gating mode for scan*/
    XMC_VADC_GROUP_ScanSetGatingMode(VADC_G0,XMC_VADC_GATEMODE_IGNORE);

    XMC_VADC_GROUP_ScanInit(VADC_G1, &VADC_Group_ScanConfig);
    /* Configure the gating mode for scan*/
    XMC_VADC_GROUP_ScanSetGatingMode(VADC_G1,XMC_VADC_GATEMODE_IGNORE);
}

/* This function initializes the synchronous conversion for the three phase currents */
#if(USER_CURRENT_SENSING == THREE_SHUNT_SYNC_CONV)
void PMSM_FOC_VADC_PhCurrentSyncConfig(uint16_t svpwm_sector_num)
{
    /* Rotating ADC alias */
    switch (svpwm_sector_num)
    {
        case 0:
        case 5:
            /* Sectors A and F. ADC sequences - Iw -> Iv -> Iu */
            VADC_G1->ALIAS = (((uint32_t) VADC_IU_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G1_CHANNEL);
            VADC_G0->ALIAS = (((uint32_t) VADC_G0_CHANNEL_ALIAS1 << VADC_G_ALIAS_ALIAS1_Pos) |VADC_IW_G0_CHANNEL);
            break;

        case 1:
        case 2:
            /*  Sectors B and C. ADC sequences - Iw -> Iu -> Iv */
            VADC_G1->ALIAS = (((uint32_t) VADC_IV_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G1_CHANNEL);
            VADC_G0->ALIAS = (((uint32_t) VADC_G0_CHANNEL_ALIAS1 << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G0_CHANNEL);

            #if (USER_MOTOR_BI_DIRECTION_CTRL == ENABLED )
            /* May need swap SVPWM CCU8 duty cycles of phase V and W, so that motor reverses */
            if (PMSM_FOC_CTRL.rotation_dir == DIRECTION_DEC)
            {
              /* ADC sequences - Iu -> Iv -> Iw */
              VADC_G1->ALIAS = (((uint32_t) VADC_IW_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G1_CHANNEL);
              VADC_G0->ALIAS = (((uint32_t) VADC_G0_CHANNEL_ALIAS1 << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL);
            }
            #endif
            break;

        default:
            /* Process for all other cases, Sectors D and E. ADC sequences - Iu -> Iv -> Iw */
            VADC_G1->ALIAS = (((uint32_t) VADC_IW_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G1_CHANNEL);
            VADC_G0->ALIAS = (((uint32_t) VADC_G0_CHANNEL_ALIAS1 << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G0_CHANNEL);

            #if (USER_MOTOR_BI_DIRECTION_CTRL == ENABLED )
            /* May need swap SVPWM CCU8 duty cycles of phase V and W, so that motor reverses */
            if (PMSM_FOC_CTRL.rotation_dir == DIRECTION_DEC)
            {
              /* ADC sequences - Iu -> Iw -> Iv */
              VADC_G1->ALIAS = (((uint32_t) VADC_IV_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G1_CHANNEL);
              VADC_G0->ALIAS = (((uint32_t) VADC_G0_CHANNEL_ALIAS1 << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IW_G0_CHANNEL);
            }
            #endif
            break;
    }
}
#endif //End Of #if(USER_CURRENT_SENSING == THREE_SHUNT_SYNC_CONV)


/* API to initialize VADC channel for potentiometer voltage sensing that is used to set user speed */
void PMSM_FOC_VADC_PotInit(void)
{
    /* Initializes the POT VADC channel for conversion */
    XMC_VADC_GROUP_ChannelInit(VADC_POT_GROUP, VADC_POT_CHANNEL, &VADC_CH_Pot_Init);
    XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_POT_GROUP,VADC_POT_CHANNEL);
}

/* API to initialize VADC channel for DC link voltage sensing */
void PMSM_FOC_VADC_VDCInit(void)
{
    /* Initializes the DC Link VADC channel for conversion */
    XMC_VADC_GROUP_ChannelInit(VADC_VDC_GROUP, VADC_VDC_CHANNEL, &VADC_CH_VDC_Init);
    XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_VDC_GROUP,VADC_VDC_CHANNEL);
}

#if(MCUTYPE == MCU_IMD701)
/* API to initialize VADC channel for BEMF sensing */
void PMSM_FOC_VADC_BEMF_Init(void)
{
    /* Initializes the BEMF VADC channel for conversion */
    XMC_VADC_GROUP_ChannelInit(VADC_BEMF_U_GROUP, VADC_BEMF_U_CHANNEL, &VADC_CH_BEMFU_Init);
    XMC_VADC_GROUP_ChannelInit(VADC_BEMF_V_GROUP, VADC_BEMF_V_CHANNEL, &VADC_CH_BEMFV_Init);
    XMC_VADC_GROUP_ChannelInit(VADC_BEMF_W_GROUP, VADC_BEMF_W_CHANNEL, &VADC_CH_BEMFW_Init);

    XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_BEMF_U_GROUP,VADC_BEMF_U_CHANNEL);
    XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_BEMF_V_GROUP,VADC_BEMF_V_CHANNEL);
    XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_BEMF_W_GROUP,VADC_BEMF_W_CHANNEL);
}

void PMSM_FOC_VADC_BEMF_Disable(void)
{
    /* Remove BEMF sensing ADC channel from VADC scan request source */
    XMC_VADC_GROUP_ScanRemoveChannel(VADC_BEMF_U_GROUP,VADC_BEMF_U_CHANNEL);
    XMC_VADC_GROUP_ScanRemoveChannel(VADC_BEMF_V_GROUP,VADC_BEMF_V_CHANNEL);
    XMC_VADC_GROUP_ScanRemoveChannel(VADC_BEMF_W_GROUP,VADC_BEMF_W_CHANNEL);
}
#endif

/* API to initialize VADC channel for board temperature sensing */
void PMSM_FOC_VADC_BoardTempSensor_Init(void)
{
    XMC_VADC_GROUP_ChannelInit(VADC_TEMP_GROUP, VADC_TEMP_CHANNEL, &VADC_CH_TEMP_Init);
    XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, SHS_GAIN_FACTOR_1,(XMC_VADC_GROUP_INDEX_t)VADC_TEMP_GROUP_NO, VADC_TEMP_CHANNEL);
    XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_TEMP_GROUP,VADC_TEMP_CHANNEL);
}

#if(E_BIKE_REF == ENABLED)
void PMSM_FOC_VADC_MOTORTempSensor_Init(void)
{
    XMC_VADC_GROUP_ChannelInit(VADC_TEMP_MOTOR_GROUP, VADC_TEMP_MOTOR_CHANNEL, &VADC_CH_TEMP_MOTOR_Init);
    XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, SHS_GAIN_FACTOR_1,(XMC_VADC_GROUP_INDEX_t)VADC_TEMP_MOTOR_GROUP_NO, VADC_TEMP_MOTOR_CHANNEL);
    XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_TEMP_MOTOR_GROUP,VADC_TEMP_MOTOR_CHANNEL);
}

void PMSM_FOC_VADC_E_BIKE_BRAKE_Init(void)
{
    XMC_VADC_GROUP_ChannelInit(VADC_E_BIKE_BRAKE_GROUP, VADC_E_BIKE_BRAKE_CHANNEL, &VADC_CH_E_BIKE_BRAKE_Init);
    XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_E_BIKE_BRAKE_GROUP,VADC_E_BIKE_BRAKE_CHANNEL);
}
#endif

#if(PMSM_FOC_HARDWARE_KIT == LEV_4KW_KIT)
/* API to initialize VADC channel for IDClink */
void pmsm_adc_idc_init(void)
{
  /* Initializes the POT VADC channel for conversion */
  XMC_VADC_GROUP_ChannelInit(VADC_IDC_GROUP, VADC_IDC_CHANNEL, &VADC_channel_idc_init);

  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, SHS_GAIN_FACTOR_1,VADC_IDC_GROUP_NO, VADC_IDC_CHANNEL);
  XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC,VADC_IDC_GROUP_NO,VADC_IDC_CHANNEL);
  //XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_IDC_GROUP,VADC_IDC_CHANNEL);

}


/* API to initialize VADC channel for 1.65V VREF voltage */
void pmsm_adc_vref_init(void)
{
  /* Initializes the VREF VADC channel for conversion */
  XMC_VADC_GROUP_ChannelInit(VADC_VREF_GROUP, VADC_VREF_CHANNEL, &VADC_channel_vref_init);
  XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC,VADC_VREF_GROUP_NO,VADC_VREF_CHANNEL);
}
#endif

/*
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Turn on Group 0 converter to calibrates VADC by triggering dummy conversions.
 * This startup calibration is required only for XMC13 AA STEP micro-controllers as per ADC_AI.004 errata.
 */
void PMSM_FOC_VADC_StartupCalib(void)
{
   volatile uint32_t dealy_counter; /* Delay counter for the startup calibration to complete. */
   *SHS0_CALOC0 = REG_RESET;
   *SHS0_CALOC1 = REG_RESET;

   #if(VADC_ENABLE_GROUP_QUEUE_0 != 1U)
   /* Enable Group 0 for Calibration */
   XMC_VADC_GROUP_SetPowerMode(VADC_G0,XMC_VADC_GROUP_POWERMODE_NORMAL);
   #endif

   /* Enable the StartUp calibration in the VADC */
   VADC->GLOBCFG |= (((uint32_t)1 << (uint32_t)VADC_GLOBCFG_SUCAL_Pos) & (uint32_t)VADC_GLOBCFG_SUCAL_Msk) |
                    (((uint32_t)1 << (uint32_t)VADC_GLOBCFG_DPCAL0_Pos) & (uint32_t)VADC_GLOBCFG_DPCAL0_Msk);

   /* Wait for 1920cycles or 60us for the startup calibration to complete. */
   dealy_counter = VADC_CALIBRATION_CYCLE_DELAY;

   while (dealy_counter > 0U)
   {
       dealy_counter--;
       /* Clear offset calibration values */
       CLEAR_OFFSET_CALIB_VALUES;
   }
   PMSM_FOC_VADC_GainCalib();

   /* Switch off Group 0 converter if it is not used for any conversions */
   #if(VADC_ENABLE_GROUP_QUEUE_0 != 1U)
   XMC_VADC_GROUP_SetPowerMode(VADC_G0,XMC_VADC_GROUP_POWERMODE_OFF);
   #endif
}



/* This API is used for VADC gain calibration. */
/*
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Calibrates VADC by triggering dummy conversion for XMC13 AA STEP micro-controllers as per ADC_AI.004 errata.
 */
void PMSM_FOC_VADC_GainCalib(void)
{
    volatile uint16_t dummy_conv_counter = VADC_DUMMY_CONVERSION_COUNTER;  /* Used for keeping track for dummy conversions */
    volatile uint32_t adc_result_aux;                                      /* Used for reading ADC values after dummy conversions */

    /* ADC_AI.004 errata*/
    *SHS0_CALCTR = 0X3F100400U;

    /* add a channel in group-0 for dummy conversion*/
    VADC->BRSSEL[0] = VADC_BRSSEL_CHSELG0_Msk;

    /*Clear the DPCAL0, DPCAL1 and SUCAL bits*/
    VADC->GLOBCFG &= ~( (uint32_t)VADC_GLOBCFG_DPCAL0_Msk | (uint32_t)VADC_GLOBCFG_DPCAL1_Msk | (uint32_t)VADC_GLOBCFG_SUCAL_Msk);

    /* Clear offset calibration values*/
    CLEAR_OFFSET_CALIB_VALUES;

    VADC->BRSMR = ((uint32_t)1 << (uint32_t)VADC_BRSMR_ENGT_Pos);
    #if UC_SERIES != XMC11
    VADC_G0->ARBPR = (VADC_G_ARBPR_ASEN2_Msk);
    #endif
    /*Trigger dummy conversion for 9* 2000 times*/

    while (dummy_conv_counter > 0U)
    {
        /*load event */
        VADC->BRSMR |= (uint32_t)VADC_BRSMR_LDEV_Msk;
        #if (UC_SERIES != XMC11)
        /*Wait until a new result is available*/
        while (VADC_G0->VFR == 0U)
        {

        }

        /*dummy read of result */
        adc_result_aux = VADC_G0->RES[0];
        #else
        /*Wait untill a new result is available*/
        while ((VADC->GLOBRES & VADC_GLOBRES_VF_Msk) == 0)
        {

        }

        /*dummy read of result */
        adc_result_aux = VADC->GLOBRES;
        #endif

        /* Clear offset calibration values*/
        CLEAR_OFFSET_CALIB_VALUES;
        dummy_conv_counter--;
    }

    /* To avoid a warning*/
    adc_result_aux &= 0U;

    /* Wait until last gain calibration step is finished */
    while ( (SHS0->SHSCFG & (uint32_t)SHS_SHSCFG_STATE_Msk) != 0U )
    {
        /* Clear offset calibration values*/
        CLEAR_OFFSET_CALIB_VALUES;
    }
    /* Re enable SUCAL DPCAL */
    VADC->GLOBCFG |= ( (uint32_t)VADC_GLOBCFG_DPCAL0_Msk | (uint32_t)VADC_GLOBCFG_DPCAL1_Msk);
    VADC->BRSMR     = (uint32_t)0x00;
    VADC->BRSSEL[0] = (uint32_t)0x00;
    #if (UC_SERIES != XMC11)
    VADC_G0->REFCLR = 1U;
    VADC_G0->ARBPR &= ~((uint32_t)VADC_G_ARBPR_ASEN2_Msk);
    #endif
}


