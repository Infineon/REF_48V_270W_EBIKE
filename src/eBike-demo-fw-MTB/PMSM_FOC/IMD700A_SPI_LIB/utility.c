/**
    @file: utility.c

    Implementation of utility
*/

/* ===========================================================================
** Copyright (C) 2021-2023 Infineon Technologies AG
** All rights reserved.
** ===========================================================================
**
** ===========================================================================
** This document contains proprietary information of Infineon Technologies AG.
** Passing on and copying of this document, and communication of its contents
** is not permitted without Infineon's prior written authorisation.
** ===========================================================================
*/   

#include <PMSM_FOC/Configuration/pmsm_foc_config.h>
#include <PMSM_FOC/Configuration/pmsm_foc_mcuhw_params.h>
#include "6EDL7141_register.h"
#include "utility.h"
#include "SPI.h"
#include "xmc_usic.h"
#include "xmc_spi.h"

 /*****************************************************************************/

// Define Global Variables


//extern Global Variables
extern uint8_t SPI_Data[3];
extern uint8_t SPIRxData[3];


/*****************************************************************************/
// Function Definitions

// SUPPLY_CFG Register
SUPPLY_CFG_REG_t SUPPLY_Config =
{
    .PVCC_SETPT = USER_GD_PVCC_SETPT,                /* gate driving voltage, 0 - 12V, 1 - 15V, 2 - 10V, 3 - 7V */
    .CS_REF_CFG = VREF_DVDD_1_2,            /* Current sense reference, 0 - DVDD/2 */
    .DVDD_OCP_CFG = OCP_THR_450,          /* DVDD OCP threshold configuration, 0x00 -450mA */
    .DVDD_SFTSTRT = DVDD_SFTSTRT_100us,          /* DVDD soft start, 0x000 - 100uS, 0x001 - 200uS */
    #if(CONTROLLER_DEVICE_ID == IMD700A_DVDD_3_3_V)
    .DVDD_SETPT = DVDD_3_3,            /* DVDD set point, 0,1 - VSENSE pin, 2 - 3.3V, 3 - 5V digitally programmed*/
    #else
    .DVDD_SETPT = DVDD_5,            /* DVDD set point, 0,1 - VSENSE pin, 2 - 3.3V, 3 - 5V digitally programmed*/
    #endif
    .BK_FREQ = BK_FREQ_500K,               /* Buck converter switching frequency, 0 - Low Freq(500KHz), 1 - High(2 MHz) */
    .DVDD_TON_DELAY = DVDD_TON_DELAY_400us,        /* DVDD turn on delay configuration - 0x00 -200uS, 0x01-400uS, 0x02-600uS, 0x03-800uS */
    .CP_PRECHARGE_EN = CP_PRECHAR_EN,       /* Charge pump pre-charge, 0x0 - Disable, 0x01- enable */
};

// ADC_CFG Register
ADC_CFG_REG_t ADC_Config =
{
    .ADC_OD_REQ = NO_ADC_REQ,
    .ADC_OD_INSEL = ADC_IN_IDIGITAL,
    .ADC_EN_FILT = ADC_DIS_FILTER,
    .ADC_FILT_CFG = ADC_FILT_SAMP_8,
    .ADC_FILT_CFG_P = ADC_FILT_PSAMP_32,
};

// PWM_CFG Register
PWM_CFG_REG_t PWM_Config =
{
    .PWM_MODE = 0x00,                /* 0x00 - 6PWM, 0x01 - 3PWM, 0x02 - 1 PWM .. */
    .PWM_FREEW_CFG = 0x01,           /* PWM Freewheeling,  0x00 - Active Freewheeling, 0x01 - Diode Freewheeling */
    .BRAKE_CFG = 0x00,               /* Brake Config -0x00 - Low side, 0x01 - High Side, 0x02 - High Z(no power), 0x03 - ALternate low & High */
    .PWM_RECIRC = 0x00,              /* PWM recirculation */
};

// SENSOR_CFG Register
SENSOR_CFG_REG_t SENSOR_Config =
{
    .HALL_DEGLITCH = HALL_DEGLITCH_0ns,           /* Hall sensor deglitch config */
    .OTS_DIS = OTEMP_PROT_EN,                /* Over temperature shutdown enabled - 0, disable - 1*/
    .CS_TMODE = CS_ACTIVE_GLx_H                /* Current sense amplifier timing mode,
                                          0x00 - Amplifier o/p active when GLx signal is high
                                          0x01 - Amplifier o/p active when GHx signal is low
                                          0x02 - Amplifier o/p is always active */
};

// WD_CFG Register
WD_CFG_REG_t WD_Config =
{
    .BIT_WD_EN = WD_DIS,                 /* Watchdog disabled - 0, enabled - 1 */
    .WD_INSEL = WDIN_DRV,
    .WD_FLTCFG = WDOUT_STATUS,
    .WD_TIMER_VAL = WD_PERIOD_100us,
};

// WD_CFG2 Register
WD_CFG2_REG_t WD_Config2 =
{
    .WD_BRAKE = WD_REACT_NFAULT,
    .WD_EN_LATCH = WD_FLT_LATCH_DIS,
    .WD_DVDD_RSTRT_ATT = RSTRT_ATT0,
    .WD_DVDD_RSTRT_DLY = DVDD_RSTRT_DLY_0ms5,
    .WD_RLOCK_EN = RLOCK_DIS,
    .WD_RLOCK_T = RLOCK_WD_TMOUT_1s,
    .WD_BK_DIS = BUCK_EN,
};

IDRIVE_CFG_REG_t IDRIVE_Config =
{
#if (MCUTYPE == MCU_IMD_1_5kw_ref)
		/*For 1.5kw ref board*/
    .IHS_SRC = 0x0B,              /* High side gate driver pull-up current,0x0B - 200mA */
    .IHS_SINK = 0x0B,             /* High side gate driver pull-down current,0x0B - 200mA */
    .ILS_SRC = 0x0C,              /* Low side gate driver pull-up current,0x0B - 200mA */
    .ILS_SINK = 0x0B,             /* Low side gate driver pull-down current,0x0B - 200mA */
#else
    .IHS_SRC = USER_GD_IHS_SRC,              /* High side gate driver pull-up current */
    .IHS_SINK = USER_GD_IHS_SINK,             /* High side gate driver pull-down current */
    .ILS_SRC = USER_GD_ILS_SRC,              /* Low side gate driver pull-up current */
    .ILS_SINK = USER_GD_ILS_SINK             /* Low side gate driver pull-down current */
#endif
};

// IDRIVE_PRE_CFG Register
IDRIVE_PRE_CFG_REG_t IDRIVE_PRE_Config =
{
#if (MCUTYPE == MCU_IMD_1_5kw_ref)
	.I_PRE_SRC = 0x0B,           /* Pre charge pull up current setting  ,0x0B - 200mA */
	.I_PRE_SINK = 0x0F,          /* Pre charge pull down current setting  ,0x0F - 500mA */
	.I_PRE_EN = 0x00,            /* Pre charge mode 0x00 - Enabled, 0x1 - Disabled */
#else
    .I_PRE_SRC = USER_GD_IPRE_SRC,           /* Pre charge pull up current setting */
    .I_PRE_SINK = USER_GD_IPRE_SINK,          /* Pre charge pull down current setting */
    .I_PRE_EN = USER_GD_I_PRE_EN,            /* Pre charge mode 0x00 - Enabled, 0x1 - Disabled */
#endif
};

// TDRIVE_SRC_CFG Register
TDRIVE_SRC_CFG_REG_t TDRIVE_SRC_Config =
{
#if (MCUTYPE == MCU_IMD_1_5kw_ref)
	.TDRIVE1 = 0x01,           /* 0x0 - 0nS, 0x01-50nS, 0x02-60nS...0xFF-2590nS */
	.TDRIVE2 = 0xF,           /* 0x0 - 0nS, 0x01-50nS, 0x02-60nS...0xFF-2590nS */
#else
    .TDRIVE1 = USER_GD_TDRIVE1,           /* 0x0 - 0nS, 0x01-50nS, 0x02-60nS...0xFF-2590nS */
    .TDRIVE2 = USER_GD_TDRIVE2
#endif
};

// TDRIVE_SINK_CFG Register
TDRIVE_SINK_CFG_REG_t TDRIVE_SINK_Config =
{
#if (MCUTYPE == MCU_IMD_1_5kw_ref)
	.TDRIVE3 = 0x19,           /* 0x0 - 0nS, 0x01-50nS, 0x02-60nS, 0x03-70nS...0xFF-2590nS */
	.TDRIVE4 = 0x19,           /* 0x0 - 0nS, 0x01-50nS, 0x02-60nS...0xFF-2590nS */
#else
    .TDRIVE3 = USER_GD_TDRIVE3,           /* 0x0 - 0nS, 0x01-50nS, 0x02-60nS, 0x03-70nS...0xFF-2590nS */
    .TDRIVE4 = USER_GD_TDRIVE4
#endif
};

// DT_CFG Register
DT_CFG_REG_t DT_Config =
{
    .DT_RISE = USER_GD_DT_RISE,            /* Dead time rise(Low to high),  */
    .DT_FALL = USER_GD_DT_FALL,            /* Dead time fall(high to low),  */
};

// Charge Pump Configuration Register
CP_CFG_REG_t CP_Config =
{
    .CP_CLK_CFG = CP_CLK_195_3125kHz,       /* Charge pump clock freq. 0x0-781.25KHz,..0x02-1.5625MHz */
    .BIT_CP_CLK_SS_DIS = CP_CLK_SS_EN,    /* Charge pump clock spread spectrum enabled */
};

// CSAMP_CFG Register
CSAMP_CFG_REG_t CSAMP_Config =
{
    .CS_GAIN = USER_GD_CSOPAMP_GAIN,  /* Current Amplifier Gain */
    .CS_GAIN_ANA = CS_GAIN_PROG_DIG,      /* analog programming disabled */
    .CS_EN = CS_A_EN_B_EN_C_EN,            /* Amplifier enable/disable, 0x0->All disabled, 0x07-> All enabled */
    .CS_BLANK = CS_BLANK_0ns,         /* Current sense blanking time, 0->no blank.,0x02->100ns*/
    .CS_EN_DCCAL = CS_CALIB_DIS,      /* DC Calibration, 0x00 - Disabled, 0x01 - Enabled */
    .CS_OCP_DEGLITCH = CS_DEGLITCH_4us,  /* OCP deglitch, 0-> 0uS,0x1->2uS,0x2->4uS,0x3->8uS */
    .CS_OCPFLT_CFG = OCP_FLT_TRIG_8,    /* OCP fault, 0x0-> 8 OCP count,0x01->16 OCP, 0x2-> all OCP,0x03->no fault trigger */
};

// CSAMP_CFG2 Register
CSAMP_CFG2_REG_t CSAMP_Config2 =
{
#if (MCUTYPE == MCU_IMD_1_5kw_ref)
    .CS_OCP_PTHR = OCP_POS_THR_40mV,    /* Positive threshold for OCP, 0x0->+0.3V, 0xF->0.02V */
    .CS_OCP_NTHR = OCP_POS_THR_40mV,    /* Negative threshold for OCP, 0x0->-0.3V, 0xF->0.02V */
#else
    .CS_OCP_PTHR = OCP_POS_THR_300mV,    /* Positive threshold for OCP, 0x0->+0.3V, 0xF->0.02V */
    .CS_OCP_NTHR = OCP_POS_THR_300mV,    /* Negative threshold for OCP, 0x0->-0.3V, 0xF->0.02V */
#endif
    .CS_OCP_LATCH = OCP_FLT_LATCH_EN,   /* OCP fault, 0x0-> unlatched,0x1->Latched */
    .CS_MODE = CS_SENSE_SHUNT_RES,        /* Current sense, 0x0-> shunt register,0x01 - Rds_on sensing */
    .CS_OCP_BRAKE = OCP_FLT_BRAKE_DIS,   /* OCP Brake, 0-> No braking upon OCP */
    .CS_TRUNC_DIS = OCP_PWM_TRUNC_DIS,   /* PWM Truncation, 0x0-> enabled, 0x1-> disabled */
    .VREF_INSEL = CS_VREF_INT,          /* VREF source, 0x0->Internal */
    .CS_NEG_OCP_DIS = OCP_NEG_EN,       /* negative OC Fault, 0x0 - enabled, 0x01-disabled */
    .CS_AZ_CFG = CS_AUTOZERO_EN,      /* Auto zero, 0-> enabled with internal sync, 1-> disabled, 2->external sync, 3-> external sync with CP clock gating */
};

// OTP_PROG Register
OTP_PROG_REG_t OTP_PROG_Config =
{
    .USER_ID = OTP_PROG_USER_ID,
    .OTP_PROG = OTP_PROG_DIS,
};

// FAULT_ST Register
FAULT_ST_REG_t FAULT_ST_Config =
{
    .FAULT_ST = REGISTER_ZERO_VALUE,
};

// Die temperature sensor,TEMP_ST Register
TEMP_ST_REG_t TEMP_ST_Config =
{
      .TEMP_VAL = REGISTER_ZERO_VALUE,
};

// SUPPLY_ST Register
SUPPLY_ST_REG_t SUPPLY_ST_Config =
{
    .SUPPLY_ST = REGISTER_ZERO_VALUE,
};

// FUNCT_ST Register
FUNCT_ST_REG_t FUNCT_ST_Config =
{
    .FUNCT_ST = REGISTER_ZERO_VALUE,
};

// OTP_ST Register
OTP_ST_REG_t OTP_ST_Config =
{
    .OTP_ST = REGISTER_ZERO_VALUE,
};

// ADC_ST Register
ADC_ST_REG_t ADC_ST_Config =
{
    .ADC_ST = REGISTER_ZERO_VALUE,
};

// CP_ST Register
CP_ST_REG_t CP_ST_Config =
{
    .CP_ST = REGISTER_ZERO_VALUE,
};

// DEVICE_ID Register
DEVICE_ID_REG_t DEVICE_ID_Config =
{
    .DEV_ID		= REGISTER_ZERO_VALUE,
};

// FAULTS_CLR Register
FAULTS_CLR_REG_t FAULTS_CLR_Config =
{
    .BIT_CLR_FLTS = CLR_FLTS,    /* Clear all faults except latch faults */
    .BIT_CLR_LATCH	= CLR_LATCH,  /* Clear latched faults */
};



void Driver_CSAmp_Init()
{
    uint8_t SPI_Datas[2];

    // Write CSAMP_CFG_Register
    SPI_Datas[0] = (CSAMP_Config.CSAMP_CFG) >> 8;
    SPI_Datas[1] = CSAMP_Config.CSAMP_CFG;
    SPI_Write_Word(addr_CSAMP_CFG, &SPI_Datas[0]);

    // Write CSAMP_CFG2_Register
    SPI_Datas[0] = CSAMP_Config2.CSAMP_CFG2 >> 8;
    SPI_Datas[1] = CSAMP_Config2.CSAMP_CFG2;
    SPI_Write_Word(addr_CSAMP_CFG2, &SPI_Datas[0]);
}

void Driver_CSDCCalib_Enable()
{
    uint8_t SPI_Datas[2];

    /* Enable DC Calibration */
    CSAMP_Config.CS_EN_DCCAL = 0x01;

    // Write CSAMP_CFG_Register
    SPI_Datas[0] = (CSAMP_Config.CSAMP_CFG) >> 8;
    SPI_Datas[1] = CSAMP_Config.CSAMP_CFG;
    SPI_Write_Word(addr_CSAMP_CFG, &SPI_Datas[0]);
}

void Driver_CSDCCalib_Disable()
{
    uint8_t SPI_Datas[2];

    /* Disable DC Calibration */
    CSAMP_Config.CS_EN_DCCAL = 0x00;

    // Write CSAMP_CFG_Register
    SPI_Datas[0] = (CSAMP_Config.CSAMP_CFG) >> 8;
    SPI_Datas[1] = CSAMP_Config.CSAMP_CFG;
    SPI_Write_Word(addr_CSAMP_CFG, &SPI_Datas[0]);
}

void Driver_PowerSupply_Init()
{
    uint8_t SPI_Datas[2];

    // Write SUPPLY_CFG_Register
    SPI_Datas[0] = SUPPLY_Config.SUPPLY_CFG >> 8;
    SPI_Datas[1] = SUPPLY_Config.SUPPLY_CFG;
    SPI_Write_Word(addr_SUPPLY_CFG, &SPI_Datas[0]);
}

void Driver_ChargePump_Init()
{
    uint8_t SPI_Datas[2];

    // Write CP_CFG_Register
    SPI_Datas[0] = CP_Config.CP_CFG >> 8;
    SPI_Datas[1] = CP_Config.CP_CFG;
    SPI_Write_Word(addr_CP_CFG, &SPI_Datas[0]);
}


void Driver_WatchDog_Init()
{
    uint8_t SPI_Datas[2];

    // Write WD_CFG_Register
    SPI_Datas[0] = WD_Config.WD_CFG >> 8;
    SPI_Datas[1] = WD_Config.WD_CFG;
    SPI_Write_Word(addr_WD_CFG, &SPI_Datas[0]);

    // Write WD_CFG2_Register
    SPI_Datas[0] = WD_Config2.WD_CFG2 >> 8;
    SPI_Datas[1] = WD_Config2.WD_CFG2;
    SPI_Write_Word(addr_WD_CFG2, &SPI_Datas[0]);
}

void Driver_GateDriverStage_Init()
{
    uint8_t SPI_Datas[2];

    // Write IDRIVE_CFG_Register
    SPI_Datas[0] = IDRIVE_Config.IDRIVE_CFG >> 8;
    SPI_Datas[1] = IDRIVE_Config.IDRIVE_CFG;
    SPI_Write_Word(addr_IDRIVE_CFG, &SPI_Datas[0]);

    // Write IDRIVE_PRE_CFG_Register
    SPI_Datas[0] = IDRIVE_PRE_Config.IDRIVE_PRE_CFG >> 8;
    SPI_Datas[1] = IDRIVE_PRE_Config.IDRIVE_PRE_CFG;
    SPI_Write_Word(addr_IDRIVE_PRE_CFG, &SPI_Datas[0]);

    // Write TDRIVE_SRC_CFG_Register
    SPI_Datas[0] = TDRIVE_SRC_Config.TDRIVE_SRC_CFG >> 8;
    SPI_Datas[1] = TDRIVE_SRC_Config.TDRIVE_SRC_CFG;
    SPI_Write_Word(addr_TDRIVE_SRC_CFG, &SPI_Datas[0]);

    // Write TDRIVE_SINK_CFG_Register
    SPI_Datas[0] = TDRIVE_SINK_Config.TDRIVE_SINK_CFG >> 8;
    SPI_Datas[1] = TDRIVE_SINK_Config.TDRIVE_SINK_CFG;
    SPI_Write_Word(addr_TDRIVE_SINK_CFG, &SPI_Datas[0]);
}

void Driver_Sensor_Init()
{
    uint8_t SPI_Datas[2];

    // Write SENSOR_CFG_Register
    SPI_Datas[0] = SENSOR_Config.SENSOR_CFG >> 8;
    SPI_Datas[1] = SENSOR_Config.SENSOR_CFG;
    SPI_Write_Word(addr_SENSOR_CFG, &SPI_Datas[0]);
}

void Driver_PWM_Init()
{
    uint8_t SPI_Datas[2];

    // Write PWM_CFG_Register
    SPI_Datas[0] = PWM_Config.PWM_CFG >> 8;
    SPI_Datas[1] = PWM_Config.PWM_CFG;
    SPI_Write_Word(addr_PWM_CFG, &SPI_Datas[0]);

    // Write DT_CFG_Register
    SPI_Datas[0] = DT_Config.DT_CFG >> 8;
    SPI_Datas[1] = DT_Config.DT_CFG;
    SPI_Write_Word(addr_DT_CFG, &SPI_Datas[0]);
}

void Driver_Brake_Init(BRAKING_MODE_t brake_mode)
{
    uint8_t SPI_Datas[2];

    /* Configure the brake mode */
    PWM_Config.BRAKE_CFG = brake_mode;

    /* Write PWM_CFG_Register */
    SPI_Datas[0] = PWM_Config.PWM_CFG >> 8;
    SPI_Datas[1] = PWM_Config.PWM_CFG;

    SPI_Write_Word(addr_PWM_CFG, &SPI_Datas[0]);
}

/* Clears all faults including latched faults */
void Driver_Faults_Clear()
{
    uint8_t SPI_Datas[2];
    
    #ifdef INVERTER_EN_PIN
    XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, USER_INVERTER_DISABLE_LEVEL);
    #endif

    // Clear FAULTS_CLR_Register
    SPI_Datas[0] = FAULTS_CLR_Config.FAULTS_CLR >> 8;
    SPI_Datas[1] = FAULTS_CLR_Config.FAULTS_CLR;
    SPI_Write_Word(addr_FAULTS_CLR, &SPI_Datas[0]);

    /* Toggle Chip enable pin to clear latched faults if any. */
    #ifdef INVERTER_EN_PIN
    XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, USER_INVERTER_ENABLE_LEVEL);
    #endif
}

void Driver_ProgramOTP()
{
    uint8_t SPI_Datas[2];

    // Write OTP_PROG_Register
    SPI_Datas[0] = OTP_PROG_Config.OTP_PROG >> 8;
    SPI_Datas[1] = OTP_PROG_Config.OTP_PROG;
    SPI_Write_Word(addr_OTP_PROG, &SPI_Datas[0]);
}

void Driver_GetFaultStatus()
{
    uint8_t SPI_Data[3] = {0x00, 0x00, 0x00};

    // Read FAULT_ST_Register
    SPI_Read_Word(addr_FAULT_ST, &SPI_Data[0]);
    FAULT_ST_Config.BIT15_8 = SPI_Data[1];
    FAULT_ST_Config.BIT7_0 = SPI_Data[2];
}

void Driver_GetDeviceStatus()
{
    uint8_t SPI_Data[3] = {0x00, 0x00, 0x00};

    // Read FAULT_ST_Register
    SPI_Read_Word(addr_FAULT_ST, &SPI_Data[0]);
    FAULT_ST_Config.BIT15_8 = SPI_Data[1];
    FAULT_ST_Config.BIT7_0 = SPI_Data[2];

    // Read TEMP_ST_Register
    SPI_Read_Word(addr_TEMP_ST, &SPI_Data[0]);
    TEMP_ST_Config.BIT7_0 = SPI_Data[2];

    // Read SUPPLY_ST_Register
    SPI_Read_Word(addr_SUPPLY_ST, &SPI_Data[0]);
    SUPPLY_ST_Config.BIT15_8 = SPI_Data[1];
    SUPPLY_ST_Config.BIT7_0 = SPI_Data[2];

    // Read FUNCT_ST_Register
    SPI_Read_Word(addr_FUNCT_ST, &SPI_Data[0]);
    FUNCT_ST_Config.BIT7_0 = SPI_Data[2];

    // Read OTP_ST_Register
    SPI_Read_Word(addr_OTP_ST, &SPI_Data[0]);
    OTP_ST_Config.BIT7_0 = SPI_Data[2];
}

void Driver_GetADCStaus(void)
{
    // Read ADC_ST_Register
    SPI_Read_Word(addr_ADC_ST, &SPI_Data[0]);
    ADC_ST_Config.BIT7_0 = SPI_Data[2];
}

void Driver_GetCPStaus(void)
{
    // Read CP_ST_Register
    SPI_Read_Word(addr_CP_ST, &SPI_Data[0]);
    CP_ST_Config.BIT15_8 = SPI_Data[1];
    CP_ST_Config.BIT7_0 = SPI_Data[2];
}

void Driver_GetDeviceID(void)
{
    uint8_t SPI_Data[3] = {0x00, 0x00, 0x00};

    /* Read DEVICE_ID_Register */
    SPI_Read_Word(addr_DEVICE_ID, &SPI_Data[0]);
    DEVICE_ID_Config.BIT7_0 = SPI_Data[2];
}

// @Function      void Read_Word (uint8_t RegName, uint8_t* Data)
//----------------------------------------------------------------------------
// @Description   This function send the read command to the SPI channel
//                The RegAddr is the address of register to read,
//                Data parameter is a pointer to store the two bytes of data read
//                from the register
//----------------------------------------------------------------------------
// @Returnvalue   none
//----------------------------------------------------------------------------
// @Parameters    RegName: the address of register
//                Data: the data array pointer read from register via SPI
//


void SPI_Read_Word(uint8_t RegName, uint8_t* Data)
{
    XMC_USIC_CH_RXFIFO_Flush(SPI_MAS_CH);
    #if (GATE_DRIVER == IFX_6EDL7141)
    SPI_MAS_CH->IN[0] = RegName & RegRead;
    #elif (GATE_DRIVER == IFX_NN)
    SPI_MAS_CH->IN[0] = RegName | RegRead;
    #endif
    SPI_MAS_CH->IN[0] = 0x00;
    SPI_MAS_CH->IN[0] = 0x00;
    while((XMC_USIC_CH_RXFIFO_GetEvent(SPI_MAS_CH) & XMC_USIC_CH_RXFIFO_EVENT_STANDARD) == 0);//wait for received data /2 words

    //	Read_Slave(SPI_MAS_CH, SPIRxData);
    XMC_USIC_CH_RXFIFO_ClearEvent(SPI_MAS_CH, XMC_USIC_CH_RXFIFO_EVENT_STANDARD);
    *Data = SPI_MAS_CH->OUTR;               //data read of invalid data
    *Data = SPI_MAS_CH->OUTR;             //data read of MSB
    *(Data+1) = SPI_MAS_CH->OUTR;           //data read of LSB
}

// @Function      void Write_Word (uint8_t RegName, uint8_t* Data)
//----------------------------------------------------------------------------
// @Description   This function write data to register via SPI channel
//                The RegAddr is the address of register to write to,
//                Data parameter is a pointer which stored the two bytes of data
//                written to the register
//----------------------------------------------------------------------------
// @Returnvalue   none
//----------------------------------------------------------------------------
// @Parameters    RegName: the address of register
//                Data: the data array pointer to store data write to register via SPI
//
void SPI_Write_Word(uint8_t RegName, uint8_t* Data)
{
    uint8_t readData;
    XMC_USIC_CH_RXFIFO_Flush(SPI_MAS_CH);
  #if (GATE_DRIVER == IFX_6EDL7141)
    SPI_MAS_CH->IN[0] = RegName | RegWrite;
  #elif (GATE_DRIVER == IFX_NN)
    SPI_MAS_CH->IN[0] = RegName & RegWrite;
  #endif
    SPI_MAS_CH->IN[0] = *Data;
    SPI_MAS_CH->IN[0] = *(Data+1);
    while((XMC_USIC_CH_RXFIFO_GetEvent(SPI_MAS_CH) & XMC_USIC_CH_RXFIFO_EVENT_STANDARD) == 0);//wait for received data /2 words

    //	Read_Slave(SPI_MAS_CH, SPIRxData);
    XMC_USIC_CH_RXFIFO_ClearEvent(SPI_MAS_CH, XMC_USIC_CH_RXFIFO_EVENT_STANDARD);
    readData = SPI_MAS_CH->OUTR;          //data read to clear buffer
    readData |= SPI_MAS_CH->OUTR;          //data read of clear buffer
    readData |= SPI_MAS_CH->OUTR;          //data read of clear buffer
}
