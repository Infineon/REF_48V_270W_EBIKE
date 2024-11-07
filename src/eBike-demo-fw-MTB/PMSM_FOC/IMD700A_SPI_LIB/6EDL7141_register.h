/**
    @file: 6EDL7141_register.h

    Header file of 6EDL7141_register
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

#include <stdint.h>

#ifndef REGISTERS_H_
#define REGISTERS_H_

// 6EDL7141 Registers address
#define addr_FAULT_ST 		   0x0
#define addr_TEMP_ST 	       0x1
#define addr_SUPPLY_ST	     0x2
#define addr_FUNCT_ST		     0x3
#define addr_OTP_ST 		     0x4
#define addr_ADC_ST 	       0x5
#define addr_CP_ST 		       0x6
#define addr_DEVICE_ID 		   0x7
#define addr_FAULTS_CLR      0x10

#define addr_SUPPLY_CFG      0x11
#define addr_ADC_CFG 		     0x12
#define addr_PWM_CFG 		     0x13
#define addr_SENSOR_CFG 	   0x14
#define addr_WD_CFG 		     0x15
#define addr_WD_CFG2 	       0x16
#define addr_IDRIVE_CFG	     0x17
#define addr_IDRIVE_PRE_CFG  0x18
#define addr_TDRIVE_SRC_CFG  0x19
#define addr_TDRIVE_SINK_CFG 0x1A
#define addr_DT_CFG          0x1B
#define addr_CP_CFG          0x1C
#define addr_CSAMP_CFG       0x1D
#define addr_CSAMP_CFG2      0x1E
#define addr_OTP_PROG 	     0x1F


// FAULT_ST Register
/*****************************************************************************
* Structure for initialize the FAULT_ST register
*/
typedef struct FAULT_ST_REG
{
    union
    {
        struct
        {
            uint32_t CS_OCP_FLT   : 3;   //bit 2-0
            uint32_t CP_FLT 		  : 1;   //bit 3
            uint32_t DVDD_OCP_FLT	: 1;   //bit 4
            uint32_t DVDD_UV_FLT 	: 1;   //bit 5
            uint32_t DVDD_OV_FLT 	: 1;   //bit 6
            uint32_t BK_OCP_FLT 	: 1;   //bit 7
            uint32_t OTS_FLT 	    : 1;   //bit 8
            uint32_t OTW_FLT 		  : 1;   //bit 9
            uint32_t RLOCK_FLT 	  : 1;   //bit 10
            uint32_t WD_FLT 	    : 1;   //bit 11
            uint32_t OTP_FLT      : 1;   //bit 12
            uint32_t RESERV       :19;   //reserve bit 31-13
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t FAULT_ST;
    };
} FAULT_ST_REG_t;

// TEMP_ST Register
typedef struct TEMP_ST_REG
{
    union
    {
        struct
        {
            uint32_t TEMP_VAL :7;	 // bit 6-0
            uint32_t RESERV   :25; // reserve bit 31-7

        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t TEMP_ST;
    };
}TEMP_ST_REG_t;

// SUPPLY_ST Register
typedef struct SUPPLY_ST_REG
{
    union
    {
        struct
        {
            uint32_t VCCLS_UVST :1;	// bit 0
            uint32_t VCCHS_UVST:1;	// bit 1
            uint32_t DVDD_UVST :1;	// bit 3
            uint32_t DVDD_OVST :1;	// bit 4
            uint32_t VDDB_UVST :1;	// bit 6
            uint32_t VDDB_OVST :1;	// bit 7
            uint32_t PVDD_VAL  :7;	// bit 12-6
            uint32_t RESERV    :19; //reserve bit 31-13
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t SUPPLY_ST;
    };
}SUPPLY_ST_REG_t;

// FUNCT_ST Register
typedef struct FUNCT_ST_REG
{
    union
    {
        struct
        {
            uint32_t HALLIN_ST   :3;  // bit 2-0
            uint32_t HALLPOL_ST  :1;  // bit 3
            uint32_t DVDD_ST     :1;  // bit 4
            uint32_t CS_GAIN_ST  :3;  // bit 5-7
            uint32_t RESERV      :24;   //reserve bit 31-8
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t FUNCT_ST;
    };
}FUNCT_ST_REG_t;

// OTP_ST Register
typedef struct OTP_ST_REG
{
    union
    {
        struct
        {
            uint32_t OTP_USED       :1;	// bit 0
            uint32_t OTP_PASS       :1;	// bit 1
            uint32_t OTP_PROG_BLOCK :1;	// bit 2
            uint32_t OTP_PROG_FAIL  :1; // bit 3
            uint32_t RESERV         :28;//reserve bit 31-4
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t OTP_ST;
    };
}OTP_ST_REG_t;

// ADC_ST Register
typedef struct ADC_ST_REG
{
    union
    {
        struct
        {
            uint32_t ADC_OD_RDY :1;	// bit 0
            uint32_t ADC_OD_VAL :7;	// bit 7-1
            uint32_t RESERV     :24; //reserve bit 31-8
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t ADC_ST;
    };
}ADC_ST_REG_t;

// CP_ST Register
typedef struct CP_ST_REG
{
    union
    {
        struct
        {
            uint32_t VCCHS_VAL  :7; // bit 6-0
            uint32_t VCCLS_VAL  :7;	// bit 13-7
            uint32_t RESERV     :18; //reserve bit 31-14
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t CP_ST;
    };
}CP_ST_REG_t;

// DEVICE_ID Register
typedef struct DEVICE_ID_REG
{
    union
    {
        struct
        {
            uint32_t DEV_ID  :4;	// bit 3-0
            uint32_t RESERV  :28;   //reserve bit 31-4
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t DEVICE_ID;
    };
}DEVICE_ID_REG_t;

// FAULTS_CLR Register
typedef struct FAULTS_CLR_REG
{
    union
    {
        struct
        {
            uint32_t BIT_CLR_FLTS  :1;	// bit 0
            uint32_t BIT_CLR_LATCH :1;	// bit 1
            uint32_t RESERV    :30;  //reserve bit 31-2
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t FAULTS_CLR;
    };
}FAULTS_CLR_REG_t;

// SUPPLY_CFG Register
typedef struct SUPPLY_CFG_REG
{
    union
    {
        struct
        {
            uint32_t PVCC_SETPT       :2; // bit 1-0
            uint32_t CS_REF_CFG       :2; // bit 3-2
            uint32_t DVDD_OCP_CFG     :2; // bit 5-4
            uint32_t DVDD_SFTSTRT     :4; // bit 9-6
            uint32_t DVDD_SETPT       :2; // bit 11-10
            uint32_t BK_FREQ          :1; // bit 12
            uint32_t DVDD_TON_DELAY   :2; // bit 14-13
            uint32_t CP_PRECHARGE_EN  :1; // bit 15
            uint32_t RESERV           :16;//reserve bit 31-16
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t SUPPLY_CFG;
    };
}SUPPLY_CFG_REG_t;

// ADC_CFG Register
typedef struct ADC_CFG_REG
{
    union
    {
        struct
        {
            uint32_t ADC_OD_REQ       :1;	// bit 0
            uint32_t ADC_OD_INSEL     :2;	// bit 2-1
            uint32_t ADC_EN_FILT      :1;	// bit 3
            uint32_t ADC_FILT_CFG     :2;	// bit 5-4
            uint32_t ADC_FILT_CFG_P   :2;	// bit 7-6
            uint32_t RESERV           :24;//reserve bit 31-8
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t ADC_CFG;
    };
}ADC_CFG_REG_t;

// PWM_CFG Register
typedef struct PWM_CFG_REG
{
    union
    {
        struct
        {
            uint32_t PWM_MODE         :3;	// bit 2-0
            uint32_t PWM_FREEW_CFG    :1;	// bit 3
            uint32_t BRAKE_CFG        :2;	// bit 5-4
            uint32_t PWM_RECIRC       :1;	// bit 6
            uint32_t RESERV           :25;//reserve bit 31-7
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t PWM_CFG;
    };
}PWM_CFG_REG_t;

// SENSOR_CFG Register
typedef struct SENSOR_CFG_REG
{
    union
    {
        struct
        {
            uint32_t HALL_DEGLITCH :4;	// bit 3-0
            uint32_t OTS_DIS       :1;	// bit 5
            uint32_t CS_TMODE	     :2;	// bit 6-5
            uint32_t RESERV        :25; //reserve bit 31-7
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t SENSOR_CFG;
    };
}SENSOR_CFG_REG_t;

// WD_CFG Register
typedef struct WD_CFG_REG
{
    union
    {
        struct
        {
            uint32_t BIT_WD_EN          :1;	// bit 0
            uint32_t WD_INSEL       :3;	// bit 3-1
            uint32_t WD_FLTCFG      :1;	// bit 4
            uint32_t WD_TIMER_VAL   :10;// bit 14-5
            uint32_t RESERV         :17;//reserve bit 31-15
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t WD_CFG;
    };
}WD_CFG_REG_t;

// WD_CFG2 Register
typedef struct WD_CFG2_REG
{
    union
    {
        struct
        {
            uint32_t WD_BRAKE          :1;  // bit 0
            uint32_t WD_EN_LATCH       :1;  // bit 1
            uint32_t WD_DVDD_RSTRT_ATT :2;  // bit 3-2
            uint32_t WD_DVDD_RSTRT_DLY :4;  // bit 7-4
            uint32_t WD_RLOCK_EN       :1;  // bit 8
            uint32_t WD_RLOCK_T        :3;  // bit 11-9
            uint32_t WD_BK_DIS         :1;  // bit 12
            uint32_t RESERV            :19; //reserve bit 31-13
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t WD_CFG2;
    };
} WD_CFG2_REG_t;

// IDRIVE_CFG Register
typedef struct IDRIVE_CFG_REG
{
    union
    {
        struct
        {
            uint32_t IHS_SRC        :4;	// bit 3-0
            uint32_t IHS_SINK       :4;	// bit 7-4
            uint32_t ILS_SRC        :4; // bit 11-8
            uint32_t ILS_SINK       :4; // bit 15-12
            uint32_t RESERV         :16;//reserve bit 31-16
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t IDRIVE_CFG;
    };
}IDRIVE_CFG_REG_t;

// IDRIVE_PRE_CFG Register
typedef struct IDRIVE_PRE_CFG_REG
{
    union
    {
        struct
        {
            uint32_t I_PRE_SRC        :4; // bit 3-0
            uint32_t I_PRE_SINK       :4; // bit 7-4
            uint32_t I_PRE_EN         :1; // bit 8
            uint32_t RESERV           :23;//reserve bit 31-9
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t IDRIVE_PRE_CFG;
    };
}IDRIVE_PRE_CFG_REG_t;

// TDRIVE_SRC_CFG Register
typedef struct TDRIVE_SRC_CFG_REG
{
    union
    {
        struct
        {
            uint32_t TDRIVE1       :8; // bit 7-0
            uint32_t TDRIVE2       :8; // bit 15-8
            uint32_t RESERV        :16;//reserve bit 31-16
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t TDRIVE_SRC_CFG;
    };
}TDRIVE_SRC_CFG_REG_t;

// TDRIVE_SINK_CFG Register
typedef struct TDRIVE_SINK_CFG_REG
{
    union
    {
        struct
        {
            uint32_t TDRIVE3       :8; // bit 7-0
            uint32_t TDRIVE4       :8; // bit 15-8
            uint32_t RESERV        :16;//reserve bit 31-16
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t TDRIVE_SINK_CFG;
    };
}TDRIVE_SINK_CFG_REG_t;

// DT_CFG Register
typedef struct DT_CFG_REG
{
    union
    {
        struct
        {
            uint32_t DT_RISE       :8; // bit 7-0
            uint32_t DT_FALL       :8; // bit 15-8
            uint32_t RESERV        :16;//reserve bit 31-16
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t DT_CFG;
    };
}DT_CFG_REG_t;


// Charge Pump Configuration Register
typedef struct CP_CFG_REG
{
    union
    {
        struct
        {
            uint32_t CP_CLK_CFG       :2;  // bit 1-0
            uint32_t BIT_CP_CLK_SS_DIS    :1;  // bit 2
            uint32_t RESERV           :29; //reserve bit 31-3
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t CP_CFG;
    };

}CP_CFG_REG_t;

// CSAMP_CFG Register
typedef struct CSAMP_CFG_REG
{
    union
    {
        struct
        {
            uint32_t CS_GAIN		     :3;  // bit 2-0
            uint32_t CS_GAIN_ANA     :1;	// bit 3
            uint32_t CS_EN           :3;	// bit 6-4
            uint32_t CS_BLANK        :4;	// bit 10-7
            uint32_t CS_EN_DCCAL     :1;	// bit 11
            uint32_t CS_OCP_DEGLITCH :2;	// bit 13-12
            uint32_t CS_OCPFLT_CFG   :2;	// bit 15-14
            uint32_t RESERV          :16; //reserve bit 31-16
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t CSAMP_CFG;
    };
}CSAMP_CFG_REG_t;

// CSAMP_CFG2 Register
typedef struct CSAMP_CFG2_REG
{
    union
    {
        struct
        {
            uint32_t CS_OCP_PTHR     :4; // bit 3-0
            uint32_t CS_OCP_NTHR     :4; // bit 7-4
            uint32_t CS_OCP_LATCH    :1; // bit 8
            uint32_t CS_MODE         :1; // bit 9
            uint32_t CS_OCP_BRAKE    :1; // bit 10
            uint32_t CS_TRUNC_DIS    :1; // bit 11
            uint32_t VREF_INSEL      :1; // bit 12
            uint32_t CS_NEG_OCP_DIS  :1; //bit 13
            uint32_t CS_AZ_CFG       :2; //bit 15-14
            uint32_t RESERV          :16;//reserve bit 31-16
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t CSAMP_CFG2;
    };
}CSAMP_CFG2_REG_t;

// OTP_PROG Register
typedef struct OTP_PROG_REG
{
    union
    {
        struct
        {
            uint32_t OTP_PROG        :1;  // bit 0
            uint32_t USER_ID         :4;	// bit 4-1
            uint32_t RESERV          :27; //reserve bit 31-5
        };
        struct
        {
            uint32_t BIT7_0         : 8;
            uint32_t BIT15_8        : 8;
            uint32_t BIT24_16       : 8;
            uint32_t BIT31_25       : 8;
        };
        uint32_t PROG_OTP;
    };
}OTP_PROG_REG_t;


#endif /* REGISTERS_H_ */
