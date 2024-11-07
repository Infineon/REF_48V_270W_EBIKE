/**
    @file: IMD700_SPI_LIB.c

    Implementation of IMD700_SPI_LIB
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

#include <PMSM_FOC/IMD700A_SPI_LIB/IMD700_SPI_LIB.h>
#include "../Configuration/pmsm_foc_config.h"

#define FW_VER 0x02   /* for FW version storage*/
#define XMC_UART_CH_OVERSAMPLING (16UL)


/*********************************************************************************************************************
  * DATA STRUCTURES
*********************************************************************************************************************/


///* GPIO LED pin configuration */
XMC_GPIO_CONFIG_t LED_pin_config =
{
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level= (XMC_GPIO_OUTPUT_LEVEL_t)0U
};

extern XMC_UART_CH_CONFIG_t   uart_config;

/*********************************************************************************************************************
  * GLOBAL VARIABLES
*********************************************************************************************************************/
void SendByte(uint8_t data);
void device_id_read(void);
void bauderate_change_response(XMC_USIC_CH_t *const channel, uint32_t baudrate, uint32_t oversampling);
//void poll_gui_message(void);
uint8_t checksum (uint8_t *msg, uint16_t MsgSize);

uint8_t UARTTxData[5] = {0,0,0,0,0};
uint8_t UARTRxData[5] = {0,0,0,0,0};

uint8_t SPI_Data[3] = {0,0,0};
uint8_t RegData[3];
uint8_t chk, CSResult;

uint8_t fw_ver;
bool RW;

/*********************************************************************************************************************
  * LOCAL ROUTINES
*********************************************************************************************************************/



/*********************************************************************************************************************
  * INITIALIZATION & FUNCTION IMPLEMENTATION
*********************************************************************************************************************/
#include <xmc_uart.h>


//int main(void)
//{
////  uint16_t count = 0;
// // fw_ver = FW_VER;
//  //SystemCoreClockUpdate();
//
//  ASC_Init();  //ASC initialization
////  XMC_GPIO_Init(XMC_GPIO_PORT0, 10, &LED_pin_config);
////  XMC_GPIO_Init(XMC_GPIO_PORT0, 11, &LED_pin_config);
//  //SPI master init
//  SPI_Master_Init(SPI_MAS_CH);
//
//
//  while(1)
//  {
//	  poll_gui_message();
//  }   //end of while(1)
//
//
//}

void SendByte(uint8_t data)
{
    while(!((USIC_CH->TRBSR & (0x01UL << 11)) >> 11) ) {}; //check if Tx FIFO is empty
    {/* Clear the Transmit Buffer indication flag */
        USIC_CH->IN[0] = data;
    }
}


/****************************************************************************
// @Function      uint8_t checksum (uint8_t *msg, uint16_t MsgSize)
//----------------------------------------------------------------------------
// @Description   This function calculates the message error checking byte
//                using Checksum
//----------------------------------------------------------------------------
// @Returnvalue   chk: the calculated checksum byte
//----------------------------------------------------------------------------
// @Parameters    *msg: the message that need to be calculated
//                 MsgSize: the size of the message
//----------------------------------------------------------------------------*/
uint8_t checksum(uint8_t *msg, uint16_t MsgSize)
{
    uint8_t chk = 0;                //initial value
    while (MsgSize-- != 0) chk -= *msg++;
    return chk;
}

/****************************************************************************
// @Function      void device_id_read (XMC_USIC_CH_t *const channel, uint32_t baudrate, uint32_t oversampling)
//----------------------------------------------------------------------------
// @Description   This function read the FW version and Chimera device ID
//                and send the read data in request baud rate
//----------------------------------------------------------------------------
// @Returnvalue   void
//----------------------------------------------------------------------------
// @Parameters    *channel: the uart channel pointer
//                 baudrate: the request baudrate
//                 oversampling: the number of samples for a symbol
//----------------------------------------------------------------------------*/
void device_id_read(void)
{
    //uint16_t readData = 0;


    XMC_USIC_CH_RXFIFO_ClearEvent(SPI_MAS_CH, XMC_USIC_CH_RXFIFO_EVENT_STANDARD);
    #if (GATE_DRIVER == IFX_6EDL7141)
      SPI_Data[0] = 0x07;  // do a read back of 6EDL7141's Device ID addr 0x07
    #elif (GATE_DRIVER == IFX_NN)
      SPI_Data[0] = 0x2A;  // do a read back of NN gate driver status addr 0x2A
    #endif
    SPI_Data[1] = 0x00;
    SPI_Data[2] = 0x00;
    XMC_USIC_CH_RXFIFO_Flush(SPI_MAS_CH);
    SPI_Read_Word(SPI_Data[0], &SPI_Data[1]);
    UARTTxData[0] = 0x55;
    UARTTxData[1] = (uint8_t)FW_VER;
    UARTTxData[2] = (SPI_Data[1]);
    UARTTxData[3] = (SPI_Data[2]);
    CSResult = checksum(&UARTTxData[0],4);

    XMC_GPIO_SetOutputHigh(XMC_GPIO_PORT0, 11);

    //send the response to PC
    //write ACK to PC
    SendByte(UARTTxData[0]);
    SendByte(UARTTxData[1]);
    SendByte(UARTTxData[2]);
    SendByte(UARTTxData[3]);
    SendByte(CSResult);
}

/****************************************************************************
// @Description  1. send ACK to PC after receiving the baudrate change command
//               2. change the baudrate to the new one
//----------------------------------------------------------------------------*/

void bauderate_change_response(XMC_USIC_CH_t *const channel, uint32_t baudrate, uint32_t oversampling)
{
    //	int i;

    UARTTxData[0] = 0x55;
    UARTTxData[1] = 0x0;
    UARTTxData[2] = 0x0;
    UARTTxData[3] = 0x0;
    CSResult = checksum(&UARTTxData[0],4);

    //send the response to PC
    //write ACK to PC
    SendByte(UARTTxData[0]);
    SendByte(UARTTxData[1]);
    SendByte(UARTTxData[2]);
    SendByte(UARTTxData[3]);
    SendByte(CSResult);

    /* Check uart tx and rx flag is idle before change baudrate */
    while(!(((XMC_UART_CH_GetStatusFlag(channel) & XMC_UART_CH_STATUS_FLAG_TRANSMISSION_IDLE)>>USIC_CH_PSR_ASCMode_TXIDLE_Pos)| \
          ((XMC_UART_CH_GetStatusFlag(channel) & XMC_UART_CH_STATUS_FLAG_RECEPTION_IDLE)>>USIC_CH_PSR_ASCMode_RXIDLE_Pos)) ) {};
    if (XMC_USIC_CH_SetBaudrate(channel, baudrate, oversampling)) {};
}

void poll_gui_message()
{
	  uint16_t count = 0;
	  //uint16_t readData = 0;
    if ((XMC_USIC_CH_RXFIFO_GetEvent(UART_CH) & XMC_USIC_CH_RXFIFO_EVENT_STANDARD) == 1)
    {
        XMC_GPIO_SetOutputHigh(XMC_GPIO_PORT0, 11);
        for (count=0; count<5; count++)
        {
           UARTRxData[count] = (uint16_t)UART_CH->OUTR;
        }
        XMC_USIC_CH_RXFIFO_ClearEvent(UART_CH, XMC_USIC_CH_RXFIFO_EVENT_STANDARD);
        XMC_USIC_CH_RXFIFO_Flush(UART_CH);

        XMC_GPIO_SetOutputLow(XMC_GPIO_PORT0, 11);
        chk = checksum (&UARTRxData[0],4);
        XMC_GPIO_SetOutputHigh(XMC_GPIO_PORT0, 11);

        if (chk == UARTRxData[4])
        {
            if((UARTRxData[0] != 0x7F) && (UARTRxData[0] != 0x5F) && (UARTRxData[0] != 0xA1) && (UARTRxData[0] != 0xA2))
            {
                UARTTxData[0] = 0x9E;
                UARTTxData[1] = 0xF1;  //header error
                UARTTxData[2] = 0x00;
                UARTTxData[3] = 0x00;
                CSResult = checksum(&UARTTxData[0],4);
                UARTTxData[4] = CSResult;

                SendByte(UARTTxData[0]);
                SendByte(UARTTxData[1]);
                SendByte(UARTTxData[2]);
                SendByte(UARTTxData[3]);
                SendByte(CSResult);
            }
            else
            {
                switch (UARTRxData[0])
                {
                    case 0xA1:                                             /* send Read Register command*/
                        RW=1;
                        SPI_Data[0] = UARTRxData[1];
                        SPI_Data[1] = UARTRxData[2];
                        SPI_Data[2] = UARTRxData[3];
                        SPI_Read_Word(SPI_Data[0], &SPI_Data[1]);
                        XMC_GPIO_SetOutputLow(XMC_GPIO_PORT0, 11);
                        UARTTxData[0]=0x93;
                        UARTTxData[1]=UARTRxData[1];
                        UARTTxData[2]=SPI_Data[1];
                        UARTTxData[3]=SPI_Data[2];
                        CSResult = checksum(&UARTTxData[0],4);

                        XMC_GPIO_SetOutputHigh(XMC_GPIO_PORT0, 11);
                        SendByte(UARTTxData[0]);
                        SendByte(UARTTxData[1]);
                        SendByte(UARTTxData[2]);
                        SendByte(UARTTxData[3]);
                        SendByte(CSResult);
                        XMC_GPIO_SetOutputLow(XMC_GPIO_PORT0, 11);
                        break;
                    case 0xA2:                            /* send Write register header receive*/
                        RW=0;
                        SPI_Data[0] = UARTRxData[1];
                        SPI_Data[1] = UARTRxData[2];
                        SPI_Data[2] = UARTRxData[3];
                        SPI_Write_Word(SPI_Data[0], &SPI_Data[1]);
                        SPI_Data[0] = UARTRxData[1];
                        SPI_Data[1] = 0x00;
                        SPI_Data[2] = 0x00;
                        SPI_Read_Word(SPI_Data[0], &SPI_Data[1]);     /*do a read back to verify write command*/
                        XMC_GPIO_SetOutputLow(XMC_GPIO_PORT0, 11);
                        UARTTxData[0]=0x93;
                        UARTTxData[1]=UARTRxData[1];
                        UARTTxData[2]=SPI_Data[1];
                        UARTTxData[3]=SPI_Data[2];
                        CSResult = checksum(&UARTTxData[0],4);

                        XMC_GPIO_SetOutputHigh(XMC_GPIO_PORT0, 11);
                        SendByte(UARTTxData[0]);                 /*send the read back data for write verification*/
                        SendByte(UARTTxData[1]);
                        SendByte(UARTTxData[2]);
                        SendByte(UARTTxData[3]);
                        SendByte(CSResult);

                        XMC_GPIO_SetOutputLow(XMC_GPIO_PORT0, 11);
                        break;
                    case 0x7F:
                        if(UARTRxData[1] > 0x3 )
                        {
                            UARTTxData[0] = 0x9E;
                            UARTTxData[1] = 0xF5;  //baudrate error
                            UARTTxData[2] = 0x00;
                            UARTTxData[3] = 0x00;
                            CSResult = checksum(&UARTTxData[0],4);
                            UARTTxData[4] = CSResult;

                            SendByte(UARTTxData[0]);
                            SendByte(UARTTxData[1]);
                            SendByte(UARTTxData[2]);
                            SendByte(UARTTxData[3]);
                            SendByte(CSResult);
                            break;
                        }
                        switch (UARTRxData[1])
                        {
                            case 0x00:
                                bauderate_change_response(UART_CH, 115200, XMC_UART_CH_OVERSAMPLING); //UART baudrate at default 115.2kbps
                                break;
                            case 0x01:
                                bauderate_change_response(UART_CH, 230400, XMC_UART_CH_OVERSAMPLING); //change UART baudrate to 230.4kbps
                                break;
                            case 0x02:
                                bauderate_change_response(UART_CH, 460800, XMC_UART_CH_OVERSAMPLING); //change UART baudrate to 460.8kbps
                                break;
                            case 0x03:
                                bauderate_change_response(UART_CH, 921600, XMC_UART_CH_OVERSAMPLING); //change UART baudrate to 921.6kbps
                                break;
                            default:
                                break;
                        }
                        break;

                    case 0x5F:                               //header to read ID from device over SPI
                        device_id_read();
                        break;

                    default:
                    break;
                }   //end of case for UART header received
            }
        }
        else   // error frame for incorrect checksum
        {
            UARTTxData[0] = 0x9E;
            UARTTxData[1] = 0xF2;  //checksum error
            UARTTxData[2] = 0x00;
            UARTTxData[3] = 0x00;
            CSResult = checksum(&UARTTxData[0],4);
            UARTTxData[4] = CSResult;

            SendByte(UARTTxData[0]);
            SendByte(UARTTxData[1]);
            SendByte(UARTTxData[2]);
            SendByte(UARTTxData[3]);
            SendByte(CSResult);
        }
    }  // end of if loop for detect GUI message
}

/* This function will configure driver IC. */
void Config_Driver_6EDL7141(void)
{
    Driver_Faults_Clear();
    Driver_PowerSupply_Init();
    Driver_ChargePump_Init();
    Driver_PWM_Init();
    Driver_CSAmp_Init();
    Driver_GateDriverStage_Init();
    Driver_Sensor_Init();
    Driver_WatchDog_Init();
    /* Configures braking mode in 6EDL7141 gate driver through nBRAKE pin */
    //Driver_Brake_Init((BRAKING_MODE_t)USER_MOTOR_STOP_METHOD);
    Driver_Brake_Init(BRAKE_HIGH_Z);
}
