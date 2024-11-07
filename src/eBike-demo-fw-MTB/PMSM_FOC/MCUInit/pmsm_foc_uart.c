/**
 * @file pmsm_foc_uart.c
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
#include <PMSM_FOC/Configuration/pmsm_foc_config.h>
#include <PMSM_FOC/Configuration/pmsm_foc_const.h>
#include "../MCUInit/pmsm_foc_uart.h"
#include "../ControlModules/pmsm_foc_functions.h"
/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/

extern PMSM_FOC_CTRL_t PMSM_FOC_CTRL; /* motor control information */

/**
  * @brief	Initialize USIC Module
  *
  * @param  None
  * @retval None
  */
#define XMC_SCU_GCU_PASSWD_PROT_DISABLE (0x000000C0UL)  /*
                                                         * Password for disabling protection.
                                                         * Access to protected bits allowed.
                                                         */
#define XMC_SCU_GCU_PASSWD_PROT_ENABLE  (0x000000C3UL)  /* Password for enabling protection. */

#if(UART_ENABLE == USIC0_CH1_P1_2_P1_3)
/**
 * @brief	Initialize USIC Module
 *
 * @param  None
 * @retval None
 */
void pmsm_foc_uart_init (void)
{
    /* Disable clock gating to USIC0: */
    SCU_GENERAL->PASSWD = XMC_SCU_GCU_PASSWD_PROT_DISABLE;

    /* Stop gating USIC0 */
    SCU_CLK->CGATCLR0 = 0x00000008UL;

    /* Wait if VDDC is too low, for VDDC to stabilise */
    while (SCU_CLK->CLKCR & 0x40000000UL)
    {
      continue;
    }

    /* Enable bit protection */
    SCU_GENERAL->PASSWD = XMC_SCU_GCU_PASSWD_PROT_ENABLE;

    /* Enable the module kernel clock and the module functionality: */
    USIC0_CH1->KSCFG |= USIC_CH_KSCFG_MODEN_Msk | USIC_CH_KSCFG_BPMODEN_Msk;

    /* fFD = fPB. */
    /* FDR.DM = 02b (Fractional divider mode). */
    USIC0_CH1->FDR &= ~(USIC_CH_FDR_DM_Msk | USIC_CH_FDR_STEP_Msk);
    USIC0_CH1->FDR |= (0x02UL << USIC_CH_FDR_DM_Pos) | (FDR_STEP << USIC_CH_FDR_STEP_Pos);

    /* Configure baud rate generator: */
    /* BAUDRATE = fCTQIN/(BRG.PCTQ x BRG.DCTQ). */
    /* CLKSEL = 0 (fPIN = fFD), CTQSEL = 00b (fCTQIN = fPDIV), PPPEN = 0 (fPPP=fPIN). */
    USIC0_CH1->BRG &= ~(USIC_CH_BRG_PCTQ_Msk | USIC_CH_BRG_DCTQ_Msk | USIC_CH_BRG_PDIV_Msk | USIC_CH_BRG_CLKSEL_Msk |
                      USIC_CH_BRG_PPPEN_Msk);
    USIC0_CH1->BRG |= (BRG_PCTQ << USIC_CH_BRG_PCTQ_Pos) | (BRG_DCTQ << USIC_CH_BRG_DCTQ_Pos) |
                      (BRG_PDIV << USIC_CH_BRG_PDIV_Pos);

    /* Configuration of USIC Shift Control: */
    /* SCTR.FLE = 8 (Frame Length). */
    /* SCTR.WLE = 8 (Word Length). */
    /* SCTR.TRM = 1 (Transmission Mode). */
    /*
     * SCTR.PDL = 1 (This bit defines the output level at the shift data output signal when no data is available
     * for transmission).
     */
    USIC0_CH1->SCTR &= ~(USIC_CH_SCTR_TRM_Msk | USIC_CH_SCTR_FLE_Msk | USIC_CH_SCTR_WLE_Msk);
    USIC0_CH1->SCTR |= USIC_CH_SCTR_PDL_Msk | (0x01UL << USIC_CH_SCTR_TRM_Pos) | (0x07UL << USIC_CH_SCTR_FLE_Pos) |
                       (0x07UL << USIC_CH_SCTR_WLE_Pos);

    /* Configuration of USIC Transmit Control/Status Register: */
    /* TBUF.TDEN = 1 (TBUF Data Enable: A transmission of the data word in TBUF can be started if TDV = 1. */
    /*
     * TBUF.TDSSM = 1 (Data Single Shot Mode: allow word-by-word data transmission which avoid sending the same data
     * several times.
     */
    USIC0_CH1->TCSR &= ~(USIC_CH_TCSR_TDEN_Msk);
    USIC0_CH1->TCSR |= USIC_CH_TCSR_TDSSM_Msk | (0x01UL << USIC_CH_TCSR_TDEN_Pos);

    /* Configuration of Protocol Control Register: */
    /* PCR.SMD = 1 (Sample Mode based on majority). */
    /* PCR.STPB = 0 (1x Stop bit). */
    /* PCR.SP = 5 (Sample Point). */
    /* PCR.PL = 0 (Pulse Length is equal to the bit length). */
    USIC0_CH1->PCR &= ~(USIC_CH_PCR_ASCMode_STPB_Msk | USIC_CH_PCR_ASCMode_SP_Msk | USIC_CH_PCR_ASCMode_PL_Msk);
    USIC0_CH1->PCR |= USIC_CH_PCR_ASCMode_SMD_Msk | (9 << USIC_CH_PCR_ASCMode_SP_Pos);

    /* Configure Transmit Buffer: */
    /* Standard transmit buffer event is enabled. */
    /* Define start entry of Transmit Data FIFO buffer DPTR = 0. */
    USIC0_CH1->TBCTR &= ~(USIC_CH_TBCTR_SIZE_Msk | USIC_CH_TBCTR_DPTR_Msk);

    /* Set Transmit Data Buffer size and set data pointer to position 0. */
    USIC0_CH1->TBCTR |= ((UART_FIFO_SIZE << USIC_CH_TBCTR_SIZE_Pos) | (0x00 << USIC_CH_TBCTR_DPTR_Pos));

    /* Init UART_RX (P1.3 --> DX0A, or P2.11 --> DX0E): */
    XMC_GPIO_SetMode(P1_3, XMC_GPIO_MODE_INPUT_TRISTATE);
    USIC0_CH1->DX0CR = 0x00000000; /* USIC0_CH1.DX0A <-- P1.3. */

    /* Configure Receive Buffer: */
    /* Standard Receive buffer event is enabled. */
    /* Define start entry of Receive Data FIFO buffer DPTR. */
    USIC0_CH1->RBCTR &= ~(USIC_CH_RBCTR_SIZE_Msk | USIC_CH_RBCTR_DPTR_Msk);

    /* Set Receive Data Buffer Size and set data pointer to position max. */
    USIC0_CH1->RBCTR |= ((UART_FIFO_SIZE << USIC_CH_RBCTR_SIZE_Pos) | ((1 << UART_FIFO_SIZE) << USIC_CH_RBCTR_DPTR_Pos));

    /* Init UART_TX (P1.2 --> DOUT): */
    XMC_GPIO_SetMode(P1_2, XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7);

    /* Configuration of Channel Control Register: */
    /* CCR.PM = 00 ( Disable parity generation). */
    /* CCR.MODE = 2 (ASC mode enabled). */
    USIC0_CH1->CCR &= ~(USIC_CH_CCR_PM_Msk | USIC_CH_CCR_MODE_Msk);
    USIC0_CH1->CCR |= 0x02UL << USIC_CH_CCR_MODE_Pos;
}

#elif(UART_ENABLE == USIC0_CH0_P1_4_P1_5)
void pmsm_foc_uart_init (void)
{
    /* Disable clock gating to USIC0: */
    SCU_GENERAL->PASSWD = XMC_SCU_GCU_PASSWD_PROT_DISABLE;

    /* Stop gating USIC0 */
    SCU_CLK->CGATCLR0 = 0x00000008UL;

    /* Wait if VDDC is too low, for VDDC to stabilise */
    while (SCU_CLK->CLKCR & 0x40000000UL) continue;

    /* Enable bit protection */
    SCU_GENERAL->PASSWD = XMC_SCU_GCU_PASSWD_PROT_ENABLE;

    /* Enable the module kernel clock and the module functionality: */
    USIC0_CH0->KSCFG |= USIC_CH_KSCFG_MODEN_Msk | USIC_CH_KSCFG_BPMODEN_Msk;

    // fFD = fPB.
    // FDR.DM = 02b (Fractional divider mode).
    USIC0_CH0->FDR &= ~(USIC_CH_FDR_DM_Msk | USIC_CH_FDR_STEP_Msk);
    USIC0_CH0->FDR |= (0x02UL << USIC_CH_FDR_DM_Pos) | (FDR_STEP << USIC_CH_FDR_STEP_Pos);

    // Configure baud rate generator:
    // BAUDRATE = fCTQIN/(BRG.PCTQ x BRG.DCTQ).
    // CLKSEL = 0 (fPIN = fFD), CTQSEL = 00b (fCTQIN = fPDIV), PPPEN = 0 (fPPP=fPIN).
    USIC0_CH0->BRG &= ~(USIC_CH_BRG_PCTQ_Msk | USIC_CH_BRG_DCTQ_Msk | USIC_CH_BRG_PDIV_Msk | USIC_CH_BRG_CLKSEL_Msk | USIC_CH_BRG_PPPEN_Msk);
    USIC0_CH0->BRG |= (BRG_PCTQ << USIC_CH_BRG_PCTQ_Pos) | (BRG_DCTQ << USIC_CH_BRG_DCTQ_Pos) | (BRG_PDIV << USIC_CH_BRG_PDIV_Pos);

    // Configuration of USIC Shift Control:
    // SCTR.FLE = 8 (Frame Length).
    // SCTR.WLE = 8 (Word Length).
    // SCTR.TRM = 1 (Transmission Mode).
    // SCTR.PDL = 1 (This bit defines the output level at the shift data output signal when no data is available for transmission).
    USIC0_CH0->SCTR &= ~(USIC_CH_SCTR_TRM_Msk | USIC_CH_SCTR_FLE_Msk | USIC_CH_SCTR_WLE_Msk);
    USIC0_CH0->SCTR |= USIC_CH_SCTR_PDL_Msk | (0x01UL << USIC_CH_SCTR_TRM_Pos) | (0x07UL << USIC_CH_SCTR_FLE_Pos) | (0x07UL << USIC_CH_SCTR_WLE_Pos);

    // Configuration of USIC Transmit Control/Status Register:
    // TBUF.TDEN = 1 (TBUF Data Enable: A transmission of the data word in TBUF can be started if TDV = 1.
    // TBUF.TDSSM = 1 (Data Single Shot Mode: allow word-by-word data transmission which avoid sending the same data several times.
    USIC0_CH0->TCSR &= ~(USIC_CH_TCSR_TDEN_Msk);
    USIC0_CH0->TCSR |= USIC_CH_TCSR_TDSSM_Msk | (0x01UL << USIC_CH_TCSR_TDEN_Pos);

    // Configuration of Protocol Control Register:
    // PCR.SMD = 1 (Sample Mode based on majority).
    // PCR.STPB = 0 (1x Stop bit).
    // PCR.SP = 5 (Sample Point).
    // PCR.PL = 0 (Pulse Length is equal to the bit length).
    USIC0_CH0->PCR &= ~(USIC_CH_PCR_ASCMode_STPB_Msk | USIC_CH_PCR_ASCMode_SP_Msk | USIC_CH_PCR_ASCMode_PL_Msk);
    USIC0_CH0->PCR |= USIC_CH_PCR_ASCMode_SMD_Msk | (9 << USIC_CH_PCR_ASCMode_SP_Pos);

    // Configure Transmit Buffer:
    // Standard transmit buffer event is enabled.
    // Define start entry of Transmit Data FIFO buffer DPTR = 0.
    USIC0_CH0->TBCTR &= ~(USIC_CH_TBCTR_SIZE_Msk | USIC_CH_TBCTR_DPTR_Msk);

    // Set Transmit Data Buffer size and set data pointer to position 0.
    USIC0_CH0->TBCTR |= ((UART_FIFO_SIZE << USIC_CH_TBCTR_SIZE_Pos)|(0x00 << USIC_CH_TBCTR_DPTR_Pos));

    // Init UART_RX (P1.4 --> DX5E, or P1.5 --> DOUT0):
    XMC_GPIO_SetMode(P1_4, XMC_GPIO_MODE_INPUT_TRISTATE);
    USIC0_CH0->DX0CR |= 0x00000016;      // USIC0_CH0.DX0E <-- P1.4.
    USIC0_CH0->DX3CR |= 0x00000015;      // USIC0_CH0.DX3E <-- P1.4.
    USIC0_CH0->DX5CR |= 0x00000014;      // USIC0_CH0.DX5E <-- P1.4.

    // Configure Receive Buffer:
    // Standard Receive buffer event is enabled.
    // Define start entry of Receive Data FIFO buffer DPTR.
    USIC0_CH0->RBCTR &= ~(USIC_CH_RBCTR_SIZE_Msk | USIC_CH_RBCTR_DPTR_Msk);

    // Set Receive Data Buffer Size and set data pointer to position max.
    USIC0_CH0->RBCTR |= ((UART_FIFO_SIZE << USIC_CH_RBCTR_SIZE_Pos)|((1<<UART_FIFO_SIZE) << USIC_CH_RBCTR_DPTR_Pos));

    // Init UART_TX (P1.2 --> DOUT):
    XMC_GPIO_SetMode(P1_5, XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2);

    // Configuration of Channel Control Register:
    // CCR.PM = 00 ( Disable parity generation).
    // CCR.MODE = 2 (ASC mode enabled).
    USIC0_CH0->CCR &= ~(USIC_CH_CCR_PM_Msk | USIC_CH_CCR_MODE_Msk);
    USIC0_CH0->CCR |= 0x02UL << USIC_CH_CCR_MODE_Pos;
}
#endif

