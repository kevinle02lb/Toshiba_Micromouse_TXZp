/**
 * @file        timer32A.c
 * @brief       Timer32A driver implementation for TMPM4Ky microcontrollers.
 * @version     V1.0.0
 * @date        11-05-2026
 *
 * @details
 *   Configures T32A channels 0 and 3 for PPG (PWM) output on motor drivers.
 *   16-bit mode, up-counting, double-buffered, 40 kHz @ 80 MHz / 1:1 prescaler.
 *
 *   Pin Assignments:
 *   - T32A0: PA3 (OUTA), PA4 (OUTB) — Left motor
 *   - T32A3: PC2 (OUTA), PC3 (OUTB) — Right motor
 *
 *   Reference:
 *   - RM-T32A-C: https://toshiba.semicon-storage.com/info/RM-T32A-C_en_20241129.pdf?did=160771
 *   - App Note:   https://toshiba.semicon-storage.com/info/COM_T32A_PPG-ANE_application_note_en_20240716.pdf?did=156401&prodName=TMPM4KNF10AFG
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "timer32A.h"

/* ==========================================================================
 *   Initialization
 * ========================================================================== */

void T32A_Init(void)
{
    T32A0_Init(T32A_CH0_PERIOD);   /* Left motor PWM */
    T32A3_Init(T32A_CH3_PERIOD);   /* Right motor PWM */
}

/**
 * @brief  Initialize T32A channel 0 for PPG output.
 * @param  period  PWM period in timer counts (e.g., 2000 = 40 kHz @ 80 MHz).
 *
 *   Setup sequence (per RM-T32A-C §5.3):
 *   1. Enable clock, stop timer
 *   2. Set 16-bit mode
 *   3. Configure prescaler, up-count, double-buffer
 *   4. Set reload = 0, RG1 = period (sets PWM frequency)
 *   5. Set RG0 = 0 (initial 0% duty)
 *   6. Configure output: SET on CMP0, CLEAR on CMP1
 *
 *   PPG constraints (up-counting):
 *     RG1 ≥ RELD + 2
 *     RELD ≤ RG0 ≤ RG1
 */
void T32A0_Init(uint16_t period)
{
    /* [1] Clock enable, stop timer */
    TSB_CG->FSYSMENA |= T32A0_CG_FSYSMENA_IPMENA28;
    TSB_T32A0->RUNA &= ~T32A_RUNx_MASK;
    while (TSB_T32A0->RUNA & T32A_RUNx_RUNFLGx_MASK) { ; }   /* Wait for stop */

    /* [2] 16-bit mode */
    TSB_T32A0->MOD &= ~T32A_MODE_MASK;
    TSB_T32A0->MOD |= (T32A_MODE32 & T32A_MODE_MASK);

    /* [3] Prescaler, direction, double-buffer */
    TSB_T32A0->CRA &= ~T32A_CRx_PRSCLC_MASK;
    TSB_T32A0->CRB &= ~T32A_CRx_PRSCLC_MASK;
    TSB_T32A0->CRA |= T32A0_CRA_PRSCLA;
    TSB_T32A0->CRB |= T32A0_CRB_PRSCLB;

    TSB_T32A0->CRA &= ~T32A_CRx_UPDNx_MASK;
    TSB_T32A0->CRA |= T32A_UPDNx;           /* Up-count */

    TSB_T32A0->CRA |= T32A_CRx_WBFx_MASK;    /* Double-buffer A */
    TSB_T32A0->CRB |= T32A_CRx_WBFx_MASK;    /* Double-buffer B */

    /* [4] Reload behavior: reload at RG1 match */
    TSB_T32A0->CRA |= T32A_CRx_RELDx_MASK;
    TSB_T32A0->CRB |= T32A_CRx_RELDx_MASK;

    TSB_T32A0->RELDA = 0;
    TSB_T32A0->RELDB = 0;

    /* [5] Compare values: RG0 = duty, RG1 = period */
    TSB_T32A0->RGA0 = 0;          /* 0% duty initially */
    TSB_T32A0->RGB0 = 0;
    TSB_T32A0->RGA1 = period;     /* Period = PWM frequency */
    TSB_T32A0->RGB1 = period;

    /* [6] Output control: SET on CMP0, CLEAR on CMP1 */
    TSB_T32A0->OUTCRA0 &= ~T32A_OUTCRx0_ORCx_MASK;
    TSB_T32A0->OUTCRB0 &= ~T32A_OUTCRx0_ORCx_MASK;

    TSB_T32A0->OUTCRA1 = (TSB_T32A0->OUTCRA1 & ~T32A_OUTCRx1_MASK) |
                         T32A_OUTPUT_PPG;
    TSB_T32A0->OUTCRB1 = (TSB_T32A0->OUTCRB1 & ~T32A_OUTCRx1_MASK) |
                         T32A_OUTPUT_PPG;
}

/**
 * @brief  Initialize T32A channel 3 for PPG output.
 * @param  period  PWM period in timer counts.
 * @note   Same sequence as T32A0_Init(). See that function for details.
 */
void T32A3_Init(uint16_t period)
{
    /* [1] Clock enable, stop timer */
    TSB_CG->FSYSMENA |= T32A3_CG_FSYSMENA_IPMENA31;
    TSB_T32A3->RUNA &= ~T32A_RUNx_MASK;
    while (TSB_T32A3->RUNA & T32A_RUNx_RUNFLGx_MASK) { ; }

    /* [2] 16-bit mode */
    TSB_T32A3->MOD &= ~T32A_MODE_MASK;
    TSB_T32A3->MOD |= (T32A_MODE32 & T32A_MODE_MASK);

    /* [3] Prescaler, direction, double-buffer */
    TSB_T32A3->CRA &= ~T32A_CRx_PRSCLC_MASK;
    TSB_T32A3->CRB &= ~T32A_CRx_PRSCLC_MASK;
    TSB_T32A3->CRA |= T32A3_CRA_PRSCLA;
    TSB_T32A3->CRB |= T32A3_CRB_PRSCLB;

    TSB_T32A3->CRA &= ~T32A_CRx_UPDNx_MASK;
    TSB_T32A3->CRA |= T32A_UPDNx;

    TSB_T32A3->CRA |= T32A_CRx_WBFx_MASK;
    TSB_T32A3->CRB |= T32A_CRx_WBFx_MASK;

    /* [4] Reload at RG1 match */
    TSB_T32A3->CRA |= T32A_CRx_RELDx_MASK;
    TSB_T32A3->CRB |= T32A_CRx_RELDx_MASK;

    TSB_T32A3->RELDA = 0;
    TSB_T32A3->RELDB = 0;

    /* [5] Compare values */
    TSB_T32A3->RGA0 = 0;
    TSB_T32A3->RGB0 = 0;
    TSB_T32A3->RGA1 = period;
    TSB_T32A3->RGB1 = period;

    /* [6] Output control: PPG mode */
    TSB_T32A3->OUTCRA0 &= ~T32A_OUTCRx0_ORCx_MASK;
    TSB_T32A3->OUTCRB0 &= ~T32A_OUTCRx0_ORCx_MASK;

    TSB_T32A3->OUTCRA1 = (TSB_T32A3->OUTCRA1 & ~T32A_OUTCRx1_MASK) |
                         T32A_OUTPUT_PPG;
    TSB_T32A3->OUTCRB1 = (TSB_T32A3->OUTCRB1 & ~T32A_OUTCRx1_MASK) |
                         T32A_OUTPUT_PPG;
}

/* ==========================================================================
 *   Run Control
 * ========================================================================== */

void T32A0_Start(void)  { TSB_T32A0->RUNA |= T32A_RUNx_MASK; }
void T32A3_Start(void)  { TSB_T32A3->RUNA |= T32A_RUNx_MASK; }
void T32A0_Stop(void)   { TSB_T32A0->RUNA &= ~T32A_RUNx_MASK; }
void T32A3_Stop(void)   { TSB_T32A3->RUNA &= ~T32A_RUNx_MASK; }

/* ==========================================================================
 *   Compare Value Setters (Duty Cycle)
 * ========================================================================== */

void T32A0_SetTimerA0(uint16_t val)  { TSB_T32A0->RGA0 = val; }   /* Left A duty */
void T32A0_SetTimerA1(uint16_t val)  { TSB_T32A0->RGA1 = val; }   /* Left A period */
void T32A0_SetTimerB0(uint16_t val)  { TSB_T32A0->RGB0 = val; }   /* Left B duty */
void T32A0_SetTimerB1(uint16_t val)  { TSB_T32A0->RGB1 = val; }   /* Left B period */

void T32A3_SetTimerA0(uint16_t val)  { TSB_T32A3->RGA0 = val; }   /* Right A duty */
void T32A3_SetTimerA1(uint16_t val)  { TSB_T32A3->RGA1 = val; }   /* Right A period */
void T32A3_SetTimerB0(uint16_t val)  { TSB_T32A3->RGB0 = val; }   /* Right B duty */
void T32A3_SetTimerB1(uint16_t val)  { TSB_T32A3->RGB1 = val; }   /* Right B period */

/* ==========================================================================
 *   Output Control Setters
 * ========================================================================== */

void T32A0_SetOutCRA1(uint32_t mode)
{
    TSB_T32A0->OUTCRA1 = (TSB_T32A0->OUTCRA1 & ~T32A_OUTCRx1_MASK) | mode;
}

void T32A0_SetOutCRB1(uint32_t mode)
{
    TSB_T32A0->OUTCRB1 = (TSB_T32A0->OUTCRB1 & ~T32A_OUTCRx1_MASK) | mode;
}

void T32A3_SetOutCRA1(uint32_t mode)
{
    TSB_T32A3->OUTCRA1 = (TSB_T32A3->OUTCRA1 & ~T32A_OUTCRx1_MASK) | mode;
}

void T32A3_SetOutCRB1(uint32_t mode)
{
    TSB_T32A3->OUTCRB1 = (TSB_T32A3->OUTCRB1 & ~T32A_OUTCRx1_MASK) | mode;
}