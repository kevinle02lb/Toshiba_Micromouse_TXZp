/**
 * @file        gpio.c
 * @brief       GPIO driver implementation for TMPM4Ky microcontrollers.
 * @version     V1.0.0
 * @date        11-05-2026
 *
 * @details
 *   Pin configuration for motor drivers, encoders, ADC inputs, and IR emitters.
 *
 *   Pin Map:
 *   - Motor PWM (T32A):  PA3/PA4 (Left), PC2/PC3 (Right)
 *   - Encoders:          PN0/PN1 (Left ENC0), PD3/PD4 (Right ENC2)
 *   - ADC (AIN):         PL0/PL1 (Unit A), PJ0/PJ1 (Unit C)
 *   - IR Emitters:       PU0/PU1 (Left), PG4/PG5 (Right)
 *
 *   Reference:
 *   - PORT-M4K(2): https://toshiba.semicon-storage.com/info/TXZP-PORT-M4K(2)_en_20250620.pdf?did=70850
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "gpio.h"

/* ==========================================================================
 *   Initialization
 * ========================================================================== */

void GPIO_Init(void)
{
    PORT_A_Init();   /* Motor PWM (T32A ch0) */
    PORT_C_Init();   /* Motor PWM (T32A ch3) */
    PORT_D_Init();   /* Encoder inputs (ENC2) */
    PORT_G_Init();   /* IR emitters (Right) */
    PORT_J_Init();   /* ADC inputs (Unit C) */
    PORT_L_Init();   /* ADC inputs (Unit A) */
    PORT_N_Init();   /* Encoder inputs (ENC0) */
    PORT_U_Init();   /* IR emitters (Left) */
}

/* ==========================================================================
 *   Port-Specific Init
 * ========================================================================== */

void PORT_A_Init(void)
{
    TSB_CG->FSYSMENA |= CG_PORTA;

    /* PA3 = T32A00OUTA, PA4 = T32A00OUTB — PWM outputs */
    TSB_PA->CR |= Px3_MASK | Px4_MASK;
    PA_FRn_Clear(Px3_MASK | Px4_MASK);
    TSB_PA->FR4 |= Px3_MASK | Px4_MASK;   /* FR4 = T32A00 function */
}

void PORT_C_Init(void)
{
    TSB_CG->FSYSMENA |= CG_PORTC;

    /* PC2 = T32A30OUTA, PC3 = T32A30OUTB — PWM outputs */
    TSB_PC->CR |= Px2_MASK | Px3_MASK;
    PC_FRn_Clear(Px2_MASK | Px3_MASK);
    TSB_PC->FR5 |= Px2_MASK | Px3_MASK;   /* FR5 = T32A30 function */
}

void PORT_D_Init(void)
{
    TSB_CG->FSYSMENA |= CG_PORTD;

    /* PD3 = ENC2A, PD4 = ENC2B — encoder inputs with pull-up */
    TSB_PD->IE |= Px3_MASK | Px4_MASK;
    PD_FRn_Clear(Px3_MASK | Px4_MASK);
    TSB_PD->FR6 |= Px3_MASK | Px4_MASK;   /* FR6 = ENC2 function */
    TSB_PD->PDN &= ~(Px3_MASK | Px4_MASK);
    TSB_PD->PUP |= Px3_MASK | Px4_MASK;
}

void PORT_N_Init(void)
{
    TSB_CG->FSYSMENA |= CG_PORTN;

    /* PN0 = ENC0A, PN1 = ENC0B — encoder inputs with pull-up */
    TSB_PN->IE |= Px0_MASK | Px1_MASK;
    PN_FRn_Clear(Px0_MASK | Px1_MASK);
    TSB_PN->FR6 |= Px0_MASK | Px1_MASK;   /* FR6 = ENC0 function */
    TSB_PN->PDN &= ~(Px0_MASK | Px1_MASK);
    TSB_PN->PUP |= Px0_MASK | Px1_MASK;
}

void PORT_G_Init(void)
{
    TSB_CG->FSYSMENA |= CG_PORTG;

    /* PG4/PG5 = IR emitters (Right) — regular GPIO outputs */
    PG_FRn_Clear(Px4_MASK | Px5_MASK);
    TSB_PG->CR |= Px4_MASK | Px5_MASK;
    TSB_PG->DATA = 0;
}

void PORT_U_Init(void)
{
    TSB_CG->FSYSMENA |= CG_PORTU;

    /* PU0/PU1 = IR emitters (Left) — regular GPIO outputs */
    PU_FRn_Clear(Px0_MASK | Px1_MASK);
    TSB_PU->CR |= Px0_MASK | Px1_MASK;
    TSB_PU->DATA = 0;
}

void PORT_J_Init(void)
{
    TSB_CG->FSYSMENA |= CG_PORTJ;

    /* PJ0 = AINC00, PJ1 = AINC01 — analog inputs
     * Per PORT-M4K(2) §4.2.10: disable CR, IE, PUP, PDN for analog mode */
    TSB_PJ->CR  &= ~(Px0_MASK | Px1_MASK);
    TSB_PJ->IE  &= ~(Px0_MASK | Px1_MASK);
    TSB_PJ->PUP &= ~(Px0_MASK | Px1_MASK);
    TSB_PJ->PDN &= ~(Px0_MASK | Px1_MASK);
}

void PORT_L_Init(void)
{
    TSB_CG->FSYSMENA |= CG_PORTL;

    /* PL0 = AINA16, PL1 = AINA15 — analog inputs
     * Per PORT-M4K(2) §4.2.12: disable CR, IE, PUP, PDN for analog mode */
    TSB_PL->CR  &= ~(Px0_MASK | Px1_MASK);
    TSB_PL->IE  &= ~(Px0_MASK | Px1_MASK);
    TSB_PL->PUP &= ~(Px0_MASK | Px1_MASK);
    TSB_PL->PDN &= ~(Px0_MASK | Px1_MASK);
}

/* ==========================================================================
 *   FRn Clear Helpers
 * ========================================================================== */

void PA_FRn_Clear(uint32_t pin_mask)
{
    TSB_PA->FR1 &= ~pin_mask; TSB_PA->FR4 &= ~pin_mask;
    TSB_PA->FR5 &= ~pin_mask; TSB_PA->FR6 &= ~pin_mask;
    TSB_PA->FR7 &= ~pin_mask;
    /* FR2/FR3 reserved for Port A */
}

void PC_FRn_Clear(uint32_t pin_mask)
{
    TSB_PC->FR1 &= ~pin_mask; TSB_PC->FR2 &= ~pin_mask;
    TSB_PC->FR3 &= ~pin_mask; TSB_PC->FR4 &= ~pin_mask;
    TSB_PC->FR5 &= ~pin_mask; TSB_PC->FR6 &= ~pin_mask;
    TSB_PC->FR7 &= ~pin_mask;
}

void PD_FRn_Clear(uint32_t pin_mask)
{
    TSB_PD->FR1 &= ~pin_mask; TSB_PD->FR2 &= ~pin_mask;
    TSB_PD->FR3 &= ~pin_mask; TSB_PD->FR4 &= ~pin_mask;
    TSB_PD->FR5 &= ~pin_mask; TSB_PD->FR6 &= ~pin_mask;
    /* FR7 reserved for Port D */
}

void PG_FRn_Clear(uint32_t pin_mask)
{
    TSB_PG->FR1 &= ~pin_mask;
    TSB_PG->FR4 &= ~pin_mask; TSB_PG->FR5 &= ~pin_mask;
    /* FR2, FR3, FR6, FR7 reserved for Port G */
}

void PN_FRn_Clear(uint32_t pin_mask)
{
    TSB_PN->FR1 &= ~pin_mask; TSB_PN->FR2 &= ~pin_mask;
    TSB_PN->FR3 &= ~pin_mask; TSB_PN->FR4 &= ~pin_mask;
    TSB_PN->FR5 &= ~pin_mask; TSB_PN->FR6 &= ~pin_mask;
    TSB_PN->FR7 &= ~pin_mask;
}

void PU_FRn_Clear(uint32_t pin_mask)
{
    TSB_PU->FR1 &= ~pin_mask; TSB_PU->FR2 &= ~pin_mask;
    TSB_PU->FR3 &= ~pin_mask; TSB_PU->FR4 &= ~pin_mask;
    TSB_PU->FR5 &= ~pin_mask; TSB_PU->FR6 &= ~pin_mask;
    TSB_PU->FR7 &= ~pin_mask;
}


/* ==========================================================================
 *   Port Logic Controls
 * ========================================================================== */
void GPIO_G_SetData(uint8_t data)
{
    TSB_PG->DATA |= data;
}
void GPIO_U_SetData(uint8_t data)
{
    TSB_PU->DATA |= data;
}

void GPIO_G_ClrData(uint8_t data)
{
    TSB_PG->DATA &= ~data;
}
void GPIO_U_ClrData(uint8_t data)
{
    TSB_PU->DATA &= ~data;
}

void GPIO_G_ToggleData(uint8_t data)
{
    TSB_PG->DATA ^= data;
}
void GPIO_U_ToggleData(uint8_t data)
{
    TSB_PU->DATA ^= data;
}