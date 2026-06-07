/**
 * @file        adc.c
 * @brief       ADC-I driver implementation for TMPM4Ky RISC microcontrollers.
 * @version     V1.0.0
 * @date        29-05-2026
 *
 * @details
 *   Configures ADC Units A and C for single-conversion mode with DMA.
 *   ADCLK = 160 MHz raw; SCLK = 40 MHz after /4 prescaler.
 *
 *   Pin Assignments:
 *   - Unit A: PL0 (AINA16), PL1 (AINA15) — Left IR sensors
 *   - Unit C: PJ0 (AINC00), PJ1 (AINC01) — Right IR sensors
 *
 *   Reference:
 *   - Product Info:  https://toshiba.semicon-storage.com/info/TXZP-PINFO-M4K(2)_en_20231225.pdf
 *   - ADC-I RM:     https://toshiba.semicon-storage.com/info/RM-ADC-I_en_20251205.pdf
 *   - App Note:     https://toshiba.semicon-storage.com/info/COM_ADC_MON-ANE_application_note_en_20231016.pdf
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "adc.h"
#include "systick.h"
#include "dma.h"

/* ==========================================================================
 *   Initialization
 * ========================================================================== */

/**
 * @brief  Initialize both ADC units (A and C) for single-conversion DMA.
 * @note   DMA channel control data is re-armed inside @ref Start_ADC() before
 *         each conversion pair. Do not call @ref DMA_SetupForADC() here.
 */
void ADC_Init(void)
{
    AINA_Init();
    AINC_Init();
}

/**
 * @brief  Initialize ADC Unit A (AINA15, AINA16).
 * @details
 *   Follows Section 3.2.2 of the ADC-I RM for single-conversion setup.
 *   Steps: clock enable → disable ADEN → set prescaler → configure sampling
 *   time → set mode → program TSET registers → enable DMA request → enable ADC.
 */
void AINA_Init(void)
{
    /* [1] Clock configuration */
    TSB_CG->FSYSMENB |= AINA_CG_FSYSMENB_IPMENB02;
    TSB_ADA->CR0 &= ~ADxCR0_ADEN;                       /* Disable before config */
    TSB_CG->SPCLKEN |= CG_CGSPCLKEN_ADCKEN0;            /* Enable ADCLK */
    TSB_ADA->CLK &= ~ADxCLK_VADCLK_MASK;                /* Prescaler = /4 (3b000) */
    TSB_ADA->CLK |= ADXCLK_EXAZ0_40MHZ | ADXCLK_EXAZ1_40MHZ;

    /* [2] ADC circuit configuration */
    TSB_ADA->MOD0 |= ADxMOD0_DACON;
    TSB_ADA->MOD0 &= ~ADxMOD0_RCUT;
    SysTick_us(3U);                                     /* 3 µs stabilization */

    /* Sampling & conversion time: 0.96 µs @ 40 MHz SCLK (RM Table 6-1) */
    TSB_ADA->MOD1 = ADxMOD1_40MHZ;
    TSB_ADA->MOD2 = 0;

    /* Use EXAZ0 for both channels */
    TSB_ADA->EXAZSEL &= ~(ADxEXAZSEL_AINA15 | ADxEXAZSEL_AINA16);

    /* [3] TSET — conversion program */
    TSB_ADA->TSET0 = (ADxTSETn_TRGSn_SGL | ADxTSETn_AINSTn_AINx16);          /* AINA16 → REG0 */
    TSB_ADA->TSET1 = (ADxTSETn_TRGSn_SGL | ADxTSETn_AINSTn_AINx15 |
                        ADxTSETn_ENINTn_MASK);                               /* AINA15 → REG1, INT */

    /* [4] DMA request enable */
    TSB_ADA->CR1 = ADxCR1_SGLDMEN;

    /* [5] Enable ADC */
    TSB_ADA->CR0 |= ADxCR0_ADEN;
}

/**
 * @brief  Initialize ADC Unit C (AINC00, AINC01).
 * @details  Same sequence as @ref AINA_Init(), applied to Unit C.
 */
void AINC_Init(void)
{
    /* [1] Clock configuration */
    TSB_CG->FSYSMENB |= AINC_CG_FSYSMENB_IPMENB04;
    TSB_ADC->CR0 &= ~ADxCR0_ADEN;
    TSB_CG->SPCLKEN |= CG_CGSPCLKEN_ADCKEN2;
    TSB_ADC->CLK &= ~ADxCLK_VADCLK_MASK;
    TSB_ADC->CLK |= ADXCLK_EXAZ0_40MHZ | ADXCLK_EXAZ1_40MHZ;

    /* [2] ADC circuit configuration */
    TSB_ADC->MOD0 |= ADxMOD0_DACON;
    TSB_ADC->MOD0 &= ~ADxMOD0_RCUT;
    SysTick_us(3U);

    TSB_ADC->MOD1 = ADxMOD1_40MHZ;
    TSB_ADC->MOD2 = 0;

    TSB_ADC->EXAZSEL &= ~(ADxEXAZSEL_AINC00 | ADxEXAZSEL_AINC01);

    /* [3] TSET — conversion program */
    TSB_ADC->TSET0 = (ADxTSETn_TRGSn_SGL | ADxTSETn_AINSTn_AINx01);          /* AINC01 → REG0 */
    TSB_ADC->TSET1 = (ADxTSETn_TRGSn_SGL | ADxTSETn_AINSTn_AINx00 |
                        ADxTSETn_ENINTn_MASK);                               /* AINC00 → REG1, INT */

    /* [4] DMA request enable */
    TSB_ADC->CR1 = ADxCR1_SGLDMEN;

    /* [5] Enable ADC */
    TSB_ADC->CR0 |= ADxCR0_ADEN;
}

/* ==========================================================================
 *   Conversion Control
 * ========================================================================== */

/**
 * @brief  Trigger a single conversion on ADC Unit A.
 */
void AINA_StartSGL(void)
{
    TSB_ADA->CR0 |= ADxCR0_SGL;
}

/**
 * @brief  Trigger a single conversion on ADC Unit C.
 */
void AINC_StartSGL(void)
{
    TSB_ADC->CR0 |= ADxCR0_SGL;
}

/**
 * @brief  Start a paired ADC conversion with DMA re-arm.
 * @details
 *   Must be called every time fresh samples are needed.
 *   The PL230/DMAC-B controller invalidates (cycle_ctrl = 0) the channel
 *   control structure after each transfer, so @ref DMA_SetupForADC() is
 *   invoked first to restore the descriptors.
 */
void Start_ADC(void)
{
    DMA_SetupForADC();   /* Re-arm DMA channels (restore cycle_ctrl) */
    AINA_StartSGL();
    AINC_StartSGL();
}

/* ==========================================================================
 *   Blocking Read (DMA bypass / debug fallback)
 * ========================================================================== */

/**
 * @brief  Blocking read from ADC Unit A (no DMA).
 * @param  channel  15 for AINA15, 16 for AINA16.
 * @return 12-bit result right-aligned in a 16-bit word.
 * @note   Polls ADxST.SNGF; do not use while DMA is active on the same unit.
 */
uint16_t AINA_Read(uint8_t channel)
{
    uint16_t result = 0;

    AINA_StartSGL();
    while (TSB_ADA->ST & ADxST_SNGF) { ; }

    if (channel == 16) {
        result = (uint16_t)(TSB_ADA->REG0 & ADxREGn_ADRn);
    } else if (channel == 15) {
        result = (uint16_t)(TSB_ADA->REG1 & ADxREGn_ADRn);
    }
    return result;
}

/**
 * @brief  Blocking read from ADC Unit C (no DMA).
 * @param  channel  0 for AINC00, 1 for AINC01.
 * @return 12-bit result right-aligned in a 16-bit word.
 * @note   Polls ADxST.SNGF; do not use while DMA is active on the same unit.
 */
uint16_t AINC_Read(uint8_t channel)
{
    uint16_t result = 0;

    AINC_StartSGL();
    while (TSB_ADC->ST & ADxST_SNGF) { ; }

    if (channel == 1) {
        result = (uint16_t)(TSB_ADC->REG0 & ADxREGn_ADRn);
    } else if (channel == 0) {
        result = (uint16_t)(TSB_ADC->REG1 & ADxREGn_ADRn);
    }
    return result;
}

/* ==========================================================================
 *   Interrupt Handlers
 * ========================================================================== */

/**
 * @brief  ADC Unit A single-conversion interrupt handler.
 * @note   Currently a stub. DMA handles the data movement; no CPU action required.
 */
void INTADASGL_IRQHandler(void)
{
    /* Add acknowledge / flag-clear if needed by application layer */
}

/**
 * @brief  ADC Unit C single-conversion interrupt handler.
 * @note   Currently a stub. DMA handles the data movement; no CPU action required.
 */
void INTADCSGL_IRQHandler(void)
{
    /* Add acknowledge / flag-clear if needed by application layer */
}