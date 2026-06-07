/**
 *************************************************************************
 * @file        adc.c
 * @brief       ADC-I driver implementation for TMPM4Ky RISC microcontrollers.
 * @version     V1.0.0
 * @date        29-05-2026
 * 
 * 
 * @details
 *  This driver provides initialization and control functions for the ADC-I
 *  https://toshiba.semicon-storage.com/info/TXZP-PINFO-M4K(2)_en_20231225.pdf?did=70854
 *  https://toshiba.semicon-storage.com/info/RM-ADC-I_en_20251205.pdf?did=166835
 *  https://toshiba.semicon-storage.com/info/COM_ADC_MON-ANE_application_note_en_20231016.pdf?did=156383&prodName=TMPM4KNF10AFG
 *  
 *  Example of Usage: 6.1. Single Conversion
 * Copyright (c) [Kevin Le] 2026
 **************************************************************************
 */

/* Assignments
 *  Unit A & C:
 * - PL0, PL1 	 Left IR ADC inputs (AINA16, AINA15)
 * - PJ1, PJ0 	 Right IR ADC inputs (AINC01, AINC00)
 * 
 * Check Product Info. for ADC
 * 
 * Target: Single Conversion ADC
 * Use DMA
 * */

/* Inclusions */
#include "adc.h"   
#include "systick.h"
#include "dma.h"   


void ADC_Init(void)
{
    AINA_Init();        // With DMA
    AINC_Init();        // With DMA
    /* ADC DMA */
    DMA_SetupForADC();
}

/**
 * @brief        Sets up Unit/Channel A ADC      
 * @details      Sets up ADC Inputs pins 15 & 16
 *               Follow 3.2.2. Control Registers setup operation for Single Conversion. No General Purpose Trig
 *               ADCLK is raw 160Mhz. Not Fsysm (Check CG Datasheet Diagram)
 */
void AINA_Init(void)
{
    /* [1] CLOCK CONFIGURATION */
    
    /* [1.1] Enable fsysm clock for register access */
    TSB_CG->FSYSMENB |= AINA_CG_FSYSMENB_IPMENB02;

    /* [1.2] Disable Conversion before other ADx R/W */
    TSB_ADA->CR0 &= ~ADxCR0_ADEN;
    
    /* [1.3] Enable ADC conversion clock (ADCLK = 160MHz raw) */
    TSB_CG->SPCLKEN |= CG_CGSPCLKEN_ADCKEN0;
    
    /* [1.4] Configure prescaler */
    /* Ex: 3b000 = ADCLK/4. Must be done BEFORE ADEN=1 */
    TSB_ADA->CLK &= ~ADxCLK_VADCLK_MASK;

    /* [1.5] Sampling Time Clock Config */
    TSB_ADA->CLK |= ADXCLK_EXAZ0_40MHZ | ADXCLK_EXAZ1_40MHZ;

    /* [2] AD converter Ckt. Config */
    /* [2.1] Mode Config*/
    TSB_ADA->MOD0 |= ADxMOD0_DACON;                     // Set for ADC
    TSB_ADA->MOD0 &= ~ADxMOD0_RCUT;                     // Clear for normal mode 
    SysTick_us(3U);                                     // wait for 3 uS 

    /* [2.2] Sampling & Conversion Time per Datasheet Recommendation
     * Sampling time = SCLK period × m × n
     * (m: <EXAZ0[3:0]> or <EXAZ1[3:0]> setting, n: <MOD1[31:0]> setting)
     * 
     * Target: kHz sampling rate for IR Sensors
     * Use Example for AD conversion time 0.96[µs] at ADCLK 160[Mhz] w/ 40[Mhz] SCLK
     */
    TSB_ADA->MOD1 = ADxMOD1_40MHZ;
    TSB_ADA->MOD2 = 0;

    /* [2.3] Selection of AIN sampling time setting. Each bit corresponds to the AIN channel.
     * 0: [ADxCLK]<EXAZ0[3:0]> setting is used
     * 1: [ADxCLK]<EXAZ1[3:0]> setting is used
     * 
     * Use EXAZ0
     */
    TSB_ADA->EXAZSEL &= ~(ADxEXAZSEL_AINA15 | ADxEXAZSEL_AINA16);

    
    /* [3] General Purpose Start-up Factor Program Register - Conversion program setting */
    TSB_ADA->TSET0 = (ADxTSETn_TRGSn_SGL | ADxTSETn_AINSTn_AINx16);                                   //AINA16
    TSB_ADA->TSET1 = (ADxTSETn_TRGSn_SGL | ADxTSETn_AINSTn_AINx15 | ADxTSETn_ENINTn_MASK);            //AINA15 - Interrupt Enable

    /* [4] D.M.A Request */
    TSB_ADA->CR1 = ADxCR1_SGLDMEN;

    /* [5] Enable Conversion */
    TSB_ADA->CR0 |= ADxCR0_ADEN;
}

/**
 * @brief        Sets up Unit/Channel C ADC   
 * @details      Sets up ADC Inputs pins 0 & 1
 */
void AINC_Init(void)
{
    /* [1:] CLOCK CONFIGURATION */
    /* [1.1] Enable fsysm clock for register access */
    TSB_CG->FSYSMENB |= AINC_CG_FSYSMENB_IPMENB04;

    /* [1.2] Disable Conversion before other ADx R/W */
    TSB_ADC->CR0 &= ~ADxCR0_ADEN;

    /* [1.3] Enable ADC conversion clock */
    TSB_CG->SPCLKEN |= CG_CGSPCLKEN_ADCKEN2;

    /* [1.4] Configure prescaler: 160MHz/4 = 40MHz */
    TSB_ADC->CLK &= ~ADxCLK_VADCLK_MASK;

    /* [1.5] Sampling & Conversion Time */
    TSB_ADC->CLK |= ADXCLK_EXAZ0_40MHZ | ADXCLK_EXAZ1_40MHZ;


    /* [2:] AD converter Ckt. Config */
    /* [2.1] Mode Config */
    TSB_ADC->MOD0 |= ADxMOD0_DACON;
    TSB_ADC->MOD0 &= ~ADxMOD0_RCUT;
    SysTick_us(3U);                                     // wait for 3 uS 

    /* [2.2] Sampling & Conversion Time per Datasheet Recommendation */
    TSB_ADC->MOD1 = ADxMOD1_40MHZ;
    TSB_ADC->MOD2 = 0;

    /* [2.3] Selection of AIN sampling time setting */
    TSB_ADC->EXAZSEL &= ~(ADxEXAZSEL_AINC00 | ADxEXAZSEL_AINC01);


    /* [3:] General Purpose Start-up Factor Program Register */
    /* Configure TSET0 for AINC01 */
    TSB_ADC->TSET0 = (ADxTSETn_TRGSn_SGL | ADxTSETn_AINSTn_AINx01);
    
    /* Configure TSET1 for AINC00 */
    TSB_ADC->TSET1 = (ADxTSETn_TRGSn_SGL | ADxTSETn_AINSTn_AINx00 | ADxTSETn_ENINTn_MASK);      // Interrupt Enable

    /* [4] D.M.A Request */
    TSB_ADC->CR1 = ADxCR1_SGLDMEN;

    /* [5] Enable Conversion */
    TSB_ADC->CR0 |= ADxCR0_ADEN;
}


void AINA_StartSGL(void)
{
    TSB_ADA->CR0 |= ADxCR0_SGL;
}
void AINC_StartSGL(void)
{
    TSB_ADC->CR0 |= ADxCR0_SGL;
}


/* Start a conversion — DMA will automatically transfer results when done */
void Start_ADC(void)
{
    DMA_SetupForADC();  /* Re-arm DMA */
    AINA_StartSGL();    /* Writes ADACR0.SGL = 1 */
    AINC_StartSGL();    /* Writes ADCCR0.SGL = 1 */
}


/**
 *  @brief Read ADC values only when DMA is not used
 *  @details Returns 16 bit value from 12 bit ADC reg through bitmasking
 */
uint16_t AINA_Read(uint8_t channel)
{
    uint16_t result = 0;
    
    /* Start single conversion for AINA16 */
    AINA_StartSGL();
    /* Wait for conversion complete (check ST register or poll) */
    while (TSB_ADA->ST & ADxST_SNGF) 
    { ; }

    // Read Values
    if (channel == 16) 
    {
        /* TSET0 maps to REG0 */
        result = (uint16_t)(TSB_ADA->REG0 & ADxREGn_ADRn);
    }
    else if (channel == 15) 
    {
        /* TSET1 maps to REG1 */
        result = (uint16_t)(TSB_ADA->REG1 & ADxREGn_ADRn);
    }
    
    return result;
}
uint16_t AINC_Read(uint8_t channel)
{
    uint16_t result = 0;
    
    /* Start single conversion for AINA16 */
    AINC_StartSGL();
    /* Wait for conversion complete (check ST register or poll) */
    while (TSB_ADC->ST & ADxST_SNGF) 
    { ; }

    // Read Values
    if (channel == 1) 
    {
        /* TSET0 maps to REG0 */
        result = (uint16_t)(TSB_ADC->REG0 & ADxREGn_ADRn);
    }
    else if (channel == 0) 
    {
        /* TSET1 maps to REG1 */
        result = (uint16_t)(TSB_ADC->REG1 & ADxREGn_ADRn);
    }
    
    return result;
}



/* IRQ Handler */
void INTADASGL_IRQHandler(void)
{
    /* Acknowledge */
}
void INTADCSGL_IRQHandler(void)
{
    /* Acknowledge */
}


// Will be transfered to IR sensor module instead of driver file
// void process_results(void)
// {
//     volatile uint16_t* a_buf = DMA_GetADCABuffer();
//     volatile uint16_t* c_buf = DMA_GetADCCBuffer();
    
//     uint16_t aina16 = (a_buf[0] >> 4) & 0xFFF;  /* Result is in bits [15:4] */
//     uint16_t aina15 = (a_buf[1] >> 4) & 0xFFF;
//     uint16_t ainc01 = (c_buf[0] >> 4) & 0xFFF;
//     uint16_t ainc00 = (c_buf[1] >> 4) & 0xFFF;
// }