/**
 *************************************************************************
 * @file        dma.c
 * @brief       DMAC-B driver implementation for TMPM4Ky RISC microcontrollers.
 * @version     V1.0.0
 * @date        01-06-2026
 * 
 * 
 * @details
 *  This driver provides initialization and control functions for the ADC-I
 *  https://toshiba.semicon-storage.com/info/TXZP-PINFO-M4K(2)_en_20231225.pdf?did=70854
 *  https://toshiba.semicon-storage.com/info/RM-DMAC-B_en_20241031.pdf?did=160537
 *  https://developer.arm.com/documentation/ddi0417/a/Functional-Overview/Functional-operation/DMA-control
 *  
 *  Target: Burst Mode only availiable for DMA for ADC
 *  ARM PrimeCell DMA (PL230) Architecture
 *  Refer to 5.1. Basic Operation in DMAC-B datasheet
 * Copyright (c) [Kevin Le] 2026
 **************************************************************************
 */

/* Inclusions */
#include "dma.h"

/* 3.3.2. Preparation of Channel Control Data
 * 1. Size is 64 entries (32 Primary + 32 Alternate) = 1024 bytes total. 
 * 32 (DMA Channel #) * 2U   ->   64 * 16 bytes (from sturct DMA_ChnlCtrlData_TypeDef) = 1024 bytes
 * 2. Alignment is 1024 bytes (2^10) because CTRLBASEPTR ignores bits [9:0]. Ex: Make it a multiple for 0x400 (1024 bytes)
 */
__attribute__((aligned(1024))) static DMA_ChnlCtrlData_TypeDef DMA_CtrlTable[DMA_CHANNEL_COUNT * 2U];


/* Define necessary variables */
/* Buffers for ADC results - placed in RAM for DMA access (2 Bytes)*/
static volatile uint16_t adc_a_buffer[2];               /* AINA16, AINA15 */
static volatile uint16_t adc_c_buffer[2];               /* AINC01, AINC00 */

void DMAC_Init(void)
{
    /* [1] Clock Enable (Section 3.3.1) */
    TSB_CG->FSYSMENB |= DMAC_CG_FSYSMENB_IPMENB17;

    /* [2] Preparation of Channel Control Data (Section 3.3.2 / 4.2.3) */
    /* Tell the DMA controller where your 1024-byte aligned array is located */
    TSB_DMAA->CTRLBASEPTR = (uint32_t)DMA_CtrlTable;

    /* [3] Common Initialization of Register (Section 3.3.3) */
    /* The datasheet mandates these three writes for all units first: */
    TSB_DMAA->CFG = DMAxCfg_Master_enable;                  // Enable DMA master operation
    TSB_DMAA->CHNLREQMASKSET = DMAxChnlReqMaskSet_MASK;     // Mask all requests initially (0xFFFFFFFF)
    TSB_DMAA->CHNLENABLESET = DMAxChnlEnableSet_MASK;       // Set for make the corresponding channel valid.

    /* [4] Un-mask specific channels (Section 3.9) */
    /* Clear mask to allow DMA requests from ADC Unit A (ch16) and Unit C (ch18) */
    TSB_DMAA->CHNLREQMASKCLR = (DMA_CHANNEL_MASK(DMA_ADASLG_DMAREQ) | 
                                DMA_CHANNEL_MASK(DMA_ADCSLG_DMAREQ));     

    /* [5] Enable Burst Transmission (Section 4.2.6 / 4.2.7) */
    /* Force burst mode for ADC channels  */
    TSB_DMAA->CHNLUSEBURSTSET = (DMA_CHANNEL_MASK(DMA_ADASLG_DMAREQ)   // Ch 16
                           | DMA_CHANNEL_MASK(DMA_ADCSLG_DMAREQ));     // Ch 18 
}



/**
 * @brief  Set up DMA channels 16 and 18 for ADC Unit A and Unit C
 * @details
 *   ADC Unit A (AINA16, AINA15) → DMA Ch16 → adc_a_buffer[2]
 *   ADC Unit C (AINC01, AINC00) → DMA Ch18 → adc_c_buffer[2]
 *
 *   Each ADC result register is 32 bits wide, but the 12-bit result lives in 
 *   bits [15:4]. We DMA-transfer only the lower 16 bits (src_size=HWORD).
 *
 *   REG0 and REG1 are 4 bytes apart, so src_inc=WORD (+4 bytes).
 *   The buffer is uint16_t[], so dst_inc=HWORD (+2 bytes).
 *
 *   The DMA uses "end address" arithmetic:
 *     - 2 transfers, src_end=&REG1, src_inc=4:
 *       Xfer 1: &REG1 - (2-1)*4 = &REG0  (AINA16 result)
 *       Xfer 2: &REG1 - (1-1)*4 = &REG1  (AINA15 result)
 *     - 2 transfers, dst_end=&buffer[1], dst_inc=2:
 *       Xfer 1: &buffer[1] - (2-1)*2 = &buffer[0]
 *       Xfer 2: &buffer[1] - (1-1)*2 = &buffer[1]
 * 
 *   DMA_N_MINUS_1 Is used for "end address" arithmetic.
 *   All info can be found in ARM PrimeCell DMA (PL230) Architecture. (ARM website)
 *  @note After DMA cycle, cycle_ctrl enters invalid state/suspend state (Refer to ARM DMA Control) 
 */
void DMA_SetupForADC(void)
{
    /* Common ChnlCfg for both ADC units */
    uint32_t chnl_cfg = DMA_DST_INC_HWORD      /* dest: +2 bytes per transfer */
                      | DMA_DST_SIZE_HWORD     /* dest: 16-bit writes - Destination of adc_x_buffer array */
                      | DMA_SRC_INC_WORD       /* src: +4 bytes (REG0 → REG1) */
                      | DMA_SRC_SIZE_HWORD     /* src: 16-bit reads (lower half of 32-bit reg) */
                      | DMA_R_POWER_2          /* arbitrate after 2 transfers */
                      | DMA_N_MINUS_1(2)       /* 2 transfers total */
                      | DMA_NEXT_USEBURST      /* bit 3 = 1: force burst */
                      | DMA_CYCLE_CTRL_CNT;    /* cycle_ctrl = 010: Continuation Normal */

    /* Channel 16: ADC Unit A - Single Conversion DMA request (ADASGL_DMAREQ) */
    DMA_ConfigChannel(DMA_ADASLG_DMAREQ,
                     (uint32_t)&TSB_ADA->REG1,       /* Source end address */
                     (uint32_t)&adc_a_buffer[1],     /* Dest end address */
                     chnl_cfg);

    /* Channel 18: ADC Unit C - Single Conversion DMA request (ADCSGL_DMAREQ) */
    DMA_ConfigChannel(DMA_ADCSLG_DMAREQ,
                     (uint32_t)&TSB_ADC->REG1,
                     (uint32_t)&adc_c_buffer[1],
                     chnl_cfg);
}


/**
 * @brief  Configure a DMA channel's primary control data
 * @note   Writes SrcEndPtr, DstEndPtr, and ChnlCfg into the control table.
 *         The DMA controller reads this table autonomously when a request fires.
 */
void DMA_ConfigChannel(uint8_t channel, uint32_t src_end, uint32_t dst_end, uint32_t chnl_cfg)
{
    if (channel >= DMA_CHANNEL_COUNT) return;
    
    DMA_CtrlTable[channel].SrcEndPtr = src_end;
    DMA_CtrlTable[channel].DstEndPtr = dst_end;
    DMA_CtrlTable[channel].ChnlCfg   = chnl_cfg;
    /* RESERVED field at offset +12 - do NOT write to it */
}


/**
 * @brief  Get pointer to ADC Unit A DMA result buffer
 * @return volatile uint16_t* pointing to adc_a_buffer[0]
 * @note   Buffer layout: [0] = AINA16 (from REG0), [1] = AINA15 (from REG1)
 */
volatile uint16_t* DMA_GetADCABuffer(void)
{
    return adc_a_buffer;
}
/**
 * @brief  Get pointer to ADC Unit C DMA result buffer
 * @return volatile uint16_t* pointing to adc_c_buffer[0]
 * @note   Buffer layout: [0] = AINC01 (from REG0), [1] = AINC00 (from REG1)
 */
volatile uint16_t* DMA_GetADCCBuffer(void)
{
    return adc_c_buffer;
}