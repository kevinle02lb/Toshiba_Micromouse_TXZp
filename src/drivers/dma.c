/**
 * @file        dma.c
 * @brief       DMAC-B driver implementation for TMPM4Ky RISC microcontrollers.
 * @version     V1.0.0
 * @date        01-06-2026
 *
 * @details
 *   Configures the DMAC-B (PL230-compatible) for ADC burst transfers.
 *   The control table must reside in RAM and be 1024-byte aligned because
 *   CTRLBASEPTR ignores bits [9:0].
 *
 *   Reference:
 *   - DMAC-B RM:    https://toshiba.semicon-storage.com/info/RM-DMAC-B_en_20241031.pdf?did=160537
 *   - ARM PL230:    https://developer.arm.com/documentation/ddi0417/a?lang=en
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "dma.h"

/* ==========================================================================
 *   Control Table (1024-byte aligned)
 * ========================================================================== */
/**
 * @brief  Primary + alternate control table (32 channels × 2 = 64 entries).
 * @note   64 entries × 16 bytes = 1024 bytes. Alignment = 1024 because
 *         CTRLBASEPTR masks off bits [9:0].
 */
__attribute__((aligned(1024))) static DMA_ChnlCtrlData_TypeDef DMA_CtrlTable[DMA_CHANNEL_COUNT * 2U];

/* ==========================================================================
 *   ADC Result Buffers
 * ========================================================================== */
static volatile uint16_t adc_a_buffer[2];   /*!< AINA16 [0], AINA15 [1] */
static volatile uint16_t adc_c_buffer[2];   /*!< AINC01 [0], AINC00 [1] */

/* ==========================================================================
 *   Initialization
 * ========================================================================== */

/**
 * @brief  Initialize the DMAC-B controller.
 * @details
 *   1. Enable peripheral clock.
 *   2. Point CTRLBASEPTR to the aligned control table.
 *   3. Enable DMA master, mask all channels, then enable all channels.
 *   4. Unmask ADC request lines (ch16, ch18).
 *   5. Force burst mode for ADC channels.
 */
void DMAC_Init(void)
{
    /* [1] Clock enable */
    TSB_CG->FSYSMENB |= DMAC_CG_FSYSMENB_IPMENB17;

    /* [2] Control table base address */
    TSB_DMAA->CTRLBASEPTR = (uint32_t)DMA_CtrlTable;

    /* [3] Common init: master enable, mask all, then enable all */
    TSB_DMAA->CFG = DMAxCfg_Master_enable;
    TSB_DMAA->CHNLREQMASKSET = DMAxChnlReqMaskSet_MASK;
    TSB_DMAA->CHNLENABLESET = DMAxChnlEnableSet_MASK;

    /* [4] Unmask ADC DMA requests */
    TSB_DMAA->CHNLREQMASKCLR = (DMA_CHANNEL_MASK(DMA_ADASLG_DMAREQ) |
                                DMA_CHANNEL_MASK(DMA_ADCSLG_DMAREQ));

    /* [5] Force burst for ADC channels */
    TSB_DMAA->CHNLUSEBURSTSET = (DMA_CHANNEL_MASK(DMA_ADASLG_DMAREQ) |
                                 DMA_CHANNEL_MASK(DMA_ADCSLG_DMAREQ));
}

/* ==========================================================================
 *   ADC Channel Setup
 * ========================================================================== */

/**
 * @brief  Re-arm DMA channels 16 and 18 for ADC transfer.
 * @details
 *   Must be called before every ADC sample because the PL230 controller
 *   auto-clears cycle_ctrl to 0b000 (Invalid/Stop) after each cycle.
 *
 *   Transfer configuration:
 *   - Source: ADC result registers (REG0, REG1) — 32-bit apart, 16-bit read.
 *   - Dest:   uint16_t buffers — 16-bit apart, 16-bit write.
 *   - Mode:   Auto-request (CNT), 2 transfers, arbitrate every 2.
 *
 *   End-pointer arithmetic (PL230):
 *   - 2 transfers, src_end = &REG1, src_inc = WORD (+4):
 *       xfer 1: &REG1 - (1 × 4) = &REG0
 *       xfer 2: &REG1 - (0 × 4) = &REG1
 *   - 2 transfers, dst_end = &buffer[1], dst_inc = HWORD (+2):
 *       xfer 1: &buffer[1] - (1 × 2) = &buffer[0]
 *       xfer 2: &buffer[1] - (0 × 2) = &buffer[1]
 */
void DMA_SetupForADC(void)
{
    /* Common ChnlCfg for both ADC units */
    uint32_t chnl_cfg = DMA_DST_INC_HWORD      /* dest: +2 bytes per transfer */
                      | DMA_DST_SIZE_HWORD     /* dest: 16-bit writes */
                      | DMA_SRC_INC_WORD       /* src: +4 bytes (REG0 → REG1) */
                      | DMA_SRC_SIZE_HWORD     /* src: 16-bit reads (lower half of 32-bit reg) */
                      | DMA_R_POWER_2          /* arbitrate after 2 transfers */
                      | DMA_N_MINUS_1(2)       /* 2 transfers total */
                      | DMA_NEXT_USEBURST      /* bit 3 = 1: force burst */
                      | DMA_CYCLE_CTRL_CNT;    /* cycle_ctrl = 010: Auto-request / Continuous Normal */

    /* Channel 16: ADC Unit A → adc_a_buffer */
    DMA_ConfigChannel(DMA_ADASLG_DMAREQ,
                      (uint32_t)&TSB_ADA->REG1,
                      (uint32_t)&adc_a_buffer[1],
                      chnl_cfg);

    /* Channel 18: ADC Unit C → adc_c_buffer */
    DMA_ConfigChannel(DMA_ADCSLG_DMAREQ,
                      (uint32_t)&TSB_ADC->REG1,
                      (uint32_t)&adc_c_buffer[1],
                      chnl_cfg);
}

/* ==========================================================================
 *   Low-Level Channel Configuration
 * ========================================================================== */

/**
 * @brief  Write a single channel's primary control data.
 * @param  channel   DMA channel number (0–31).
 * @param  src_end   Source end address (used for end-pointer arithmetic).
 * @param  dst_end   Destination end address.
 * @param  chnl_cfg  32-bit DMAChnlCfg word.
 * @note   Does NOT write the RESERVED word at offset +12.
 */
void DMA_ConfigChannel(uint8_t channel, uint32_t src_end, uint32_t dst_end, uint32_t chnl_cfg)
{
    if (channel >= DMA_CHANNEL_COUNT) {
        return;
    }

    DMA_CtrlTable[channel].SrcEndPtr = src_end;
    DMA_CtrlTable[channel].DstEndPtr = dst_end;
    DMA_CtrlTable[channel].ChnlCfg   = chnl_cfg;
}

/* ==========================================================================
 *   Buffer Accessors
 * ========================================================================== */

/**
 * @brief  Get pointer to ADC Unit A DMA result buffer.
 * @return volatile uint16_t*  Layout: [0]=AINA16, [1]=AINA15.
 */
volatile uint16_t* DMA_GetADCABuffer(void)
{
    return adc_a_buffer;
}

/**
 * @brief  Get pointer to ADC Unit C DMA result buffer.
 * @return volatile uint16_t*  Layout: [0]=AINC01, [1]=AINC00.
 */
volatile uint16_t* DMA_GetADCCBuffer(void)
{
    return adc_c_buffer;
}