/**
 *************************************************************************
 * @file        dma.h
 * @brief       DMAC-B driver implementation for TMPM4Ky RISC microcontrollers.
 * @version     V1.0.0
 * @date        01-06-2026
 * 
 * 
 * @details
 *  This driver provides initialization and control functions for the ADC-I
 *  https://toshiba.semicon-storage.com/info/TXZP-PINFO-M4K(2)_en_20231225.pdf?did=70854
 *  https://toshiba.semicon-storage.com/info/RM-DMAC-B_en_20241031.pdf?did=160537
 *  
 *  
 * Copyright (c) [Kevin Le] 2026
 **************************************************************************
 */


#ifndef DMA_H
#define DMA_H

/* Inclusion */
#include "TMPM4KyA.h"
#include <stdint.h>


/* Defines */

/* =========================================================================
 * [DMAChnlCfg] Transfer Mode Setup Register bit-field macros  (Section 3.2.2.3)
 * ========================================================================= */

/* 3.2.1. Channel Control Data Memory Map */
typedef struct
{
    __IO uint32_t SrcEndPtr;     /*!< Source data end address                        */
    __IO uint32_t DstEndPtr;     /*!< Destination data end address                   */
    __IO uint32_t ChnlCfg;       /*!< Transfer mode setup                            */
         uint32_t RESERVED;      /*!< (padding — must not be written)                */
} DMA_ChnlCtrlData_TypeDef;

/* dst_inc [31:30] - Increment of a transfer destination address  */
#define DMA_DST_INC_BYTE                            ((uint32_t)0x0U << 30U)     /*!< Increment +1 byte    */
#define DMA_DST_INC_HWORD                           ((uint32_t)0x1U << 30U)     /*!< Increment +2 bytes   */
#define DMA_DST_INC_WORD                            ((uint32_t)0x2U << 30U)     /*!< Increment +4 bytes   */
#define DMA_DST_INC_NONE                            ((uint32_t)0x3U << 30U)     /*!< No increment         */

/* dst_size [29:28] - destination transfer data size */
#define DMA_DST_SIZE_BYTE                           ((uint32_t)0x0U << 28U)     /*!< 8-bit                */
#define DMA_DST_SIZE_HWORD                          ((uint32_t)0x1U << 28U)     /*!< 16-bit               */
#define DMA_DST_SIZE_WORD                           ((uint32_t)0x2U << 28U)     /*!< 32-bit               */

/* src_inc [27:26] - source address increment */                           
#define DMA_SRC_INC_BYTE                            ((uint32_t)0x0U << 26U)     /*!< Increment +1 byte    */
#define DMA_SRC_INC_HWORD                           ((uint32_t)0x1U << 26U)     /*!< Increment +2 bytes   */
#define DMA_SRC_INC_WORD                            ((uint32_t)0x2U << 26U)     /*!< Increment +4 bytes   */
#define DMA_SRC_INC_NONE                            ((uint32_t)0x3U << 26U)     /*!< No increment         */

/* src_size [25:24] - source transfer data size */                         
#define DMA_SRC_SIZE_BYTE                           ((uint32_t)0x0U << 24U)     /*!< 8-bit                */
#define DMA_SRC_SIZE_HWORD                          ((uint32_t)0x1U << 24U)     /*!< 16-bit               */
#define DMA_SRC_SIZE_WORD                           ((uint32_t)0x2U << 24U)     /*!< 32-bit               */

/* Reserved [23:18] - Please set up "000000." */

/* R_power [17:14] - arbitration interval (transfers between re-arbitrations) */
#define DMA_R_POWER_1                               ((uint32_t)0x0U << 14U)     /*!< Arbitrate after 1    */
#define DMA_R_POWER_2                               ((uint32_t)0x1U << 14U)     /*!< Arbitrate after 2    */
#define DMA_R_POWER_4                               ((uint32_t)0x2U << 14U)     /*!< Arbitrate after 4    */
#define DMA_R_POWER_8                               ((uint32_t)0x3U << 14U)     /*!< Arbitrate after 8    */

/* n_minus_1 [13:4] - (total transfers - 1), max 1023 */
#define DMA_N_MINUS_1(n)                            ((uint32_t)((n) - 1U) << 4U)

/* next_useburst [3] - Single transfer setting change (Note4)*/
#define DMA_NEXT_USEBURST                           ((uint32_t)0x01 << 3U)      /*!< Single transfer setting change - Single or Burst */

/* cycle_ctrl [2:0] - transfer mode */
#define DMA_CYCLE_CTRL_SUSPEND                      ((uint32_t)0x0U << 0U)      /*!< Channel disabled      */
#define DMA_CYCLE_CTRL_UNIT                         ((uint32_t)0x1U << 0U)      /*!< Unit (normal)         */
#define DMA_CYCLE_CTRL_CNT                          ((uint32_t)0x2U << 0U)      /*!< Continuation (normal) */
#define DMA_CYCLE_CTRL_REPEAT                       ((uint32_t)0x3U << 0U)      /*!< Repeat                */
/* ============================================================================================== */


/* [FSYSMENx] Clock Gate (CG) */
#define DMAC_CG_FSYSMENB_IPMENB17                   ((uint32_t)0x01 << 17U)     /*<! CG FSYSMENB DMAC Unit A */

/* [DMAxStatus] (DMAC Status Register) */
#define DMAxStatus_Master_enable                    ((uint32_t)0x01 << 0U)      /*!< DMA Status Reg. for Master DMA Operation*/

/* [DMAxCfg] (DMAC Configuration Register) */
#define DMAxCfg_Master_enable                       ((uint32_t)0x01 << 0U)      /*!< DMA Config Reg. for Master DMA Operation*/

/* [DMAxChnlUseburstSet] (Channel Useburst Set Register) */
#define DMAxChnlUseburstSet_MASK                    ((uint32_t)0xFFFFFFFF)      /*!< DMA Channel Use Burst Mask Set Mask */

/* [DMAxChnlUseburstClr] (Channel Useburst Clear Register) */
#define DMAxChnlUseburstClr_MASK                    ((uint32_t)0xFFFFFFFF)      /*!< DMA Channel Use Burst Mask Clear Mask */

/* [DMAxChnlReqMaskSet] (Channel Request Mask Set Register)*/
#define DMAxChnlReqMaskSet_MASK                     ((uint32_t)0xFFFFFFFF)      /*!< DMA Channel Request Mask Set Mask */

/* [DMAxChnlReqMaskClr / ChnlEnableSet] convenience mask for a single channel */
#define DMAxChnlReqMaskClr_MASK                     ((uint32_t)0xFFFFFFFF)      /*!< DMA Channel Request Mask Clear Mask */

/* [DMAxChnlEnableSet] (Channel Enable Set Register */
#define DMAxChnlEnableSet_MASK                      ((uint32_t)0xFFFFFFFF)      /*!< DMA Channel Enable Set Mask */

/* Universal Channel mask Clear/Set*/
#define DMA_CHANNEL_MASK(ch)                        ((uint32_t)0x1U << (ch))

/* DMA Transfer Request Channel */
#define DMA_ADASLG_DMAREQ                           16                           /* Channel 16 */
#define DMA_ADCSLG_DMAREQ                           18                           /* Channel 18 */

/* Number of channels in the control table (fixed by DMAC-B architecture) */
#define DMA_CHANNEL_COUNT                           (32U)

/* Function Prototypes */
void DMAC_Init(void);
void DMA_ConfigChannel(uint8_t channel, uint32_t src_end, uint32_t dst_end, uint32_t chnl_cfg);
void DMA_SetupForADC(void);

volatile uint16_t* DMA_GetADCABuffer(void);
volatile uint16_t* DMA_GetADCCBuffer(void);

#endif /* DMA_H */