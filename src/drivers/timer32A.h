/**
 * @file        timer32A.h
 * @brief       Timer32A driver for TMPM4Ky microcontrollers.
 * @version     V1.0.0
 * @date        11-05-2026
 *
 * @details
 *   PPG (PWM) configuration for motor control.
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef TIMER32A_H
#define TIMER32A_H

#include "TMPM4KyA.h"
#include <stdint.h>

/* ==========================================================================
 *   User Config
 * ========================================================================== */
#define T32A_CH0_PERIOD     2000    /*!< 80 MHz / 1:1 / 2000 = 40 kHz PWM */
#define T32A_CH3_PERIOD     2000    /*!< Same for right motor */

#define T32A_MODE_SEL       0       /*!< 0 = 16-bit, 1 = 32-bit */
#define T32A_PRSCL_SEL      0       /*!< 0 = 1:1, see RM for others */

/* ==========================================================================
 *   Clock Gating
 * ========================================================================== */
#define T32A0_CG_FSYSMENA_IPMENA28  ((uint32_t)0x01 << 28U)   /*!< T32A ch0 clock */
#define T32A3_CG_FSYSMENA_IPMENA31  ((uint32_t)0x01 << 31U)   /*!< T32A ch3 clock */

/* ==========================================================================
 *   Mode / Run Registers
 * ========================================================================== */
#define T32A_MODE_MASK          ((uint32_t)0x03)          /*!< Mode bits [1:0] */
#define T32A_MODE32             ((uint32_t)T32A_MODE_SEL) /*!< 16/32-bit select */
#define T32A_RUNx_RUNFLGx_MASK  ((uint32_t)0x01 << 4U)    /*!< RUN flag (read-only) */
#define T32A_RUNx_MASK          ((uint32_t)0x01 << 0U)    /*!< RUN control */

/* ==========================================================================
 *   Counter Control (CRA/CRB)
 * ========================================================================== */
#define T32A_CRx_PRSCLC_MASK    ((uint32_t)0x07 << 28U)   /*!< Prescaler mask */
#define T32A0_CRA_PRSCLA        ((uint32_t)T32A_PRSCL_SEL << 28U)
#define T32A0_CRB_PRSCLB        ((uint32_t)T32A_PRSCL_SEL << 28U)
#define T32A3_CRA_PRSCLA        ((uint32_t)T32A_PRSCL_SEL << 28U)
#define T32A3_CRB_PRSCLB        ((uint32_t)T32A_PRSCL_SEL << 28U)

#define T32A_CRx_RELDx_MASK     ((uint32_t)0x03 << 8U)    /*!< Reload timing mask */
#define T32A_CRx_UPDNx_MASK     ((uint32_t)0x03 << 16U)   /*!< Count direction mask */
#define T32A_UPDNx              ((uint32_t)0x00 << 16U)   /*!< 00 = Up, 01 = Down */
#define T32A_CRx_WBFx_MASK      ((uint32_t)0x01 << 20U)   /*!< Double-buffer enable */

/* ==========================================================================
 *   Output Control
 * ========================================================================== */
#define T32A_OUTCRx0_ORCx_MASK  ((uint32_t)0x03 << 0U)    /*!< OUTCRx0 mask */
#define T32A_OUTCRx1_MASK       ((uint32_t)0xFF << 0U)    /*!< OUTCRx1 full mask */

/* Compare match output actions */
#define OUTCRxx_FTN_INVALID     0x00
#define OUTCRxx_FTN_SET         0x01
#define OUTCRxx_FTN_CLEAR       0x02
#define OUTCRxx_FTN_REVERSED    0x03

#define T32A_OUTCRx1_OCRCMPx0_SET   (OUTCRxx_FTN_SET   << 0U)
#define T32A_OUTCRx1_OCRCMPx0_CLEAR (OUTCRxx_FTN_CLEAR << 0U)
#define T32A_OUTCRx1_OCRCMPx1_SET   (OUTCRxx_FTN_SET   << 2U)
#define T32A_OUTCRx1_OCRCMPx1_CLEAR (OUTCRxx_FTN_CLEAR << 2U)

/* Preset output modes */
#define T32A_OUTPUT_HIGH        (T32A_OUTCRx1_OCRCMPx0_SET   | T32A_OUTCRx1_OCRCMPx1_SET)
#define T32A_OUTPUT_LOW         (T32A_OUTCRx1_OCRCMPx0_CLEAR | T32A_OUTCRx1_OCRCMPx1_CLEAR)
#define T32A_OUTPUT_PPG         (T32A_OUTCRx1_OCRCMPx0_SET   | T32A_OUTCRx1_OCRCMPx1_CLEAR)   /*!< SET on CMP0, CLEAR on CMP1 */
#define T32A_OUTPUT_INVERTED    (T32A_OUTCRx1_OCRCMPx0_CLEAR | T32A_OUTCRx1_OCRCMPx1_SET)     /*!< CLEAR on CMP0, SET on CMP1 */

/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */
void T32A_Init(void);
void T32A0_Init(uint16_t period);
void T32A3_Init(uint16_t period);
void T32A0_Start(void);
void T32A3_Start(void);
void T32A0_Stop(void);
void T32A3_Stop(void);

/* Timer compare registers (duty/period) */
void T32A0_SetTimerA0(uint16_t val);   /* RGA0 */
void T32A0_SetTimerA1(uint16_t val);   /* RGA1 */
void T32A0_SetTimerB0(uint16_t val);   /* RGB0 */
void T32A0_SetTimerB1(uint16_t val);   /* RGB1 */
void T32A3_SetTimerA0(uint16_t val);
void T32A3_SetTimerA1(uint16_t val);
void T32A3_SetTimerB0(uint16_t val);
void T32A3_SetTimerB1(uint16_t val);

/* Output control */
void T32A0_SetOutCRA1(uint32_t mode);  /* OUTCRA1 */
void T32A0_SetOutCRB1(uint32_t mode);  /* OUTCRB1 */
void T32A3_SetOutCRA1(uint32_t mode);
void T32A3_SetOutCRB1(uint32_t mode);

#endif /* TIMER32A_H */