/**
 *************************************************************************
 * @file        adc.h
 * @brief       ADC-I driver implementation for TMPM4Ky RISC microcontrollers.
 * @version     V1.0.0
 * @date        29-05-2026
 * 
 * 
 * @details
 *  This driver provides initialization and control functions for the ADC-I
 *  https://toshiba.semicon-storage.com/info/RM-ADC-I_en_20251205.pdf?did=166835
 *  https://toshiba.semicon-storage.com/info/COM_ADC_MON-ANE_application_note_en_20231016.pdf?did=156383&prodName=TMPM4KNF10AFG
 *  
 * 
 * Copyright (c) [Kevin Le] 2026
 **************************************************************************
 */


#ifndef ADC_H
#define ADC_H

#include "TMPM4KyA.h"
#include <stdint.h>
#include <stdbool.h>

#define AINA15_POS                              15U
#define AINA16_POS                              16U
#define AINC00_POS                              0U
#define AINC01_POS                              1U


/* [FSYSMENx] Clock Gate (CG) */
#define AINA_CG_FSYSMENB_IPMENB02               ((uint32_t)0x01 << 2U)          /*!< CG FSYSMENB AINA */
#define AINC_CG_FSYSMENB_IPMENB04               ((uint32_t)0x01 << 4U)          /*!< CG FSYSMENB AINC */

/* [CGSPCLKEN] (Clock Supply and Stop Register for ADC and Debug Circuit) */
#define CG_CGSPCLKEN_ADCKEN0                    ((uint32_t)0x01 << 16U)         /*!< CG FSYSMENB ADC Unit A*/
#define CG_CGSPCLKEN_ADCKEN1                    ((uint32_t)0x01 << 17U)         /*!< CG FSYSMENB ADC Unit B*/
#define CG_CGSPCLKEN_ADCKEN2                    ((uint32_t)0x01 << 18U)         /*!< CG FSYSMENB ADC Unit C*/

/* [ADxCLK] (Conversion Clock Setting Register) */
#define ADxCLK_VADCLK_MASK                      ((uint32_t)0x07 << 0U)          /*!< ADxCLK Conversion Clock Setting. Prescale Val*/
#define ADxCLK_VADCLK                           ((uint32_t)0x01 << 0U)          /* AD prescaler output (SCLK) selection - 000: ADCLK/4    001: ADCLK/8 */
#define ADxCLK_EXAZ0_MASK                       ((uint32_t)0x0F << 3U)          /*!< ADxCLK AIN sampling time setting 0*/
#define ADxCLK_EXAZ1_MASK                       ((uint32_t)0x0F << 8U)          /*!< ADxCLK AIN sampling time setting 1*/
#define ADXCLK_EXAZ0_40MHZ                      ((uint32_t)0x01 << 3U)          /*ADxCLK AIN sampling time setting 0 for 0.96[µs] at 40Mhz */
#define ADXCLK_EXAZ1_40MHZ                      ((uint32_t)0x01 << 8U)          /*ADxCLK AIN sampling time setting 1 for 0.96[µs] at 40Mhz */

/* [ADxCR0] (Control Register0) */
#define ADxCR0_ADEN                             ((uint32_t)0x01 << 7U)          /*!< ADx Control Reg 0 AD Enable*/
#define ADxCR0_SGL                              ((uint32_t)0x01 << 1U)          /*!< ADx Control Reg 0 Single conversion control */
#define ADxCR0_CNT                              ((uint32_t)0x01 << 0U)          /*!< ADx Control Reg 0 Continuous conversion control */

/* [ADxCR1] (Control Register1) */
#define ADxCR1_TRGDMEN                          ((uint32_t)0x01 << 4U)          /*!< ADxCR1 Control Reg 1 General purpose trigger DMA request control*/
#define ADxCR1_SGLDMEN                          ((uint32_t)0x01 << 5U)          /*!< ADxCR1 Control Reg 1 Single conversion DMA request control */
#define ADxCR1_CNTDMEN                          ((uint32_t)0x01 << 6U)          /*!< ADxCR1 Control Reg 1 Continuous conversion DMA request control */

/* [ADxST] (Status Register) */
#define ADxST_TRGF                              ((uint32_t)0x01 << 1U)          /*!< ADxST Status Register General purpose trigger program flag */
#define ADxST_SNGF                              ((uint32_t)0x01 << 2U)          /*!< ADxST Status Register Single conversion program flag */
#define ADxST_CNTF                              ((uint32_t)0x01 << 3U)          /*!< ADxST Status Register Continuous conversion program flag */
#define ADxST_ADBF                              ((uint32_t)0x01 << 7U)          /*!< ADxST Status Register AD operation flag */

/* [ADxMOD0] (Mode Setting Register0) */
#define ADxMOD0_DACON                           ((uint32_t)0x01 << 0U)          /*!< ADx Mode 0 Setting DAC Control*/
#define ADxMOD0_RCUT                            ((uint32_t)0x01 << 1U)          /*!< ADx Mode 0 Setting Low power control*/

/* [ADxMOD1] (Mode Setting Register1) */
#define ADxMOD1_40MHZ                           ((uint32_t)0x00306122)          /*!< ADx Mode 1 "n" value setting for sampling when ADCLK = 160Mhz, SCLK = 40Mhz*/

/* [ADxEXAZSEL] (AIN Sampling Time Selection Register) */
#define ADxEXAZSEL_AINA15                       ((uint32_t)0x01 << AINA15_POS)  /*!< AIN Sampling Time Selection Register. AINA15 Sel */
#define ADxEXAZSEL_AINA16                       ((uint32_t)0x01 << AINA16_POS)  /*!< AIN Sampling Time Selection Register. AINA16 Sel */
#define ADxEXAZSEL_AINC00                       ((uint32_t)0x01 << AINC00_POS)  /*!< AIN Sampling Time Selection Register. AINC00 Sel */
#define ADxEXAZSEL_AINC01                       ((uint32_t)0x01 << AINC01_POS)  /*!< AIN Sampling Time Selection Register. AINC01 Sel */

/* [ADxTSETn] (General Purpose Start-up Factor Program Register0) */
#define ADxTSETn_AINSTn_AINx15                  ((uint32_t)0x0F << 0U)          /*!< ADxTSETn AINx15 Selection */
#define ADxTSETn_AINSTn_AINx16                  ((uint32_t)0x10 << 0U)          /*!< ADxTSETn AINx16 Selection */
#define ADxTSETn_AINSTn_AINx00                  ((uint32_t)0x00 << 0U)          /*!< ADxTSETn AINx00 Selection */
#define ADxTSETn_AINSTn_AINx01                  ((uint32_t)0x01 << 0U)          /*!< ADxTSETn AINx01 Selection */

#define ADxTSETn_TRGSn_MASK                     ((uint32_t)0x03 << 5U)          /*!< ADxTSETn TRGSn Conversion control */
#define ADxTSETn_TRGSn_CNT                      ((uint32_t)0x01 << 5U)          /*!< ADxTSETn TRGSn Continuous Conversion control */
#define ADxTSETn_TRGSn_SGL                      ((uint32_t)0x02 << 5U)          /*!< ADxTSETn TRGSn Single Conversion control */

#define ADxTSETn_ENINTn_MASK                    ((uint32_t)0x01 << 7U)          /*!< ADxTSETn ENINTn Interrupt control */

/* [ADxREGn] (Conversion Result Storage Register0) */
#define ADxREGn_ADRFn                           ((uint32_t)0x01 << 0U)          /*!< ADxREGn Conversion Result Storage Register - AD conversion result storage flag*/
#define ADxREGn_ADOVRFn                         ((uint32_t)0x01 << 1U)          /*!< ADxREGn Conversion Result Storage Register - Overrun flag */
#define ADxREGn_ADRn                            ((uint32_t)0xFFF << 4U)         /*!< ADxREGn Conversion Result Storage Register - AD conversion result is stored. */

/* Function Prototypes */
void ADC_Init(void);
void AINA_Init(void);
void AINC_Init(void);

void AINA_StartSGL(void);
void AINC_StartSGL(void);

void Start_ADC(void);

uint16_t AINA_Read(uint8_t channel);
uint16_t AINC_Read(uint8_t channel);



#endif /* ADC_H */