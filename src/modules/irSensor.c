/**
 * @file        irSensor.c
 * @brief       IR Sensor driver implementation for TMPM4Ky micromouse.
 * @version     V1.0.0
 * @date        11-06-2026
 *
 * @details
 *   Sequenced emitter control and ADC sampling with ambient cancellation.
 *   Uses DMA channels 16 (ADC Unit A) and 18 (ADC Unit C) for automatic
 *   data transfer without CPU intervention.
 *
 *   Pin Map:
 *   - Emitters (GPIO):
 *     - PU1 = Far Left IR Emitter, PU0 = Left IR Emitter  (Left side)
 *     - PG5 = Right IR Emitter, PG4 = Far Right IR Emitter (Right side)
 *   - Receivers (ADC):
 *     - Unit A: PL0 (AINA16) = Far Left, PL1 (AINA15) = Left
 *     - Unit C: PJ0 (AINC00) = Far Right, PJ1 (AINC01) = Right
 *
 *   DMA Buffer Layout:
 *   - adc_a_buffer[0] = AINA16 (Far Left)
 *   - adc_a_buffer[1] = AINA15 (Left)
 *   - adc_c_buffer[0] = AINC01 (Right)
 *   - adc_c_buffer[1] = AINC00 (Far Right)
 *
 *   Reference Documents (Toshiba):
 *   - ADC-I RM:     https://toshiba.semicon-storage.com/info/RM-ADC-I_en_20251205.pdf?did=166835
 *   - DMAC-B RM:    https://toshiba.semicon-storage.com/info/RM-DMAC-B_en_20241031.pdf?did=160537
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "irSensor.h"
#include "drivers/adc.h"
#include "drivers/dma.h"
#include "drivers/gpio.h"
#include "systick.h"

/* ==========================================================================
 *   Module Data
 * ========================================================================== */

static IR_SensorData ir_data;

/* ==========================================================================
 *   Initialization
 * ========================================================================== */

/**
 * @brief  Initializes ADC and GPIO for IR emitters.
 * @note   Make sure DMAC is initialized before this is called.
 */
void IR_Init(void)
{
    DMAC_Init();         /* Initialize DMAC */
    ADC_Init();
    PORT_U_Init();       /* [1] Left emitters  */
    PORT_G_Init();       /* [2] Right emitters */
}

/* ==========================================================================
 *   Sequenced Sampling
 * ========================================================================== */

/**
 * @brief  Full ON/OFF sampling cycle for all four IR sensors.
 * @details
 *   Performs ambient cancellation to reject background IR:
 *   1. Emitters OFF → sample ambient light
 *   2. Emitters ON  → sample raw (ambient + reflected)
 *   3. reflected = raw - ambient
 *
 *   Timing:
 *   - IR decay after OFF:  50 µs
 *   - IR propagation ON: 100 µs
 *   - ADC conversion:       ~2 µs per channel @ 40 MHz SCLK
 *   - Total: ~200 µs
 */
void IR_SampleAll(void)
{
    volatile uint16_t *bufA;
    volatile uint16_t *bufC;
    int i;

    /* [1] Ambient reading — all emitters OFF */
    IR_AllEmittersOff();
    SysTick_us(150U);             /* Wait for previous IR to decay */

    Start_ADC();                 /* Trigger both ADC units, DMA re-armed */
    SysTick_us(20U);             /* Wait for 2 conversions + DMA burst */

    bufA = DMA_GetADCABuffer();
    bufC = DMA_GetADCCBuffer();

    ir_data.ambient[IR_FAR_LEFT]  = bufA[0];   /* AINA16 */
    ir_data.ambient[IR_LEFT]      = bufA[1];   /* AINA15 */
    ir_data.ambient[IR_RIGHT]     = bufC[0];   /* AINC01 */
    ir_data.ambient[IR_FAR_RIGHT] = bufC[1];   /* AINC00 */

    /* [2] Reflected reading — all emitters ON */
    IR_AllEmittersOn();
    SysTick_us(100U);            /* Allow IR to reach wall and reflect back */

    Start_ADC();                 /* Re-arm DMA + trigger both units */
    SysTick_us(20U);             /* Wait for conversion + DMA */

    ir_data.raw[IR_FAR_LEFT]  = bufA[0];
    ir_data.raw[IR_LEFT]      = bufA[1];
    ir_data.raw[IR_RIGHT]     = bufC[0];
    ir_data.raw[IR_FAR_RIGHT] = bufC[1];

    /* [3] Calculate reflected values (ambient cancellation) */
    for (i = 0; i < IR_COUNT; i++)
    {
        if (ir_data.raw[i] > ir_data.ambient[i])
        {
            ir_data.reflected[i] = ir_data.raw[i] - ir_data.ambient[i];
        }
        else
        {
            ir_data.reflected[i] = 0U;
        }
    }

    /* [4] IIR low-pass filter on reflected values */
    for (i = 0; i < IR_COUNT; i++)
    {
        ir_data.filtered[i] = IR_FilterIIR(ir_data.filtered[i],
                                              ir_data.reflected[i],
                                              IR_FILTER_SHIFT);
    }

    /* [5] Power save — turn off emitters between samples */
    IR_AllEmittersOff();
}

/* ==========================================================================
 *   Data Accessors
 * ========================================================================== */

/**
 * @brief  Get pointer to the latest IR sensor data structure.
 * @return const IR_SensorData*  Pointer to internal ir_data.
 */
const IR_SensorData* IR_GetData(void)
{
    return &ir_data;
}

/**
 * @brief  Get raw ADC value (emitter ON) for a specific channel.
 * @param  ch  Sensor channel to read.
 * @return uint16_t  Raw 12-bit ADC value, right-aligned.
 */
uint16_t IR_GetRaw(IR_Channel ch)
{
    return ir_data.raw[ch];
}

/**
 * @brief  Get ambient-corrected reflected value for a specific channel.
 * @param  ch  Sensor channel to read.
 * @return uint16_t  Reflected IR intensity (0 if raw <= ambient).
 */
uint16_t IR_GetReflected(IR_Channel ch)
{
    return ir_data.reflected[ch];
}

/**
 * @brief  Check if a wall is detected based on reflected threshold.
 * @param  ch         Sensor channel to check.
 * @param  threshold  Minimum reflected value to count as wall.
 * @return bool  true if reflected value >= threshold.
 */
bool IR_IsWallDetected(IR_Channel ch, uint16_t threshold)
{
    return (ir_data.reflected[ch] >= threshold);
}

/* ==========================================================================
 *   Emitter Control
 * ========================================================================== */

/**
 * @brief  Turn OFF all four IR emitters simultaneously.
 */
void IR_AllEmittersOff(void)
{
    FarLeftEmitterOff();
    LeftEmitterOff();
    RightEmitterOff();
    FarRightEmitterOff();
}

/**
 * @brief  Turn ON all four IR emitters simultaneously.
 */
void IR_AllEmittersOn(void)
{
    FarLeftEmitterOn();
    LeftEmitterOn();
    RightEmitterOn();
    FarRightEmitterOn();
}

/* ==========================================================================
 *   Low-Level GPIO Emitter Control
 * ========================================================================== */

/**
 * @brief  Control functions to toggle data outpins for IR emitters.
 */

void FarLeftEmitterOn(void)     { GPIO_U_SetData((uint8_t)Px1_MASK); }
void LeftEmitterOn(void)        { GPIO_U_SetData((uint8_t)Px0_MASK); }
void RightEmitterOn(void)       { GPIO_G_SetData((uint8_t)Px5_MASK); }
void FarRightEmitterOn(void)    { GPIO_G_SetData((uint8_t)Px4_MASK); }

void FarLeftEmitterOff(void)    { GPIO_U_ClrData((uint8_t)Px1_MASK); }
void LeftEmitterOff(void)       { GPIO_U_ClrData((uint8_t)Px0_MASK); }
void RightEmitterOff(void)      { GPIO_G_ClrData((uint8_t)Px5_MASK); }
void FarRightEmitterOff(void)   { GPIO_G_ClrData((uint8_t)Px4_MASK); }

void FarLeftEmitterToggle(void) { GPIO_U_ToggleData((uint8_t)Px1_MASK); }
void LeftEmitterToggle(void)    { GPIO_U_ToggleData((uint8_t)Px0_MASK); }
void RightEmitterToggle(void)   { GPIO_G_ToggleData((uint8_t)Px5_MASK); }
void FarRightEmitterToggle(void){ GPIO_G_ToggleData((uint8_t)Px4_MASK); }



/* ==========================================================================
 *   IR Filtering from Spikes
 * ========================================================================== */

/**
 * @brief  Apply IIR low-pass filter to a single ADC channel.
 * @param  prev  Previous filtered value (y[n-1]).
 * @param  curr  New raw sample (x[n]).
 * @param  shift Right-shift for alpha divisor (e.g., 3 = /8).
 * @return uint16_t  Filtered output y[n].
 * @details
 *   Exponential moving average with power-of-two alpha:
 *     y[n] = y[n-1] + alpha * (x[n] - y[n-1])
 *          = y[n-1] + (x[n] - y[n-1]) >> shift
 *
 *   Equivalent to: y[n] = ((2^shift - 1) * y[n-1] + x[n]) / 2^shift
 *
 *   Shift values:
 *   - 2: alpha = 0.25, fast response, light smoothing
 *   - 3: alpha = 0.125, balanced (default)
 *   - 4: alpha = 0.0625, slow response, heavy smoothing
 *
 *   No saturation needed — uint16_t in, uint16_t out, difference
 *   computed in unsigned arithmetic with branch on direction.
 */
uint16_t IR_FilterIIR(uint16_t prev, uint16_t curr, uint8_t shift)
{
    uint16_t delta;
    uint16_t result;

    if (curr > prev)
    {
        delta = curr - prev;
        result = prev + (delta >> shift);
    }
    else
    {
        delta = prev - curr;
        result = prev - (delta >> shift);
    }

    return result;
}