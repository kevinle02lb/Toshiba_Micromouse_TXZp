/**
 * @file        Encoder.h
 * @brief       Encoder control module – speed and position from quadrature encoders
 * @version     V1.0.0
 * @date        17-06-2026
 *
 * @details
 *   Provides filtered speed (counts per second) and accumulated position
 *   for both motors. Uses the A-ENC32 hardware driver.
 *
 *   Pololu micro metal gearmotor encoder (Rev 6-2, pg 2):
 *   "12 CPR of the motor shaft when counting both edges of both channels"
 *   → 4x quadrature is ALREADY included in the 12 CPR figure.
 *   → A-ENC32 hardware does the 4x counting automatically.
 *
 *   12 CPR × 29.89:1 exact gear ratio = 358.68 → use 360 nominal.
 *   Calibrate by spinning wheel one full turn, reading raw count.
 *
 * @note
 *   This module must be updated every 1 kHz tick (call @ref Encoder_Update).
 * 
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "Motor.h"      /* for motor_t */

typedef struct
{
    int32_t raw_count;          /*!< Latest hardware counter value */
    int32_t prev_count;         /*!< Value from previous tick */
    int32_t delta;              /*!< raw - prev (counts per tick) */
    int32_t position;           /*!< Accumulated signed position */
    int32_t speed_filtered;     /*!< IIR-filtered speed (counts per second) */
} EncoderState_t;

/* ==========================================================================
 *   Configuration
 * ========================================================================== */

/**
 * @brief  IIR filter shift for speed smoothing.
 *         alpha = 1 / 2^SHIFT.  SHIFT=4 → alpha=1/16, gentle filtering.
 */
#define SPEED_FILTER_SHIFT  4U

/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */

void Encoder_Init(void);
void Encoder_Update(void);
int32_t Encoder_GetSpeed_CPS(motor_t motor);
int32_t Encoder_GetDelta(motor_t motor);
int32_t Encoder_GetPosition(motor_t motor);
void Encoder_ResetPosition(motor_t motor);
int32_t Encoder_GetRawCount(motor_t motor);

#endif /* ENCODER_H */