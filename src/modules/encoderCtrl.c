/**
 * @file        encoderCtrl.c
 * @brief       Encoder control module implementation.
 * @version     V1.0.0
 * @date        17-06-2026
 *
 * @details
 *   Reads A-ENC32 hardware counters, computes delta since last tick,
 *   applies IIR filtering on speed, and maintains a signed position.
 *
 * @note 
 *   File structure and Doxygen formatting assisted by AI.
 * 
 * Copyright (c) [Kevin Le] 2026
 */

#include "encoderCtrl.h"
#include "encoder32A.h"
#include <stddef.h>

/* ==========================================================================
 *   Private Data
 * ========================================================================== */

typedef struct
{
    int32_t raw_count;          /*!< Latest hardware counter value */
    int32_t prev_count;         /*!< Value from previous tick */
    int32_t delta;              /*!< raw - prev (counts per tick) */
    int32_t position;           /*!< Accumulated signed position */
    int32_t speed_filtered;     /*!< IIR-filtered speed (counts per second) */
} EncoderState_t;

static EncoderState_t enc_state[2];

/* ==========================================================================
 *   Private Helpers
 * ========================================================================== */

/**
 * @brief  Apply IIR low‑pass filter to a speed value.
 * @param  prev   Previous filtered value (y[n-1])
 * @param  curr   New raw value (x[n])
 * @param  shift  Right‑shift for alpha = 1/2^shift
 * @return int32_t  Filtered output y[n]
 *
 * @details
 *   y[n] = y[n-1] + (x[n] - y[n-1]) >> shift
 */
static int32_t speed_filter_IIR(int32_t prev, int32_t curr, uint8_t shift)
{
    int32_t delta = curr - prev;
    if (delta >= 0)
        return prev + (delta >> shift);
    else
        return prev - ((-delta) >> shift);
}

/* ==========================================================================
 *   Public Functions
 * ========================================================================== */


 /**
 * @brief  Initialise encoder hardware and internal state.
 * @note   Must be called once before any other function.
 */
void encoderCtrl_Init(void)
{
    /* Initialise the hardware drivers */
    ENC32A_Init();

    /* Clear all state */
    for (int i = 0; i < 2; i++)
    {
        enc_state[i].raw_count      = 0;
        enc_state[i].prev_count     = 0;
        enc_state[i].delta          = 0;
        enc_state[i].position       = 0;
        enc_state[i].speed_filtered = 0;
    }

    /* Read the current counter values to set prev_count */
    enc_state[MOTOR_LEFT].prev_count  = ENC0_ReadCount();
    enc_state[MOTOR_RIGHT].prev_count = ENC2_ReadCount();
    enc_state[MOTOR_LEFT].raw_count   = enc_state[MOTOR_LEFT].prev_count;
    enc_state[MOTOR_RIGHT].raw_count  = enc_state[MOTOR_RIGHT].prev_count;
}



/**
 * @brief  Read both encoders, update delta, position, and filtered speed.
 * @note   Call this exactly once per control tick (1 kHz).
 */
void encoderCtrl_Update(void)
{
    int32_t raw, delta, speed_raw;

    /* Left motor */
    raw = ENC0_ReadCount();
    delta = raw - enc_state[MOTOR_LEFT].prev_count;
    enc_state[MOTOR_LEFT].raw_count = raw;
    enc_state[MOTOR_LEFT].delta = delta;
    enc_state[MOTOR_LEFT].position += delta;

    /* Convert delta to counts per second (tick = 1 ms) and filter */
    speed_raw = delta * 1000;                           /* counts/s */
    enc_state[MOTOR_LEFT].speed_filtered = speed_filter_IIR
    (
        enc_state[MOTOR_LEFT].speed_filtered,
        speed_raw,
        ENC_SPEED_FILTER_SHIFT
    );

    enc_state[MOTOR_LEFT].prev_count = raw;

    /* Right motor */
    raw = ENC2_ReadCount();
    delta = raw - enc_state[MOTOR_RIGHT].prev_count;
    enc_state[MOTOR_RIGHT].raw_count = raw;
    enc_state[MOTOR_RIGHT].delta = delta;
    enc_state[MOTOR_RIGHT].position += delta;

    speed_raw = delta * 1000;
    enc_state[MOTOR_RIGHT].speed_filtered = speed_filter_IIR
    (
        enc_state[MOTOR_RIGHT].speed_filtered,
        speed_raw,
        ENC_SPEED_FILTER_SHIFT
    );

    enc_state[MOTOR_RIGHT].prev_count = raw;
}

/**
 * @brief  Get the filtered speed for a motor.
 * @param  motor  MOTOR_LEFT or MOTOR_RIGHT
 * @return int32_t  Speed in counts per second (signed, positive = forward).
 */
int32_t encoderCtrl_GetSpeed_cps(motor_t motor)
{
    if (motor != MOTOR_LEFT && motor != MOTOR_RIGHT)
        return 0;
    return enc_state[motor].speed_filtered;
}

/**
 * @brief  Get the raw delta (counts since last tick) for a motor.
 * @param  motor  MOTOR_LEFT or MOTOR_RIGHT
 * @return int32_t  Signed delta counts in one tick period.
 */
int32_t encoderCtrl_GetDelta(motor_t motor)
{
    if (motor != MOTOR_LEFT && motor != MOTOR_RIGHT)
        return 0;
    return enc_state[motor].delta;
}

/**
 * @brief  Get the accumulated position for a motor.
 * @param  motor  MOTOR_LEFT or MOTOR_RIGHT
 * @return int32_t  Signed position in counts (increments on forward motion).
 */
int32_t encoderCtrl_GetPosition(motor_t motor)
{
    if (motor != MOTOR_LEFT && motor != MOTOR_RIGHT)
        return 0;
    return enc_state[motor].position;
}

/**
 * @brief  Reset the position counter for a motor to zero.
 * @param  motor  MOTOR_LEFT or MOTOR_RIGHT
 */
void encoderCtrl_ResetPosition(motor_t motor)
{
    if (motor != MOTOR_LEFT && motor != MOTOR_RIGHT)
        return;
    enc_state[motor].position = 0;
}

/**
 * @brief  Get the raw hardware count for a motor.
 * @param  motor  MOTOR_LEFT or MOTOR_RIGHT
 * @return int32_t  Current counter value from the encoder peripheral.
 */
int32_t encoderCtrl_GetRawCount(motor_t motor)
{
    if (motor == MOTOR_LEFT)
        return ENC0_ReadCount();
    else if (motor == MOTOR_RIGHT)
        return ENC2_ReadCount();
    else
        return 0;
}