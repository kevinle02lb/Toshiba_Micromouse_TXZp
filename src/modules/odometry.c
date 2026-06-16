/**
 * @file        odometry.h
 * @brief       Wheel odometry and pose estimation for micromouse.
 * @version     V1.0.0
 * @date        15-06-2026
 *
 * @details
 *   Reads A-ENC32 hardware counters, computes deltas, velocities,
 *   and integrates differential-drive odometry.
 *
 *   Odometry math (differential drive):
 *     delta_left  = left_delta  × ENC_MM_PER_COUNT
 *     delta_right = right_delta × ENC_MM_PER_COUNT
 *     delta_dist  = (delta_left + delta_right) / 2
 *     delta_theta = (delta_right - delta_left) / WHEEL_TRACK_MM
 *
 *     x += delta_dist × cos(heading + delta_theta/2)
 *     y += delta_dist × sin(heading + delta_theta/2)
 *     heading += delta_theta
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "odometry.h"
#include "drivers/encoder32A.h"
#include <math.h>

/* ==========================================================================
 *   Module Data
 * ========================================================================== */

static Encoder_State_t enc_state;
static int32_t prev_left_counts;
static int32_t prev_right_counts;

/* Control tick period in seconds (1 kHz → 1 ms) */
#define CTRL_PERIOD_S   0.001f

/* ==========================================================================
 *   Initialization
 * ========================================================================== */

/**
 * @brief  Initialize encoders and zero odometry state.
 */
void Encoder_Init(void)
{
    ENC32A_Init();

    enc_state.left_counts   = 0;
    enc_state.right_counts  = 0;
    enc_state.left_delta    = 0;
    enc_state.right_delta   = 0;
    enc_state.left_dist_mm  = 0.0f;
    enc_state.right_dist_mm = 0.0f;
    enc_state.left_vel_mm_s  = 0.0f;
    enc_state.right_vel_mm_s = 0.0f;
    enc_state.x_mm          = 0.0f;
    enc_state.y_mm          = 0.0f;
    enc_state.heading_rad   = 0.0f;
    enc_state.heading_deg   = 0.0f;

    prev_left_counts  = ENC0_ReadCount();
    prev_right_counts = ENC2_ReadCount();
}

/**
 * @brief  Reset only odometry (x, y, heading). Preserves encoder hardware state.
 */
void Encoder_ResetOdometry(void)
{
    enc_state.x_mm        = 0.0f;
    enc_state.y_mm        = 0.0f;
    enc_state.heading_rad = 0.0f;
    enc_state.heading_deg = 0.0f;
}

/* ==========================================================================
 *   Update — Call from 1 kHz control tick
 * ========================================================================== */

/**
 * @brief  Read encoders, compute deltas, velocities, and update odometry.
 * @note   Must be called at consistent 1 ms intervals for velocity accuracy.
 */
void Encoder_Update(void)
{
    int32_t curr_left;
    int32_t curr_right;
    float delta_left_mm;
    float delta_right_mm;
    float delta_dist;
    float delta_theta;
    float cos_h;
    float sin_h;

    /* [1] Read raw counts */
    curr_left  = ENC0_ReadCount();
    curr_right = ENC2_ReadCount();

    /* [2] Compute deltas (handle 32-bit wrap — A-ENC32 reloads at max) */
    enc_state.left_delta  = curr_left  - prev_left_counts;
    enc_state.right_delta = curr_right - prev_right_counts;

    prev_left_counts  = curr_left;
    prev_right_counts = curr_right;

    enc_state.left_counts  = curr_left;
    enc_state.right_counts = curr_right;

    /* [3] Convert to wheel distances */
    delta_left_mm  = (float)enc_state.left_delta  * ENC_MM_PER_COUNT;
    delta_right_mm = (float)enc_state.right_delta * ENC_MM_PER_COUNT;

    enc_state.left_dist_mm  += delta_left_mm;
    enc_state.right_dist_mm += delta_right_mm;

    /* [4] Velocities (mm/s) */
    enc_state.left_vel_mm_s  = delta_left_mm  / CTRL_PERIOD_S;
    enc_state.right_vel_mm_s = delta_right_mm / CTRL_PERIOD_S;

    /* [5] Odometry integration */
    delta_dist  = (delta_left_mm + delta_right_mm) * 0.5f;
    delta_theta = (delta_right_mm - delta_left_mm) / WHEEL_TRACK_MM;

    /* Use midpoint heading for integration (reduces error) */
    cos_h = cosf(enc_state.heading_rad + delta_theta * 0.5f);
    sin_h = sinf(enc_state.heading_rad + delta_theta * 0.5f);

    enc_state.x_mm += delta_dist * cos_h;
    enc_state.y_mm += delta_dist * sin_h;

    enc_state.heading_rad += delta_theta;
    enc_state.heading_deg  = enc_state.heading_rad * (180.0f / 3.14159265f);
}

/* ==========================================================================
 *   Accessors
 * ========================================================================== */

const Encoder_State_t* Encoder_GetState(void)
{
    return &enc_state;
}

float Encoder_GetLeftVel(void)
{
    return enc_state.left_vel_mm_s;
}

float Encoder_GetRightVel(void)
{
    return enc_state.right_vel_mm_s;
}

float Encoder_GetForwardVel(void)
{
    return (enc_state.left_vel_mm_s + enc_state.right_vel_mm_s) * 0.5f;
}

float Encoder_GetAngularVel(void)
{
    return (enc_state.right_vel_mm_s - enc_state.left_vel_mm_s) / WHEEL_TRACK_MM * (180.0f / 3.14159265f);
}