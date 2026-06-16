/**
 * @file        odometry.h
 * @brief       Wheel odometry and pose estimation for micromouse.
 * @version     V1.0.0
 * @date        16-06-2026
 *
 * @details
 *   High-level odometry module. Wraps A-ENC32 driver (encoder32A).
 *   Computes wheel distance, velocity, and robot pose from encoder counts.
 *
 *   Hardware:
 *   - Left encoder:  ENC0 (PN0/PN1), 12 PPR × 30:1 gear, 4× quadrature
 *   - Right encoder: ENC2 (PD3/PD4), same
 *   - Wheel: Pololu 80×10mm, circumference ≈ 251.33 mm
 *   - Counts per mm: 1440 / 251.33 ≈ 5.73
 *
 * @note
 *   Call @ref Odometry_Init() after @ref SystemInit().
 *   Call @ref Odometry_Update() from 1 kHz control tick.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 *   Physical Constants
 * ========================================================================== */

/**
 * @brief  Encoder counts per wheel revolution.
 * @note   Pololu #5211 N20 30:1 + 12 PPR magnetic encoder.
 *         A-ENC32 2-phase decode: 12 × 30 = 360 counts/rev.
 */
#define ENC_COUNTS_PER_REV      360U

/**
 * @brief  Wheel diameter in mm.
 * @note   Pololu 80×10mm wheel.
 */
#define WHEEL_DIAMETER_MM       80.0f

/**
 * @brief  Wheel circumference in mm.
 */
#define WHEEL_CIRCUMFERENCE_MM  (3.14159265f * WHEEL_DIAMETER_MM)

/**
 * @brief  Counts per mm.
 */
#define ENC_COUNTS_PER_MM       ((float)ENC_COUNTS_PER_REV / WHEEL_CIRCUMFERENCE_MM)

/**
 * @brief  mm per count.
 */
#define ENC_MM_PER_COUNT        (WHEEL_CIRCUMFERENCE_MM / (float)ENC_COUNTS_PER_REV)

/**
 * @brief  Wheel track (distance between wheel centers) in mm.
 * @note   MEASURE THIS ON YOUR CHASSIS. Typical micromouse: 65–80 mm.
 *         Placeholder — update after measuring.
 */
#define WHEEL_TRACK_MM          75.0f

/* ==========================================================================
 *   Data Structure
 * ========================================================================== */

/**
 * @brief  Encoder-derived robot state.
 */
typedef struct
{
    /* Wheel-level */
    int32_t  left_counts;       /*!< Raw encoder count (left) */
    int32_t  right_counts;      /*!< Raw encoder count (right) */
    int32_t  left_delta;        /*!< Counts since last update */
    int32_t  right_delta;       /*!< Counts since last update */
    float    left_dist_mm;      /*!< Cumulative distance (left wheel) */
    float    right_dist_mm;     /*!< Cumulative distance (right wheel) */
    float    left_vel_mm_s;     /*!< Instantaneous velocity (left) */
    float    right_vel_mm_s;    /*!< Instantaneous velocity (right) */

    /* Robot pose (odometry) */
    float    x_mm;              /*!< Global X position */
    float    y_mm;              /*!< Global Y position */
    float    heading_rad;       /*!< Heading angle (radians, CCW positive) */
    float    heading_deg;       /*!< Heading angle (degrees) */
} Encoder_State_t;

/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */

/**
 * @brief  Initialize encoders and odometry state.
 * @note   Calls ENC32A_Init() internally.
 */
void Encoder_Init(void);

/**
 * @brief  Update encoder readings and odometry.
 * @note   Call at 1 kHz from control tick. Computes deltas, velocities,
 *         and integrates pose.
 */
void Encoder_Update(void);

/**
 * @brief  Get pointer to current encoder state.
 * @return const Encoder_State_t*  Read-only access to internal state.
 */
const Encoder_State_t* Encoder_GetState(void);

/**
 * @brief  Reset odometry to (0, 0, 0).
 */
void Encoder_ResetOdometry(void);

/**
 * @brief  Get left wheel velocity (mm/s).
 */
float Encoder_GetLeftVel(void);

/**
 * @brief  Get right wheel velocity (mm/s).
 */
float Encoder_GetRightVel(void);

/**
 * @brief  Get robot forward velocity (mm/s).
 */
float Encoder_GetForwardVel(void);

/**
 * @brief  Get robot angular velocity (deg/s).
 */
float Encoder_GetAngularVel(void);

#endif /* ODOMETRY_H */