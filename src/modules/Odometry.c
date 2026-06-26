/**
 * @file        Odometry.c
 * @brief       Position tracking
 * @version     V1.0.0
 * @date        18-06-2026
 *
 * @details
 *   Uses Encoder to find positioning.
 *   Tracks robot position (X, Y) and heading angle using left/right
 *   encoder counts. Uses Pololu 80×10mm wheels (diameter = 80 mm).
 *
 *   Kinematic model (one control tick):
 *   -------------------------------------------------------------------------
 *   Let  d_L  = distance traveled by left wheel  during the tick (mm)
 *   Let  d_R  = distance traveled by right wheel during the tick (mm)
 *   Let  L    = wheelbase, distance between wheel contact points (mm)
 *   Let  theta = robot heading at the start of the tick (rad)
 *
 *   1. Heading change:
 *          delta_theta = (d_R - d_L) / L
 *
 *      Derivation: both wheels sweep the same angle delta_theta around the
 *      ICC (Instantaneous Center of Curvature), but on different radii
 *      (R +/- L/2).  Subtracting the two arc-length equations eliminates R
 *      and leaves delta_theta = (d_R - d_L) / L.
 *
 *   2. Center displacement:
 *          d_avg = (d_L + d_R) / 2
 *
 *      The robot's center moves along an arc of radius R by distance
 *      R * delta_theta.  Averaging the two wheel distances gives exactly
 *      that value.
 *
 *   3. Global coordinate update:
 *          x += d_avg * cos(theta)
 *          y += d_avg * sin(theta)
 *
 *      The scalar d_avg is projected onto the global X and Y axes using
 *      the heading angle.  This converts "forward motion in the robot
 *      frame" into "motion in the world frame".
 *   -------------------------------------------------------------------------
 *
 *   From Pololu Micro Metal Gearmotor Datasheet (Rev 6-2, Page 2):
 *
 *   "The two-channel Hall effect encoder senses the rotation of a 6-pole
 *    magnetic disc on a rear protrusion of the motor shaft, providing a
 *    resolution of 12 counts per revolution (CPR) of the motor shaft when
 *    counting both edges of both channels."
 *
 *   "To compute the gearbox output CPR, multiply 12 by the gearbox
 *    reduction factor."
 *
 *   Gearbox ratio: 30:1 (nominal)
 *   Exact gear ratio: 29.89:1 (from Page 3 gearbox table)
 *   Wheel CPR = 12 * 29.89 = 358.68 ~ 359 counts per wheel revolution.
 * 
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "Odometry.h"

Odometry_t odo;

#define AVG_DIVSOR          2

/* ==========================================================================
 *   Initialization 
 * ========================================================================== */

/**
 * @brief  Initialize odometry state
 * @note   Resets all position and heading values to zero
 */
void Odometry_Init(void)
{
    odo.x_mm = 0.0f;
    odo.y_mm = 0.0f;
    odo.heading_rad = 0.0f;
    odo.left_dist_mm = 0.0f;
    odo.right_dist_mm = 0.0f;
    odo.prev_left_pos = Encoder_GetPosition(MOTOR_LEFT);
    odo.prev_right_pos = Encoder_GetPosition(MOTOR_RIGHT);
}


/**
 * @brief  Reset odometry to zero
 * @note   Same as Odometry_Init()
 */
void Odometry_Reset(void)
{
    Odometry_Init();
}


/* ==========================================================================
 *   Odometry Update
 * ========================================================================== */
/**
 * @brief  Update odometry from encoder positions.
 * @details
 *   This is the heart of the differential-drive kinematics.  The function
 *   executes the following pipeline every 1 ms control tick:
 *
 *   Step 1 - Read encoders
 *   ----------------------
 *   Grab the absolute encoder counts from the hardware.
 *
 *   Step 2 - Compute deltas
 *   -----------------------
 *   delta = current - previous
 *   These deltas are in "counts" and represent how many encoder ticks
 *   happened during the last 1 ms.
 *
 *   Step 3 - Convert counts to physical distance
 *   --------------------------------------------
 *   d_L = delta_L * MM_PER_COUNT
 *   d_R = delta_R * MM_PER_COUNT
 *
 *   MM_PER_COUNT is the wheel circumference divided by counts per revolution.
 *   It bridges the abstract encoder world to the physical millimeter world.
 *
 *   Step 4 - Accumulate total wheel distances
 *   -----------------------------------------
 *   Optional odometer-style totals.  Useful for debugging or distance-
 *   triggered events.
 *
 *   Step 5 - Compute average forward distance
 *   -----------------------------------------
 *   d_avg = (d_L + d_R) / AVG_DIVISOR
 *
 *   This is how far the robot's center moved along its heading direction.
 *
 *   Step 6 - Compute heading change
 *   -------------------------------
 *   delta_heading = (d_R - d_L) / WHEELBASE_MM
 *
 *   This is the famous "right minus left over wheelbase" formula.
 *   If the right wheel traveled farther, the robot turned left (CCW),
 *   so delta_heading is positive.
 *
 *   Step 7 - Update heading
 *   -----------------------
 *   theta_new = theta_old + delta_heading
 *   Normalise to [-PI, +PI] to prevent unbounded growth.
 *
 *   Step 8 - Update global position
 *   -------------------------------
 *   x += d_avg * cos(theta)
 *   y += d_avg * sin(theta)
 *
 *   The scalar d_avg is projected onto the global X and Y axes using the
 *   heading angle.  cos(theta) gives the X fraction, sin(theta) gives the
 *   Y fraction.
 *
 *   Step 9 - Store state for next tick
 *   ----------------------------------
 *   prev_left_pos  = current_left
 *   prev_right_pos = current_right
 *
 * @note
 *   The current implementation uses the END-of-tick heading for the
 *   cos/sin projection (heading is updated before x/y).  For small
 *   delta_heading this is negligible.  For higher accuracy you may use the
 *   midpoint heading instead:
 *
 *       heading_avg = odo.heading_rad + delta_heading / 2.0f;
 *       odo.x_mm += avg_dist * cosf(heading_avg);
 *       odo.y_mm += avg_dist * sinf(heading_avg);
 *       odo.heading_rad += delta_heading;
 *
 *   This is sometimes called the "midpoint" or "Runge-Kutta" odometry
 *   update and is geometrically more accurate during fast turns.
 */
void Odometry_Update(void) {
    int32_t left_pos, right_pos;        // Current absolute encoder counts
    int32_t left_delta, right_delta;    // Counts since last tick
    float left_dist, right_dist;        // Wheel distances this tick (mm)
    float delta_heading;                // Change in heading (rad)
    float avg_dist;                     // Distance traveled by robot center

    // Step 1: Read current encoder positions
    left_pos = Encoder_GetPosition(MOTOR_LEFT);
    right_pos = Encoder_GetPosition(MOTOR_RIGHT);

    // Step 2: Calculate delta counts since last update
    left_delta = left_pos - odo.prev_left_pos;
    right_delta = right_pos - odo.prev_right_pos;

    // Step 3: Convert counts to distance (mm)
    //   d = counts * (mm / count)
    //   MM_PER_COUNT = pi * D / COUNTS_PER_REV
    left_dist = (float)left_delta * MM_PER_COUNT;
    right_dist = (float)right_delta * MM_PER_COUNT;

    // Step 4: Accumulate total wheel distances (odometer style)
    odo.left_dist_mm += left_dist;
    odo.right_dist_mm += right_dist;

    // Step 5: Average distance = center displacement
    //   d_avg = (d_L + d_R) / 2
    avg_dist = (left_dist + right_dist) / AVG_DIVISOR;

    // Step 6: Heading change from wheel difference
    //   delta_theta = (d_R - d_L) / L
    //
    //   Sign convention:
    //     right > left  ->  delta_theta > 0  ->  turn left (CCW)
    //     left  > right ->  delta_theta < 0  ->  turn right (CW)
    delta_heading = (right_dist - left_dist) / WHEELBASE_MM;

    // Step 7: Update heading and normalise to [-PI, PI]
    odo.heading_rad += delta_heading;

    if (odo.heading_rad > M_PI) 
    {
        odo.heading_rad -= M_2PI;
    } 
    else if (odo.heading_rad < -M_PI) 
    {
        odo.heading_rad += M_2PI;
    }

    // Step 8: Update global position
    //   delta_x = d_avg * cos(theta)
    //   delta_y = d_avg * sin(theta)
    //
    //   cos(theta) and sin(theta) project the forward scalar d_avg onto the
    //   global X and Y axes.
    odo.x_mm += avg_dist * cosf(odo.heading_rad);
    odo.y_mm += avg_dist * sinf(odo.heading_rad);

    // Step 9: Store encoder snapshots for the next tick
    odo.prev_left_pos = left_pos;
    odo.prev_right_pos = right_pos;
}


/* ==========================================================================
 *   Accessors / Getters
 * ========================================================================== */

/**
 * @brief  Get the global X coordinate.
 * @return float  X position in mm.
 * @note   Positive X is "forward" when heading = 0.
 */
float Odometry_GetX_mm(void) {
    return odo.x_mm;
}

/**
 * @brief  Get the global Y coordinate.
 * @return float  Y position in mm.
 * @note   Positive Y is "left" when heading = 0.
 */
float Odometry_GetY_mm(void) {
    return odo.y_mm;
}

/**
 * @brief  Get the heading in radians.
 * @return float  Heading angle in radians.
 * @details
 *   0 rad   = pointing along the global +X axis.
 *   +pi/2 rad = pointing along the global +Y axis (turned left / CCW).
 *   Range is normalised to [-PI, +PI].
 */
float Odometry_GetHeading_rad(void) {
    return odo.heading_rad;
}

/**
 * @brief  Get the heading in degrees.
 * @return float  Heading angle in degrees.
 * @note   0 deg = forward, +90 deg = left, -90 deg = right.
 */
float Odometry_GetHeading_deg(void) {
    return odo.heading_rad * RAD_TO_DEG;
}