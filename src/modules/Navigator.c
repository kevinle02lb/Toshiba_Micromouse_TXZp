/**
 * @file        Navigator.c
 * @brief       Cell-level motion sequencer for micromouse.
 * @version     V1.0.0
 * @date        02-07-2026
 *
 * @details
 *   Bridges FloodFill (planner) with Motion/Odometry (execution).
 *   Runs a state machine at 1 kHz to sequence: plan → turn → drive → pause → repeat.
 *
 * @note
 *   Call Navigator_Update() exactly once per 1 kHz control tick.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "Navigator.h"
#include "FloodFill.h"
#include "Motion.h"
#include "Odometry.h"

#define CELL_SIZE_MM        180.0f
#define TURN_TOLERANCE_RAD  0.05f
#define DIST_TOLERANCE_MM   5.0f
#define PAUSE_TICKS         50U

/* ==========================================================================
 *   State Machine
 * ========================================================================== */

/**
 * @brief  Navigator internal states.
 */
typedef enum
{
    NAV_PLANNING,
    NAV_TURNING,
    NAV_DRIVING,
    NAV_PAUSED,
    NAV_FINISHED
} nav_state_t;

/* ==========================================================================
 *   Private Data
 * ========================================================================== */

static nav_state_t nav_state;
static floodfill_t last_action;

static float target_heading_rad;
static float drive_start_x_mm;
static float drive_start_y_mm;
static uint16_t pause_tick_count;

/* ==========================================================================
 *   Private Helpers
 * ========================================================================== */

/**
 * @brief  Normalize angle to [-pi, +pi].
 * @param  angle  Input angle in radians.
 * @return float  Normalized angle.
 */
static float NormalizeAngle(float angle)
{
    while (angle > M_PI)  
        angle -= M_2PI;
    while (angle < -M_PI) 
        angle += M_2PI;
    return angle;
}

/* ==========================================================================
 *   Public Functions
 * ========================================================================== */

/**
 * @brief  Initialize navigator state machine.
 * @note   Call once after FloodFill_Init() and Motion_Init().
 */
void Navigator_Init()
{
    nav_state = NAV_PLANNING;
    last_action = FLOODFILL_STOP;
    target_heading_rad = 0.0f;
    drive_start_x_mm = 0.0f;
    drive_start_y_mm = 0.0f;
    pause_tick_count = 0;
}

/**
 * @brief  Run one state machine step.
 * @note   Call exactly once per 1 kHz tick.
 */
void Navigator_Update(void)
{
    switch(nav_state)
    {
        /* ==============================================================
         *  NAV_PLANNING
         * ============================================================== */
        case (NAV_PLANNING):
            last_action = FloodFill_Plan();

            switch(last_action)
            {
                case (FLOODFILL_STOP):
                    Motion_Stop();
                    nav_state = NAV_FINISHED;
                    break;

                case (FLOODFILL_FORWARD):
                    drive_start_x_mm = Odometry_GetX_mm();
                    drive_start_y_mm = Odometry_GetY_mm();
                    Motion_SetMoveForwardSpeed(MOVE_SPEED);
                    nav_state = NAV_DRIVING;
                    break;
                
                case (FLOODFILL_TURN_LEFT):
                    target_heading_rad = Odometry_GetHeading_rad() + (M_PI_DIV_2);
                    Motion_SetTurnLeftSpeed(TURN_SPEED);
                    nav_state = NAV_TURNING;
                    break;
                
                case (FLOODFILL_TURN_RIGHT):
                    target_heading_rad = Odometry_GetHeading_rad() + (M_PI_DIV_2);
                    Motion_SetTurnRightSpeed(TURN_SPEED);
                    nav_state = NAV_TURNING;
                    break;

                case FLOODFILL_TURN_AROUND:
                    target_heading_rad = M_PI;
                    Motion_TurnLeft(TURN_SPEED);
                    nav_state = NAV_TURNING;
                    break;
            }
            break;


        default:
            break;
    }
}
