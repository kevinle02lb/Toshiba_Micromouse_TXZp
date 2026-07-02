/**
 * @file        Navigator.c
 * @brief       
 * @version     V1.0.0
 * @date        02-07-2026
 *
 * @details
 *   
 * @note
 *   File structure and Doxygen formatting assisted by AI.
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

typedef enum
{
    NAV_PLANNING,
    NAV_TURNING,
    NAV_DRIVING,
    NAV_PAUSED,
    NAV_FINISHED
} nav_state_t;

static nav_state_t nav_state;
static flood_action_t last_action;

static float target_heading_rad;
static float drive_start_x_mm;
static float drive_start_y_mm;
static uint16_t pause_tick_count;

static float NormalizeAngle(float angle);

void Navigator_Init(void)
{
    nav_state = NAV_PLANNING;
    last_action = FLOOD_STOP;
    target_heading_rad = 0.0f;
    drive_start_x_mm = 0.0f;
    drive_start_y_mm = 0.0f;
    pause_tick_count = 0;
}

void Navigator_Update(void)
{
    switch (nav_state)
    {
        case NAV_PLANNING:
            last_action = FloodFill_Plan();

            switch (last_action)
            {
                case FLOOD_STOP:
                    Motion_Stop();
                    nav_state = NAV_FINISHED;
                    break;

                case FLOOD_FORWARD:
                    drive_start_x_mm = Odometry_GetX_mm();
                    drive_start_y_mm = Odometry_GetY_mm();
                    Motion_MoveForward(MOVE_SPEED);
                    nav_state = NAV_DRIVING;
                    break;

                case FLOOD_LEFT:
                    target_heading_rad = Odometry_GetHeading_rad() + (M_PI / 2.0f);
                    Motion_TurnLeft(TURN_SPEED);
                    nav_state = NAV_TURNING;
                    break;

                case FLOOD_RIGHT:
                    target_heading_rad = Odometry_GetHeading_rad() - (M_PI / 2.0f);
                    Motion_TurnRight(TURN_SPEED);
                    nav_state = NAV_TURNING;
                    break;

                case FLOOD_UTURN:
                    target_heading_rad = Odometry_GetHeading_rad() + M_PI;
                    Motion_TurnLeft(TURN_SPEED);
                    nav_state = NAV_TURNING;
                    break;
            }
            break;

        case NAV_TURNING:
        {
            float error = NormalizeAngle(target_heading_rad - Odometry_GetHeading_rad());
            if (fabsf(error) < TURN_TOLERANCE_RAD)
            {
                drive_start_x_mm = Odometry_GetX_mm();
                drive_start_y_mm = Odometry_GetY_mm();
                Motion_MoveForward(MOVE_SPEED);
                nav_state = NAV_DRIVING;
            }
            break;
        }

        case NAV_DRIVING:
        {
            float dx = Odometry_GetX_mm() - drive_start_x_mm;
            float dy = Odometry_GetY_mm() - drive_start_y_mm;
            float traveled = sqrtf((dx * dx) + (dy * dy));

            if (traveled >= (CELL_SIZE_MM - DIST_TOLERANCE_MM))
            {
                Motion_Stop();
                pause_tick_count = 0;
                nav_state = NAV_PAUSED;
            }
            break;
        }

        case NAV_PAUSED:
            if (++pause_tick_count >= PAUSE_TICKS)
            {
                FloodFill_ReportDone(last_action);
                nav_state = NAV_PLANNING;
            }
            break;

        case NAV_FINISHED:
        default:
            break;
    }
}

static float NormalizeAngle(float angle)
{
    while (angle > M_PI)  
        angle -= (2.0f * M_PI);
    while (angle < -M_PI) 
        angle += (2.0f * M_PI);
    return angle;
}