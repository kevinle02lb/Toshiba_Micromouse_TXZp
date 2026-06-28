/**
 * @file        Motion.c
 * @brief       Speed control for the Micromouse
 * @version     V1.0.0
 * @date        27-06-2026
 *
 * @details
 *   Bridges Encoder feedback, PID control, and Motor output.
 *   Runs the inner speed loop at 1 kHz.
 *
 *   Caller sets targets via Motion_SetSpeed or convenience wrappers,
 *   then polls sensors/odometry and calls Motion_Stop() when done.
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "Motion.h"
#include "Encoder.h"
#include "PID.h"
#include "Motor.h"

/* ==========================================================================
 *   Private Data
 * ========================================================================== */

static PID_t pid_left;
static PID_t pid_right;

static float target_left = 0.0f;
static float target_right = 0.0f;

/* ==========================================================================
 *   Private Helpers
 * ========================================================================== */

/**
 * @brief  Convert PID output to motor direction and duty.
 * @param  motor   MOTOR_LEFT or MOTOR_RIGHT
 * @param  output  PID output in range [-100.0, +100.0]
 * @details
 *   Outputs below MOTION_DEADZONE are treated as brake.
 */
static void Motion_ApplyOutput(motor_t motor, float output)
{
    direction_t dir;
    uint8_t duty;

    if (output > MOTION_DEADZONE)
    {
        dir = FORWARD;
        duty = (uint8_t)(output + MOTION_ROUND_OFFSET);
    }
    else if (output < -MOTION_DEADZONE)
    {
        dir = REVERSE;
        duty = (uint8_t)(-output + MOTION_ROUND_OFFSET);
    }
    else
    {
        dir = BRAKE;
        duty = 0U;
    }

    Motor_Set(motor, dir, duty);
}

/* ==========================================================================
 *   Initialization
 * ========================================================================== */

/**
 * @brief  Initialize PID controllers.
 * @note   Call once after Encoder, Motor, and PID modules are initialized.
 */
void Motion_Init(void)
{
    PID_Create(&pid_left);
    PID_Create(&pid_right);

    target_left = 0.0f;
    target_right = 0.0f;
}

/* ==========================================================================
 *   Motion Update (1 kHz)
 * ========================================================================== */

/**
 * @brief  Run one PID control tick.
 * @details
 *   1. Read wheel speeds from Encoder
 *   2. Compute error = target - actual
 *   3. Run PID
 *   4. Apply to motors
 */
void Motion_Update(void)
{
    float actual_left, actual_right;
    float error_left, error_right;
    float output_left, output_right;

    actual_left = (float)Encoder_GetSpeed_CPS(MOTOR_LEFT);
    actual_right = (float)Encoder_GetSpeed_CPS(MOTOR_RIGHT);

    error_left = CalculateError(target_left, actual_left);
    error_right = CalculateError(target_right, actual_right);

    output_left = PID_Update(&pid_left, error_left);
    output_right = PID_Update(&pid_right, error_right);

    Motion_ApplyOutput(MOTOR_LEFT, output_left);
    Motion_ApplyOutput(MOTOR_RIGHT, output_right);
}

/* ==========================================================================
 *   Speed Commands
 * ========================================================================== */

/**
 * @brief  Set target wheel speeds.
 * @param  left_cps   Target left speed (CPS). Positive = forward.
 * @param  right_cps  Target right speed (CPS). Positive = forward.
 */
void Motion_SetSpeed(float left_cps, float right_cps)
{
    target_left = left_cps;
    target_right = right_cps;
}

/**
 * @brief  Move both wheels forward at the same speed.
 * @param  speed_cps  Target speed (CPS).
 */
void Motion_MoveForward(float speed_cps)
{
    Motion_SetSpeed(speed_cps, speed_cps);
}

/**
 * @brief  Move both wheels backward at the same speed.
 * @param  speed_cps  Target speed (CPS).
 */
void Motion_MoveBackward(float speed_cps)
{
    Motion_SetSpeed(-speed_cps, -speed_cps);
}

/**
 * @brief  Turn left (CCW) in place.
 * @param  speed_cps  Wheel speed during turn (CPS).
 */
void Motion_TurnLeft(float speed_cps)
{
    Motion_SetSpeed(-speed_cps, speed_cps);
}

/**
 * @brief  Turn right (CW) in place.
 * @param  speed_cps  Wheel speed during turn (CPS).
 */
void Motion_TurnRight(float speed_cps)
{
    Motion_SetSpeed(speed_cps, -speed_cps);
}

/**
 * @brief  Stop immediately.
 * @details  Resets PID, clears targets, applies motor brake.
 */
void Motion_Stop(void)
{
    PID_Reset(&pid_left);
    PID_Reset(&pid_right);
    target_left = 0.0f;
    target_right = 0.0f;
    Motor_Set(MOTOR_LEFT, STOP, 0U);
    Motor_Set(MOTOR_RIGHT, STOP, 0U);
}