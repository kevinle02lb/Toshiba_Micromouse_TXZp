/**
 * @file        Motion.c
 * @brief       Speed control & Logic control for the Micromouse
 * @version     V1.0.0
 * @date        18-06-2026
 *
 * @details
 *   Logic Bridge control that combines Motor controls & PID control
 * 
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
 *   Private Variables
 * ========================================================================== */
static PID_t pid_left;                  /*!< PID controller for left motor */
static PID_t pid_right;                 /*!< PID controller for right motor */

static float target_left = 0.0f;        /*!< Target speed for left motor (CPS) */
static float target_right = 0.0f;       /*!< Target speed for right motor (CPS) */

/* ==========================================================================
 *   Initialization 
 * ========================================================================== */
/**
 * @brief  Initialize the motion control system
 * @note   Must be called once before any other Motion functions.
 *         Initializes PID controllers with default gains.
 */
void Motion_Init(void)
{
    PID_Create(&pid_left);
    PID_Create(&pid_right);

    target_left = 0.0f;
    target_right = 0.0f;
}

/**
 * @brief  Sets the target speed in Counts Per Second (CPS)
 * @param  left_cps     Sets left pid controller target
 * @param  right_cps    Sets right pid controller target
 * @note   
 */
void Motion_SetSpeed(float left_cps, float right_cps)
{
    target_left = left_cps;
    target_right = right_cps;
}


/**
 * @brief  Stops motion control
 * @note   
 */
void Motion_Stop(void)
{
    PID_Reset(&pid_left);
    PID_Reset(&pid_right);
    target_left = 0.0f;
    target_right = 0.0f;
    Motor_Set(MOTOR_LEFT, STOP, 0);
    Motor_Set(MOTOR_RIGHT, STOP, 0);
}

/* ==========================================================================
 *   Motion Update
 * ========================================================================== */
/**
 * @brief  Motion Update called repeatedly in control loop  
 * @note  
 */
void Motion_Update(void)
{
    float actual_left, actual_right;
    float error_left, error_right;
    float output_left, output_right;

    actual_left = (float)Encoder_GetSpeed_cps(MOTOR_LEFT);
    actual_right = (float)Encoder_GetSpeed_cps(MOTOR_RIGHT);

    error_left = CalculateError(&target_left, &actual_left);
    error_right = CalculateError(&target_right, &actual_right);

    output_left = PID_Update(&pid_left, error_left);
    output_right = PID_Update(&pid_right, error_right);

    Motion_ApplyOutput(MOTOR_LEFT, output_left);
    Motion_ApplyOutput(MOTOR_RIGHT, output_right);
}



/* ==========================================================================
 *   Motion Helper functions
 * ========================================================================== */
/**
 * @brief  Bridge between Motor and PID controller
 * @param motor
 * @param output
 * @note   
 */
static void Motion_ApplyOutput(motor_t motor, float output)
{
    direction_t dir;
    uint8_t duty;

    if (output > MOTION_DEADZONE) 
    {
        dir = FORWARD;
        duty = (uint8_t)(output + 0.5f);
    } 
    else if (output < -MOTION_DEADZONE) 
    {
        dir = REVERSE;
        duty = (uint8_t)(-output + 0.5f);
    } 
    else 
    {
        dir = BRAKE;
        duty = 0;
    }

    Motor_Set(motor, dir, duty);
}