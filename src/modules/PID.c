/**
 * @file        PID.c
 * @brief       PID controller
 * @version     V1.0.0
 * @date        18-06-2026
 *
 * @details
 * 
 *   Formula:  output (MV) = Kp * error + Ki * integral + Kd * derivivative
 *   MV = manipulated variable
 * 
 *   error = e(t) = SP - PV
 *   Setpoint (SP)
 *   Process Variable (PV)
 * 
 * 
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "PID.h"


/* ==========================================================================
 *   Initialization
 * ========================================================================== */

/**
 * @brief  Initialize a PID controller with gains and output limits
 * @param  pid      Pointer to PID_t structure
 * @param  Kp       Proportional gain
 * @param  Ki       Integral gain
 * @param  Kd       Derivative gain
 * @param  dt       Sample time in seconds (should be 0.001f for 1 kHz)
 * @param  out_min  Minimum output clamp (e.g., -100.0f)
 * @param  out_max  Maximum output clamp (e.g., 100.0f)
 * @note  dt = 0.001f because the control loop runs at 1 kHz
 */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}


/* ==========================================================================
 *   PID Controller
 * ========================================================================== */

/**
 * @brief  Reset the PID controller state
 * @param  pid  Pointer to PID_t structure
 * @note  Resets integral and previous error to zero
 */
void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

