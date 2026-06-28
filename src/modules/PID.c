/**
 * @file        PID.c
 * @brief       PID controller
 * @version     V1.0.0
 * @date        18-06-2026
 *
 * @details
 * 
 *   Formula:  output (MV) = (Kp * error) + (Ki * integral) + (Kd * derivivative)
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
 * @brief  Create and initialize a PID controller with default gains
 * @param  pid  Pointer to PID_t structure
 */
void PID_Create(PID_t *pid)
{
    PID_Init(pid,
             PID_KP,
             PID_KI,
             PID_KD,
             PID_DT,
             PID_OUT_MIN,
             PID_OUT_MAX);
}


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
    pid->derivative = 0.0f;
    pid->prev_error = 0.0f;
}


/* ==========================================================================
 *   PID Controller
 * ========================================================================== */

/**
 * @brief  Reset the PID controller state
 * @param  pid  Pointer to PID_t structure
 * @note  Resets integral, deriviative, and previous error to zero
 */
void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->prev_error = 0.0f;
}



/**
 * @brief  
 * @param  pid  
 * @note  
 */
float PID_Update(PID_t *pid, float error)
{
    float proportional_term, integral_term, derivative_term, output;

    /* [1] Proportional Term */
    proportional_term = pid->Kp * error;

    /* [2] Integral Term */
    pid->integral += error * pid->dt;

    /* Integral clamping (anti-windup) - only if Ki is non-zero */
    if (pid->Ki != 0.0f) 
    {
        float max_integral = pid->out_max / pid->Ki;
        float min_integral = pid->out_min / pid->Ki;
        if (pid->integral > max_integral) 
        {
            pid->integral = max_integral;
        } 
        else if (pid->integral < min_integral) 
        {
            pid->integral = min_integral;
        }
    }
    
    integral_term = pid->Ki * pid->integral;

    /* [3] Derivative Term */
    pid->derivative = (error - pid->prev_error) / pid->dt;
    derivative_term = pid->Kd * pid->derivative;
    
    /* [4] Store current error for next cycle */
    pid->prev_error = error;

    /* [5] Output */
    output = proportional_term + integral_term + derivative_term;

    /* Clamp output */
    if (output > pid->out_max)
        output = pid->out_max;
    else if (output < pid->out_min)
        output = pid->out_min;
    
    return output;
}


/**
 * @brief  Calculates error as Setpoint - Process Variable
 * @param  SP  Pointer to target speed (CPS)
 * @param  PV  Pointer to actual speed (CPS)
 * @return     Error value (SP - PV)
 */
float CalculateError(float SP, float PV)
{
    return SP - PV;
}