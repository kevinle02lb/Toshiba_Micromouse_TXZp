/**
 * @file        PID.c
 * @brief       PID controller
 * @version     V1.1.0
 * @date        18-07-2026
 *
 * @details
 *   Formula:  output (MV) = (kp * error) + (ki * integral) + (kd * derivative)
 *   MV = manipulated variable
 *
 *   error = e(t) = SP - PV
 *   Setpoint (SP)
 *   Process Variable (PV)
 *
 *   Derivative runs on PV, not error, so a setpoint step doesn't spike it.
 *   D term is low-pass filtered (PID_D_FILTER_ALPHA) against encoder noise.
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
 * @param  pid  Pointer to pid_t structure
 */
void PID_Create(pid_t *pid)
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
 * @param  pid      Pointer to pid_t structure
 * @param  kp       Proportional gain
 * @param  ki       Integral gain
 * @param  kd       Derivative gain
 * @param  dt       Sample time in seconds (should be 0.001f for 1 kHz)
 * @param  out_min  Minimum output clamp (e.g., -100.0f)
 * @param  out_max  Maximum output clamp (e.g., 100.0f)
 * @note  dt = 0.001f because the control loop runs at 1 kHz
 */
void PID_Init(pid_t *pid, float kp, float ki, float kd, float dt, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->d_filtered = 0.0f;
    pid->first_run = true;
}


/* ==========================================================================
 *   PID Controller
 * ========================================================================== */

/**
 * @brief  Reset the PID controller state
 * @param  pid  Pointer to pid_t structure
 * @note  Clears integral, derivative, and re-arms first_run so the next
 *        update seeds prev_measurement instead of spiking the D term
 */
void PID_Reset(pid_t *pid)
{
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->d_filtered = 0.0f;
    pid->first_run = true;
}


/**
 * @brief  Run one PID tick
 * @param  pid          Pointer to pid_t structure
 * @param  setpoint     Target value (SP)
 * @param  measurement  Actual value (PV)
 * @return              Clamped output in [out_min, out_max]
 */
float PID_Update(pid_t *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;
    float proportional_term, integral_term, derivative_term, output;
    float d_raw;

    // Seed PV history on first tick so derivative starts at 0
    if (pid->first_run)
    {
        pid->prev_measurement = measurement;
        pid->first_run = false;
    }

    // [1] Proportional
    proportional_term = pid->kp * error;

    // [2] Integral (accumulate, then clamp to spendable range)
    pid->integral += error * pid->dt;

    if (pid->ki != 0.0f)                             // skip clamp math if I-term unused
    {
        float max_integral = pid->out_max / pid->ki;
        float min_integral = pid->out_min / pid->ki;

        if (pid->integral > max_integral)
            pid->integral = max_integral;
        else if (pid->integral < min_integral)
            pid->integral = min_integral;
    }

    integral_term = pid->ki * pid->integral;

    // [3] Derivative on measurement (negated), low-pass filtered
    d_raw = -(measurement - pid->prev_measurement) / pid->dt;
    pid->d_filtered += PID_D_FILTER_ALPHA * (d_raw - pid->d_filtered);
    pid->derivative = pid->d_filtered;
    derivative_term = pid->kd * pid->d_filtered;

    // [4] Save measurement for next tick's derivative
    pid->prev_measurement = measurement;

    // [5] Sum and clamp
    output = proportional_term + integral_term + derivative_term;

    if (output > pid->out_max)
        output = pid->out_max;
    else if (output < pid->out_min)
        output = pid->out_min;

    return output;
}


/**
 * @brief  Calculates error as Setpoint - Process Variable
 * @param  SP  Target speed (CPS)
 * @param  PV  Actual speed (CPS)
 * @return     Error value (SP - PV)
 */
float CalculateError(float SP, float PV)
{
    return SP - PV;
}