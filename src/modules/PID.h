/**
 * @file        PID.h
 * @brief       PID controller
 * @version     V1.1.0
 * @date        18-07-2026
 *
 * @details
 *   Derivative runs on PV (not error) so a setpoint step doesn't spike it,
 *   and is low-pass filtered against encoder noise.
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>


typedef struct
{
    float kp;                   /*!< Proportional gain */
    float ki;                   /*!< Integral gain */
    float kd;                   /*!< Derivative gain */
    float integral;             /*!< Accumulated integral term */
    float derivative;           /*!< Last filtered derivative */
    float prev_measurement;     /*!< PV from previous tick */
    float d_filtered;           /*!< Low-pass state of derivative */
    float dt;                   /*!< Sample time in seconds (0.001f) */
    float out_min;              /*!< Minimum output clamp (e.g., -100.0f) */
    float out_max;              /*!< Maximum output clamp (e.g., 100.0f) */
    bool  first_run;            /*!< Seeds prev_measurement after reset */
} pid_t;

/* ==========================================================================
 *   PID Default Gains
 * ========================================================================== */

#define PID_KP               0.5f
#define PID_KI               0.1f
#define PID_KD               0.02f
#define PID_DT               0.001f
#define PID_OUT_MIN          -100.0f
#define PID_OUT_MAX          100.0f

/*!< Derivative LPF coeff (EMA alpha). ~0.15 = ~30 Hz @ 1 kHz. Lower = smoother, more lag. */
#define PID_D_FILTER_ALPHA   0.15f


/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */
void  PID_Create(pid_t *pid);
void  PID_Init(pid_t *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max);
void  PID_Reset(pid_t *pid);
float PID_Update(pid_t *pid, float setpoint, float measurement);
float CalculateError(float SP, float PV);


#endif /* PID_H */