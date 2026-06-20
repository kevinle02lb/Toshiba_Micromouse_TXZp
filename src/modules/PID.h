/**
 * @file        PID.h
 * @brief       PID controller
 * @version     V1.0.0
 * @date        18-06-2026
 *
 * @details
 * @note
 *   
 *
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef PID_H
#define PID_H


typedef struct 
{
    float Kp;           /*!< Proportional gain */
    float Ki;           /*!< Integral gain */
    float Kd;           /*!< Derivative gain */
    float integral;     /*!< Accumulated integral term */
    float derivative;   /*!< Derivative of Error */
    float prev_error;   /*!< Previous error (for derivative) */
    float dt;           /*!< Sample time in seconds (0.001f) */
    float out_min;      /*!< Minimum output clamp (e.g., -100.0f) */
    float out_max;      /*!< Maximum output clamp (e.g., 100.0f) */
} PID_t;


/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max);
void PID_Reset(PID_t *pid);
float PID_Update(PID_t *pid, float error);


#endif /* PID_H */