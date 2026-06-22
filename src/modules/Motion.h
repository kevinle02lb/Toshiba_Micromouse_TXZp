/**
 * @file        Motion.h
 * @brief       Handles speed controlling and integegration of multiple modules
 * @version     V1.0.0
 * @date        18-06-2026
 *
 * @details
 * 
 * 
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "Motor.h"

#define MOTION_DEADZONE         0.5f        /*!< Below this, motor stops */
#define MOTION_ROUND_OFFSET     0.5f        /*!< For rounding float to nearest int */


/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */

void Motion_Init(void);
void Motion_Update(void);
void Motion_SetSpeed(float left_cps, float right_cps);
void Motion_Stop(void);
static void Motion_ApplyOutput(motor_t motor, float output);


#endif /* MOTION_H */