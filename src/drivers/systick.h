/**
 * @file        systick.h
 * @brief       SysTick delay driver for TMPM4Ky RISC microcontrollers.
 * @version     V1.0.0
 * @date        29-05-2026
 *
 * @details
 *   Blocking delay prototypes. Uses ARM SysTick (core_cm4.h).
 *
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef SYSTICK_H
#define SYSTICK_H

#include "TMPM4KyA.h"   /* Includes core_cm4.h for SysTick */
#include <stdint.h>

void SysTick_us(uint32_t val);
void SysTick_ms(uint32_t val);

#endif /* SYSTICK_H */