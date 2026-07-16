/**
 * @file        systick.c
 * @brief       SysTick delay driver for TMPM4Ky RISC microcontrollers.
 * @version     V1.0.0
 * @date        29-05-2026
 *
 * @details
 *   Blocking delay using ARM SysTick timer. No ISR required.
 *   System clock: 160 MHz → 160 cycles = 1 µs.
 *
 *   Reference:
 *   - ARM DDI0403: https://developer.arm.com/documentation/ddi0403/ee
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "systick.h"

/* ==========================================================================
 *   Delay Functions
 * ========================================================================== */

/**
 * @brief  Blocking delay in microseconds.
 * @param  val  Delay in µs (max ~104 ms at 160 MHz).
 *
 *   LOAD = (val × 160) - 1 cycles.
 *   Uses processor clock (CLKSOURCE = 1), no interrupt (TICKINT = 0).
 */
void SysTick_us(uint32_t val)
{
    if (val == 0U) { return; }

    /* Clamp to max LOAD (0xFFFFFF) */
    if (val > (SysTick_LOAD_RELOAD_Msk / 160U)) 
    {
        val = SysTick_LOAD_RELOAD_Msk / 160U;
    }

    SysTick->LOAD = (uint32_t)((val * 160U) - 1U);
    SysTick->VAL  = 0;                                            /* Clear current value */
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk |                 /* Processor clock */
                     SysTick_CTRL_ENABLE_Msk);                    /* Start counter */

    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) { ; }   /* Poll until done */

    SysTick->CTRL = 0;                                            /* Disable for next call */
}

/**
 * @brief  Blocking delay in milliseconds.
 * @param  val  Delay in ms.
 * @note   Loops SysTick_us(1000) because max single delay is ~104 ms.
 */
void SysTick_ms(uint32_t val)
{
    for (uint32_t i = 0U; i < val; i++) {
        SysTick_us(1000U);
    }
}