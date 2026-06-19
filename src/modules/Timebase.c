/**
 * @file        Timebase.c
 * @brief       control Loop in main.c to indicate when logic should update.
 * @version     V1.0.0
 * @date        29-05-2026
 *
 * @details
 * @note
 *   
 *
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "Timebase.h"
#include "drivers/timer32A.h"

/**
 * @brief  Starts T32A1 Timer 
 * @note   
 */
void Timebase_Init(void)
{
    T32A1_Init();       /* Interval Timer Interrupt */
    T32A1_Start();      /* Timer already configured by T32A_Init() */
}

/**
 * @brief  
 * @note   
 */
bool Timebase_GetAndClear(void)
{
    __disable_irq();
    bool wasSet = T32A01AC_IRQ_Fire;
    if (wasSet) 
        T32A01AC_IRQ_Fire = false;
    __enable_irq();
    return wasSet;
}
