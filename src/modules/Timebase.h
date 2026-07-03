/**
 * @file        Timebase.h
 * @brief       ctrlTick in main.c to indicate when logic should update.
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


#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdbool.h>

void Timebase_Init(void);
bool Timebase_GetAndClear(void);

#endif /* TIMEBASE_H */