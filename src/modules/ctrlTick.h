/**
 * @file        ctrlTick.h
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


#ifndef CtrlTick_H
#define CtrlTick_H

#include <stdbool.h>

void CtrlTick_Init(void);
bool CtrlTick_GetAndClear(void);

#endif /* CtrlTick_H */