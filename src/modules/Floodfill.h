/**
 * @file        FloodFill.h
 * @brief       Flood fill path planner for a micromouse maze.
 * @version     V1.0.0
 * @date        27-06-2026
 *
 * @details
 *   
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef FLOODFILL_H
#define FLOODFILL_H

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 *   Defines
 * ========================================================================== */

#define MAZE_SIZE                   16U
#define CELL_UNVISITED              255U        /*!< 2^8 - 1 = 256 - 1 = 255 */

typedef enum
{
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
} direction_t;

typedef enum
{
    WALL_NORTH_BIT  = 0x01U,   /*!< bit 0 */
    WALL_EAST_BIT   = 0x02U,   /*!< bit 1 */
    WALL_SOUTH_BIT  = 0x04U,   /*!< bit 2 */
    WALL_WEST_BIT   = 0x08U    /*!< bit 3 */
} wall_t;

/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */
void FloodFill_Init(uint8_t goal_x, uint8_t goal_y);
void FloodFill_Run(void);
bool FloodFill_IsAtGoal(void);


#endif /* FLOODFILL_H */