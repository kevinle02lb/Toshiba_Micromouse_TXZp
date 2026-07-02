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

/**
 * @brief  Bit position wall detection
 */
typedef enum
{
    WALL_NORTH_BIT  = 0x01U,   /*!< bit 0 */
    WALL_EAST_BIT   = 0x02U,   /*!< bit 1 */
    WALL_SOUTH_BIT  = 0x04U,   /*!< bit 2 */
    WALL_WEST_BIT   = 0x08U    /*!< bit 3 */
} wall_t;

/**
 * @brief  Action returned by FloodFill_Plan().
 */
typedef enum
{
    FLOOD_STOP = 0,     /* Mouse is inside the 2x2 goal area */
    FLOOD_FORWARD,      /* Already facing the best direction — drive straight */
    FLOOD_LEFT,         /* Turn 90° CCW, then drive */
    FLOOD_RIGHT,        /* Turn 90° CW, then drive */
    FLOOD_UTURN         /* Turn 180°, then drive */
} flood_action_t;

/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */
void FloodFill_Init(void);
flood_action_t FloodFill_Plan(void);
void FloodFill_ReportDone(flood_action_t action);
bool FloodFill_IsAtGoal(void);


#endif /* FLOODFILL_H */