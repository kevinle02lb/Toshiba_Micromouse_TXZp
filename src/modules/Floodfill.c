/**
 * @file        FloodFill.c
 * @brief       Flood fill maze solver for IEEE 16x16 micromouse.
 * @version     V1.0.0
 * @date        27-06-2026
 *
 * @details
 *   IEEE goal area: 2x2 block at maze center (cells 7-8, 7-8).
 *   BFS seeds all four goal cells simultaneously.
 *
 *   This module is a pure planner. It decides which direction to face next.
 *   The Navigator module executes motion and reports completion.
 *
 *   Algorithm Flow:
 *   1. FloodFill_Plan():
 *      a. Scan walls from IR sensors
 *      b. Run BFS from 2x2 goal area
 *      c. Pick lowest-distance neighbor
 *      d. Return action: FORWARD / LEFT / RIGHT / UTURN / STOP
 *   2. Navigator executes turn/drive physically
 *   3. Navigator calls FloodFill_ReportDone() when arrived
 *   4. Repeat until goal reached
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "FloodFill.h"
#include "IrSensor.h"

#define GOAL_MIN_X   7U
#define GOAL_MIN_Y   7U
#define GOAL_MAX_X   8U
#define GOAL_MAX_Y   8U

/* ==========================================================================
 *   Private Data
 * ========================================================================== */

static uint8_t walls[MAZE_SIZE][MAZE_SIZE];

/**
 * @brief  Distance to nearest goal tile for each cell.
 * @details
 *   - 0 = inside 2x2 goal area
 *   - 1 = adjacent to goal
 *   - 255 = unreachable
 */
static uint8_t dist[MAZE_SIZE][MAZE_SIZE];

static uint8_t mouse_x, mouse_y;
static direction_t mouse_heading;

/* ==========================================================================
 *   Queue
 * ========================================================================== */

#define QUEUE_SIZE  (MAZE_SIZE * MAZE_SIZE)

typedef struct
{
    uint8_t x;
    uint8_t y;
} Cell_t;

static Cell_t queue[QUEUE_SIZE];
static uint16_t queue_head, queue_tail;

/* ==========================================================================
 *   Queue Operations
 * ========================================================================== */

static void Queue_Init(void)
{
    queue_head = 0;
    queue_tail = 0;
}

static void Queue_Enqueue(uint8_t x, uint8_t y)
{
    if (queue_tail < QUEUE_SIZE)
    {
        queue[queue_tail].x = x;
        queue[queue_tail].y = y;
        queue_tail++;
    }
}

static bool Queue_Dequeue(Cell_t *cell)
{
    if (queue_head == queue_tail)
        return false;

    cell->x = queue[queue_head].x;
    cell->y = queue[queue_head].y;
    queue_head++;
    return true;
}

/* ==========================================================================
 *   Wall Helpers
 * ========================================================================== */

static uint8_t FloodFill_GetFrontWall(direction_t heading)
{
    switch (heading)
    {
        case NORTH: return WALL_NORTH_BIT;
        case EAST:  return WALL_EAST_BIT;
        case SOUTH: return WALL_SOUTH_BIT;
        case WEST:  return WALL_WEST_BIT;
        default:    return WALL_NORTH_BIT;
    }
}

static uint8_t FloodFill_GetLeftWall(direction_t heading)
{
    switch (heading)
    {
        case NORTH: return WALL_WEST_BIT;
        case EAST:  return WALL_NORTH_BIT;
        case SOUTH: return WALL_EAST_BIT;
        case WEST:  return WALL_SOUTH_BIT;
        default:    return WALL_WEST_BIT;
    }
}

static uint8_t FloodFill_GetRightWall(direction_t heading)
{
    switch (heading)
    {
        case NORTH: return WALL_EAST_BIT;
        case EAST:  return WALL_SOUTH_BIT;
        case SOUTH: return WALL_WEST_BIT;
        case WEST:  return WALL_NORTH_BIT;
        default:    return WALL_EAST_BIT;
    }
}

/* ==========================================================================
 *   Wall Management
 * ========================================================================== */

static void FloodFill_SetWall(uint8_t x, uint8_t y, wall_t wall_bit)
{
    walls[x][y] |= (uint8_t)wall_bit;

    switch(wall_bit)
    {
        case (WALL_NORTH_BIT):
            if (y + 1 < MAZE_SIZE)
                walls[x][y + 1] |= (uint8_t)WALL_SOUTH_BIT;
            break;

        case (WALL_EAST_BIT):
            if (x + 1 < MAZE_SIZE)
                walls[x + 1][y] |= (uint8_t)WALL_WEST_BIT;
            break;

        case (WALL_SOUTH_BIT):
            if (y > 0)
                walls[x][y - 1] |= (uint8_t)WALL_NORTH_BIT;
            break;

        case (WALL_WEST_BIT):
            if (x > 0)
                walls[x - 1][y] |= (uint8_t)WALL_EAST_BIT;
            break;

        default:
            break;
    }
}

/* ==========================================================================
 *   Scan Walls
 * ========================================================================== */

static void FloodFill_ScanWalls(void)
{
    bool front, left, right;

    IR_SampleAll();

    front = IR_IsWallPresent(IR_LEFT) && IR_IsWallPresent(IR_RIGHT);
    left  = IR_IsWallPresent(IR_FAR_LEFT);
    right = IR_IsWallPresent(IR_FAR_RIGHT);

    if (front) FloodFill_SetWall(mouse_x, mouse_y, (wall_t)FloodFill_GetFrontWall(mouse_heading));
    if (left)  FloodFill_SetWall(mouse_x, mouse_y, (wall_t)FloodFill_GetLeftWall(mouse_heading));
    if (right) FloodFill_SetWall(mouse_x, mouse_y, (wall_t)FloodFill_GetRightWall(mouse_heading));
}

/* ==========================================================================
 *   BFS Update
 * ========================================================================== */

static void FloodFill_Update(void)
{
    uint8_t x, y;
    Cell_t cell;

    for (x = 0; x < MAZE_SIZE; x++)
    {
        for (y = 0; y < MAZE_SIZE; y++)
        {
            dist[x][y] = CELL_UNVISITED;
        }
    }

    Queue_Init();
    Queue_Enqueue(GOAL_MIN_X, GOAL_MIN_Y);  dist[GOAL_MIN_X][GOAL_MIN_Y] = 0;
    Queue_Enqueue(GOAL_MAX_X, GOAL_MIN_Y);  dist[GOAL_MAX_X][GOAL_MIN_Y] = 0;
    Queue_Enqueue(GOAL_MIN_X, GOAL_MAX_Y);  dist[GOAL_MIN_X][GOAL_MAX_Y] = 0;
    Queue_Enqueue(GOAL_MAX_X, GOAL_MAX_Y);  dist[GOAL_MAX_X][GOAL_MAX_Y] = 0;

    while (Queue_Dequeue(&cell))
    {
        x = cell.x; y = cell.y;

        if ((y + 1 < MAZE_SIZE) && ((walls[x][y] & WALL_NORTH_BIT) == 0))
        {
            if (dist[x][y + 1] == CELL_UNVISITED)
            {
                dist[x][y + 1] = dist[x][y] + 1;
                Queue_Enqueue(x, y + 1);
            }
        }

        if ((x + 1 < MAZE_SIZE) && ((walls[x][y] & WALL_EAST_BIT) == 0))
        {
            if (dist[x + 1][y] == CELL_UNVISITED)
            {
                dist[x + 1][y] = dist[x][y] + 1;
                Queue_Enqueue(x + 1, y);
            }
        }

        if ((y > 0) && ((walls[x][y] & WALL_SOUTH_BIT) == 0))
        {
            if (dist[x][y - 1] == CELL_UNVISITED)
            {
                dist[x][y - 1] = dist[x][y] + 1;
                Queue_Enqueue(x, y - 1);
            }
        }

        if ((x > 0) && ((walls[x][y] & WALL_WEST_BIT) == 0))
        {
            if (dist[x - 1][y] == CELL_UNVISITED)
            {
                dist[x - 1][y] = dist[x][y] + 1;
                Queue_Enqueue(x - 1, y);
            }
        }
    }
}

/* ==========================================================================
 *   Pick Next Direction
 * ========================================================================== */

static direction_t FloodFill_GetNextDir(void)
{
    uint8_t best_dist;
    direction_t best_dir;

    best_dist = CELL_UNVISITED;
    best_dir  = mouse_heading;

    if ((mouse_y + 1 < MAZE_SIZE) && ((walls[mouse_x][mouse_y] & WALL_NORTH_BIT) == 0))
    {
        if (dist[mouse_x][mouse_y + 1] < best_dist)
        {
            best_dist = dist[mouse_x][mouse_y + 1];
            best_dir  = NORTH;
        }
    }

    if ((mouse_x + 1 < MAZE_SIZE) && ((walls[mouse_x][mouse_y] & WALL_EAST_BIT) == 0))
    {
        if (dist[mouse_x + 1][mouse_y] < best_dist)
        {
            best_dist = dist[mouse_x + 1][mouse_y];
            best_dir  = EAST;
        }
    }

    if ((mouse_y > 0) && ((walls[mouse_x][mouse_y] & WALL_SOUTH_BIT) == 0))
    {
        if (dist[mouse_x][mouse_y - 1] < best_dist)
        {
            best_dist = dist[mouse_x][mouse_y - 1];
            best_dir  = SOUTH;
        }
    }

    if ((mouse_x > 0) && ((walls[mouse_x][mouse_y] & WALL_WEST_BIT) == 0))
    {
        if (dist[mouse_x - 1][mouse_y] < best_dist)
        {
            best_dist = dist[mouse_x - 1][mouse_y];
            best_dir  = WEST;
        }
    }

    return best_dir;
}

/* ==========================================================================
 *   Public Functions
 * ========================================================================== */

/**
 * @brief  Initialize flood fill with Manhattan distances to nearest goal tile.
 * @details
 *   IEEE goal is 2x2 block at center (7,7) to (8,8).
 *   Mouse starts at (0,0) facing NORTH.
 */
void FloodFill_Init(void)
{
    uint8_t x, y;
    uint8_t dx, dy;

    mouse_x = 0;
    mouse_y = 0;
    mouse_heading = NORTH;

    for (x = 0; x < MAZE_SIZE; x++)
    {
        for (y = 0; y < MAZE_SIZE; y++)
        {
            walls[x][y] = 0;

            if (x < GOAL_MIN_X)      
                dx = GOAL_MIN_X - x;
            else if (x > GOAL_MAX_X) 
                dx = x - GOAL_MAX_X;
            else                     
                dx = 0;

            if (y < GOAL_MIN_Y)      
                dy = GOAL_MIN_Y - y;
            else if (y > GOAL_MAX_Y) 
                dy = y - GOAL_MAX_Y;
            else                       
                dy = 0;

            dist[x][y] = dx + dy;
        }
    }
}

/**
 * @brief  Check if mouse is inside the 2x2 goal area.
 */
bool FloodFill_IsAtGoal(void)
{
    return ((mouse_x == GOAL_MIN_X || mouse_x == GOAL_MAX_X) &&
            (mouse_y == GOAL_MIN_Y || mouse_y == GOAL_MAX_Y));
}

/**
 * @brief  Plan the next move.
 * @return flood_action_t  Action for Navigator to execute.
 * @details
 *   1. Scan walls with IR
 *   2. Run BFS from 2x2 goal
 *   3. Pick neighbor with lowest distance
 *   4. Compare to current heading, return relative action
 *
 *   Does NOT start any motion. Navigator must execute and report done.
 */
flood_action_t FloodFill_Plan(void)
{
    direction_t next_dir;

    if (FloodFill_IsAtGoal())
        return FLOOD_STOP;

    FloodFill_ScanWalls();
    FloodFill_Update();
    next_dir = FloodFill_GetNextDir();

    /* Update internal heading immediately. Position only updates on ReportDone. */
    if (next_dir == mouse_heading)
    {
        return FLOOD_FORWARD;
    }
    else if ((next_dir == NORTH && mouse_heading == EAST)  ||
             (next_dir == EAST  && mouse_heading == SOUTH) ||
             (next_dir == SOUTH && mouse_heading == WEST)  ||
             (next_dir == WEST  && mouse_heading == NORTH))
    {
        mouse_heading = next_dir;
        return FLOOD_LEFT;
    }
    else if ((next_dir == NORTH && mouse_heading == WEST)  ||
             (next_dir == WEST  && mouse_heading == SOUTH) ||
             (next_dir == SOUTH && mouse_heading == EAST)  ||
             (next_dir == EAST  && mouse_heading == NORTH))
    {
        mouse_heading = next_dir;
        return FLOOD_RIGHT;
    }
    else
    {
        mouse_heading = next_dir;
        return FLOOD_UTURN;
    }
}

/**
 * @brief  Report that the last planned action is physically complete.
 * @param  action  The action that was executed.
 * @details
 *   Only FLOOD_FORWARD changes cell position.
 *   Turns only changed heading, already done in Plan().
 */
void FloodFill_ReportDone(flood_action_t action)
{
    if (action == FLOOD_FORWARD)
    {
        switch (mouse_heading)
        {
            case NORTH: 
                mouse_y++; break;
            case EAST:  
                mouse_x++; break;
            case SOUTH: 
                mouse_y--; break;
            case WEST:  
                mouse_x--; break;
        }
    }
    /* LEFT, RIGHT, UTURN: heading already updated in Plan(), no position change */
}