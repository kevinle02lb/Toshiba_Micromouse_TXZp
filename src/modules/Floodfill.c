/**
 * @file        FloodFill.c
 * @brief       Flood fill maze solver implementation.
 * @version     V1.0.0
 * @date        27-06-2026
 *
 * @details
 *   Flood fill algorithm for 16x16 micromouse maze.
 *
 *   Key concepts:
 *
 *   Manhattan Distance: |goal_x - x| + |goal_y - y|
 *   - Used for initialization
 *   - Assumes no walls exist (best-case scenario)
 *
 *   BFS (Breadth-First Search):
 *   - Explores the grid level by level from the goal
 *   - Respects discovered walls
 *   - Each cell gets its shortest distance to goal
 *   - Uses a FIFO queue (first-in, first-out)
 *
 *   Queue Operations:
 *   - Enqueue: Add cell to back of queue
 *   - Dequeue: Remove cell from front of queue
 *   - This ensures level-by-level exploration
 *
 *   Algorithm Flow:
 *   1. Initialize with Manhattan distances
 *   2. Each FloodFill_Run() call:
 *      a. Scan walls from IR sensors
 *      b. Run BFS from goal to recalculate distances
 *      c. Pick lowest-distance neighbor
 *      d. Turn and move to that cell
 *   3. Repeat until goal reached
 *
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */


#include "FloodFill.h"
#include "Motion.h"
#include "Odometry.h"
#include "IrSensor.h"

/* ==========================================================================
 *   Private Data
 * ========================================================================== */
/**
 * @brief  walls[16][16] - Wall bitmask for each cell
 * @details
 *   Each cell stores 4 bits:
 *   - Bit 0 (0x01): North wall
 *   - Bit 1 (0x02): East wall
 *   - Bit 2 (0x04): South wall
 *   - Bit 3 (0x08): West wall
 *
 *   Example: walls[3][5] = 0x0B means:
 *   - North (0x01): yes
 *   - East  (0x02): yes
 *   - South (0x04): no
 *   - West  (0x08): yes
 *
 *   Only SOUTH is open.
 */
static uint8_t walls[MAZE_SIZE][MAZE_SIZE];

/**
 * @brief  dist[16][16] - Distance to goal for each cell
 * @details
 *   - 0 = goal cell
 *   - 1 = adjacent to goal
 *   - 2 = two steps from goal
 *   - ...
 *   - 255 (CELL_UNVISITED) = unreachable (blocked by walls)
 *
 *   Values are calculated by BFS from the goal.
 *   The mouse always moves to the neighbor with the lowest dist.
 */
static uint8_t dist[MAZE_SIZE][MAZE_SIZE];


/**
 * @brief  Mouse position and heading
 * @details
 *   - mouse_x, mouse_y: Current cell (0-15)
 *   - mouse_heading: Which way the mouse faces (NORTH/EAST/SOUTH/WEST)
 *
 *   These are updated by FloodFill_Move() after each move.
 *   They are NOT read from Odometry to keep the module self-contained.
 */
static uint8_t mouse_x, mouse_y;
static direction_t mouse_heading;

static uint8_t goal_x, goal_y;


/* ==========================================================================
 *   Queue
 * ========================================================================== */
/**
 * @brief  Queue for BFS (Breadth-First Search)
 * @details
 *   BFS uses a FIFO queue to process cells level by level:
 *
 *   1. Start with goal cell in queue
 *   2. Dequeue a cell, check its 4 neighbors
 *   3. If a neighbor is reachable (no wall) and unvisited:
 *      - Set its distance = current distance + 1
 *      - Enqueue it
 *   4. Repeat until queue is empty
 *
 *   QUEUE_SIZE = 256 (max cells in 16x16 maze)
 *   No overflow possible if BFS is correct.
 */
#define QUEUE_SIZE         (MAZE_SIZE * MAZE_SIZE)

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
/**
 * @brief  Reset queue to empty
 * @details
 *   Sets head = tail = 0. Queue is empty when head == tail.
 */
static void Queue_Init(void)
{
    queue_head = 0;
    queue_tail = 0;
}

/**
 * @brief  Add cell to back of queue.
 * @param  x  Cell x coordinate
 * @param  y  Cell y coordinate
 * @details
 *   Enqueue = add to tail. tail increments.
 *   If tail reaches QUEUE_SIZE, queue is full (shouldn't happen).
 */
static void Queue_Enqueue(uint8_t x, uint8_t y)
{
    if (queue_tail < QUEUE_SIZE)
    {
        queue[queue_tail].x = x;
        queue[queue_tail].y = y;
        queue_tail++;
    }
}

/**
 * @brief  Remove cell from front of queue.
 * @param  cell  Pointer to store dequeued cell
 * @return bool  true if cell was dequeued, false if queue empty
 * @details
 *   Dequeue = remove from head. head increments.
 *   Returns false if head == tail (queue empty).
 */
static bool Queue_Dequeue(Cell_t *cell)
{
    if (queue_head == queue_tail)
    {
        return false;   /* queue empty */
    }

    cell->x = queue[queue_head].x;
    cell->y = queue[queue_head].y;
    queue_head++;
    return true;
}


/* ==========================================================================
 *   Private Helper
 * ========================================================================== */

/**
 * @brief  Get absolute wall bit for "front" relative to heading.
 * @param  heading  Current mouse heading (NORTH/EAST/SOUTH/WEST)
 * @return uint8_t  Corresponding wall bit
 *
 * @details
 *   Example: If mouse faces NORTH, front wall = NORTH wall.
 *            If mouse faces EAST,  front wall = EAST wall.
 *   This converts the mouse's relative "front" to an absolute direction.
 */
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

/**
 * @brief  Get absolute wall bit for "left" relative to heading.
 * @param  heading  Current mouse heading (NORTH/EAST/SOUTH/WEST)
 * @return uint8_t  Corresponding wall bit
 *
 * @details
 *   Example: If mouse faces NORTH, left wall = WEST wall.
 *            If mouse faces EAST,  left wall = NORTH wall.
 */
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

/**
 * @brief  Get absolute wall bit for "right" relative to heading.
 * @param  heading  Current mouse heading (NORTH/EAST/SOUTH/WEST)
 * @return uint8_t  Corresponding wall bit
 *
 * @details
 *   Example: If mouse faces NORTH, right wall = EAST wall.
 *            If mouse faces EAST,  right wall = SOUTH wall.
 */
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


/**
 * @brief  Sample IR sensors and store discovered walls.
 * @details
 *   Reads all 4 IR sensors and detects walls:
 *   - FRONT wall: Both IR_LEFT and IR_RIGHT detect a wall
 *   - LEFT wall:  IR_FAR_LEFT detects a wall
 *   - RIGHT wall: IR_FAR_RIGHT detects a wall
 *
 *   Then converts relative walls (front/left/right) to absolute walls
 *   (NORTH/EAST/SOUTH/WEST) based on mouse_heading.
 *
 *   Stores walls using FloodFill_SetWall() which also writes
 *   symmetric walls to neighboring cells.
 */
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

/**
 * @brief  Recalculate distances from goal using BFS.
 * @details
 *   This is the core BFS (Breadth-First Search) flood fill algorithm.
 *
 *   Steps:
 *   1. Reset all distances to CELL_UNVISITED (255)
 *   2. Start with goal cell: distance = 0
 *   3. Enqueue goal cell
 *   4. While queue not empty:
 *      a. Dequeue a cell
 *      b. For each of 4 neighbors:
 *         - Check if neighbor is within maze bounds
 *         - Check if wall blocks the direction
 *         - If no wall and unvisited (dist == 255):
 *             Set dist[neighbor] = dist[current] + 1
 *             Enqueue neighbor
 *
 *   Result: Each reachable cell has its shortest distance to goal.
 *   The mouse picks the neighbor with the lowest dist value.
 */
static void FloodFill_Update(void)
{
    uint8_t x,y;
    Cell_t cell;

    /* Reset all Distances */
    for (x = 0 ; x < MAZE_SIZE ; x++)
    {
        for (y = 0 ; y < MAZE_SIZE ; y++)
        {
            dist[x][y] = CELL_UNVISITED;
        }
    }

    /* [1] Start BFS from goal */
    Queue_Init();
    Queue_Enqueue(goal_x, goal_y);
    dist[goal_x][goal_y] = 0;

    /* Process queue */
    while (Queue_Dequeue(&cell))
    {
        x = cell.x; y = cell.y;

        /* NORTH neighbor */
        if ( (y + 1 < MAZE_SIZE) && ((walls[x][y] & WALL_NORTH_BIT) == 0) )
        {
            if ( dist[x][y + 1] == CELL_UNVISITED)
            {
                dist[x][y + 1] = dist[x][y] + 1;
                Queue_Enqueue(x , y + 1);
            }
        }

        /* EAST neighbor */
        if ((x + 1 < MAZE_SIZE) && ((walls[x][y] & WALL_EAST_BIT) == 0))
        {
            if (dist[x + 1][y] == CELL_UNVISITED)
            {
                dist[x + 1][y] = dist[x][y] + 1;
                Queue_Enqueue(x + 1, y);
            }
        }

        /* SOUTH neighbor */
        if ((y > 0) && ((walls[x][y] & WALL_SOUTH_BIT) == 0))
        {
            if (dist[x][y - 1] == CELL_UNVISITED)
            {
                dist[x][y - 1] = dist[x][y] + 1;
                Queue_Enqueue(x, y - 1);
            }
        }

        /* WEST neighbor */
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

/**
 * @brief  Pick the neighbor with the lowest distance to goal.
 * @return direction_t  Best direction to move (NORTH/EAST/SOUTH/WEST)
 * @details
 *   Checks all 4 neighbors of the current cell.
 *   - Must be within maze bounds (0-15)
 *   - Must not be blocked by a known wall
 *   - Picks the one with the smallest dist value
 *
 *   If multiple neighbors have the same lowest distance,
 *   the first one checked wins (NORTH, EAST, SOUTH, WEST order).
 *
 *   The mouse always moves "downhill" toward lower distances,
 *   which guarantees it's following the shortest path to goal.
 */
static direction_t FloodFill_GetNextDir(void)
{
    uint8_t best_dist;
    direction_t best_dir;

    best_dist = CELL_UNVISITED;
    best_dir = mouse_heading;

    if ((mouse_y + 1 < MAZE_SIZE) && ((walls[mouse_x][mouse_y] & WALL_NORTH_BIT) == 0))
    {
        if (dist[mouse_x][mouse_y + 1] < best_dist)
        {
            best_dist = dist[mouse_x][mouse_y + 1];
            best_dir = NORTH;
        }
    }

    if ((mouse_x + 1 < MAZE_SIZE) && ((walls[mouse_x][mouse_y] & WALL_EAST_BIT) == 0))
    {
        if (dist[mouse_x + 1][mouse_y] < best_dist)
        {
            best_dist = dist[mouse_x + 1][mouse_y];
            best_dir = EAST;
        }
    }

    if ((mouse_y > 0) && ((walls[mouse_x][mouse_y] & WALL_SOUTH_BIT) == 0))
    {
        if (dist[mouse_x][mouse_y - 1] < best_dist)
        {
            best_dist = dist[mouse_x][mouse_y - 1];
            best_dir = SOUTH;
        }
    }

    if ((mouse_x > 0) && ((walls[mouse_x][mouse_y] & WALL_WEST_BIT) == 0))
    {
        if (dist[mouse_x - 1][mouse_y] < best_dist)
        {
            best_dist = dist[mouse_x - 1][mouse_y];
            best_dir = WEST;
        }
    }

    return best_dir;
}

/**
 * @brief  Turn to face a direction, move forward, update position.
 * @param  next_dir  Direction to move (NORTH/EAST/SOUTH/WEST)
 * @details
 *   Steps:
 *   1. Determine required turn based on current heading:
 *      - Same direction: no turn
 *      - Left: Motion_TurnLeft()
 *      - Right: Motion_TurnRight()
 *      - 180°: Two left turns (or two right turns)
 *   2. Update mouse_heading to next_dir
 *   3. Move forward one cell: Motion_MoveForward()
 *   4. Update mouse_x, mouse_y based on heading
 *
 *   The caller (FloodFill_Run) decides which direction to move.
 *   This function executes the movement and tracks position internally.
 */
static void FloodFill_Move(direction_t next_dir)
{
    switch (next_dir)
    {
        case NORTH:
            if (mouse_heading == EAST)       Motion_TurnLeft(TURN_SPEED);
            else if (mouse_heading == WEST)  Motion_TurnRight(TURN_SPEED);
            else if (mouse_heading == SOUTH) { Motion_TurnLeft(TURN_SPEED); Motion_TurnLeft(TURN_SPEED); }
            break;

        case EAST:
            if (mouse_heading == NORTH)      Motion_TurnRight(TURN_SPEED);
            else if (mouse_heading == SOUTH) Motion_TurnLeft(TURN_SPEED);
            else if (mouse_heading == WEST)  { Motion_TurnLeft(TURN_SPEED); Motion_TurnLeft(TURN_SPEED); }
            break;

        case SOUTH:
            if (mouse_heading == EAST)       Motion_TurnRight(TURN_SPEED);
            else if (mouse_heading == WEST)  Motion_TurnLeft(TURN_SPEED);
            else if (mouse_heading == NORTH) { Motion_TurnLeft(TURN_SPEED); Motion_TurnLeft(TURN_SPEED); }
            break;

        case WEST:
            if (mouse_heading == NORTH)      Motion_TurnLeft(TURN_SPEED);
            else if (mouse_heading == SOUTH) Motion_TurnRight(TURN_SPEED);
            else if (mouse_heading == EAST)  { Motion_TurnLeft(TURN_SPEED); Motion_TurnLeft(TURN_SPEED); }
            break;
    }

    mouse_heading = next_dir;

    Motion_MoveForward(MOVE_SPEED);

    switch (mouse_heading)
    {
        case NORTH: mouse_y++; break;
        case EAST:  mouse_x++; break;
        case SOUTH: mouse_y--; break;
        case WEST:  mouse_x--; break;
    }
}

/* ==========================================================================
 *   Functions
 * ========================================================================== */
/**
 * @brief  Initialize flood fill with Manhattan distances.
 * @param  gx  Goal x coordinate (0-15)
 * @param  gy  Goal y coordinate (0-15)
 *
 * @details
 *   Manhattan distance: |goal_x - x| + |goal_y - y|
 *
 *   Example: Goal at (7,7), cell at (0,0):
 *   dx = |7 - 0| = 7, dy = |7 - 0| = 7, dist = 14
 *
 *   This gives the "best case" distance if no walls exist.
 *   As walls are discovered, FloodFill_Update() will recalculate
 *   distances using BFS to reflect actual shortest paths.
 *
 *   Also clears all walls and positions mouse at (0,0) facing NORTH.
 */
void FloodFill_Init(uint8_t gx, uint8_t gy)
{
    uint8_t x, y;
    uint8_t dx, dy;

    goal_x = gx; goal_y = gy;

    mouse_x = 0; mouse_y = 0;
    mouse_heading = NORTH;

    for (x = 0 ; x < MAZE_SIZE ; x++)
    {
        for (y = 0 ; y < MAZE_SIZE ; y++)
        {
            walls[x][y] = 0;

            /* Manhattan distance: |goal_x - x| + |goal_y - y| */
            dx = (x > goal_x) ? (x - goal_x) : (goal_x - x);
            dy = (y > goal_y) ? (y - goal_y) : (goal_y - y);

            dist[x][y] = dx + dy;
        }
    }
}


/* ==========================================================================
 *   Wall Management
 * ========================================================================== */
/**
 * @brief  Store a wall and its symmetric neighbor wall.
 * @param  x         Cell x coordinate (0-15)
 * @param  y         Cell y coordinate (0-15)
 * @param  wall_bit  WALL_NORTH_BIT, WALL_EAST_BIT, WALL_SOUTH_BIT, or WALL_WEST_BIT
 * @details
 *   Writes wall to current cell AND the adjacent cell.
 *
 *   Example: NORTH wall at (3,5) also writes SOUTH wall at (3,6).
 *   This is because walls are shared between cells:
 *   - The north edge of cell (3,5) is the south edge of cell (3,6)
 *
 *   Without this symmetry, BFS would think cells on the other side
 *   are open when they're actually blocked.
 *
 *   Bounds checks prevent writing outside the 16x16 maze.
 */
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
 *   FloodFill Run
 * ========================================================================== */

/**
 * @brief  One exploration step: scan walls, flood, decide move.
 * @details
 *   This is the main function called from the control loop.
 *
 *   Each call performs one complete exploration cycle:
 *
 *   1. FloodFill_ScanWalls():
 *      - Sample IR sensors
 *      - Detect front/left/right walls
 *      - Convert to absolute (N/E/S/W)
 *      - Store in walls array
 *
 *   2. FloodFill_Update():
 *      - Reset all distances to 255
 *      - BFS from goal
 *      - Fill dist[][] with shortest distances
 *
 *   3. FloodFill_GetNextDir():
 *      - Check 4 neighbors
 *      - Pick one with lowest dist
 *
 *   4. FloodFill_Move():
 *      - Turn to face direction
 *      - Move forward one cell
 *      - Update position
 *
 *   This repeats until the goal is reached.
 */
void FloodFill_Run(void)
{
    direction_t next_dir;

    FloodFill_ScanWalls();
    FloodFill_Update();
    next_dir = FloodFill_GetNextDir();
    FloodFill_Move(next_dir);
}