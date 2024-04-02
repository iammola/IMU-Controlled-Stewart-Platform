/**
 * @file Maze.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "STATIONS.h"

#define CONNECTED_STATE_X      10
#define CONNECTED_STATE_Y      60
#define CONNECTED_STATE_WIDTH  185
#define CONNECTED_STATE_HEIGHT 20

#define CONTROL_METHOD_X      10
#define CONTROL_METHOD_Y      80
#define CONTROL_METHOD_WIDTH  215
#define CONTROL_METHOD_HEIGHT 20

/**
 * @brief
 * @param dataLength
 * @param buffer
 */
void Maze_UpdateControlMethod(MAZE_CONTROL_METHOD newControl);

/**
 * @brief
 * @param
 */
void Maze_MoveToPosition(void);

/**
 * @brief
 * @param connected
 */
void Maze_UpdateConnectedState(bool connected);
