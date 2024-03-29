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

#include <stdint.h>

#include "STATIONS.h"

/**
 * @brief 
 * @param dataLength 
 * @param buffer 
 */
void Maze_ChangeControlMethod(uint8_t dataLength, uint8_t *buffer);

/**
 * @brief 
 * @param dataLength 
 * @param buffer 
 */
void Maze_ReadNewPosition(uint8_t dataLength, uint8_t *buffer);

/**
 * @brief
 * @param
 */
void Maze_MoveToPosition(void);
