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
#include "UTILS/UTILS.h"

#define CONNECTED_STATE_X      10
#define CONNECTED_STATE_Y      60
#define CONNECTED_STATE_WIDTH  185
#define CONNECTED_STATE_HEIGHT 20

#define CONTROL_METHOD_X      10
#define CONTROL_METHOD_Y      80
#define CONTROL_METHOD_WIDTH  215
#define CONTROL_METHOD_HEIGHT 20

#define MAZE_X             400
#define MAZE_Y             50
#define MAZE_CELL_SIZE     40
#define MAZE_COLUMNS_COUNT 9
#define MAZE_ROWS_COUNT    9
#define MAZE_WIDTH         (MAZE_COLUMNS_COUNT * MAZE_CELL_SIZE)
#define MAZE_HEIGHT        (MAZE_ROWS_COUNT * MAZE_CELL_SIZE)

#define SCREEN_TIME_X      10
#define SCREEN_TIME_Y      100
#define SCREEN_TIME_WIDTH  215
#define SCREEN_TIME_HEIGHT 20

typedef enum CONNECTED_STATE {
  CONNECTED = 0x81,
  DISCONNECTED = 0x96,
} CONNECTED_STATE;

typedef struct TIME {
  uint8_t  minutes;      // 0 to 60
  uint8_t  seconds;      // 0 to 60
  uint16_t milliseconds; // 0 to 999
  bool updated;
} TIME;

extern volatile TIME                time;
extern volatile Position            position;
extern volatile MAZE_CONTROL_METHOD CTL_METHOD;
extern volatile CONNECTED_STATE     connectionState;

/**
 * @brief
 * @param SYS_CLOCK
 */
void Maze_Init(const uint32_t SYS_CLOCK);

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
void Maze_MoveToNeutralPosition(void);

/**
 * @brief
 * @param
 */
void Maze_MoveToPosition(void);

/**
 * @brief
 * @param connected
 */
void Maze_UpdateConnectedState(CONNECTED_STATE connected);

/**
 * @brief 
 * @param  
 */
void Maze_UpdateGameTime(void);

/**
 * @brief
 * @param
 */
extern inline void Maze_ConfirmedInitialized(void);
