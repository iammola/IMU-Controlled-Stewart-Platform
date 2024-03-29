/**
 * @file STATIONS.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-22
 *
 * @copyright Copyright (c) 2024
 *
 */

typedef enum MAZE_CONTROL_METHOD {
  JOYSTICK_CTL_METHOD = 0xDE,
  IMU_CTL_METHOD = 0xAC,
} MAZE_CONTROL_METHOD;

static const MAZE_CONTROL_METHOD DEFAULT_CTL_METHOD = IMU_CTL_METHOD;
