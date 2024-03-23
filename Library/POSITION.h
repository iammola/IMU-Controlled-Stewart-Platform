/**
 * @file POSITION.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef POSITION__H

#include <stdbool.h>

#include "Quaternion/Quaternion.h"

#define POSITION__H

typedef struct Coords {
  float x;
  float y;
  float z;
} Coords;

typedef struct Position {
  Quaternion quaternion;
  Coords     translation;
  bool       isNew;
} Position;

#define POSITION_BYTE_SIZE 28 // (4 bytes per flaot * (4 quaternion floats) + (3 translation floats))

#endif
