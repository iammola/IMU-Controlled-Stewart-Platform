/**
 * @file Quaternion.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-20
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef QUATERNION_H
#define QUATERNION_H

typedef struct {
  float w;
  float x;
  float y;
  float z;
} Quaternion;

Quaternion normalizeQuaternion(float w, float x, float y, float z);

#endif
