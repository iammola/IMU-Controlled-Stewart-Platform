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
#include <math.h>

#ifndef QUATERNION_H
#define QUATERNION_H

typedef struct {
  float w;
  float x;
  float y;
  float z;
} Quaternion;

#define sqr(g) (g * g)

/**
 * @brief Normalizes a Quaternion
 * @param w
 * @param x
 * @param y
 * @param z
 * @return
 */
static inline Quaternion normalizeQuaternion(float w, float x, float y, float z) {
  Quaternion result = {0};
  float      norm = sqrtf(sqr(w) + sqr(x) + sqr(y) + sqr(z));

  result.w = w / norm;
  result.x = x / norm;
  result.y = y / norm;
  result.z = z / norm;

  return result;
}

#endif
