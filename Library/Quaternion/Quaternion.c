/**
 * @file Quaternion.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-20
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <math.h>

#include "Quaternion.h"

#define sqr(num) (num * num)

/**
 * @brief
 * @param w
 * @param x
 * @param y
 * @param z
 * @return
 */
Quaternion normalizeQuaternion(float w, float x, float y, float z) {
  Quaternion result = {0.0f};
  float      norm = sqrtf(sqr(w) + sqr(x) + sqr(y) + sqr(z));

  result.w = w / norm;
  result.x = x / norm;
  result.y = y / norm;
  result.z = z / norm;

  return result;
}
