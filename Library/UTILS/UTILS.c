/**
 * @file UTILS.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <stdint.h>

#include "UTILS.h"

/**
 * @brief Normalizes the quaternion to have |Q| = 1 as long as the norm is not zero
 * @link https://github.com/rawify/Quaternion.js/blob/c3834673b502e64e1866dbbf13568c0be93e52cc/quaternion.js#L385-L405
 * @param w `w` component
 * @param x `x` component
 * @param y `y` component
 * @param z `z` component
 * @return The normalized Quaternion
 */
Quaternion normalizeQuaternion(float w, float x, float y, float z) {
  Quaternion result = {0};
  float      norm = sqrtf(sqr(w) + sqr(x) + sqr(y) + sqr(z));

  if (norm >= FLT_EPSILON) {
    result.w = w / norm;
    result.x = x / norm;
    result.y = y / norm;
    result.z = z / norm;
  }

  return result;
}

/**
 * @brief Creates quaternion by a rotation given as axis-angle orientation
 * @link https://github.com/rawify/Quaternion.js/blob/c3834673b502e64e1866dbbf13568c0be93e52cc/quaternion.js#L1107-L1123
 * @param axis The axis around which to rotate
 * @param angle The angle in radians
 * @return The rotated quaternion
 */
Quaternion QuaternionFromAxisAngle(float x, float y, float z, float angle) {
  Quaternion result = {0};

  float halfAngle = angle / 2.0f;
  float sin_norm = sinf(halfAngle) / sqrtf(sqr(x) + sqr(y) + sqr(z));

  result.w = cosf(halfAngle);
  result.x = x * sin_norm;
  result.y = y * sin_norm;
  result.z = z * sin_norm;

  return result;
}

/**
 * @brief Rotates a vector according to the quaternion, assumes |q|=1
 * @link https://github.com/rawify/Quaternion.js/blob/c3834673b502e64e1866dbbf13568c0be93e52cc/quaternion.js#L1004-L1025
 * @param q Quaternion
 * @param v Vector
 */
Coords QuaternionRotateVector(Quaternion q, Coords v) {
  Coords result = {0};

  // t = 2q x v
  float tx = 2 * (q.y * v.z - q.z * v.y);
  float ty = 2 * (q.z * v.x - q.x * v.z);
  float tz = 2 * (q.x * v.y - q.y * v.x);

  // v + w t + q x t
  result.x = v.x + q.w * tx + q.y * tz - q.z * ty;
  result.y = v.y + q.w * ty + q.z * tx - q.x * tz;
  result.z = v.z + q.w * tz + q.x * ty - q.y * tx;

  return result;
}
