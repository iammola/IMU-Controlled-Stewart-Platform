/**
 * @file UTILS/UTILS.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-20
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef UTILS_H
#define UTILS_H

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD_TO_DEG 180.0f / (float)M_PI
#define DEG_TO_RAD (float)M_PI / 180.0f

typedef struct {
  float w;
  float x;
  float y;
  float z;
} Quaternion;

typedef struct Coords {
  float x;
  float y;
  float z;
} Coords;

typedef struct Position {
  Quaternion quaternion;
  Coords     translation;
  bool       inUse;
  uint32_t   count;
} Position;

#define sqr(num) ((num) * (num))

#define clamp(a, min, max) ((a) < min ? min : ((a) > max ? max : a))

/**
 * @brief Normalizes the quaternion to have |Q| = 1 as long as the norm is not zero
 * @link https://github.com/rawify/Quaternion.js/blob/c3834673b502e64e1866dbbf13568c0be93e52cc/quaternion.js#L385-L405
 * @param w `w` component
 * @param x `x` component
 * @param y `y` component
 * @param z `z` component
 * @return The normalized Quaternion
 */
Quaternion normalizeQuaternion(float w, float x, float y, float z);

/**
 * @brief Creates quaternion by a rotation given as axis-angle orientation
 * @link https://github.com/rawify/Quaternion.js/blob/c3834673b502e64e1866dbbf13568c0be93e52cc/quaternion.js#L1107-L1123
 * @param axis The axis around which to rotate
 * @param angle The angle in radians
 * @return The rotated quaternion
 */
Quaternion QuaternionFromAxisAngle(float x, float y, float z, float angle);

/**
 * @brief Calculates the Hamilton product of two quaternions. Leaving out the imaginary part results in just scaling the quat
 * Not commutative because cross(v1, v2) != cross(v2, v1)!
 * @link https://github.com/rawify/Quaternion.js/blob/c3834673b502e64e1866dbbf13568c0be93e52cc/quaternion.js#L406-L439
 * @param Q1 The LHS of the equation
 * @param Q2 The RHS of the equation
 * @param scale Value to scale result by
 * @return The result of the subtraction
 */
inline Quaternion QuaternionMultiply(Quaternion Q1, Quaternion Q2, float scale) {
  Quaternion result = {
      .w = (Q1.w * Q2.w - Q1.x * Q2.x - Q1.y * Q2.y - Q1.z * Q2.z) * scale,
      .x = (Q1.w * Q2.x + Q1.x * Q2.w + Q1.y * Q2.z - Q1.z * Q2.y) * scale,
      .y = (Q1.w * Q2.y + Q1.y * Q2.w + Q1.z * Q2.x - Q1.x * Q2.z) * scale,
      .z = (Q1.w * Q2.z + Q1.z * Q2.w + Q1.x * Q2.y - Q1.y * Q2.x) * scale,
  };

  return result;
}

/**
 * @brief Conjugate of Quaternion. This is equals to the Quaternion's inverse
 * when borth are normalized
 * @param w `w` component
 * @param x `x` component
 * @param y `y` component
 * @param z `z` component
 * @return Quaternion's conjugate
 */
inline Quaternion QuaternionConjugate(float w, float x, float y, float z) {
  return ((Quaternion){.w = w, .x = -x, .y = -y, .z = -z});
}

/**
 * @brief Rotates a vector according to the quaternion, assumes |q|=1
 * @link https://github.com/rawify/Quaternion.js/blob/c3834673b502e64e1866dbbf13568c0be93e52cc/quaternion.js#L1004-L1025
 * @param q Quaternion
 * @param v Vector
 */
Coords QuaternionRotateVector(Quaternion q, Coords v);
#endif
