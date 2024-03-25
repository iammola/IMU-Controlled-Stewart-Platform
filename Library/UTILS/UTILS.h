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
  bool       isNew;
} Position;

#define sqr(num) ((num) * (num))

#define clamp(a, min, max) ((a) < min ? min : ((a) > max ? max : a))

#define POSITION_BYTE_SIZE 28 // (4 bytes per flaot * (4 quaternion floats) + (3 translation floats))

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
 * @brief Rotates a vector according to the quaternion, assumes |q|=1
 * @link https://github.com/rawify/Quaternion.js/blob/c3834673b502e64e1866dbbf13568c0be93e52cc/quaternion.js#L1004-L1025
 * @param q Quaternion
 * @param v Vector
 */
Coords QuaternionRotateVector(Quaternion q, Coords v);
#endif
