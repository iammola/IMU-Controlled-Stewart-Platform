/**
 * @file StewartPlatform.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-24
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <math.h>
#include <stdint.h>

#include "StewartPlatform.h"

#define LEGS_COUNT 6

#define sqr(num)                 (num * num)
#define InRange(angle, min, max) (angle < min ? min : angle > max ? max : angle)

#define BASE_OUTER_RADIUS     83.2665f
#define BASE_INNER_RADIUS     60.0f
#define PLATFORM_OUTER_RADIUS 70.946f
#define PLATFORM_INNER_RADIUS 45.0f
#define ROD_LENGTH            150.0f
#define HORN_LENGTH           38.1f
#define SHAFT_DISTANCE        19.05f  // Distance between servo pair on base
#define ANCHOR_DISTANCE       22.225f // Distance between anchor points on platform
#define HORN_DIRECTION        0       // If horns are pointed outwards 0, otherwise 1

#define SERVO_ANGLE_MIN -90
#define SERVO_ANGLE_MAX 90

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

Legs legs[LEGS_COUNT] = {0};

static const float RAD_TO_DEG = 180.0f / (float)M_PI;

static StewartCoords T0 = {0};

static StewartCoords l[] = {0}; // vector from Base anchor to Platform anchor

static void StewartPlatform_RotateVector(StewartCoords *dest, Quaternion orientation, StewartCoords vector);
static void StewartPlatform_GetLegs(void);

static void getHexPlate(StewartCoords (*ret)[LEGS_COUNT], float r_o, float r_i, float rot);

void StewartPlatform_Init(void) {
  StewartPlatform_GetLegs();

  T0.z = sqrtf(                                            //
      sqr(ROD_LENGTH) +                                    // sqr(∣d∣)
      sqr(HORN_LENGTH) -                                   // sqr(∣h∣)
      sqr(legs[0].platformJoint.x - legs[0].baseJoint.x) - // sqr(pk(x) - bk(x))
      sqr(legs[0].platformJoint.y - legs[0].baseJoint.y)   // sqr(pk(y) - bk(y))
  );
}

static void getHexPlate(StewartCoords (*ret)[LEGS_COUNT], float r_o, float r_i, float rot) {
  uint8_t legIdx = 0;

  float ap = 0.0f, phi = 0.0f;
  float a_2 = (2.0f * r_i - r_o) / sqrtf(3.0f);

  for (legIdx = 0; legIdx < LEGS_COUNT; legIdx++) {
    phi = ((legIdx - (legIdx % 2)) / 3.0f) * M_PI + rot;
    ap = a_2 * powf(-1, legIdx);

    (*ret)[legIdx].x = r_o * cosf(phi) + ap * sinf(phi);
    (*ret)[legIdx].y = r_o * sinf(phi) - ap * cosf(phi);
  }
}

static void StewartPlatform_GetLegs(void) {
  uint8_t legIdx = 0;

  StewartCoords baseInts[LEGS_COUNT] = {0};
  StewartCoords platformInts[LEGS_COUNT] = {0};

  float baseCx, baseCy, baseNx, baseNY;
  float platCx, platCy, platNx, platNY;
  float baseMidX, baseMidY;
  float platMidX, platMidY;
  float lenBaseSide, pm;
  float baseDX, baseDY;
  float motorRotation;

  getHexPlate(&baseInts, BASE_OUTER_RADIUS, BASE_INNER_RADIUS, 0);
  getHexPlate(&platformInts, PLATFORM_OUTER_RADIUS, PLATFORM_INNER_RADIUS, 0);

  for (legIdx = 0; legIdx < 6; legIdx++) {
    uint8_t midK = legIdx | 1;

    baseCx = baseInts[midK].x;
    baseCy = baseInts[midK].y;
    baseNx = baseInts[(midK + 1) % 6].x;
    baseNY = baseInts[(midK + 1) % 6].y;

    platCx = platformInts[midK].x;
    platCy = platformInts[midK].y;
    platNx = platformInts[(midK + 1) % 6].x;
    platNY = platformInts[(midK + 1) % 6].y;

    baseDX = baseNx - baseCx;
    baseDY = baseNY - baseCy;
    lenBaseSide = hypotf(baseDX, baseDY);

    pm = powf(-1, legIdx);

    baseMidX = (baseCx + baseNx) / 2.0f;
    baseMidY = (baseCy + baseNY) / 2.0f;

    platMidX = (platCx + platNx) / 2.0f;
    platMidY = (platCy + platNY) / 2.0f;

    baseDX /= lenBaseSide;
    baseDY /= lenBaseSide;

    motorRotation = atan2f(baseDY, baseDX) + ((legIdx + HORN_DIRECTION) % 2) * M_PI;

    legs[legIdx].baseJoint.x = baseMidX + baseDX * SHAFT_DISTANCE * pm;
    legs[legIdx].baseJoint.y = baseMidY + baseDY * SHAFT_DISTANCE * pm;
    legs[legIdx].baseJoint.z = 0;

    legs[legIdx].platformJoint.x = platMidX + baseDX * ANCHOR_DISTANCE * pm;
    legs[legIdx].platformJoint.y = platMidY + baseDY * ANCHOR_DISTANCE * pm;
    legs[legIdx].platformJoint.z = 0;

    legs[legIdx].sinBeta = sinf(motorRotation);
    legs[legIdx].cosBeta = cosf(motorRotation);
  }
}

void StewartPlatform_Update(StewartCoords translation, Quaternion orientation) {
  StewartCoords coords = {0};
  uint8_t       legIdx = 0;

  float angle;
  float gk, ek, fk;

  static const float sqrROD_HORN = sqr(ROD_LENGTH) + sqr(HORN_LENGTH);
  static const float doubleHORN_LENGTH = 2 * HORN_LENGTH;

  for (legIdx = 0; legIdx < LEGS_COUNT; legIdx++) {
    StewartPlatform_RotateVector(&coords, orientation, legs[legIdx].platformJoint);

    l[legIdx].x = translation.x + coords.x - legs[legIdx].baseJoint.x;
    l[legIdx].y = translation.y + coords.y - legs[legIdx].baseJoint.y;
    l[legIdx].z = translation.z + coords.z - legs[legIdx].baseJoint.z + T0.z;

    gk = sqr(l[legIdx].x) + sqr(l[legIdx].y) + sqr(l[legIdx].z) - sqrROD_HORN;
    ek = doubleHORN_LENGTH * l[legIdx].z;
    fk = doubleHORN_LENGTH * ((legs[legIdx].cosBeta * l[legIdx].x) + (legs[legIdx].sinBeta * l[legIdx].y));

    angle = (asinf(gk / sqrtf(sqr(ek) + sqr(fk))) - atan2f(fk, ek)) * RAD_TO_DEG;
    legs[legIdx].servoAngle = InRange(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
  }
}

static void StewartPlatform_RotateVector(StewartCoords *dest, Quaternion orientation, StewartCoords vector) {
  float qw = orientation.w;
  float qx = orientation.x;
  float qy = orientation.y;
  float qz = orientation.z;

  float vx = vector.x;
  float vy = vector.y;
  float vz = vector.z;

  // t = 2q x v
  float tx = 2 * (qy * vz - qz * vy);
  float ty = 2 * (qz * vx - qx * vz);
  float tz = 2 * (qx * vy - qy * vx);

  // v + w t + q x t
  dest->x = vx + qw * tx + qy * tz - qz * ty;
  dest->y = vy + qw * ty + qz * tx - qx * tz;
  dest->z = vz + qw * tz + qx * ty - qy * tx;
}
