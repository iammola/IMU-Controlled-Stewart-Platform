#include <math.h>
#include <stdint.h>

#include "StewartPlatform.h"

#define LEGS_COUNT 6

#define sqr(num)                 (num * num)
#define InRange(angle, min, max) (angle < min ? min : angle > max ? max : angle)

#define BASE_OUTER_RADIUS     0
#define BASE_INNER_RADIUS     0
#define PLATFORM_OUTER_RADIUS 0
#define PLATFORM_INNER_RADIUS 0

#define SERVO_ANGLE_MIN -90
#define SERVO_ANGLE_MAX 90

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

typedef struct Legs {
  float  servoAngle;
  float  sinBeta;       // Sin of Pan angle of motors in base plate
  float  cosBeta;       // Cos of Pan angle of motors in base plate
  Coords baseJoint;     // base joints in base frame
  Coords platformJoint; // platform joints in platform frame
} Legs;

static float rodLength = 0;
static float hornLength = 0;

static Coords T0 = {0};
static Legs   legs[] = {0};

static Coords l[] = {0}; // vector from Base anchor to Platform anchor

static void StewartPlatform_RotateVector(Coords dest, Quaternion orientation, Coords vector);
static void StewartPlatform_GetLegs(uint8_t hornDirection, // If horns are pointed outwards 0, otherwise 1
                                    float   shaftDistance, // Distance between servo pair on base
                                    float   anchorDistance // Distance between anchor points on platform
);
static void getHexPlate(Coords ret[], float r_i, float r_o, float rot);

void StewartPlatform_Init(float _rodLength, float _hornLength, float shaftDistance, float anchorDistance) {
  StewartPlatform_GetLegs(0, shaftDistance, anchorDistance);

  rodLength = _rodLength;
  hornLength = _hornLength;

  T0.z = sqrtf(                                            //
      sqr(rodLength) +                                     // sqr(∣d∣)
      sqr(hornLength) -                                    // sqr(∣h∣)
      sqr(legs[0].platformJoint.x - legs[0].baseJoint.x) - // sqr(pk(x) - bk(x))
      sqr(legs[0].platformJoint.y - legs[0].baseJoint.y)   // sqr(pk(y) - bk(y))
  );
}

static void getHexPlate(Coords ret[], float r_i, float r_o, float rot) {
  uint8_t legIdx = 0;

  float ap, phi;
  float a_2 = (2 * r_i - r_o) / sqrtf(3);

  for (legIdx = 0; legIdx < 6; legIdx++) {
    phi = ((legIdx - (legIdx % 2)) / 3) * M_PI + rot;
    ap = a_2 * powf(-1, legIdx);

    ret[legIdx].x = r_o * cosf(phi) + ap * sinf(phi);
    ret[legIdx].y = r_o * sinf(phi) - ap * cosf(phi);
  }
}

static void StewartPlatform_GetLegs(uint8_t hornDirection, float shaftDistance, float anchorDistance) {
  uint8_t armIdx = 0;

  Coords baseInts[] = {0};
  Coords platformInts[] = {0};

  float baseCx, baseCy, baseNx, baseNY;
  float platCx, platCy, platNx, platNY;
  float baseMidX, baseMidY;
  float platMidX, platMidY;
  float lenBaseSide, pm;
  float baseDX, baseDY;
  float motorRotation;

  getHexPlate(baseInts, BASE_OUTER_RADIUS, BASE_INNER_RADIUS, 0);
  getHexPlate(platformInts, PLATFORM_OUTER_RADIUS, PLATFORM_INNER_RADIUS, 0);

  for (armIdx = 0; armIdx < 6; armIdx++) {
    uint8_t midK = armIdx | 1;

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

    pm = powf(-1, armIdx);

    baseMidX = (baseCx + baseNx) / 2;
    baseMidY = (baseCy + baseNY) / 2;

    platMidX = (platCx + platNx) / 2;
    platMidY = (platCy + platNY) / 2;

    baseDX /= lenBaseSide;
    baseDY /= lenBaseSide;

    motorRotation = atan2f(baseDY, baseDX) + ((armIdx + hornDirection) % 2) * M_PI;

    legs[armIdx].baseJoint.x = baseMidX + baseDX * shaftDistance * pm;
    legs[armIdx].baseJoint.y = baseMidY + baseDY * shaftDistance * pm;
    legs[armIdx].baseJoint.z = 0;

    legs[armIdx].platformJoint.x = platMidX + baseDX * anchorDistance * pm;
    legs[armIdx].platformJoint.y = platMidY + baseDY * anchorDistance * pm;
    legs[armIdx].platformJoint.z = 0;

    legs[armIdx].sinBeta = sinf(motorRotation);
    legs[armIdx].cosBeta = cosf(motorRotation);
  }
}

void StewartPlatform_Update(Coords translation, Quaternion orientation) {
  Coords  o = {0};
  uint8_t legIdx = 0;

  float gk, ek, fk;

  for (legIdx = 0; legIdx < LEGS_COUNT; legIdx++) {
    StewartPlatform_RotateVector(o, orientation, legs[legIdx].platformJoint);

    l[legIdx].x = translation.x + o.x - legs[legIdx].baseJoint.x;
    l[legIdx].y = translation.y + o.y - legs[legIdx].baseJoint.y;
    l[legIdx].z = translation.z + o.z - legs[legIdx].baseJoint.z + T0.z;

    gk = sqr(l[legIdx].x) + sqr(l[legIdx].y) + sqr(l[legIdx].z) - sqr(rodLength) + sqr(hornLength);
    ek = 2 * hornLength * l[legIdx].z;
    fk = 2 * hornLength * ((legs[legIdx].cosBeta * l[legIdx].x) + (legs[legIdx].sinBeta * l[legIdx].y));

    legs[legIdx].servoAngle = InRange(asinf(gk / sqrtf(sqr(ek) + sqr(fk))) - atan2f(fk, ek), SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
  }
}

static void StewartPlatform_RotateVector(Coords dest, Quaternion orientation, Coords vector) {
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
  dest.x = vx + qw * tx + qy * tz - qz * ty;
  dest.y = vy + qw * ty + qz * tx - qx * tz;
  dest.z = vz + qw * tz + qx * ty - qy * tx;
}
