/**
 * @file StewartPlatform.c
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief Library for the Inverse Kinematics of a 6-DOF Stewart Platform. Rewritten in C
 * from Robert Eisele's implementation https://github.com/rawify/Stewart.js/tree/master
 * @version 0.2
 * @date 2024-03-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <math.h>
#include <stdint.h>

#include "StewartPlatform.h"
#include "UTILS/UTILS.h"

#define BASE_OUTER_RADIUS     83.2665f // Radius (mm) of circumscribed circle of hexagonal platform plate
#define BASE_INNER_RADIUS     60.0f    // Radius (mm) of inscribed circle of hexagonal platform plate
#define PLATFORM_OUTER_RADIUS 70.946f  // Radius (mm) of circumscribed circle of hexagonal platform plate
#define PLATFORM_INNER_RADIUS 45.0f    // Radius (mm) of inscribed circle of hexagonal platform plate
#define ROD_LENGTH            150.0f   // Length (mm) of the rod attached to the servo horn and the platform
#define HORN_LENGTH           31.75f   // Length (mm) of servo horn attached to the motor shaft and the rod
#define SHAFT_DISTANCE        20.0f    // Distance (mm) from center of side to servo motor shaft/horn center
#define ANCHOR_DISTANCE       22.225f  // Distance (mm) from center of side to platform anchor point
#define HORN_DIRECTION        0        // If horns are pointed outwards 0, otherwise 1
#define SERVO_ANGLE_LIMIT     90.0f    // Min/Max range (°) for the servo motors to rotate allowed

StewartPlatformLeg legs[LEGS_COUNT] = {0};

static Coords T0 = {0};

static void StewartPlatform_GetLegs(void);
static void StewartPlatform_GetHexPlate(Coords (*ret)[LEGS_COUNT], float r_o, float r_i, float rot);

/**
 * @brief Initializes the Stewart Platform model, calculating leg coordinates and home position
 * @param None
 */
void StewartPlatform_Init(void) {
  StewartPlatform_GetLegs();

  T0.z = sqrtf(                                                //
      sqr(ROD_LENGTH) +                                        // sqr(∣d∣)
      sqr(HORN_LENGTH) -                                       // sqr(∣h∣)
      powf(legs[0].platformJoint.x - legs[0].baseJoint.x, 2) - // sqr(pk(x) - bk(x))
      powf(legs[0].platformJoint.y - legs[0].baseJoint.y, 2)   // sqr(pk(y) - bk(y))
  );

  // Initialize leg servo angles with no translation and rotation
  StewartPlatform_Update((Coords){.x = 0.0f, .y = 0.0f, .z = 0.0f}, (Quaternion){.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f});
}

/**
 * @brief Calculates the angle for each servo motor to position the platform in the desired
 * orientation & translation
 * @param translation The vector to move the model by
 * @param orientation The vector to rotate the model by
 */
void StewartPlatform_Update(Coords translation, Quaternion orientation) {
  Coords  coords = {0};
  uint8_t legIdx = 0;

  float angle;
  float x, y, z;
  float gk, ek, fk;

  static const float twiceHornLength = 2 * HORN_LENGTH;
  static const float squareRodL_HornL = sqr(ROD_LENGTH) - sqr(HORN_LENGTH);

  for (legIdx = 0; legIdx < LEGS_COUNT; legIdx++) {
    coords = QuaternionRotateVector(orientation, legs[legIdx].platformJoint);

    x = translation.x + coords.x - legs[legIdx].baseJoint.x;
    y = translation.y + coords.y - legs[legIdx].baseJoint.y;
    z = translation.z + coords.z - legs[legIdx].baseJoint.z + T0.z;

    gk = sqr(x) + sqr(y) + sqr(z) - squareRodL_HornL;
    ek = twiceHornLength * z;
    fk = twiceHornLength * ((legs[legIdx].cosBeta * x) + (legs[legIdx].sinBeta * y));

    angle = (asinf(gk / sqrtf(sqr(ek) + sqr(fk))) - atan2f(fk, ek)) * RAD_TO_DEG;
    legs[legIdx].servoAngle = clamp(angle, -SERVO_ANGLE_LIMIT, SERVO_ANGLE_LIMIT);
  }
}

/**
 * @brief Calculates the polar coordinates of each point (anchor/shaft) on the hex plate
 * @param ret Destination to store each leg vector in
 * @param r_o Outer radius of hex plate in mm
 * @param r_i Inner radius of hex plate in mm
 * @param rot Angle to rotate plate by in degrees °
 */
static void StewartPlatform_GetHexPlate(Coords (*ret)[LEGS_COUNT], float r_o, float r_i, float rot) {
  uint8_t legIdx = 0;

  float ap = 0.0f, phi = 0.0f;
  float a_2 = (2.0f * r_i - r_o) / sqrtf(3.0f);

  for (legIdx = 0; legIdx < LEGS_COUNT; legIdx++) {
    phi = ((legIdx - (legIdx % 2)) / 3.0f) * M_PI + (rot * DEG_TO_RAD);
    ap = a_2 * powf(-1, legIdx);

    (*ret)[legIdx].x = r_o * cosf(phi) + ap * sinf(phi);
    (*ret)[legIdx].y = r_o * sinf(phi) - ap * cosf(phi);
  }
}

/**
 * @brief Calculates the vector and polar angles for each leg in the model. It's base and
 * platform connection points.
 * @param None
 */
static void StewartPlatform_GetLegs(void) {
  uint8_t legIdx = 0;

  Coords baseInts[LEGS_COUNT] = {0};
  Coords platformInts[LEGS_COUNT] = {0};

  float baseCx, baseCy, baseNx, baseNY;
  float platCx, platCy, platNx, platNY;
  float baseMidX, baseMidY;
  float platMidX, platMidY;
  float lenBaseSide, pm;
  float baseDX, baseDY;
  float motorRotation;

  StewartPlatform_GetHexPlate(&baseInts, BASE_OUTER_RADIUS, BASE_INNER_RADIUS, 0);
  StewartPlatform_GetHexPlate(&platformInts, PLATFORM_OUTER_RADIUS, PLATFORM_INNER_RADIUS, 0);

  for (legIdx = 0; legIdx < LEGS_COUNT; legIdx++) {
    uint8_t midK = legIdx | 1;

    baseCx = baseInts[midK].x;
    baseCy = baseInts[midK].y;
    baseNx = baseInts[(midK + 1) % LEGS_COUNT].x;
    baseNY = baseInts[(midK + 1) % LEGS_COUNT].y;

    platCx = platformInts[midK].x;
    platCy = platformInts[midK].y;
    platNx = platformInts[(midK + 1) % LEGS_COUNT].x;
    platNY = platformInts[(midK + 1) % LEGS_COUNT].y;

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
