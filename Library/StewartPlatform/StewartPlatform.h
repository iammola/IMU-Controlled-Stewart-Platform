/**
 * @file StewartPlatform.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief
 * @version 0.2
 * @date 2024-03-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "UTILS/UTILS.h"

#define LEGS_COUNT 6

typedef struct StewartPlatformLeg {
  float  servoAngle;
  float  sinBeta;       // Sin of Pan angle of motors in base plate
  float  cosBeta;       // Cos of Pan angle of motors in base plate
  Coords baseJoint;     // base joints in base frame
  Coords platformJoint; // platform joints in platform frame
} StewartPlatformLeg;

extern StewartPlatformLeg legs[LEGS_COUNT];

/**
 * @brief
 * @param
 */
void StewartPlatform_Init(void);

/**
 * @brief
 * @param translation
 * @param orientation
 */
void StewartPlatform_Update(Coords translation, Quaternion orientation);
