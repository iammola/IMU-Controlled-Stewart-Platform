/**
 * @file StewartPlatform.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @version 0.1
 * @date 2024-02-24
 * @brief
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "UTILS/UTILS.h"

#define LEGS_COUNT 6

typedef struct Legs {
  float  servoAngle;
  float  sinBeta;       // Sin of Pan angle of motors in base plate
  float  cosBeta;       // Cos of Pan angle of motors in base plate
  Coords baseJoint;     // base joints in base frame
  Coords platformJoint; // platform joints in platform frame
} Legs;

extern Legs legs[LEGS_COUNT];

void StewartPlatform_Init(void);

void StewartPlatform_Update(Coords translation, Quaternion orientation);
