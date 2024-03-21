/**
 * @file StewartPlatform.h
 * @author Ademola Adedeji (a.mola.dev@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-02-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "Quaternion/Quaternion.h"

#define LEGS_COUNT 6
#define AXES_COUNT 3

typedef struct StewartCoords {
  float x;
  float y;
  float z;
} StewartCoords;

typedef struct Legs {
  float  servoAngle;
  float  sinBeta;       // Sin of Pan angle of motors in base plate
  float  cosBeta;       // Cos of Pan angle of motors in base plate
  StewartCoords baseJoint;     // base joints in base frame
  StewartCoords platformJoint; // platform joints in platform frame
} Legs;

extern Legs legs[LEGS_COUNT];

void StewartPlatform_Init(float _rodLength, float _hornLength, float shaftDistance, float anchorDistance);

void StewartPlatform_Update(StewartCoords translation, Quaternion orientation);
