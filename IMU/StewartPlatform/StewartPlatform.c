#include <math.h>
#include <stdint.h>

#include "StewartPlatform.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288f
#endif

#define HALF_PI        M_PI / 2
#define ONE_THIRD_PI   M_PI / 3
#define TWO_THIRDS_PI  2 * ONE_THIRD_PI
#define FOUR_THIRDS_PI 4 * ONE_THIRD_PI

static float beta[ARMS_COUNT] = {0};
static float home_pos[AXES_COUNT] = {0};
static float B[AXES_COUNT][ARMS_COUNT] = {0}; // Coordinate of the points where servo arms are attached to the corresponding servo axis.
static float P[AXES_COUNT][ARMS_COUNT] = {0}; // Coordinates of the points where the rods are attached to the platform.

static float lhl = 0;
static float ldl = 0;

static float lll[ARMS_COUNT] = {0};
static float l[AXES_COUNT][ARMS_COUNT] = {0};

static void StewartPlatform_RotX(float roll, float rotXMatrix[AXES_COUNT][AXES_COUNT]);
static void StewartPlatform_RotY(float pitch, float rotYMatrix[AXES_COUNT][AXES_COUNT]);
static void StewartPlatform_RotZ(float yaw, float rotZMatrix[AXES_COUNT][AXES_COUNT]);

/**
 * @brief
 * Yeok 2022
 * Stewart Platform Python Implementation
 * Uses 6 Rotational Servos

 * @param r_B Radius for circumscribed circle where all the anchor points for servo shaft lie on (Bottom)
 * @param r_P Radius for circumscribed circle where all anchor points for platform lie on (Top)
 * @param __lhl length of servo horn
 * @param __ldl length of rod
 * @param gamma_B Half of angle between two anchors on the base
 * @param gamma_P Half of angle between two anchors on the platform
 * @param ref_rotation rotates the entire platform in the z-axis to align with your applications
 */
void StewartPlatform_Init(float r_B, float r_P, float __lhl, float __ldl, float gamma_B, float gamma_P, float ref_rotation) {
  uint8_t armIdx;
  uint8_t axisIdx;
  float   z;

  // Psi_B (Polar coordinates)
  float_t psi_B[ARMS_COUNT] = {
      -gamma_B,                 // Arm #1
      gamma_B,                  // Arm #2
      TWO_THIRDS_PI - gamma_B,  // Arm #3
      TWO_THIRDS_PI + gamma_B,  // Arm #4
      FOUR_THIRDS_PI - gamma_B, // Arm #5
      FOUR_THIRDS_PI - gamma_B  // Arm #6
  };
  // psi_P (Polar coordinates)
  // Direction of the points where the rod is attached to the platform.
  float_t psi_P[ARMS_COUNT] = {
      ONE_THIRD_PI + FOUR_THIRDS_PI + gamma_P, // Arm #1
      ONE_THIRD_PI - gamma_P,                  // Arm #2
      ONE_THIRD_PI + gamma_P,                  // Arm #3
      ONE_THIRD_PI + TWO_THIRDS_PI - gamma_P,  // Arm #4
      ONE_THIRD_PI + TWO_THIRDS_PI + gamma_P,  // Arm #5
      ONE_THIRD_PI + FOUR_THIRDS_PI - gamma_P, // Arm #6
  };

  beta[0] = HALF_PI + M_PI;                  // Arm #1
  beta[1] = HALF_PI;                         // Arm #2
  beta[2] = TWO_THIRDS_PI + HALF_PI + M_PI;  // Arm #3
  beta[3] = TWO_THIRDS_PI + HALF_PI;         // Arm #4
  beta[4] = FOUR_THIRDS_PI + HALF_PI + M_PI; // Arm #5
  beta[5] = FOUR_THIRDS_PI + HALF_PI;        // Arm #6

  for (armIdx = 0; armIdx < ARMS_COUNT; armIdx++) {
    beta[armIdx] += ref_rotation;
    psi_B[armIdx] += ref_rotation;
    psi_P[armIdx] += ref_rotation;
  }

  for (axisIdx = 0; axisIdx < AXES_COUNT; axisIdx++) {
    for (armIdx = 0; armIdx < ARMS_COUNT; armIdx++) {
      if (axisIdx == 2) {
        // Leave Z-axis values at 0
        B[axisIdx][armIdx] = 0;
        P[axisIdx][armIdx] = 0;
      } else {
        // Transposed array with axis as rows and arms as column
        B[axisIdx][armIdx] = r_B * psi_B[armIdx];
        P[axisIdx][armIdx] = r_P * psi_B[armIdx];
      }
    }
  }

  // Definition of the platform home position.
  z = sqrtf(powf(__ldl, 2) + powf(__lhl, 2) - powf(P[0] - B[0], 2) - powf(P[1] - B[1], 2));
  home_pos[0] = 0;
  home_pos[1] = 0;
  home_pos[2] = z;

  lhl = __lhl;
  ldl = __ldl;
}

static void StewartPlatform_RotX(float roll, float rotXMatrix[AXES_COUNT][AXES_COUNT]) {
  rotXMatrix[0][0] = 1;
  rotXMatrix[0][1] = 0;
  rotXMatrix[0][2] = 0;

  rotXMatrix[1][0] = 0;
  rotXMatrix[1][1] = cosf(roll);
  rotXMatrix[1][2] = sinf(roll);

  rotXMatrix[2][0] = 0;
  rotXMatrix[2][1] = -sinf(roll);
  rotXMatrix[2][2] = cosf(roll);
}

static void StewartPlatform_RotY(float pitch, float rotYMatrix[AXES_COUNT][AXES_COUNT]) {
  rotYMatrix[0][0] = cosf(pitch);
  rotYMatrix[0][1] = 0;
  rotYMatrix[0][2] = -sinf(pitch);

  rotYMatrix[1][0] = 0;
  rotYMatrix[1][1] = 1;
  rotYMatrix[1][2] = 0;

  rotYMatrix[2][0] = sinf(pitch);
  rotYMatrix[2][1] = 0;
  rotYMatrix[2][2] = cosf(pitch);
}

static void StewartPlatform_RotZ(float yaw, float rotZMatrix[AXES_COUNT][AXES_COUNT]) {
  rotZMatrix[0][0] = cosf(yaw);
  rotZMatrix[0][1] = sinf(yaw);
  rotZMatrix[0][2] = 0;

  rotZMatrix[1][0] = -sinf(yaw);
  rotZMatrix[1][1] = cosf(yaw);
  rotZMatrix[1][2] = 0;

  rotZMatrix[2][0] = 0;
  rotZMatrix[2][1] = 0;
  rotZMatrix[2][2] = 1;
}

void StewartPlatform_Calculate(float translate[AXES_COUNT], float rotation[AXES_COUNT], float angles[ARMS_COUNT]) {
  uint8_t armIdx;
  uint8_t axisIdx;
  uint8_t newArmIdx;
  float   rotMatrixXP = 0;
  float   R[AXES_COUNT][AXES_COUNT] = {0};

  float g = 0;
  float e = 0;
  float fk = 0;

  float_t rotMatrix[AXES_COUNT][AXES_COUNT] = {
      {cosf(rotation[1]),                     0,                  -sinf(rotation[1])                   },
      {sinf(rotation[0]) * sinf(rotation[1]), cosf(rotation[0]),  sinf(rotation[0]) * cosf(rotation[1])},
      {cosf(rotation[0]) * sinf(rotation[1]), -sinf(rotation[0]), cosf(rotation[0]) * cosf(rotation[1])},
  };

  // s.l =  np.repeat(trans[:, np.newaxis], 6, axis=1) + np.repeat(s.home_pos[:, np.newaxis], 6, axis=1) + np.matmul(R, s.P) - s.B
  // Calculate length for each leg
  for (axisIdx = 0; axisIdx < AXES_COUNT; axisIdx++) {
    for (armIdx = 0; armIdx < ARMS_COUNT; armIdx++) {
      rotMatrixXP = 0;
      for (newArmIdx = 0; newArmIdx < AXES_COUNT; newArmIdx++) {
        rotMatrixXP += rotMatrix[axisIdx][newArmIdx] * P[newArmIdx][armIdx];
      }

      l[axisIdx][armIdx] = translate[axisIdx] + home_pos[axisIdx] + rotMatrixXP - B[axisIdx][armIdx];
    }
  }

  // s.lll = np.linalg.norm(s.l, axis=0)
  // Calculate the norms for each arm (square root of the sum of each squared coord)
  for (armIdx = 0; armIdx < ARMS_COUNT; armIdx++) {
    lll[armIdx] = 0;

    for (axisIdx = 0; axisIdx < AXES_COUNT; axisIdx++) {
      lll[armIdx] += powf(l[axisIdx][armIdx], 2);
    }

    lll[armIdx] = sqrtf(lll[armIdx]);
  }

  // Calculate servo angles for each leg
  for (armIdx = 0; armIdx < ARMS_COUNT; armIdx++) {
    // Calculate auxiliary quantities g, f and e
    g = powf(lll[armIdx], 2) - powf(ldl, 2) - powf(lhl, 2);
    e = 2 * lhl * l[2][armIdx];
    fk = 2 * lhl * ((cosf(beta[armIdx]) * l[0][armIdx]) + (sinf(beta[armIdx]) * l[1][armIdx]));

    // The wanted position could be achieved if the solution of this equation is real for all i
    angles[armIdx] = asinf(g / sqrtf(powf(e, 2) + powf(fk, 2))) - atan2f(fk, e);
  }
}
