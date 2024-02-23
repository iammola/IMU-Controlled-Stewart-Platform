#define ARMS_COUNT 6
#define AXES_COUNT 3

/**
 * @brief
 * Yeok 2022
 * Stewart Platform Python Implementation
 * Uses 6 Rotational Servos

 * @param r_B Radius for circumscribed circle where all the anchor points for servo shaft lie on (Bottom)
 * @param r_P Radius for circumscribed circle where all anchor points for platform lie on (Top)
 * @param lhl length of servo horn
 * @param ldl length of rod
 * @param gamma_B Half of angle between two anchors on the base
 * @param gamma_P Half of angle between two anchors on the platform
 * @param ref_rotation rotates the entire platform in the z-axis to align with your applications
 */
void StewartPlatform_Init(float r_B, float r_P, float lhl, float ldl, float gamma_B, float gamma_P, float ref_rotation);

void StewartPlatform_Calculate(float translate[AXES_COUNT], float rotation[AXES_COUNT], float angles[ARMS_COUNT]);
