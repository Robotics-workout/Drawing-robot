#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include "config.h"
#include <math.h>

// Drawing area limits
extern const float drawing_area_x_limits[2];
extern const float drawing_area_y_limits[2];

// Belt lengths (shared between cores, but only written from Core 0)
extern float L1;
extern float L2;
extern float Z1_i;
extern float Z2_i;

// Function declarations
bool inverseKinematics(float target_x, float target_y, float &Z1, float &Z2);
long beltToSteps(float dZ);

#endif // ROBOT_KINEMATICS_H


