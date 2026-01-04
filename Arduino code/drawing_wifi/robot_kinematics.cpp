#include "robot_kinematics.h"

// Drawing area limits
const float drawing_area_x_limits[2] = {DRAWING_AREA_X_BIAS, BASE - DRAWING_AREA_X_BIAS};
const float drawing_area_y_limits[2] = {DRAWING_AREA_Y_BIAS, DRAWING_AREA_HEIGHT - DRAWING_AREA_Y_BIAS};

// Belt lengths (shared between cores, but only written from Core 0)
float L1 = 500.0; // Length from motor to left belt gripper
float L2 = 500.0; // Length from motor to right belt gripper
float Z1_i = L1 + L_ARM; // Initial length from left motor to the pen
float Z2_i = L2 + L_ARM; // Initial length from right motor to the pen

bool inverseKinematics(float target_x, float target_y, float &Z1, float &Z2) {
  if (target_x < drawing_area_x_limits[0] || target_x > drawing_area_x_limits[1] || 
      target_y < drawing_area_y_limits[0] || target_y > drawing_area_y_limits[1]) {
    return false;
  }
  // Compute arm lengths
  Z1 = sqrt(pow(target_x, 2) + pow(target_y, 2));
  Z2 = sqrt(pow(BASE - target_x, 2) + pow(target_y, 2));
  return true;
}

long beltToSteps(float dZ) {
  return long(dZ / MM_PER_STEP);
}


