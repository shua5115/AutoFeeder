#include "kinematics.h"
#include <math.h>

// units in mm

static float dist_sq(float x1, float y1, float x2, float y2) {
  return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
}

static bool twoLinkIK(float x, float y, float a, float b, bool elbowup, float &t1, float &t2) {
  float D = (x * x + y * y - a * a - b * b) / (2.0 * a * b);
  if (D > 1.0) {
    return false;
  }
  t2 = atan2((elbowup ? 1.0 : -1.0) * sqrt(1.0 - D * D), D);
  t1 = atan2(y, x) - atan2(b * sin(t2), a + b * cos(t2));
  return true;
}

bool calc_ik(float x, float y, float &q1_ptr, float &q2_ptr) {
  return twoLinkIK(x, y, L1, L2, true, q1_ptr, q2_ptr);
}

bool calc_fk(float q1, float q2, float &x_ptr, float &y_ptr) {
  x_ptr = L1*cos(q1) + L2*cos(q1+q2);
  y_ptr = L1*sin(q1) + L2*sin(q1+q2);
}

bool constrain_ik_point(float &x, float &y) {
  bool modified = false;
  if (isnan(x) || isnan(y)) {
    x = 0.0;
    y = -100.0;
    return true;
  }
  const float max_mag = L1+L2;
  const float min_mag = 10;
  float mag = sqrt(x * x + y * y);
  if (mag > max_mag) {
    x *= max_mag / mag;
    y *= max_mag / mag;
    modified = true;
  } else if(mag < min_mag) {
    x *= min_mag / mag;
    y *= min_mag / mag;
    modified = true;
  }
  // Prevent ik target from going in a place that would require a q1 rotation of more than 180 degrees
  mag = sqrt(dist_sq(x, y, -L1, 0.0));
  if (mag < L1) {
    x = (x + L1) * L1 / mag - L1;
    y = (y)*L1 / mag;
    modified = true;
  }
  if (y > -0.001) {
    y = -0.001;
    modified = true;
  }
  if (x < -0.9 * (L1 + L2)) {
    x = -0.9 * (L1 + L2);
    modified = true;
  }
  return modified;
}