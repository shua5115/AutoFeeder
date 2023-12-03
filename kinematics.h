#ifndef KINEMATICS_H
#define KINEMATICS_H

#define Q1_HOME -3.1415926
#define Q2_HOME 2.1817
#define HOME_X -42.6424
#define HOME_Y -81.9152

extern const float L1;
extern const float L2;

/// Calculates forward kinematics for given joint values.
/// Returns whether the kinematic calculation was successful.
/// Places the result of fk into x_ptr and y_ptr.
bool calc_fk(float q1, float q2, float &x_ptr, float &y_ptr);

/// Calculates inverse kinematics for a given point relative to the robot's origin (the shoulder joint).
/// Returns whether the kinematic calculation was successful.
/// Places the result of ik into q1_ptr and q2_ptr.
bool calc_ik(float x, float y, float &q1_ptr, float &q2_ptr);

// Constrains an ik point to be within workspace bounds.
// Returns true if the point was modified
bool constrain_ik_point(float &x, float &y);

#endif