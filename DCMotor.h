#ifndef DCMotor_H
#define DCMotor_H
#include <stdint.h>

namespace DCMotor {
  /// Sets the pinout for the motor.
  /// Direction pin: 13,
  /// PWM pin: 11,
  /// Brake pin: 8
  void attach();
  /// Sets the speed of the motor.
  void set_speed(uint8_t speed);
  /// Sets the direction of the motor.
  void set_direction(bool dir);
  /// Engages or disengages the brake of the motor
  void set_brake(bool brake);
};

#endif