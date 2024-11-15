#ifndef DCMotor_H
#define DCMotor_H
#include <stdint.h>

namespace DCMotor {
  /**
   * @brief Sets the pinout for the motor.
   * @note Direction pin: 13, PWM pin: 11, Brake pin: 8
   */
  void attach();

  /**
   * @brief Sets the speed of the motor.
   */
  void set_speed(uint8_t speed);

  /**
   * @brief Sets the direction of the motor.
   */
  void set_direction(bool dir);
  
  /**
   * @brief Engages or disengages the brake of the motor
   */
  void set_brake(bool brake);
};

#endif