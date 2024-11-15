#ifndef JOYSTICK_H
#define JOYSTICK_H
#include <stdint.h>

#define JOY_X_PIN 2
#define JOY_Y_PIN 3
#define JOYSTICK_BUTTON_PIN 7

extern uint16_t X_CENTER, Y_CENTER;

/**
 * @brief Returns -1 if left of center, 1 if right of center, 0 if at center.
 */
int read_joystick_x();

/**
 * @brief Returns -1 if below of center, 1 if above of center, 0 if at center.
 */
int read_joystick_y();

/**
 * Returns 1 if the button is down, 0 if otherwise.
 */
int read_joystick_button();

#endif