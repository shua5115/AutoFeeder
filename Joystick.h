#ifndef JOYSTICK_H
#define JOYSTICK_H
#include <stdint.h>

#define JOY_X_PIN 2
#define JOY_Y_PIN 3
#define JOYSTICK_BUTTON_PIN 7

// Returns -1 if left of center, 1 if right of center, 0 if at center
int read_joystick_x();
// Returns -1 if below of center, 1 if above of center, 0 if at center
int read_joystick_y();

int read_joystick_button();

#endif