#include "Joystick.h"
#include <Arduino.h>

// Change these values to tune for your specific joystick
#define X_CENTER 503
#define Y_CENTER 504
#define X_DEADZONE 100
#define Y_DEADZONE 100
// X is inverted because of the orientation of the joystick
#define X_SIGN (-1)
#define Y_SIGN 1

int read_joystick_x() {
  int val = X_SIGN*(analogRead(JOY_X_PIN)-X_CENTER);
  if (val < -X_DEADZONE) {
    return -1;
  }
  if (val > X_DEADZONE) {
    return 1;
  }
  return 0;
}

int read_joystick_y() {
  int val = Y_SIGN*(analogRead(JOY_Y_PIN)-Y_CENTER);
  if (val < -Y_DEADZONE) {
    return -1;
  }
  if (val > Y_DEADZONE) {
    return 1;
  }
  return 0;
}

int read_joystick_button() {
  return digitalRead(JOYSTICK_BUTTON_PIN) == LOW;
}