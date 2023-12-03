#include "DCMotor.h"
#include <Arduino.h>

#define DC_DIR_PIN 13
#define DC_PWM_PIN 11
#define DC_BRAKE_PIN 8

namespace DCMotor {

void attach() {
  pinMode(DC_DIR_PIN, OUTPUT);
  pinMode(DC_PWM_PIN, OUTPUT);
  pinMode(DC_BRAKE_PIN, OUTPUT);
}

void set_brake(bool brake) {
  digitalWrite(DC_BRAKE_PIN, brake);
}

void set_speed(uint8_t speed) {
  analogWrite(DC_PWM_PIN, speed);
}

void set_direction(bool dir) {
  digitalWrite(DC_DIR_PIN, dir);
}

};