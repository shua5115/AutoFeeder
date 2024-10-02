#include <Servo.h>
#include <stdint.h>
#include "DCMotor.h"
#include "kinematics.h"
#include "Profile.h"
#include "Joystick.h"

// Digital Pins
#define INPUT_PIN 2
#define DEBUG_PIN 4
#define SERVO_POWER_PWM 3
#define SERVO_POWER_DIR 12
#define WARNING_LED_PIN 10
// Analog Pins
#define PROFILE_POT_PIN 5
#define SERVO_VOLTAGE_PIN 4

// Lengths of linkages, in mm
const float L1 = 100.0;
const float L2 = 100.0;

// Constants
#define Q_EPSILON 0.00001   // Two angles are considered equal if their absolute difference is less than this value
#define DIST_EPSILON 0.001  // Two points are considered at the same position if they are this close
// If servos are not moving from 0 to 180 degrees, then change these values
#define SERVO_MIN_PW 544
#define SERVO_MAX_PW 2400
// Units in radians/step (timestep dependent on code performance)
#define MAX_JOINT_SPEED 0.00025  // In radians per step
#define DC_MOTOR_SPEED 64        // From 0-255, an analogWrite value
// Units in mm/step
#define IK_STEP_SIZE 0.25
// Current sensing
// Linearly maps from 0-3.3 volts -> 0-2 amps
// Servos "normally" draw 0.2 amps. Drawing 1 amp is usually when they are stuck on something.
// The current values below are actually analogRead values.
#define THRESHOLD_CURRENT 215  // This is deliberately above actual idle current. Setting this too low will result in speed reduction too early.
#define OVERLOAD_CURRENT 337.6
constexpr float SPEED_REDUCTION_STRENGTH = log(0.05)/(THRESHOLD_CURRENT-OVERLOAD_CURRENT); // Speed is 20x slower at overload. See https://www.desmos.com/calculator/5mldupvozq
// Battery voltage sensing
#define LOW_POWER_VOLTAGE 562  // Calculated as half of 5.5 volts mapped from (0-5) -> (0-1023). If the voltage divider circuit measures below this value, then the device will shut off.

// GLOBAL VARIABLES
Profile profile = profiles[0];  // Stores the keypoints of the currently selected profile
int profile_idx = 0;            // Stores the index of the selected profile in the list of profiles
unsigned long timestamp;        // Keeps track of when states are entered
Servo j1, j2;                   // Servo joints
float q1 = 0;                   // current q1 position
float q2 = 0;                   // current q2 position
float q1_speed = 0;             // current q1 speed
float q2_speed = 0;             // current q2 speed
float fk_target_q1 = 0;         // target q1 position
float fk_target_q2 = 0;         // target q2 position
float ik_target_x = L1 + L2;    // target x
float ik_target_y = 0;          // target y
size_t fk_step = 0;             // keeps track of forward kinematics progress
size_t ik_step = 0;             // keeps track of inverse kinematics progress
uint8_t prev_profile_idx = 0;   // keeps track of the previous profile index to detect when the selection changes

#pragma region Step Code
/// Returns 1 if finished, 0 otherwise
int step_ik_target(float x_target, float y_target, float step_length) {
  float x_delta, y_delta;
  // current position is stored in ik_target_x, ik_target_y

  x_delta = x_target - ik_target_x;
  y_delta = y_target - ik_target_y;
  float mag = sqrt(x_delta * x_delta + y_delta * y_delta);

  if (mag < step_length || mag < DIST_EPSILON) {
    ik_target_x = x_target;
    ik_target_y = y_target;
    return 1;
  }

  x_delta *= step_length / mag;
  y_delta *= step_length / mag;

  ik_target_x += x_delta;
  ik_target_y += y_delta;

  return 0;
}

/// Returns 1 if finished, 0 otherwise
int step_joint_positions(float q1_target, float q2_target, float q1_max_step, float q2_max_step) {
  // Step q1 towards q1_target by q1_max_step, and the same for q2. Both do not exceed target values.
  float q1_delta, q2_delta;
  bool q1_done = false, q2_done = false;
  // current position is stored in q1, q2
  q1_delta = q1_target - q1;
  if (q1_delta > 0) {
    if (q1_delta > q1_max_step) {
      q1 += q1_max_step;
    } else {
      q1 = q1_target;
      q1_done = true;
    }
  } else {
    if (q1_delta < -q1_max_step) {
      q1 -= q1_max_step;
    } else {
      q1 = q1_target;
      q1_done = true;
    }
  }

  q2_delta = q2_target - q2;
  if (q2_delta > 0) {
    if (q2_delta > q2_max_step) {
      q2 += q2_max_step;
    } else {
      q2 = q2_target;
      q2_done = true;
    }
  } else {
    if (q2_delta < -q2_max_step) {
      q2 -= q2_max_step;
    } else {
      q2 = q2_target;
      q2_done = true;
    }
  }

  return q1_done && q2_done;
}

void balance_speed(float q1_target, float q2_target, float max_speed, float &q1_speed, float &q2_speed) {
  // Finds speeds at which both q1 and q2 will reach their target at the same time.
  // If one of the joint speeds is very close to 0, then both speeds are just set to the maximum.

  // Make the target store the difference between the current and target (the delta), then make the value positive
  q1_target -= q1;
  if (q1_target < 0) q1_target = -q1_target;
  q2_target -= q2;
  if (q2_target < 0) q2_target = -q2_target;

  // If either value is too small, just use max speed.
  if (q1_target < Q_EPSILON || q2_target < Q_EPSILON) {
    q1_speed = max_speed;
    q2_speed = max_speed;
    return;
  }
  // Find which delta is larger and set that speed to maximum.
  // Then, set the smaller delta's speed to the ratio of max_speed*big_delta/small_delta.
  if (q1_target > q2_target) {
    q1_speed = max_speed;
    q2_target /= q1_target;
    if (q2_target < Q_EPSILON) {
      q2_speed = max_speed;
    } else {
      q2_speed = max_speed * q2_target;
    }
  } else {
    q2_speed = max_speed;
    q1_target /= q2_target;
    if (q1_target < Q_EPSILON) {
      q1_speed = max_speed;
    } else {
      q1_speed = max_speed * q1_target;
    }
  }
}
#pragma endregion

// MODES
void (*cur_mode)();                   // The current code to execute in the main loop
volatile void (*next_mode)() = NULL;  // Allows interrupts to change the mode (volatile = must reference RAM, so slower but fewer race conditions)
bool pre = false;                     // True if the mode is just entered

/// Schedules to switch to the next mode, only if another section of code has not yet requested to switch.
void switch_mode(void (*new_mode)()) {
  if (next_mode == NULL) {
    next_mode = new_mode;
  }
}
/// Switches the mode regardless of whether another has already requested a switch (useful for emergency stop)
void force_switch_mode(void (*new_mode)()) {
  next_mode = new_mode;
}

// Pre-define modes for later implementation
void move_home_then_wait();  // Moves the servos back to the home position, then switches to wait_mode.
void wait_mode();            // Waits for user input, then goes into the rotate plate step or calibration mode depending on the input.
void calibration_mode();     // Sets keypoints for differently shaped plates and bowls
void low_power_mode();       // Blinks the warning LED and waits for the robot to be powered off.

void rotate_plate_step();  // Rotates the plate at a slow speed. On user input, stop the plate and switch to descend step.
void descend_step();       // Descends to edge of plate. Once motion is complete, switch to scoop step.
void scoop_step();         // Scrapes across plate. Once motion is complete, switch to pre lift step.
void lift_step_fk();       // Lifts spoon up to user with forward kinematics (sets joint states directly)
void feed_wait_step();     // Waits for user to eat the food. Switches to return step on user input.
void return_step();        // Moves arm back to starting position, then switches to wait mode.

// CODE

bool check_low_power() {
  int val = analogRead(SERVO_VOLTAGE_PIN);
  if (val < LOW_POWER_VOLTAGE) {
    switch_mode(low_power_mode);
    return true;
  }
  return false;
}

void setup() {
  pinMode(DEBUG_PIN, OUTPUT);                  // For oscilloscope debugging
  pinMode(INPUT_PIN, INPUT_PULLUP);            // Button input for scooping
  pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);  // Used for calibration
  pinMode(SERVO_POWER_DIR, OUTPUT);            // Motor direction A -> positive voltage for servos
  digitalWrite(SERVO_POWER_DIR, HIGH);
  pinMode(SERVO_POWER_PWM, OUTPUT);    // Motor enable A -> power for servos
  digitalWrite(SERVO_POWER_PWM, LOW);  // Disable servos during initialization
  pinMode(WARNING_LED_PIN, OUTPUT);
  digitalWrite(WARNING_LED_PIN, LOW);
  // Attach all motors
  j1.attach(5);
  j2.attach(6);
  DCMotor::attach();  // Sets pins 8, 11, 13 for Motor B brake, enable, and direction
  DCMotor::set_brake(false);
  DCMotor::set_direction(false);
  cur_mode = lift_step_fk;  // Start the robot by moving to the zero position
  pre = true;
  // Load profiles from EEPROM
  for (int i = 0; i < NUM_PROFILES; i++) {
    load_profile(i, profiles[i]);
  }
  profile = profiles[0];
  // When the servos turn on, they snap to their start position at full speed
  // So, this position is one that is unlikely to hit an obstacle.
  write_servos(-2.09, 2.09);            // This is -120 and 120 degrees, making an equilateral triangle.
  digitalWrite(SERVO_POWER_PWM, HIGH);  // Enable servos after setting targets to ensure servos recieve signal immediately
  timestamp = millis();
  // Give servos time to reach their target before starting the state machine
  while (millis() < timestamp + 500) {
    if (check_low_power()) {
      break;
    }
  }
}

void loop() {
  if (next_mode != NULL) {
    cur_mode = next_mode;
    next_mode = NULL;
    pre = true;
  }
  cur_mode();
  pre = false;
  // Every 0.1s, check if the profile choice has changed. If so, blink the LED.
  if ((millis() % 100) == 0) {
    uint8_t val = check_profile_choice();
    if (val != prev_profile_idx) {
      prev_profile_idx = val;
      digitalWrite(WARNING_LED_PIN, HIGH);
      delay(25);
      digitalWrite(WARNING_LED_PIN, LOW);
    }
  }
}

#pragma region Servo Control
/// Linearly interpolates from (-pi/2 pi/2) -> (SERVO_MIN_PW SERVO_MAX_PW)
int map_q_to_pulse_width(float q) {
  // See https://www.desmos.com/calculator/jx9jmkxbzo
  // y = mx + b, where x is an angle from [-pi/2, pi/2], y is a pulse width from SERVO_MIN_PW to SERVO_MAX_PW
  // m = ((SERVO_MAX_PW - SERVO_MIN_PW)/(PI))
  // y1 = m*x1 + b -> b = y1 - m*x1
  // b = (SERVO_MIN_PW - (m*(-PI/2)))
  // constexpr to force compiler to do this math at compile time instead of runtime
  constexpr float m = ((SERVO_MAX_PW - SERVO_MIN_PW) / (PI));
  constexpr float b = (SERVO_MIN_PW - (m * (-PI / 2)));
  return m * q + b;
}

void write_q1(float in_q1) {
  // J1 from -PI to 0
  // j1.writeMicroseconds(map_q_to_pulse_width(in_q1 + PI * 0.5)); // Opposite sign
  j1.writeMicroseconds(map_q_to_pulse_width(-in_q1 - PI * 0.5));
  q1 = in_q1;
}

void write_q2(float in_q2) {
  // J2 from 0 to PI
  // j2.writeMicroseconds(map_q_to_pulse_width(in_q2 - PI * 0.5)); // Opposite sign
  j2.writeMicroseconds(map_q_to_pulse_width(-in_q2 + PI * 0.5));
  q2 = in_q2;
}

void write_servos(float in_q1, float in_q2) {
  write_q1(in_q1);
  write_q2(in_q2);
}
#pragma endregion

// Returns a profile index based on current potentiometer value
int check_profile_choice() {
  // Average difference between division centers: 146
  // Center of profile 1: 476
  int val = analogRead(PROFILE_POT_PIN);
  int idx = (val - (476 - 146 / 2)) / 146;  // Subtract half a width to start at the "left" of profile 1 instead of the center.
  if (idx < 0) idx = 0;
  if (idx > 3) idx = 3;
  return idx;
}

void move_home_then_wait() {
  if (pre) {
    fk_step = 0;
  }
  timestamp = millis();
  if (fk_step == 0) {
    int ik_done = step_ik_target(HOME_X, HOME_Y, IK_STEP_SIZE);
    if (ik_done) {
      write_servos(Q1_HOME, Q2_HOME);
      ik_target_x = HOME_X;
      ik_target_y = HOME_Y;
      switch_mode(wait_mode);
    }
    bool ik_success = calc_ik(ik_target_x, ik_target_y, fk_target_q1, fk_target_q2);
  }
  fk_step = !step_joint_positions(fk_target_q1, fk_target_q2, MAX_JOINT_SPEED * 4, MAX_JOINT_SPEED * 4);
  write_servos(q1, q2);
  check_low_power();
}

void wait_mode() {
  // Input pin is pullup, so negative logic (pressed = LOW)
  if (digitalRead(INPUT_PIN) == LOW) {
    int idx = check_profile_choice();
    profile = profiles[idx];
    switch_mode(rotate_plate_step);
  }
  if (read_joystick_button()) {
    // Wait until joystick button is released
    timestamp = millis();
    while ((read_joystick_button() && (millis() - timestamp < 10000)) || (millis() - timestamp < 100)) {
      if (millis() - timestamp >= 5000) {
        digitalWrite(WARNING_LED_PIN, LOW);
      } else if (millis() - timestamp >= 1000) {
        digitalWrite(WARNING_LED_PIN, HIGH);
      }
    }
    digitalWrite(WARNING_LED_PIN, LOW);
    timestamp = millis() - timestamp;
    if (timestamp >= 10000) {
      reset_profiles();
      for (int i = 0; i < 4; i++) {
        digitalWrite(WARNING_LED_PIN, HIGH);
        delay(125);
        digitalWrite(WARNING_LED_PIN, LOW);
        delay(125);
      }
      while (read_joystick_button()) {}
    } else if (timestamp > 1000) {
      switch_mode(calibration_mode);
    } else {
      int idx = check_profile_choice();
      profile = profiles[idx];
      switch_mode(rotate_plate_step);
    }
  }
  check_low_power();
}

void low_power_mode() {
  if (pre) {
    digitalWrite(SERVO_POWER_PWM, 0);
    DCMotor::set_speed(0);
  }
  digitalWrite(WARNING_LED_PIN, HIGH);
  delay(500);
  digitalWrite(WARNING_LED_PIN, LOW);
  delay(500);
}

void rotate_plate_step() {
  if (pre) {
    timestamp = millis();
  }
  // At this point, we are looping while the button is pressed, so we wait until the button is released
  unsigned long elapsed = millis() - timestamp;
  // We don't switch until the elapsed time exceeds a certain value to ensure a minimum amount of plate rotation
  if (digitalRead(INPUT_PIN) == HIGH && elapsed > 250) {
    // then user has deactivated input and mode should be switched
    DCMotor::set_speed(0);
    switch_mode(descend_step);
  } else {
    DCMotor::set_speed(DC_MOTOR_SPEED);
  }
  check_low_power();
}

void descend_step() {
  if (pre) {
    timestamp = millis();
    ik_target_x = profile.entry_x;
    ik_target_y = profile.entry_y;
    calc_ik(ik_target_x, ik_target_y, fk_target_q1, fk_target_q2);

    balance_speed(fk_target_q1, fk_target_q2, MAX_JOINT_SPEED, q1_speed, q2_speed);
  }
  int fk_done = step_joint_positions(fk_target_q1, fk_target_q2, q1_speed, q2_speed);
  write_servos(q1, q2);
  if (fk_done) {
    switch_mode(scoop_step);
  }
  check_low_power();
}

void scoop_step() {
  if (pre) {
    ik_step = 1;
    fk_step = 0;
    timestamp = millis();
  }
  if (fk_step == 0) {
    float x = 0, y = 0;
    bool profile_success = get_profile_step(profile, ik_step, x, y);
    if (profile_success) {
      int ik_done = step_ik_target(x, y, IK_STEP_SIZE);
      if (ik_done) {
        ik_step += 1;
      }
      bool ik_success = calc_ik(ik_target_x, ik_target_y, fk_target_q1, fk_target_q2);
    } else {
      // We are done stepping through the profile, go to next mode
      switch_mode(lift_step_fk);
    }
  }
  // exponentially decrease speed if we go over threshold current
  int current = analogRead(0);
  if (current > OVERLOAD_CURRENT) {
    q1_speed = 0;
    q2_speed = 0;
  } else if (current > THRESHOLD_CURRENT) {
    float speed_perc = exp(SPEED_REDUCTION_STRENGTH * (THRESHOLD_CURRENT - current));
    q1_speed = MAX_JOINT_SPEED * speed_perc;
    q2_speed = MAX_JOINT_SPEED * speed_perc;
  } else {
    q1_speed = MAX_JOINT_SPEED;
    q2_speed = MAX_JOINT_SPEED;
  }

  fk_step = !step_joint_positions(fk_target_q1, fk_target_q2, q1_speed, q2_speed);
  write_servos(q1, q2);
  if (digitalRead(INPUT_PIN) == LOW || read_joystick_button()) {
    switch_mode(move_home_then_wait);
  }
  check_low_power();
}

void lift_step_fk() {
  if (pre) {
    fk_target_q1 = 0.0;
    fk_target_q2 = 0.0;
    ik_target_x = L1 + L2;
    ik_target_y = 0.0;
    balance_speed(fk_target_q1, fk_target_q2, MAX_JOINT_SPEED * 0.75, q1_speed, q2_speed);
  }
  bool fk_done = step_joint_positions(fk_target_q1, fk_target_q2, q1_speed, q2_speed);
  write_servos(q1, q2);
  if (fk_done) {
    switch_mode(feed_wait_step);
  }
}

void feed_wait_step() {
  if (digitalRead(INPUT_PIN) == LOW || read_joystick_button()) {
    while (digitalRead(INPUT_PIN) == LOW || read_joystick_button()) {}
    switch_mode(return_step);
  }
}

void return_step() {
  if (pre) {
    ik_target_x = profile.end_x;
    ik_target_y = profile.end_y + 30.0; // Make sure to clear the bowl/plate
    calc_ik(ik_target_x, ik_target_y, fk_target_q1, fk_target_q2);
    balance_speed(fk_target_q1, fk_target_q2, MAX_JOINT_SPEED, q1_speed, q2_speed);
  }
  bool fk_done = step_joint_positions(fk_target_q1, fk_target_q2, q1_speed, q2_speed);
  write_servos(q1, q2);
  if (fk_done) {
    switch_mode(move_home_then_wait);
  }
}

void head_nod() {
  float og_q1 = q1;
  float og_q2 = q2;
  while (!step_joint_positions(og_q1, og_q2 + 0.25, MAX_JOINT_SPEED * 0.5, MAX_JOINT_SPEED * 0.5)) {
    write_servos(q1, q2);
  }
  delay(250);
  while (!step_joint_positions(og_q1, og_q2, MAX_JOINT_SPEED, MAX_JOINT_SPEED)) {
    write_servos(q1, q2);
  }
  delay(250);
}

void calibration_mode() {
  static uint8_t calibration_step = 0;
  static uint8_t profile_index = 0;
  static Profile profile_to_change;
  if (pre) {
    calibration_step = 0;
    fk_step = 0;
    q1_speed = MAX_JOINT_SPEED;
    q2_speed = MAX_JOINT_SPEED;
    timestamp = millis();
    profile_index = check_profile_choice();
    if (profile_index < 0 || profile_index > (NUM_PROFILES-1)) {
      switch_mode(move_home_then_wait);
      return;
    }
    profile_to_change = profiles[profile_index];
    DCMotor::set_speed(DC_MOTOR_SPEED);
  }
  constexpr float speed = 0.05;
  const float max_mag = L1 + L2;
  int joy_x = read_joystick_x();
  int joy_y = read_joystick_y();
  ik_target_x += speed * joy_x;
  ik_target_y += speed * joy_y;
  // Prevent ik target from going out of absolute workspace bounds
  constrain_ik_point(ik_target_x, ik_target_y);

  if (fk_step == 0) {
    int ik_done = step_ik_target(ik_target_x, ik_target_y, IK_STEP_SIZE);
    bool ik_success = calc_ik(ik_target_x, ik_target_y, q1, q2);
  }
  write_servos(q1, q2);

  if (millis() - timestamp > 500 && read_joystick_button()) {
    unsigned long push_time = millis();
    while (read_joystick_button() && (millis() - push_time) <= 1000) {}
    if (millis() - push_time > 1000) { // Cancel the calibration
      DCMotor::set_speed(0);
      switch_mode(move_home_then_wait);
      while (read_joystick_button()) {}
      return;
    }
    switch (calibration_step) {
      case 0:  // Set entry
        profile_to_change.entry_x = ik_target_x;
        profile_to_change.entry_y = ik_target_y;
        break;
      case 1:  // Set bottom edge
        profile_to_change.bottom_x = ik_target_x;
        profile_to_change.bottom_y = ik_target_y;
        break;
      case 2:  // Set middle
        profile_to_change.middle_x = ik_target_x;
        profile_to_change.middle_y = ik_target_y;
        break;
      case 3:  // Set bottom front
        profile_to_change.front_x = ik_target_x;
        profile_to_change.front_y = ik_target_y;
        break;
      default:  // Set end, save, and return to home
        profile_to_change.end_x = ik_target_x;
        profile_to_change.end_y = ik_target_y;
        save_profile(profile_to_change, profile_index);
        profiles[profile_index] = profile_to_change;
        DCMotor::set_speed(0);
        switch_mode(move_home_then_wait);
        break;
    }
    calibration_step += 1;
    head_nod();
  }
  check_low_power();
}
