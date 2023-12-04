#include "Profile.h"
#include "kinematics.h"
#include <Arduino.h>
#include <EEPROM.h>

#define PROFILE_EEPROM_START 0
#define PROFILE_EEPROM_LEN 4

Profile profiles[4];

Profile plate_profile = {
  -80, -155,  // start
  -76, -175,  // bottom
  0, -177.5,    // middle
  76, -180,   // front
  80, -155    // end
};

Profile bowl_profile = {
  -75, -82.5,  // start
  -70, -175,   // bottom
  0, -175,     // middle
  70, -175,    // front
  60, -90      // end
};

void reset_profiles() {
  for (int i = 0; i < NUM_PROFILES; i++) {
    profiles[i] = (i < NUM_PROFILES / 2) ? bowl_profile : plate_profile;
    save_profile(profiles[i], i);
  }
}

bool save_profile(const Profile &p, uint8_t idx) {
  if (idx >= PROFILE_EEPROM_LEN) { return false; }
  EEPROM.put(PROFILE_EEPROM_START + (sizeof(Profile) * idx), p);
  return true;
}

bool load_profile(uint8_t idx, Profile &p) {
  if (idx >= PROFILE_EEPROM_LEN) { return false; }
  EEPROM.get(PROFILE_EEPROM_START + (sizeof(Profile) * idx), p);
  constrain_ik_point(p.entry_x, p.entry_y);
  constrain_ik_point(p.bottom_x, p.bottom_y);
  constrain_ik_point(p.middle_x, p.middle_y);
  constrain_ik_point(p.front_x, p.front_y);
  constrain_ik_point(p.end_y, p.end_y);
  return true;
}

bool get_profile_step(const Profile &p, int step, float &x_addr, float &y_addr) {
  switch (step) {
    case 0:
      x_addr = p.entry_x;
      y_addr = p.entry_y;
      return true;
    case 1:
      x_addr = p.bottom_x;
      y_addr = p.bottom_y;
      return true;
    case 2:
      x_addr = p.middle_x;
      y_addr = p.middle_y;
      return true;
    case 3:
      x_addr = p.front_x;
      y_addr = p.front_y;
      return true;
    case 4:
      x_addr = p.end_x;
      if (p.end_y < -(L1 + 50)) {
        y_addr = p.end_y + 15;
      } else {
        y_addr = p.end_y + 25;
      }
      return true;
    default:
      return false;
  }
  return true;
}