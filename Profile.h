#ifndef PROFILE_H
#define PROFILE_H
#include <stdint.h>

#define NUM_PROFILES 4

typedef struct Profile {
  float entry_x, entry_y;
  float bottom_x, bottom_y;
  float middle_x, middle_y;
  float front_x, front_y;
  float end_x, end_y;
} Profile;

extern Profile profiles[4];

void reset_profiles();

/// Saves a profile to an index in EEPROM. Returns if the save was successful.
bool save_profile(const Profile &p, uint8_t idx);

/// Loads a profile from a specified index in EEPROM. Returns if the load was successful.
bool load_profile(uint8_t idx, Profile &p);

// /// Checks if a profile contains valid start and end positions.
// bool is_valid_profile(const Profile &p);

bool get_profile_step(const Profile &p, int step, float &x_addr, float &y_addr);

#endif