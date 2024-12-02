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

/**
 * @brief Resets all profiles and saves to EEPROM.
 */
void reset_profiles();

/**
 * @brief Saves a profile to an index in EEPROM. Returns if the save was successful.
 * @param p Profile to save
 * @param idx Profile index to overwrite in EEPROM
 * @return True if successful
 */
bool save_profile(const Profile &p, uint8_t idx);

/**
 * @brief Loads a profile from a specified index in EEPROM.
 * @param idx Profile index in EEPROM
 * @param p Pointer to Profile struct to initialize.
 * @return True if the load was successful.
 */
bool load_profile(uint8_t idx, Profile &p);

/**
 * @param p Profile to read
 * @param step Index of step
 * @param x_addr x value to initialize
 * @param y_addr y value to initialize
 * @returns True if the profile step exists. If true, x_addr and y_addr are written, if false they are left unmodified from their original values.
 */
bool get_profile_step(const Profile &p, int step, float &x_addr, float &y_addr);

#endif