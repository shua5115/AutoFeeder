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
 * @brief Resets all profiles
 */
void reset_profiles();

/**
 * @brief Saves a profile to an index in EEPROM. Returns if the save was successful.
 * @param p Pointer to
 * @param idx Pointer to
 */
bool save_profile(const Profile &p, uint8_t idx);

/**
 * @brief Loads a profile from a specified index in EEPROM.
 * @param idx
 * @param p Pointer to
 * @returns True if the load was successful.
 */
bool load_profile(uint8_t idx, Profile &p);

/**
 * @brief Checks if a profile contains valid start and end positions.
 * @param p Pointer to
 * @param step
 * @param x_addr Pointer to
 * @param y_addr Pointer to
 * @returns True if the profile step exists, otherwise False.
 */
bool get_profile_step(const Profile &p, int step, float &x_addr, float &y_addr);

#endif