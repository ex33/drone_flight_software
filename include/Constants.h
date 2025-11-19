#ifndef _CONSTANTS_H
#define _CONSTANTS_H


namespace CONSTANTS{
// Conversions
inline constexpr float deg2rad = 0.0174532925199433f; // pi/180


// Time constants
inline const unsigned long seconds2micro = 1000000UL;
// GPS Constants
inline constexpr float WGS84_a = 6378137.0f; // m, semi-major axis of ellpsiod
inline constexpr float WGS84_e2 = 0.006694379990140f; // n.d., eccentricity^2 of ellipsoid
// Altimeter Constants
inline constexpr float seaLevelPressure = 1013.25f; //Hardcode this for altimeter [hPa]
// IMU Constants
inline constexpr float g0 = 9.80665; //m/s^2
}
#endif