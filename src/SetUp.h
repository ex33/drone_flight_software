
#ifndef _SETUP_H
#define _SETUP_H

#include <Arduino.h> //Platformio doesn't insert this at compile time like Ardiuno does
#include <Servo.h>
namespace SETUP {
// ========================= Vehicle Parameters =========================

// Motor Parameters
inline constexpr float kM = 1.0f; 
inline constexpr float kT = 1.0f;

//Physical parameters
inline constexpr float L = 0.75f; // Pitch / Roll Moment arm

// ========================= Sensors =========================

// Sensor polling frequencies
inline constexpr float imuFrequency {100}; //Hz
inline constexpr float magFrequency {50}; //Hz
inline constexpr float altFrequency {50}; //Hz
inline constexpr float gpsFrequency {1}; //Hz

// Sensor Calibration / Orientations
inline constexpr std::array<float,3> magHardIron{0.0, 0.0, 0.0};
inline constexpr std::array<float,9> magSoftIron{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
inline constexpr std::array<float,9> rotIMU2Body{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
inline constexpr std::array<float,9> rotMag2Body{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
inline constexpr std::array<float,9> rotMag2TrueNED{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

// GPS Wiring
HardwareSerial& gpsSerial = Serial8; // GPS Connected to Tx/Rx 8 UART

//========================= Navigation =========================
inline constexpr std::array<float,3> p0 {0.0f,0.0f,0.0f};
inline constexpr std::array<float,3> v0 {0.0f,0.0f,0.0f};
inline constexpr std::array<float,4> q0 {1.0f,0.0f,0.0f,0.0f};
inline constexpr std::array<float,3> ba0 {0.0f,0.0f,0.0f};
inline constexpr std::array<float,3> bg0 {0.0f,0.0f,0.0f};
inline constexpr std::array<float,3> bm0 {0.0f,0.0f,0.0f};


inline constexpr std::array<float,324> P0 {
    3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f
};

inline constexpr float dt (0.02f); // 50hz
//Set Process Noise
inline constexpr float sig_acc(0.003f);
inline constexpr float sig_gyro(0.5f);
inline constexpr float eta_acc(0.01f);
inline constexpr float eta_gyro(0.0001f);
inline constexpr float eta_mag(0.001f);
//Set Measurement Noise
inline constexpr float sig_mag(0.003f);
inline constexpr float sig_tilt(0.08f);
inline constexpr float sig_alt(0.1f);
inline constexpr float sig_gps_pos(5.0f);
inline constexpr float sig_gps_vel(0.1f);


// ========================= Motors =========================
// Pins are 28 29 37 36 (top to bottom), left to right
inline constexpr int esc1SignalPin = 28;
inline constexpr int esc2SignalPin = 37;
inline constexpr int esc3SignalPin = 29;
inline constexpr int esc4SignalPin = 36;

inline constexpr int maxPWM = 2000;
inline constexpr int minPWM = 1000;
inline constexpr int M1StartPWM = 1050; 
inline constexpr int M2StartPWM = 1150; 
inline constexpr int M3StartPWM = 1150; 
inline constexpr int M4StartPWM = 1150; 
}
#endif
