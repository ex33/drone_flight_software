
#ifndef _SETUP_H
#define _SETUP_H


#include <Arduino.h> //Platformio doesn't insert this at compile time like Ardiuno does
#include <Servo.h>
#include <Sensors.h>
#include <Constants.h>
#include <Logger.h>
#include <Motors.h>
#define SD_CS BUILTIN_SDCARD  // Teensy 4.1 internal SD slot

namespace SETUP {
// ========================= Motors =========================
// Motor Parameters
inline constexpr float kT = 6.583e-5f; // [ N*s^2/rev^2 ] Thrust Constant
inline constexpr float kM = 7.51e-7f; // [ N*m*s^2/rev^2 ] Torque Constant

//Physical parameters
inline constexpr float L = 0.08f; // Pitch / Roll Moment arm


// Pins are 28 29 37 36 (top to bottom), left to right
inline constexpr int esc1SignalPin = 28;
inline constexpr int esc2SignalPin = 37;
inline constexpr int esc3SignalPin = 29;
inline constexpr int esc4SignalPin = 36;

inline constexpr int maxPWM = 1950; //Recommended by Hobby Wing
inline constexpr int minPWM = 1150; //Recommended by Hobby Wing
inline constexpr int saturationPWM = 1700; // ~50% of MAX. Will use a flag that if this triggers for X times, then something went wrong with vontroller... Based on testing, shouldn't need more than ~30% for hover

inline constexpr int M1StartPWM = 1195; 
inline constexpr int M2StartPWM = 1195; 
inline constexpr int M3StartPWM = 1195; 
inline constexpr int M4StartPWM = 1195; 

// Without a way to record RPM, will backcalculate from kT
// Give the max pwm of 1950 and record the max thrust (can do multiple samples and average)
// Using kT, then back calculate max spin rate
// Add some factor to knock this down. Better to under-estimate actuator capabilities than over
inline constexpr float maxSpinSquare = 160000.0f; // [rev^2 / s^2] Used to Convert Control requested to PWM




// ========================= Sensors =========================

// Sensor polling frequencies
inline constexpr float imuFrequency {1000.0f}; //Hz
inline constexpr float magFrequency {100.0f};//{50}; //Hz
inline constexpr float altFrequency {25.0f}; //Hz, 25 Hz for Indoors
inline constexpr float gpsFrequency {1.0f}; //Hz


// Sensor Calibration / Orientations
inline constexpr std::array<float,3> magHardIron{-54.756429, 7.184927, 13.532636};

inline constexpr std::array<float,9> magSoftIron {0.994268, 0.009895, 0.008865,
                                                0.009895, 1.010350, 0.031712,
                                                0.008865, 0.031712, 1.000568};


inline constexpr std::array<float,9> rotBody2IMU{1.0, 0.0, 0.0, 
                                                0.0, -1.0, 0.0, 
                                                0.0, 0.0, -1.0};
                                                
inline constexpr std::array<float,9> rotBody2Mag{1.0, 0.0, 0.0, 
                                                0.0, 1.0, 0.0, 
                                                0.0, 0.0, 1.0}; //Note that the z axis is flipped within sensors.


// GPS Wiring
HardwareSerial& gpsSerial = Serial8; // GPS Connected to Tx/Rx 8 UART
bool gpsFlag = false; //Are we flying with GPS or not

//========================= Navigation =========================
inline constexpr std::array<float,3> p0 {0.0f,0.0f,0.0f};
inline constexpr std::array<float,3> v0 {0.0f,0.0f,0.0f};
inline constexpr std::array<float,4> q0 {1.0f,0.0f,0.0f,0.0f};
// inline constexpr std::array<float,3> ba0 {0.0f,0.0f,0.0f};
// inline constexpr std::array<float,3> bg0 {0.0f,0.0f,0.0f};
// inline constexpr std::array<float,3> bm0 {0.0f,0.0f,0.0f};


inline constexpr std::array<float,81> P0 {
    5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f
};

//Set Process Noise
inline constexpr float sig_acc(0.045f); //From static calibration, this is closer to ~0.044for highest axis
inline constexpr float sig_gyro(0.004f); // From static calibratation, this is closer to ~0.004
// inline constexpr float eta_acc(0.001f);
// inline constexpr float eta_gyro(0.0005f);
// inline constexpr float eta_mag(0.0001f);
//Set Measurement Noise
inline constexpr float sig_mag(0.5f); //From static calibration, closer to ~0.45
inline constexpr float sig_tilt(0.5f);
inline constexpr float sig_alt(0.1f); // From static calibration, this is ~0.04m
inline constexpr float sig_gps_pos(3.0f);
inline constexpr float sig_gps_vel(0.1f);

// This is the Reference vector for EKF. Either user provided, or passed to from Sensor Calibrations, in which We aren't really pointed at True North or Mag north, and using a local frame instead that is rotated from Mag North.
//inline constexpr Vector3f magRef{11.4299 * PI/180}; 
inline constexpr bool nis_gating_flag (false);

// ========================= Controller =========================
inline constexpr float controlFrequency {500}; // Hz
inline constexpr std::array<float,3> pRef {0.0f, 0.0f, 0.0f};
inline constexpr std::array<float,3> vRef {0.0f, 0.0f, 0.0f};
inline constexpr std::array<float,4> qRef {1.0f, 0.0f, 0.0f, 0.0f};
inline constexpr std::array<float,3> wRef {0.0f, 0.0f, 0.0f};
inline constexpr std::array<float,4> uRef {8.14f, 0.0f, 0.0f, 0.0f}; //[Ft, Mx, My. Mz]. Ft is the THRUST magnitude.
// Gains for u = Ft Mx My Mz
// Everywhere there is a 0 has a magntitude of <1e-13.
inline constexpr std::array<float,48> K {0.0f , 0.0f, -9.235291726374717f , 0.0f, 0.0f, -26.459385705162603f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                         0.0f, 0.063349178891246 , 0.0f, 0.0f, 0.216910884742852, 0.0f, 2.292320283163765, 0.0f, 0.0f, 0.300548926668002f, 0.0f, 0.0f,
                                        -0.082236680485872f, 0.0f, 0.0f, -0.281605587634183f, 0.0f, 0.0f, 0.0f, 2.977310240689001 , 0.0f, 0.0f, 0.390567835752289f, 0.0f, 
                                         0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.308247131137511f, 0.0f, 0.0f, 0.311508647946585f};

// inline constexpr std::array<float,48> K {
//    -0.0000,   -0.0000,   -1.0951,   -0.0000,    0.0000,   -3.3836,    0.0000,    0.0000    ,0.0000 ,   0.0000 ,   0.0000,    0.0000,
//    -0.0000,    0.0257,    0.0000,   -0.0000,    0.0883,    0.0000 ,   0.9551,    -0.0000  , -0.0000 ,   0.1288,    0.0000,   -0.0000,
//    -0.0273,   -0.0000,   -0.0000,   -0.0941,    0.0000,   -0.0000 ,   0.0000,     1.0267 ,   0.0000,    0.0000,    0.1400,    0.0000,
//    -0.0000,    0.0000,   -0.0000,   -0.0000,   -0.0000,   -0.0000 ,  -0.0000,    0.0000 ,   0.1289 ,  -0.0000,    0.0000,    0.1317};

// inline constexpr std::array<float,48> K {
//     0.0000,    0.0000,   -9.2353,    0.0000,    0.0000,  -26.4594,    0.0000,     0.0000,    0.0000,    0.0000,    0.0000,   -0.0000,
//    -0.0000,    0.0311,   -0.0000,   -0.0000,    0.0982,   -0.0000,    0.5894,    0.0000,   -0.0000,    0.0448,    0.0000,   -0.0000,
//    -0.0314,    0.0000,    0.0000,   -0.0999,    0.0000,    0.0000,    0.0000,    0.6465,   -0.0000,    0.0000,    0.0532,   -0.0000,
//    -0.0000,    0.0000,   -0.0000,   -0.0000,   -0.0000,    0.0000,   -0.0000,    0.0000,    0.0147,   -0.0000,    0.0000,    0.0171};





// Can independently turn on/off controller for NE + D positions.
// Disable everything if testing attitude controller
// Enable verticalControllerFlag once attitude ocntroller is good
// Enable everything once confident things are working

bool horizontalControllerFlag = false; //Flag for Controller to determine if position controller is being used
bool verticalControllerFlag = false; // Flag for Controller to determine if height (hover / altitude hold) is disabled




// ========================= LOGGER / RINGBUFFER SIZE =========================
// Make these power 2 since ringbuffer modulus function is for bitwise
// Based on frequencies set below. NOT loop frequencies
inline constexpr int imuRingBufferSize = 2048; //For 200Hz will hold ~2-3 seconds of data
inline constexpr int magRingBufferSize = 256; // For 50Hz, should hold ~4-5 seconds of data
inline constexpr int altRingBufferSize = 128; //For 25Hz, should hold ~ 4-5 seconds of data
inline constexpr int gpsRingBufferSize = 4;   // FOr 1 Hz, should hold ~ 4 seconds of data
inline constexpr int gncRingBufferSize = 512; //For 200Hz will hold ~2-3 seconds of data

// Should think about this...
// TODO:: In the future, might want to be able to keep a buffer of EVERY measurement / state in order to 
// back propagate if needed, In which case, it might be better to just extract every X-th measurement from that buffer based on the frequencies here
// Or maybe just have seperate buffers for less book keeping
// IF THESE CHANGE, MAKE SURE TO CHANGE SIZE OF STURCTURE INSIDE LOGGER FOR EACH LOGSENSOR
inline constexpr float logIMUDataFrequency {200}; //Hz. Logs every 4th IMU Sample
inline constexpr float logMagDataFrequency {50}; //Hz. Logs every 2nd IMU Sample
inline constexpr float logAltDataFrequency {25}; //Hz. Logs every Altmeter sample
inline constexpr float logGPSDataFrequency {1}; //Hz. Logs every GPS sample
inline constexpr float logGNCDataFrequency {100}; //Hz.  Logs State AND Controller AND Motor information every 250 times in 1 second.

inline constexpr float logFlushFrequency {10}; //Hz. This is the rate in which Data are written to the file and flushed


// Setup Functions
// ============================ Shortcuts for various Setups [START] ============================
// Subroutines to run during void setup. Actual setup further down

void SETUP_Offboard_Calibration(Sensors& sensors) {
  Wire.begin(); //Initializes default I2C bus.
  pinMode(LED_BUILTIN, OUTPUT); //Configure built in LED 
  // Start the the SD Card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card not found");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }; 
  } else {
      Serial.println("SD card found");
  }
  //Use Built in LED to show that we are starting...
  for (int i=0; i<5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
  delay(2000);

  digitalWrite(LED_BUILTIN, HIGH);
  //Be on for a pattern to show that we are starting. either blink in count down or something
  // --- Calibrate Sensors ---
  sensors.startUpSensors();
  sensors.calibrateMagnetometer();

  digitalWrite(LED_BUILTIN, LOW);

  // Forever loop since this is calibration and we don't want to go to the rest of the main loop
  while(1) {};

};

// Need to Upload, plug in battery, then open serial monitor. Cann't have battery already plugged. Do it all one by one for more consistent results
void SETUP_Calibrate_Motor_Individual(Motors& motors, int num) {
  Serial.begin(9600); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  while (!Serial) {
    //Do nothing until serial monitor is opened
  };
  // For calibration, start motor HIGH, then low. This is opposite of flight commands which must start LOW
  // Calibrate Motor 1 
  Serial.print("Starting Motor ");
  Serial.print(num);
  Serial.println(" High");
  motors.commandMotors(SETUP::maxPWM, 3000, num); // Maximum throttle for 3 seconds
  Serial.println("Starting Motor 1 Low");
  motors.commandMotors(SETUP::minPWM, 3000, num); // Minimum throttle for 3 seconds
  Serial.println("Done");
  while(1) {};
};



void SETUP_Find_Motors_Start(Motors& motors) {
  Serial.begin(9600); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  while (!Serial) {
    //Do nothing until serial monitor is opened
  };
  // Observe which PWM is when the motor starts spinning. This will be the lower bound of PWM commands
  motors.setUp();
  motors.arm();

  //Find the off-set needed after calibration
  Serial.println("Starting Motor 1 1175");
  motors.commandMotors(1175, 2000, 1);
  Serial.println("Starting Motor 1 1180");
  motors.commandMotors(1180, 2000, 1);
  Serial.println("Starting Motor 1 1185");
  motors.commandMotors(1185, 2000, 1);
  Serial.println("Starting Motor 1 1190");
  motors.commandMotors(1190, 2000, 1);
  Serial.println("Starting Motor 1 1195");
  motors.commandMotors(1195, 2000, 1);
  Serial.println("Starting Motor 1 1200");
  motors.commandMotors(1200, 2000, 1);
  Serial.println("Starting Motor 1 Low");
  motors.commandMotors(SETUP::minPWM, 2000, 1);

  // Find the off-set needed after calibration
  Serial.println("Starting Motor 2 1175");
  motors.commandMotors(1175, 2000, 2);
  Serial.println("Starting Motor 2 1180");
  motors.commandMotors(1180, 2000, 2);
  Serial.println("Starting Motor 2 1185");
  motors.commandMotors(1185, 2000, 2);
  Serial.println("Starting Motor 2 1190");
  motors.commandMotors(1190, 2000, 2);
  Serial.println("Starting Motor 2 1195");
  motors.commandMotors(1195, 2000, 2);
  Serial.println("Starting Motor 2 1200");
  motors.commandMotors(1200, 2000, 2);
  Serial.println("Starting Motor 2 Low");
  motors.commandMotors(SETUP::minPWM, 2000, 2);

  // Find the off-set needed after calibration
  Serial.println("Starting Motor 3 1175");
  motors.commandMotors(1175, 2000, 3);
  Serial.println("Starting Motor 3 1180");
  motors.commandMotors(1180, 2000, 3);
  Serial.println("Starting Motor 3 1185");
  motors.commandMotors(1185, 2000, 3);
  Serial.println("Starting Motor 3 1190");
  motors.commandMotors(1190, 2000, 3);
  Serial.println("Starting Motor 3 1195");
  motors.commandMotors(1195, 2000, 3);
  Serial.println("Starting Motor 3 1200");
  motors.commandMotors(1200, 2000, 3);
  Serial.println("Starting Motor 3 Low");
  motors.commandMotors(SETUP::minPWM, 2000, 3);

  // Find the off-set needed after calibration
  Serial.println("Starting Motor 4 1175");
  motors.commandMotors(1175, 2000, 4);
  Serial.println("Starting Motor 4 1180");
  motors.commandMotors(1180, 2000, 4);
  Serial.println("Starting Motor 4 1185");
  motors.commandMotors(1185, 2000, 4);
  Serial.println("Starting Motor 4 1190");
  motors.commandMotors(1190, 2000, 4);
  Serial.println("Starting Motor 4 1195");
  motors.commandMotors(1195, 2000, 4);
  Serial.println("Starting Motor 4 1200");
  motors.commandMotors(1200, 2000, 4);
  Serial.println("Starting Motor 4 Low");
  motors.commandMotors(SETUP::minPWM, 2000, 4);


  Serial.println("Done");
  while(1) {};
}




void SETUP_Find_Motor_Directions(Motors& motors) {
  Serial.begin(9600); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  while (!Serial) {
    //Do nothing until serial monitor is opened
  };
  motors.setUp();
  motors.arm();

  // Spin each motor slowly to observe Spin Direction. 
  // REMEMBER SPIN DIRECTION IN REFERENCE TO DOWN AXIS
  // SO IF VIEWING FROM TOP, CW --> CCW and CCW --> CW
  delay(2000);
  // Observe the Spin Direction of motors and make sure they are right
  Serial.println("Starting Motor 1 Minimum PWM");
  motors.commandMotors(SETUP::M1StartPWM, 2000, 1);
  Serial.println("Starting Motor 1 Low");
  motors.commandMotors(SETUP::minPWM, 2000, 1);

  // Find the off-set needed after calibration
  Serial.println("Starting Motor 2 Minimum PWM");
  motors.commandMotors(SETUP::M2StartPWM, 2000, 2);
  Serial.println("Starting Motor 2 Low");
  motors.commandMotors(SETUP::minPWM, 2000, 2);


  // Find the off-set needed after calibration
  Serial.println("Starting Motor 3 Minimum PWM");
  motors.commandMotors(SETUP::M3StartPWM, 2000, 3);
  Serial.println("Starting Motor 3 Low");
  motors.commandMotors(SETUP::minPWM, 2000, 3);

  // 
  Serial.println("Starting Motor 4 Minimum PWM");
  motors.commandMotors(SETUP::M3StartPWM, 2000, 4);
  Serial.println("Starting Motor 4 Low");
  motors.commandMotors(SETUP::minPWM, 2000, 4);

  Serial.println("Done");
  while(1) {};
}

void SETUP_Find_Thrust_Constant(Motors& motors,int PWM) {
  Serial.begin(9600);
  while (!Serial) {};
  pinMode(LED_BUILTIN, OUTPUT); //Configure built in LED 

  motors.setUp();
  motors.arm();

  Serial.println("STARTING IN");
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
  //1950 is max
  // 1150 is min
  // X percent throttle is (1950 - 1150 ) * X +1150 
  // Do 0, 10, 20, 30, 40, 50, 60. 
  motors.commandMotors(std::array<int,4>{ PWM, PWM, PWM, PWM},3000);
  while(1) {};
}


void SETUP_Find_Motor_Max_Spin(Motors& motors) {
  Serial.begin(9600); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  while (!Serial) {
    //Do nothing until serial monitor is opened
  };

  pinMode(LED_BUILTIN, OUTPUT); //Configure built in LED 
  for (int i=0; i<3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }

  motors.setUp();
  motors.arm();

  //1950 is max
  // 1150 is min
  // ~70 percent throttle is (1950 - 1150 ) * 0.7 +1150 =  1710
  for (int i=0; i<3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
  Serial.println("STARTING!");
  //motors.commandMotors(std::array<int,4>{ SETUP::M3StartPWM,  SETUP::M3StartPWM,  SETUP::M3StartPWM, SETUP::M3StartPWM},2000);
  motors.commandMotors(std::array<int,4>{ 1790, 1790, 1790, 1790},5000);

  while(1) {};
};


// Run to just check out all Sensors and make sure motors are connected properly
void SETUP_Test_Sensors_Motors(Sensors& sensors, Motors& motors) {
  Serial.begin(9600); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  while (!Serial) {
    //Do nothing until serial monitor is opened
  };
  Wire.begin(); //Initializes default I2C bus (SDA / SCL pins)

  // Determine if we want to run with GPS or not
  sensors.setGPSFlag(false);

  // ------------- Sensor Checkout and Setup -----------------
  sensors.startUpSensors();
  sensors.setUpSensors(SETUP::magHardIron, SETUP::magSoftIron, SETUP::rotBody2IMU, SETUP::rotBody2Mag); //Also sets up frequencies of sensors / ODR [HARDCODED]

  // Run Calibrations
  sensors.calibrateSensors(30); // Get IMU Bias, checks GPS lock


  Serial.println("Done Test, Forever looping...");
  while (1) { //Eventually replace with error
    delay(10); //Infinite loop catches IMU not initized 
  };
  motors.setUp();
  motors.arm();

  // Do some delay before the start
  pinMode(LED_BUILTIN, OUTPUT); //Configure built in LED 
  //Use Built in LED to show that we are starting...
  for (int i=0; i<5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
  delay(2000);

};

void SETUP_SpinMotor(Motors& motors, int num, int PWM, uint32_t duration) {
  Serial.begin(9600); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  while (!Serial) {
    //Do nothing until serial monitor is opened
  };
  motors.setUp();
  motors.arm();
  delay(2000); //Delay to make sure ESCs are armed and ready. 
  Serial.print("Spinning Motor ");
  Serial.print(num);
  Serial.print(" at PWM ");
  Serial.println(PWM);
  motors.commandMotors(PWM, duration, num); // Spin for 5 seconds
  Serial.println("Done Spinning");
  while(1) {};
};


// Mostly used for Allan Variance, but also save down magnetometer and altimeter data to take a look at noise 
void SETUP_AllanVariance(Sensors& sensors) { //Using millis() to prevent overflow from bits
  // Set up Time durations
  uint32_t imu_logger_flush_rate = 1000 * 60* 10; //10 minutes
  uint32_t imu_logger_duration =  CONSTANTS::seconds2milli * 10UL * 60UL * 60UL; // 3 Hours
  uint32_t mag_alt_logger_duration = CONSTANTS::seconds2milli * 10 * 60; // 10 minutes 
  //====================Start of Actual Code====================

  //Begin Serial 
  Serial.begin(9600); 
  while (!Serial) {};
  //Begin SD Card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card not found");
    while (1) {}; 
  } else {
      Serial.println("SD card found");
  }

  // Begin Sensors
  Wire.begin(); //Initializes default I2C bus (SDA / SCL pins)
  sensors.setGPSFlag(SETUP::gpsFlag); //Not running GPS
  sensors.startUpSensors(); // Ensures communication is on
  sensors.setUpSensors(SETUP::magHardIron, SETUP::magSoftIron, SETUP::rotBody2IMU, SETUP::rotBody2Mag); //Also sets up frequencies of sensors / ODR [HARDCODED]

  // Clean and open up file for logging
  if (SD.exists("imu_allan_variance.csv")) {
      SD.remove("imu_allan_variance.csv");
  }
  File imuAllanVarianceFile = SD.open("imu_allan_variance.csv", FILE_WRITE); //imu .csv
  if (!imuAllanVarianceFile) {
      Serial.println("Failed to open IMU Allan Variance file");
      return;
  }
  imuAllanVarianceFile.println("t,ax,ay,az,gx,gy,gz"); 

  // Set up Variables for IMU loop
  std::array<float,6> imuMeas; //Holds IMU Measurements

  uint32_t time_elapsed = 0.0; //Keeps track of time in loop
  uint32_t logger_last_flush = 0.0; //Keeps track of last SD card flush

  Serial.print("Starting IMU Logging...");
  uint32_t initial_time = millis(); //Keeps track of initial time so time starts at 0
  uint32_t loop_start_time;
  while (time_elapsed < imu_logger_duration) {
    loop_start_time = millis();

    if (sensors.imuUpdate(loop_start_time)) {
      time_elapsed = loop_start_time - initial_time; 
      imuMeas = sensors.getIMUMeas();

        //Write to file
        imuAllanVarianceFile.print(time_elapsed/CONSTANTS::seconds2milli); //Convert milli seconds to seconds
        imuAllanVarianceFile.print(",");
        imuAllanVarianceFile.print(imuMeas[0],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
        imuAllanVarianceFile.print(",");
        imuAllanVarianceFile.print(imuMeas[1],6);
        imuAllanVarianceFile.print(",");
        imuAllanVarianceFile.print(imuMeas[2],6); 
        imuAllanVarianceFile.print(",");
        imuAllanVarianceFile.print(imuMeas[3],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
        imuAllanVarianceFile.print(",");
        imuAllanVarianceFile.print(imuMeas[4],6);
        imuAllanVarianceFile.print(",");
        imuAllanVarianceFile.println(imuMeas[5],6); 

      if (loop_start_time - logger_last_flush >= imu_logger_flush_rate) { //Flush every 10 minutes
          logger_last_flush = millis();
          imuAllanVarianceFile.flush();
      }
    }
  }
  imuAllanVarianceFile.close();

  Serial.println("Done IMU Logging...");
  // ------------------------- Start of magnetometer and altimeter loop ----------------------------
  // Clean and open up logger files
  if (SD.exists("mag_white_noise.csv")) {
      SD.remove("mag_white_noise.csv");
  }
  if (SD.exists("alt_white_noise.csv")) {
      SD.remove("alt_white_noise.csv");
  }
  File magWhiteNoiseFile = SD.open("mag_white_noise.csv", FILE_WRITE); //imu .csv
  if (!magWhiteNoiseFile) {
      Serial.println("Failed to open Magnetometer White Noise file");
      return;
  }
  File altWhiteNoiseFile = SD.open("alt_white_noise.csv", FILE_WRITE); //imu .csv
  if (!altWhiteNoiseFile) {
      Serial.println("Failed to open Altimeter White Noise file");
      return;
  }
  magWhiteNoiseFile.println("t,mx,my,mz"); 
  altWhiteNoiseFile.println("t,h");

  // Variables for loop
  std::array<float,3> magMeas; //Magnetometer measurements
  float altMeas; //Altimeter measurements
  time_elapsed = 0.0; // Reset Time
  initial_time = millis();
  while (time_elapsed < mag_alt_logger_duration) { //No need for periodic flushing since this is running for a shorter time

    loop_start_time = millis();

    // Magnetometer loop
    if (sensors.magUpdate(loop_start_time)) {
      time_elapsed = loop_start_time - initial_time;
      magMeas = sensors.getMagMeas();

      magWhiteNoiseFile.print(time_elapsed/1000);
      magWhiteNoiseFile.print(",");
      magWhiteNoiseFile.print(magMeas[0],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
      magWhiteNoiseFile.print(",");
      magWhiteNoiseFile.print(magMeas[1],6);
      magWhiteNoiseFile.print(",");
      magWhiteNoiseFile.println(magMeas[2],6); 

    }
    // Altimeter loop
    if (sensors.altUpdate(loop_start_time)) {
      time_elapsed = loop_start_time - initial_time;
      altMeas = sensors.getAltMeas();
      //Write to file
      altWhiteNoiseFile.print(time_elapsed/1000);
      altWhiteNoiseFile.print(",");
      altWhiteNoiseFile.println(altMeas,6); 

    }

 
  }
  magWhiteNoiseFile.close();
  altWhiteNoiseFile.close();

  Serial.println("Done, Looping Forever...");
  while(1) {
  };

}



}
#endif
