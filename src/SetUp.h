
#ifndef _SETUP_H
#define _SETUP_H


#include <Arduino.h> //Platformio doesn't insert this at compile time like Ardiuno does
#include <Servo.h>
#include <Sensors.h>
#include <Constants.h>
#include <Logger.h>
#define SD_CS BUILTIN_SDCARD  // Teensy 4.1 internal SD slot
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
inline constexpr float altFrequency {25}; //Hz, 25 Hz for Indoors
inline constexpr float gpsFrequency {1}; //Hz
//Use these in main to determine looping frequencies. Will replace with interrupts eventually
float imuLoopFrequency = 1000000UL / imuFrequency; 
float magLoopFrequency = 1000000UL / magFrequency;
float altLoopFrequency = 1000000UL / altFrequency; 
float gpsLoopFrequency = 1000000UL / gpsFrequency;

// Sensor Calibration / Orientations
//inline constexpr std::array<float,3> magHardIron{-53.550001000000002 ,  7.199998500000000 ,-14.550001500000000};
inline constexpr std::array<float,3> magHardIron{-54.756429, 7.184927, 13.532636};
// inline constexpr std::array<float,9> magSoftIron{0.022796974373613, 0.000169347121256, -0.000135727939028, 
//                                                  0.000169347121256, 0.022743898721181, -0.000716086610041,
//                                                  -0.000135727939028, -0.000716086610041, 0.022593055575845};
inline constexpr std::array<float,9> magSoftIron {0.994268, 0.009895, 0.008865,
                                                0.009895, 1.010350, 0.031712,
                                                0.008865, 0.031712, 1.000568};


inline constexpr std::array<float,9> rotBody2IMU{1.0, 0.0, 0.0, 
                                                0.0, -1.0, 0.0, 
                                                0.0, 0.0, -1.0};
                                                
inline constexpr std::array<float,9> rotBody2Mag{1.0, 0.0, 0.0, 
                                                0.0, 1.0, 0.0, 
                                                0.0, 0.0, 1.0};


// GPS Wiring
HardwareSerial& gpsSerial = Serial8; // GPS Connected to Tx/Rx 8 UART
bool gpsFlag = false; //Are we flying with GPS or not
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

// This is the Reference vector for EKF. Either user provided, or passed to from Sensor Calibrations, in which We aren't really pointed at True North or Mag north, and using a local frame instead that is rotated from Mag North.
//inline constexpr Vector3f magRef{11.4299 * PI/180}; 
// ========================= Motors =========================
// Pins are 28 29 37 36 (top to bottom), left to right
inline constexpr int esc1SignalPin = 28;
inline constexpr int esc2SignalPin = 37;
inline constexpr int esc3SignalPin = 29;
inline constexpr int esc4SignalPin = 36;

inline constexpr int maxPWM = 1950; //Recommended by Hobby Wing
inline constexpr int minPWM = 1150; //Recommended by Hobby Wing
inline constexpr int M1StartPWM = 1200; 
inline constexpr int M2StartPWM = 1200; 
inline constexpr int M3StartPWM = 1200; 
inline constexpr int M4StartPWM = 1200; 


// ========================= LOGGER / RINGBUFFER SIZE =========================
// Make these power 2 since ringbuffer modulus function is for bitwise
inline constexpr int imuRingBufferSize = 512; //Should hold last ~5 seconds of data
inline constexpr int magRingBufferSize = 256;
inline constexpr int altRingBufferSize = 128;
inline constexpr int gpsRingBufferSize = 8;  
inline constexpr int eskfStateRingBufferSize = 512;

// Should think about this...
// TODO:: In the future, might want to be able to keep a buffer of EVERY measurement / state in order to 
// back propagate if needed, In which case, it might be better to just extract every X-th measurement from that buffer based on the frequencies here
// Or maybe just have seperate buffers for less book keeping
// IF THESE CHANGE, MAKE SURE TO CHANGE SIZE OF STURCTURE INSIDE LOGGER FOR EACH LOGSENSOR
inline constexpr float logIMUDataFrequency {25}; //Hz. Logs every 4th IMU Sample
inline constexpr float logMagDataFrequency {25}; //Hz. Logs every 2nd IMU Sample
inline constexpr float logAltDataFrequency {25}; //Hz. Logs every Altmeter sample
inline constexpr float logGPSDataFrequency {1}; //Hz. Logs every GPS sample
inline constexpr float logGNCDataFrequency {10}; //Hz. 

inline constexpr float logFlushFrequency {2}; //Hz. This is the rate in which Data are written to the file and flushed


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

}

// Need to Upload, plug in battery, then open serial monitor. Cann't have battery already plugged. Do it all one by one for more consistent results
void SETUP_Calibrate_M1(Servo & motor1CW) {
  Serial.begin(9600); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  while (!Serial) {
    //Do nothing until serial monitor is opened
  };
  motor1CW.attach(SETUP::esc1SignalPin, SETUP::minPWM, SETUP::maxPWM);

  // For calibration, start motor HIGH, then low. This is opposite of flight commands which must start LOW

  // Calibrate Motor 1 
  Serial.println("Starting Motor 1 High");
  motor1CW.writeMicroseconds(SETUP::maxPWM); // Maximum throttle
  delay(3000); //Hold max for 5 seconds Until Beeps 
  Serial.println("Starting Motor 1 Low");
  motor1CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(3000); //Hold min for 5 seconds Until Beeps 

  Serial.println("Done");
  while(1) {
  };
}
void SETUP_Calibrate_M2(Servo & motor2CCW) {
  Serial.begin(9600); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  while (!Serial) {
    //Do nothing until serial monitor is opened
  };

  motor2CCW.attach(SETUP::esc2SignalPin, SETUP::minPWM, SETUP::maxPWM);


  // For calibration, start motor HIGH, then low. This is opposite of flight commands which must start LOW
  // Calibrate Motor 2
  Serial.println("Starting Motor 2 High");
  motor2CCW.writeMicroseconds(SETUP::maxPWM); // Maximum throttle
  delay(3000); ///Hold max for 5 seconds Until Beeps 
  Serial.println("Starting Motor 2 Low");
  motor2CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(3000); //Hold min for 5 seconds Until One long beep

  Serial.println("Done");
  while(1) {
  };
}
void SETUP_Calibrate_M3(Servo & motor3CW) {
  Serial.begin(9600); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  while (!Serial) {
    //Do nothing until serial monitor is opened
  };
  motor3CW.attach(SETUP::esc3SignalPin, SETUP::minPWM, SETUP::maxPWM);

  // For calibration, start motor HIGH, then low. This is opposite of flight commands which must start LOW

  // Calibrate Motor 3
  Serial.println("Starting Motor 3 High");
  motor3CW.writeMicroseconds(SETUP::maxPWM); // Maximum throttle
  delay(3000); //Hold max for 5 seconds Until Beeps 
  Serial.println("Starting Motor 3 Low");
  motor3CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(3000); //Hold min for 5 seconds Until Beeps 

  Serial.println("Done");
  while(1) {
  };
}
void SETUP_Calibrate_M4(Servo & motor4CCW) {
  Serial.begin(9600); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  while (!Serial) {
    //Do nothing until serial monitor is opened
  };

  motor4CCW.attach(SETUP::esc4SignalPin, SETUP::minPWM, SETUP::maxPWM);


  // For calibration, start motor HIGH, then low. This is opposite of flight commands which must start LOW
  // Calibrate Motor 2
  Serial.println("Starting Motor 4 High");
  motor4CCW.writeMicroseconds(SETUP::maxPWM); // Maximum throttle
  delay(3000); ///Hold max for 5 seconds Until Beeps 
  Serial.println("Starting Motor 4 Low");
  motor4CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(3000); //Hold min for 5 seconds Until One long beep

  Serial.println("Done");
  while(1) {
  };
}

void SETUP_Find_Motors_Start(Servo & motor1CW, Servo& motor2CCW, Servo& motor3CW, Servo& motor4CCW) {
  Serial.begin(9600); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  while (!Serial) {
    //Do nothing until serial monitor is opened
  };
  motor1CW.attach(SETUP::esc1SignalPin, SETUP::minPWM, SETUP::maxPWM);
  motor2CCW.attach(SETUP::esc2SignalPin, SETUP::minPWM, SETUP::maxPWM);
  motor3CW.attach(SETUP::esc3SignalPin, SETUP::minPWM, SETUP::maxPWM);
  motor4CCW.attach(SETUP::esc4SignalPin, SETUP::minPWM, SETUP::maxPWM);


  // Spin each motor slowly to observe Spin Direction. 
  // REMEMBER SPIN DIRECTION IN REFERENCE TO DOWN AXIS
  // SO IF VIEWING FROM TOP, CW --> CCW and CCW --> CW

  //Find the off-set needed after calibration
  Serial.println("Starting Motor 1 Low");
  motor1CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  Serial.println("Starting Motor 1 1150");
  motor1CW.writeMicroseconds(1150); 
  delay(2000); 
  Serial.println("Starting Motor 1 1175");
  motor1CW.writeMicroseconds(1175); 
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 1 1200");
  motor1CW.writeMicroseconds(1200); 
  delay(2000); 
  Serial.println("Starting Motor 1 Low");
  motor1CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps 

  // Find the off-set needed after calibration
  Serial.println("Starting Motor 2 Low");
  motor2CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 2 1150");
  motor2CCW.writeMicroseconds(1150); 
  delay(2000); 
  Serial.println("Starting Motor 2 1175");
  motor2CCW.writeMicroseconds(1175); 
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 2 1200");
  motor2CCW.writeMicroseconds(1200); 
  delay(2000); 
  Serial.println("Starting Motor 2 Low");
  motor2CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps   

  // Find the off-set needed after calibration
  Serial.println("Starting Motor 3 Low");
  motor3CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 3 1150");
  motor3CW.writeMicroseconds(1150); 
  delay(2000); 
  Serial.println("Starting Motor 3 1175");
  motor3CW.writeMicroseconds(1175); 
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 3 1200");
  motor3CW.writeMicroseconds(1200); 
  delay(2000); 
  Serial.println("Starting Motor 3 Low");
  motor3CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps   

  Serial.println("Starting Motor 4 Low");
  motor4CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 4 1150");
  motor4CCW.writeMicroseconds(1150); 
  delay(2000); 
  Serial.println("Starting Motor 4 1175");
  motor4CCW.writeMicroseconds(1175); 
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 4 1200");
  motor4CCW.writeMicroseconds(1200); 
  delay(2000); 
  Serial.println("Starting Motor 4 Low");
  motor4CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps   


  Serial.println("Done");
  while(1) {};
}

void SETUP_Find_Motor_Directions(Servo& motor1CW, Servo& motor2CCW, Servo& motor3CW, Servo& motor4CCW) {
  Serial.begin(9600); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  while (!Serial) {
    //Do nothing until serial monitor is opened
  };
  motor1CW.attach(SETUP::esc1SignalPin, SETUP::minPWM, SETUP::maxPWM);
  motor2CCW.attach(SETUP::esc2SignalPin, SETUP::minPWM, SETUP::maxPWM);
  motor3CW.attach(SETUP::esc3SignalPin, SETUP::minPWM, SETUP::maxPWM);
  motor4CCW.attach(SETUP::esc4SignalPin, SETUP::minPWM, SETUP::maxPWM);


  // Spin each motor slowly to observe Spin Direction. 
  // REMEMBER SPIN DIRECTION IN REFERENCE TO DOWN AXIS
  // SO IF VIEWING FROM TOP, CW --> CCW and CCW --> CW

  // Observe the Spin Direction of motors and make sure they are right
  Serial.println("Starting Motor 1 Low");
  motor1CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 1 Minimum PWM");
  motor1CW.writeMicroseconds(SETUP::M1StartPWM);
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 1 Low");
  motor1CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps 

  // Find the off-set needed after calibration
  Serial.println("Starting Motor 2 Low");
  motor2CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(5000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 2 1025");
  motor2CCW.writeMicroseconds(SETUP::M2StartPWM); 
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 2 Low");
  motor2CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps   

  // Find the off-set needed after calibration
  Serial.println("Starting Motor 3 Low");
  motor3CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(5000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 3 Start PWM");
  motor3CW.writeMicroseconds(SETUP::M3StartPWM); 
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 3 Low");
  motor3CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps   

  Serial.println("Starting Motor 4 Low");
  motor4CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(5000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 4 Start PWM");
  motor4CCW.writeMicroseconds(SETUP::M4StartPWM); 
  delay(2000); 
  Serial.println("Starting Motor 4 Low");
  motor4CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps   

  Serial.println("Done");
  while(1) {};
}

// Run to just check out all Sensors and make sure motors are connected properly
void SETUP_Test_Sensors_Motors(Sensors& sensors, Servo & motor1CW, Servo& motor2CCW, Servo& motor3CW, Servo& motor4CCW) {
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
  sensors.calibrateSensors(); // Get IMU Bias, checks GPS lock


  Serial.println("Done Test, Forever looping...");
  while (1) { //Eventually replace with error
    delay(10); //Infinite loop catches IMU not initized 
  };

  motor1CW.attach(SETUP::esc1SignalPin, SETUP::minPWM, SETUP::maxPWM);
  motor2CCW.attach(SETUP::esc2SignalPin, SETUP::minPWM, SETUP::maxPWM);
  motor3CW.attach(SETUP::esc3SignalPin, SETUP::minPWM, SETUP::maxPWM);
  motor4CCW.attach(SETUP::esc4SignalPin, SETUP::minPWM, SETUP::maxPWM);

  motor1CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  motor2CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  motor3CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  motor4CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle


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


// Mostly used for Allan Variance, but also save down magnetometer and altimeter data to take a look at noise 
void SETUP_AllanVariance(Sensors& sensors) { //Using millis() to prevent overflow from bits
  // Set up Time durations
  uint32_t imu_logger_flush_rate = 1000 * 60* 10; //10 minutes
  uint32_t imu_logger_duration =  CONSTANTS::seconds2milli * 5UL * 60UL * 60UL; // 3 Hours
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

  Serial.println("Starting IMU Logging...");
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
