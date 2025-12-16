//pio run -t upload
//pio device monitor --baud 115200
//===== Ardiuno / Standard =====
#include <Arduino.h> //Platformio doesn't insert this at compile time like Ardiuno does
#include <Wire.h> //Communicate w/ I2C devices
#include <SD.h> //SD card
#include <Servo.h> //Motor PWM command generation
//===== User / Custom =====
#include <Mathpk.h>
#include "FSM.h"
#include "Sensors.h"
#include "ESKF.h"
#include "SetUp.h"

#define SD_CS BUILTIN_SDCARD  // Teensy 4.1 internal SD slot

// ============================ Set up Containers for Global Variables ============================
//Init Flags (eventually, will switch this to states/ modes so that it can check the state.)
FSM finiteStateMachine;
FSM::State currentState;


Sensors sensors(SETUP::imuFrequency, SETUP::magFrequency, SETUP::altFrequency, SETUP::gpsFrequency, SETUP::gpsSerial); //GPS connected to port 8
bool gps_flag = false;

ESKF eskf(SETUP::p0, SETUP::v0, SETUP::q0, SETUP::ba0, SETUP::bg0, SETUP::bm0, 
          SETUP::P0, 
          SETUP::sig_acc, SETUP::sig_gyro, SETUP::eta_acc, SETUP::eta_gyro, SETUP::eta_mag,
          SETUP::sig_mag, SETUP::sig_tilt, SETUP::sig_alt, SETUP::sig_gps_pos, SETUP::sig_gps_vel);

Servo motor1CW; 
Servo motor2CCW; 
Servo motor3CW; 
Servo motor4CCW; 


//Measurements
std::array<float,6> imuMeas;
std::array<float,3> magMeas;
float altMeas;
std::array<float,4> gpsMeas;

//States
Vector3f p_k;
Vector3f v_k;
Quaternion q_k;
Vector3f w_k;

float initial_time; 
float current_time;
float loop_time_step = CONSTANTS::seconds2micro/SETUP::imuFrequency;
float loop_start_time; //Keeps track of time at the start of loop
float loop_end_time; // Keeps track of time at the end of loop for dt
float loop_delay_time; // this is loop_time_step - (loop_end_time - loop_start_time). THis is the amount needed to delay the loop to hit the desired loop frequency
float  logger_last_flush = 0;
// Open up logger files
File measurementHistoryFile = SD.open("meas_history.csv", FILE_WRITE); 
File stateHistoryFile = SD.open("state_history.csv", FILE_WRITE); 

// ============================ Shortcuts for various Setups [START] ============================
// Subroutines to run during void setup. Actual setup further down
void SETUP_Offboard_Calibration() {
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

void SETUP_Calibrate_Motors() {
  Serial.begin(9600); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  while (!Serial) {
    //Do nothing until serial monitor is opened
  };
  motor1CW.attach(SETUP::esc1SignalPin, SETUP::minPWM, SETUP::maxPWM);
  motor2CCW.attach(SETUP::esc2SignalPin, SETUP::minPWM, SETUP::maxPWM);
  motor3CW.attach(SETUP::esc3SignalPin, SETUP::minPWM, SETUP::maxPWM);
  motor4CCW.attach(SETUP::esc4SignalPin, SETUP::minPWM, SETUP::maxPWM);

  // For calibration, start motor HIGH, then low. This is opposite of flight commands which must start LOW

  // Calibrate Motor 1 
  Serial.println("Starting Motor 1 High");
  motor1CW.writeMicroseconds(SETUP::maxPWM); // Maximum throttle
  delay(5000); //Hold max for 5 seconds Until Beeps 
  Serial.println("Starting Motor 1 Low");
  motor1CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(5000); //Hold min for 5 seconds Until Beeps 

  // Calibrate Motor 2
  Serial.println("Starting Motor 2 High");
  motor2CCW.writeMicroseconds(SETUP::maxPWM); // Maximum throttle
  delay(5000); ///Hold max for 5 seconds Until Beeps 
  Serial.println("Starting Motor 2 Low");
  motor2CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(5000); //Hold min for 5 seconds Until One long beep

  // // Calibrate Motor 3 
  Serial.println("Starting Motor 3 High");
  motor3CW.writeMicroseconds(SETUP::maxPWM); // Maximum throttle
  delay(5000); //Hold max for 5 seconds
  Serial.println("Starting Motor 3 Low");
  motor3CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(5000); //Hold max for 5 seconds

  // // Calibrate Motor 4 
  Serial.println("Starting Motor 4 High");
  motor4CCW.writeMicroseconds(SETUP::maxPWM); // Maximum throttle
  delay(5000); //Hold max for 3 seconds
  Serial.println("Starting Motor 4 Low");
  motor4CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(5000); //Hold max for 5 seconds

}

void SETUP_Find_Motors_Start() {
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
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 1 1025");
  motor1CW.writeMicroseconds(1025); 
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 1 1050");
  motor1CW.writeMicroseconds(1050); 
  delay(2000); 
  Serial.println("Starting Motor 1 1075");
  motor1CW.writeMicroseconds(1075); 
  delay(2000); 
  Serial.println("Starting Motor 1 1100");
  motor1CW.writeMicroseconds(1100); 
  delay(2000); 
  Serial.println("Starting Motor 1 1125");
  motor1CW.writeMicroseconds(1125); 
  delay(2000); 
  Serial.println("Starting Motor 1 1150");
  motor1CW.writeMicroseconds(1150); 
  delay(2000); 
  Serial.println("Starting Motor 1 Low");
  motor1CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps 

  // Find the off-set needed after calibration
  Serial.println("Starting Motor 2 Low");
  motor2CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 2 1025");
  motor2CCW.writeMicroseconds(1025); 
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 2 1050");
  motor2CCW.writeMicroseconds(1050); 
  delay(2000); 
  Serial.println("Starting Motor 2 1075");
  motor2CCW.writeMicroseconds(1075); 
  delay(2000); 
  Serial.println("Starting Motor 23 1100");
  motor2CCW.writeMicroseconds(1100); 
  delay(2000); 
  Serial.println("Starting Motor 2 1125");
  motor2CCW.writeMicroseconds(1125); 
  delay(2000); 
  Serial.println("Starting Motor 2 1150");
  motor2CCW.writeMicroseconds(1150); 
  delay(2000); 
  Serial.println("Starting Motor 2 Low");
  motor2CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps   

  // Find the off-set needed after calibration
  Serial.println("Starting Motor 3 Low");
  motor3CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 3 1025");
  motor3CW.writeMicroseconds(1025); 
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 3 1050");
  motor3CW.writeMicroseconds(1050); 
  delay(2000); 
  Serial.println("Starting Motor 3 1075");
  motor3CW.writeMicroseconds(1075); 
  delay(2000); 
  Serial.println("Starting Motor 3 1100");
  motor3CW.writeMicroseconds(1100); 
  delay(2000); 
  Serial.println("Starting Motor 3 1125");
  motor3CW.writeMicroseconds(1125); 
  delay(2000); 
  Serial.println("Starting Motor 3 1150");
  motor3CW.writeMicroseconds(1150); 
  delay(2000); 
  Serial.println("Starting Motor 3 Low");
  motor3CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps   

  Serial.println("Starting Motor 4 Low");
  motor4CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 4 1025");
  motor4CCW.writeMicroseconds(1025); 
  delay(2000); //Hold min for 5 seconds Until Beeps 
  Serial.println("Starting Motor 4 1050");
  motor4CCW.writeMicroseconds(1050); 
  delay(2000); 
  Serial.println("Starting Motor 4 1075");
  motor4CCW.writeMicroseconds(1075); 
  delay(2000); 
  Serial.println("Starting Motor 4 1100");
  motor4CCW.writeMicroseconds(1100); 
  delay(2000); 
  Serial.println("Starting Motor 4 1125");
  motor4CCW.writeMicroseconds(1125); 
  delay(2000); 
  Serial.println("Starting Motor 4 1150");
  motor4CCW.writeMicroseconds(1150); 
  delay(2000); 
  Serial.println("Starting Motor 4 Low");
  motor4CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  delay(2000); //Hold min for 5 seconds Until Beeps   
}

void SETUP_Find_Motor_Directions() {
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
}



// Run to just check out all Sensors and make sure motors are connected properly
void SETUP_Test_Sensors_Motors() {
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
void SETUP_AllanVariance() { //Using millis() to prevent overflow from bits
  // Set up Time durations
  uint32_t imu_logger_flush_rate = 1000 * 60* 10; //10 minutes
  uint32_t imu_logger_duration =  CONSTANTS::seconds2milli * 3UL * 60UL * 60UL; // 3 Hours
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
  sensors.setGPSFlag(gps_flag); //Not running GPS
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
  while (time_elapsed < imu_logger_duration) {
    loop_start_time = millis();

    if (sensors.imuUpdate(loop_start_time)) {
      time_elapsed = loop_start_time - initial_time; 
      imuMeas = sensors.getIMUMeas();

        //Write to file
        imuAllanVarianceFile.print(time_elapsed/1000); //Convert milli seconds to seconds
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
  loop_time_step = CONSTANTS::seconds2micro / SETUP::magFrequency;
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
      imuAllanVarianceFile.print(",");
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


void SETUP_Preflight_Check() {
  Serial.begin(9600); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  while (!Serial) {
    //Do nothing until serial monitor is opened
  };

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

  Wire.begin(); //Initializes default I2C bus (SDA / SCL pins)
  pinMode(LED_BUILTIN, OUTPUT); //Configure built in LED 
  // ------------- Sensor Checkout and Setup -----------------
  // Determine if we want to run with GPS or not
  sensors.setGPSFlag(gps_flag);

  delay(500); //Delay a few milliseconds after sensor start-up. IMU has wonky first reading
  sensors.startUpSensors();

  Serial.println("Setting up Sensors...");
  sensors.setUpSensors(SETUP::magHardIron, SETUP::magSoftIron, SETUP::rotBody2IMU, SETUP::rotBody2Mag); //Also sets up frequencies of sensors / ODR [HARDCODED]

  // Run Calibrations
  digitalWrite(LED_BUILTIN, HIGH); // High to let us know its calibrating
  Serial.println("Calibrating Sensors...");

  sensors.calibrateSensors(); // Get IMU Bias, checks GPS lock

  // Pass Magnetometer Reference to ESKF. Either user defined, or from sensor calibration
  eskf.setMagRef(sensors.getMagRef());

  // for (int i=0; i<5; i++) { //Rapid blinks to tell us this is done

  //   digitalWrite(LED_BUILTIN, HIGH);
  //   delay(500);
  //   digitalWrite(LED_BUILTIN, LOW);
  //   delay(500);
  // }

  // // Start Motors
  // Serial.println("Setting up Motors ...");
  // motor1CW.attach(SETUP::esc1SignalPin, SETUP::minPWM, SETUP::maxPWM);
  // motor2CCW.attach(SETUP::esc2SignalPin, SETUP::minPWM, SETUP::maxPWM);
  // motor3CW.attach(SETUP::esc3SignalPin, SETUP::minPWM, SETUP::maxPWM);
  // motor4CCW.attach(SETUP::esc4SignalPin, SETUP::minPWM, SETUP::maxPWM);

  // motor1CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  // motor2CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  // motor3CW.writeMicroseconds(SETUP::minPWM); //Minimum throttle
  // motor4CCW.writeMicroseconds(SETUP::minPWM); //Minimum throttle

  // Set up Headers for logger files

  measurementHistoryFile.println("t,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,alt,gps_x, gps_y, gps_vx, gps_vy"); 
  stateHistoryFile.println("t,x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz"); 

  // Do some delay before the start

  //Use Built in LED to show that we are starting...
  for (int i=0; i<5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
  Serial.println("Starting ...");

  while(1) {};
  delay(2000);


  initial_time = micros(); //Set initial Time before starting.
}

// ============================ [END] ============================

// ============================ [Helper Functions]=====================
void saveStep(float t, const Vector3f& p, const Vector3f& v,const Quaternion& q, const Vector3f& w, const std::array<float,14> z, File stateFile, File measFile) {
  // Save down state
  stateFile.print(t);
  stateFile.print(",");
  stateFile.print(p[0],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
  stateFile.print(",");
  stateFile.print(p[1],6);
  stateFile.print(",");
  stateFile.print(p[2],6); 
  stateFile.print(",");
  stateFile.print(v[0],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
  stateFile.print(",");
  stateFile.print(v[1],6);
  stateFile.print(",");
  stateFile.print(v[2],6); 
  stateFile.print(",");
  stateFile.print(q.w(),6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
  stateFile.print(",");
  stateFile.print(q.x(),6);
  stateFile.print(",");
  stateFile.print(q.y(),6); 
  stateFile.print(",");
  stateFile.print(q.z(),6); 
  stateFile.print(",");
  stateFile.print(w[0],6);
  stateFile.print(",");
  stateFile.print(w[1],6); 
  stateFile.print(",");
  stateFile.println(w[2],6); 


  // Save down measurements
  measFile.print(t);
  measFile.print(",");
  measFile.print(z[0],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
  measFile.print(",");
  measFile.print(z[1],6);
  measFile.print(",");
  measFile.print(z[2],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
  measFile.print(",");
  measFile.print(z[3],6);
  measFile.print(",");
  measFile.print(z[4],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
  measFile.print(",");
  measFile.print(z[5],6);
  measFile.print(",");
  measFile.print(z[6],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
  measFile.print(",");
  measFile.print(z[7],6);
  measFile.print(",");
  measFile.print(z[8],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
  measFile.print(",");
  measFile.print(z[9],6);
  measFile.print(",");
  measFile.print(z[10],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
  measFile.print(",");
  measFile.print(z[11],6);
  measFile.print(",");
  measFile.println(z[12],6);

} 



// ============================ Actual Setup and Loop Calls ============================
void setup() {
  //SETUP_Test_Sensors_Motors();
  //SETUP_Calibrate_Motors();
  //SETUP_Find_Motors_Start();
  //SETUP_Find_Motor_Directions();
  //SETUP_Offboard_Calibration();
  //SETUP_AllanVariance();
  SETUP_Preflight_Check();
}

void loop() {
  loop_start_time = millis();
  // Get current time
  current_time = (loop_start_time - initial_time) / CONSTANTS::seconds2milli; 
  // // Check/Switch States

  //------ Sensors / Navigation------
  // IMU Loop
  if (sensors.imuUpdate(loop_start_time)) {
    imuMeas = sensors.processIMUMeas(sensors.getIMUMeas());
    eskf.predict(imuMeas, loop_start_time);

    // Update the tilt. Will do a check to see if accelerometer magnitude is small enough
    eskf.updateTiltMeas(std::array<float,3> {imuMeas[0], imuMeas[1], imuMeas[2]}); 

    //Push Meas to buffer / ram
  }

  // Magnetometer Loop
  if (sensors.magUpdate(loop_start_time)) {
    magMeas = sensors.processMagMeas(sensors.getMagMeas());
    //Tecnically should have a very tiny dt to predict between previous loop to this one, but that should be negliable. 
    eskf.updateMagMeas(magMeas);
    //Push Meas to buffer / ram
  }

  // Altimeter Loop
  if (sensors.altUpdate(loop_start_time)) {
    altMeas = sensors.processAltMeas(sensors.getAltMeas());
    //Tecnically should have a very tiny dt to predict between previous loop to this one, but that should be negliable. 
    eskf.updateAltMeas(altMeas);

    //Push Meas to buffer / ram
  }

  // GPS Loop
  if (gps_flag) {
    if (sensors.gpsUpdate(loop_start_time)) {
      gpsMeas = sensors.processGPSMeas(sensors.getGPSMeas(),-eskf.getPosition()[2]); //Get the best estimate of altitude, which is just our UP direction (negative of Z axis)
      //Tecnically should have a very tiny dt to predict between previous loop to this one, but that should be negliable. 
      eskf.updateGPSMeas(gpsMeas);

      //Push Meas to buffer / ram
    }
  }

  // Inject Error of the eskf if there were any updates (use a flag here)
  eskf.injectError(); // This updates the states

  //Get the states 
  p_k = eskf.getPosition();
  v_k = eskf.getVelocity();
  q_k = eskf.getQuaternion();
  w_k = eskf.getBodyRates(); 

  //Push state to buffer / ram

  // Serial.print(p_k[0],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
  // Serial.print(",");
  // Serial.print(p_k[1],6);
  // Serial.print(",");
  // Serial.print(p_k[2],6); 
  // Serial.print(",");
  // Serial.print(v_k[0],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
  // Serial.print(",");
  // Serial.print(v_k[1],6);
  // Serial.print(",");
  // Serial.print(v_k[2],6); 
  // Serial.print(",");
  // Serial.print(q_k.w(),6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
  // Serial.print(",");
  // Serial.print(q_k.x(),6);
  // Serial.print(",");
  // Serial.print(q_k.y(),6); 
  // Serial.print(",");
  // Serial.print(q_k.z(),6); 
  // Serial.print(",");
  // Serial.print(w_k[0],6);
  // Serial.print(",");
  // Serial.print(w_k[1],6); 
  // Serial.print(",");
  // Serial.println(w_k[2],6); 


  // Guidance


  // Control



  // Save down into SD card
  //saveStep(current_time, p_k, v_k, q_k, w_k, stateHistoryFile, measurementHistoryFile);

  // Force File to flush every 10Hz timesteps (for 50hz, this is about every 5 timesteps)
  if (loop_start_time - logger_last_flush >= 100000) { //100ms --> 10Hz
      logger_last_flush = loop_start_time;
      stateHistoryFile.flush();
      measurementHistoryFile.flush();
  }
}




