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
#include "Setup.h"

#define SD_CS BUILTIN_SDCARD  // Teensy 4.1 internal SD slot

// ============================ Set up Containers for Global Variables ============================
//Init Flags (eventually, will switch this to states/ modes so that it can check the state.)
FSM finiteStateMachine;
FSM::State currentState;


Sensors sensors(SETUP::imuFrequency, SETUP::magFrequency, SETUP::altFrequency, SETUP::gpsFrequency, SETUP::gpsSerial); //GPS connected to port 8

ESKF eskf(SETUP::p0, SETUP::v0, SETUP::q0, SETUP::ba0, SETUP::bg0, SETUP::bm0, 
          SETUP::P0, SETUP::dt,
          SETUP::sig_acc, SETUP::sig_gyro, SETUP::eta_acc, SETUP::eta_gyro, SETUP::eta_mag,
          SETUP::sig_mag, SETUP::sig_tilt, SETUP::sig_alt, SETUP::sig_gps_pos, SETUP::sig_gps_vel);

Servo motor1CW; 
Servo motor2CCW; 
Servo motor3CW; 
Servo motor4CCW; 

// ============================ Shortcuts for various Setups [START] ============================
// Subroutines to run during void setup. Actual setup further down
void SETUP_Off_Board_Calibration() {
  Serial.begin(115200); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  while (!Serial) {
    //Do nothing until serial monitor is opened
  };
  Wire.begin(); //Initializes default I2C bus.

  // Start the the SD Card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card not found");
    while (1); 
  } else {
      Serial.println("SD card found");
  }

  // --- Calibrate Sensors ---
  sensors.calibrateMagnetometer();



  // ------------- Sensor Checkout and Setup -----------------
  sensors.startUpSensors();
  sensors.setUpSensors(SETUP::magHardIron, SETUP::magSoftIron,SETUP::rotMag2TrueNED, SETUP::rotIMU2Body, SETUP::rotMag2Body); //Also sets up frequencies of sensors / ODR [HARDCODED]
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

  // ------------- Sensor Checkout and Setup -----------------
  sensors.startUpSensors();
  sensors.setUpSensors(SETUP::magHardIron, SETUP::magSoftIron,SETUP::rotMag2TrueNED, SETUP::rotIMU2Body, SETUP::rotMag2Body); //Also sets up frequencies of sensors / ODR [HARDCODED]

  // Run Calibrations
  sensors.calibrateSensors(); // Get IMU Bias, checks GPS lock

  //Test sensor measurements
  sensors.updateMeasurements();
  sensors.printMeasurements();

  //Get the Bias as part of the initial state for the filter
  Vector3f startUpBiasAccel = sensors.getAccelBias();
  Vector3f startUpBiasGyro = sensors.getGyroBias();

  Serial.println("Done Test, Forever looping...");
  while (1) { //Eventually replace with error
    delay(10); //Infinite loop catches IMU not initized 
  };

  motor1CW.attach(SETUP::esc1SignalPin, SETUP::minPWM, SETUP::maxPWM);
  motor2CCW.attach(SETUP::esc2SignalPin, SETUP::minPWM, SETUP::maxPWM);
  motor3CW.attach(SETUP::esc3SignalPin, SETUP::minPWM, SETUP::maxPWM);
  motor4CCW.attach(SETUP::esc4SignalPin, SETUP::minPWM, SETUP::maxPWM);

};

//Need to re-write to save down to SD card
void SETUP_AllanVariance() {
    // if (allan_variance) {
  //   unsigned long now = micros();
  //   if (now - prevTime >= sampleInterval) {
  //     prevTime += sampleInterval; //Helps slightly prevent drift from using delay(10)

  //     imu.getEvent(&imu_a, &imu_g, &imu_temp);

  //     // print CSV line
  //     Serial.print(now); Serial.print(",");
  //     Serial.print(imu_a.acceleration.x,6); Serial.print(",");
  //     Serial.print(imu_a.acceleration.y,6); Serial.print(",");
  //     Serial.print(imu_a.acceleration.z,6); Serial.print(",");
  //     Serial.print(imu_g.gyro.x,6); Serial.print(",");
  //     Serial.print(imu_g.gyro.y,6); Serial.print(",");
  //     Serial.println(imu_g.gyro.z,6);
  //     //Run pio device monitor --baud 115200 --quiet > allan_variance_imu_data.csv
  //   };
  // };

    // Flags
  //allan_variance = 1; // Mode to Test  

  //prevTime = micros();

  //Serial.print(1e6/100,10);

//   bool allan_variance;
// unsigned long prevTime = 0;
// const unsigned long sampleInterval = 10000; //100 micro-s is ~100Hz (0.01s)
}


void SETUP_Preflight_Check() {
  Wire.begin(); //Initializes default I2C bus (SDA / SCL pins)

  // ------------- Sensor Checkout and Setup -----------------
  delay(100); //Delay a few milliseconds after sensor start-up. IMU has wonky first reading
  sensors.startUpSensors();
  sensors.setUpSensors(SETUP::magHardIron, SETUP::magSoftIron,SETUP::rotMag2TrueNED, SETUP::rotIMU2Body, SETUP::rotMag2Body); //Also sets up frequencies of sensors / ODR [HARDCODED]

  //Get the Bias as part of the initial state for the filter
  Vector3f startUpBiasAccel = sensors.getAccelBias();
  Vector3f startUpBiasGyro = sensors.getGyroBias();
}

// ============================ [END] ============================



// ============================ Actual Setup and Loop Calls ============================
void setup() {
  //SETUP_Test_Sensors_Motors();
  //SETUP_Calibrate_Motors();
  //SETUP_Find_Motors_Start();
  SETUP_Find_Motor_Directions();

}

void loop() {

  // Sensors 
  //z = run_sensors(curentState, imu);

  // // Check/Switch States


  //------ Sensors ------

  // Navigation


  // Guidance


  // Control



  // Save down into SD card
  // Can save ~50 samples, so basically wrtite to SD card every ~0.5 seconds.

}




