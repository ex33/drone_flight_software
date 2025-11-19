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
//pio run -t upload
//pio device monitor --baud 115200
//Init Sensors

#define SD_CS BUILTIN_SDCARD  // Teensy 4.1 internal SD slot

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

  motor1CW.attach(SETUP::esc1SignalPin, SETUP::minPulseWidth, SETP::maxPulseWidth);
  motor2CW.attach(SETUP::esc2SignalPin, SETUP::minPulseWidth, SETP::maxPulseWidth);
  motor3CW.attach(SETUP::esc3SignalPin, SETUP::minPulseWidth, SETP::maxPulseWidth);
  motor4CW.attach(SETUP::esc4SignalPin, SETUP::minPulseWidth, SETP::maxPulseWidth);

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






void setup() {
  Serial.begin(115200); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  while (!Serial) {
    //Do nothing until serial monitor is opened
  };
  SETUP_Test_Sensors_Motors();

  // // Set up Motor
  //   esc1.attach(esc1Pin, 1000, 2000);
  //   esc1.writeMicroseconds(1000); // Minimum throttle
  //   delay(3000); // Wait to arm


  // Arming Check
  // Within here, should check that all calibration went okay, i.e. we have values for biases, references, etc for sensors
  //Check if navigation has initial states.
  // Each class should have a bool that can quickly be reference that we are ready to start. sensors will be calibratebool
}

void loop() {
  // // Slowly ramp up to spin
  // for (int us = 1000; us <= 1500; us += 10) {
  //   esc1.writeMicroseconds(us);
  //   delay(50);
  // }
  // delay(2000);

  // // Slowly ramp down
  // for (int us = 1500; us >= 1000; us -= 10) {
  //   esc1.writeMicroseconds(us);
  //   delay(50);
  // }
  // delay(2000);
  // Check Which State 
  //finiteStateMachine.update(sensors);
  //currentState = finiteStateMachine.getState();

  // Sensors 
  //z = run_sensors(curentState, imu);


  // Initialize Adafruit sensors_event_t (common struct between data from sensors)
  // Re-initialize to ensure nothing is left over from previous loop
  //sensors_event_t imu_a, imu_g, imu_temp;

  // // Check/Switch States


  //------ Sensors ------

  // Navigation


  // Guidance


  // Control



  // Save down into SD card
  // Can save ~50 samples, so basically wrtite to SD card every ~0.5 seconds.

}




