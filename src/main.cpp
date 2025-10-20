//===== Ardiuno / Standard =====
#include <Arduino.h> //Platformio doesn't insert this at compile time like Ardiuno does
#include <Wire.h> //Communicate w/ I2C devices
#include <SD.h> //SD card
//===== User / Custom =====
#include <Mathpk.h>
#include "FSM.h"
#include "Sensors.h"


//pio run -t upload
//pio device monitor --baud 115200
//Init Sensors

#define SD_CS BUILTIN_SDCARD  // Teensy 4.1 internal SD slot

//Init Flags (eventually, will switch this to states/ modes so that it can check the state.)
FSM finiteStateMachine;
FSM::State currentState;

//Init sensors
float imuFrequency {100}; //Hz
float magFrequency {50}; //Hz
float altFrequency {50}; //Hz
float gpsFrequency {1}; //Hz
Vector3f magHardIron(0.0, 0.0, 0.0);
Rotation magSoftIron(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
Rotation rotIMU2Body(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
Rotation rotMag2Body(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
Rotation rotMag2TrueNED(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
Sensors sensors(imuFrequency, magFrequency, altFrequency, gpsFrequency, Serial6); //GPS connected to port 6
bool calibrateMag = true;



void setup() {
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

  delay(1000);

  // ------------- Sensor Checkout and Setup -----------------
  sensors.startUpSensors();
  sensors.setUpSensors(magHardIron, magSoftIron,rotMag2TrueNED, rotIMU2Body, rotMag2Body); //Also sets up frequencies of sensors / ODR [HARDCODED]

  // --- Calibrate Sensors ---
  delay(100); //Delay a few milliseconds after sensor start-up. IMU has wonky first reading
  //sensors.calibrateMagnetometer();
  //Need a GPS LOCK to do the calibration for it..
  
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
  //Set up Low pass filters. Can also do this on software. Remember, nyquist frequency is ODR / 2, setting cutoff frequency above this does nothing. 
  //icm20x_accel_cutoff_t cutoff_frequency = icm20x_accel_cutoff_t::ICM20X_ACCEL_FREQ_50_4_HZ;
  //enableAccelDLPF(1, )

  // Arming Check
  // Within here, should check that all calibration went okay, i.e. we have values for biases, references, etc for sensors
  //Check if navigation has initial states.
  // Each class should have a bool that can quickly be reference that we are ready to start. sensors will be calibratebool
}

void loop() {

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
