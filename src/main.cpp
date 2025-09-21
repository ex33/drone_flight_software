//===== Ardiuno / Standard =====
#include <Arduino.h> //Platformio doesn't insert this at compile time like Ardiuno does
#include <Wire.h> //Communicate w/ I2C devices
#include <SD.h> //SD card
//===== User / Custom =====
#include "Mathpk.h"
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
double imuFrequency {100}; //Hz
double magFrequency {50}; //Hz
double altFrequency {50}; //Hz
double gpsFrequency {1}; //Hz
std::array<double,3> magHardIron{0.0, 0.0, 0.0};
std::array<double,9> magSoftIron{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
Sensors sensors(imuFrequency, magFrequency, altFrequency, gpsFrequency, Serial6); //GPS connected to port 6
bool calibrateMag = true;

void setup() {
  Serial.begin(115200); //Init. serial communication between Teensy and Computer. Only need this for debugging. 
  Serial6.begin(9600); //Init serial communication between Teensy and GPS, should match up with GPS
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


  // ------------- Sensor Checkout and Setup -----------------
  // ---Set up IMU ----
  sensors.setIMUDataRange(icm20649_accel_range_t::ICM20649_ACCEL_RANGE_16_G, icm20649_gyro_range_t::ICM20649_GYRO_RANGE_2000_DPS);
  sensors.setIMUUpdateRate(10); //102.3 Hz

  // --- Set up Magnetometer  ---
  sensors.setMagUpdateRate(lis2mdl_rate_t::LIS2MDL_RATE_50_HZ); 
  sensors.setMagCalibration(magHardIron, magSoftIron);

  // --- Set up Altimeter ---
  sensors.setAltPressureOversampling(BMP3_OVERSAMPLING_8X); //Datasheet recommended for drones to oversample 8x
  sensors.setAltUpdateRate(BMP3_ODR_50_HZ); //Datasheet recommended for drones to have ODR at 50Hz
  sensors.setAltFilterCoefficent(BMP3_IIR_FILTER_COEFF_3); //Datasheet recommended for drones to have IIR filter bit be 2 (Coeff 3, see table 4.3.21)
  
  // --- Set up GPS ---
  sensors.setGPSNMEAFormat(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Formats to output RMC (lat,long,SOG,COG) + GGA (Altitude, sat id, etc)
  sensors.setGPSNMEAUpdateRate(PMTK_SET_NMEA_UPDATE_1HZ); //1 Hz
  sensors.setGPSPositionUpdateRate(PMTK_API_SET_FIX_CTL_1HZ); //1Hz
  sensors.setGPSBaudRate(PMTK_SET_BAUD_9600); //9600 Bits per second limit. Should match with above 
  // --- Checkout all sensor ---
  sensors.checkoutSensors();


  // --- Calibrate Sensors ---
  delay(100); //Delay a few milliseconds after sensor start-up. IMU has wonky first reading
  //sensors.calibrateMagnetometer();
  sensors.calibrateSensors(); // Get IMU Bias, checks GPS lock

  //Test sensor measurements
  

  sensors.getMeasurements();

  // for (unsigned int i = 0; i < z.size(); i++) {
  //   Serial.print(z[i]);
  //   Serial.print(" ");
  // }

  

  //Set up Low pass filters. Can also do this on software. Remember, nyquist frequency is ODR / 2, setting cutoff frequency above this does nothing. 
  //icm20x_accel_cutoff_t cutoff_frequency = icm20x_accel_cutoff_t::ICM20X_ACCEL_FREQ_50_4_HZ;
  //enableAccelDLPF(1, )

  // Arming Check

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
