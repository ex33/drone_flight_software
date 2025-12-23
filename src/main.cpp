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
#include "Logger.h" //Includes RingBuffer and DataTypes

#define SD_CS BUILTIN_SDCARD  // Teensy 4.1 internal SD slot

// ============================ Set up Containers for Global Variables ============================
//Init Flags (eventually, will switch this to states/ modes so that it can check the state.)
//FiniteStateMachine fsm;

// ============================SENSORS============================
Sensors sensors(SETUP::imuFrequency, SETUP::magFrequency, SETUP::altFrequency, SETUP::gpsFrequency, SETUP::gpsSerial); //GPS connected to port 8

//============================ESKF============================
ESKF eskf(SETUP::p0, SETUP::v0, SETUP::q0, SETUP::ba0, SETUP::bg0, SETUP::bm0, 
          SETUP::P0, 
          SETUP::sig_acc, SETUP::sig_gyro, SETUP::eta_acc, SETUP::eta_gyro, SETUP::eta_mag,
          SETUP::sig_mag, SETUP::sig_tilt, SETUP::sig_alt, SETUP::sig_gps_pos, SETUP::sig_gps_vel);

//States from ESKF (keep global)
Vector3f p_k;
Vector3f v_k;
Quaternion q_k;
Vector3f w_k;


//============================MOTOR============================
//Eventually move these into MOTOR class
Servo motor1CW; 
Servo motor2CCW; 
Servo motor3CW; 
Servo motor4CCW; 


//============================LOGGERS/BUFFERS============================
//Set up Ringbuffers
RingBuffer<imuData, SETUP::imuRingBufferSize> imuBuffer; 
RingBuffer<magData, SETUP::magRingBufferSize> magBuffer; 
RingBuffer<altData, SETUP::altRingBufferSize> altBuffer; 
RingBuffer<gpsData, SETUP::gpsRingBufferSize> gpsBuffer; 
RingBuffer<eskfStateData, SETUP::eskfStateRingBufferSize> eskfStateBuffer; 


//Set up Logger(Templated on size of buffers)
Logger<SETUP::imuRingBufferSize, 
      SETUP::magRingBufferSize, 
      SETUP::altRingBufferSize, 
      SETUP::gpsRingBufferSize, 
      SETUP::eskfStateRingBufferSize> logger(SETUP::logFlushFrequency,SETUP::logIMUDataFrequency, SETUP::logMagDataFrequency, SETUP::logAltDataFrequency, SETUP::logGPSDataFrequency, SETUP::logGNCDataFrequency,
                                        imuBuffer, magBuffer, altBuffer, gpsBuffer, eskfStateBuffer);

//============================TIMING============================
float initial_time; //Get the time setup() finishes at. For logging so we are saving in SECONDS, which should take up less bytes
float current_time; //This is what is saved for loggers when post processsing. 
float loop_start_time; //Keeps track of time at the start of loop when calling all functions


void preflightCheck() {
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
  sensors.setGPSFlag(SETUP::gpsFlag);

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


  // // Do some delay before the start

  // //Use Built in LED to show that we are starting...
  // for (int i=0; i<5; i++) {
  //   digitalWrite(LED_BUILTIN, HIGH);
  //   delay(1000);
  //   digitalWrite(LED_BUILTIN, LOW);
  //   delay(1000);
  // }
  Serial.println("Starting ...");

  // // while(1) {};
  // // delay(2000);

  logger.begin(); //Start logger
  initial_time = millis(); //Set initial Time before starting.
}


// ============================ Actual Setup and Loop Calls ============================





void setup() {
  //Setup helper functions (mostly for calibration and testing)
  //SETUP::SETUP_Offboard_Calibration(sensors);
  //SETUP::SETUP_Calibrate_M1(motor1CW);
  //SETUP::SETUP_Calibrate_M2(motor2CCW);
  //SETUP::SETUP_Calibrate_M3(motor3CW);
  //SETUP::SETUP_Calibrate_M4(motor4CCW);
  
  //SETUP::SETUP_Find_Motors_Start(motor1CW, motor2CCW, motor3CW, motor4CCW);
  //SETUP::SETUP_Find_Motor_Directions(motor1CW, motor2CCW, motor3CW, motor4CCW);
  //SETUP::SETUP_Test_Sensors_Motors(sensors, motor1CW, motor2CCW, motor3CW, motor4CCW);
  SETUP::SETUP_AllanVariance(sensors);
  
  //preflightCheck();
}

void loop() {
  loop_start_time = millis();
  current_time = (loop_start_time - initial_time) / CONSTANTS::seconds2milli; 
  // // Check/Switch States
  //Flush on state changes, disarm, usb disconnect, error, etcs


  //TODO:: Consider making sensors and eskf compatable with datatypes instead, so passing around info is more seamless
  //======= Run Sensors / Navigation ======
  //-------IMU Loop (Frequency Determined by SETUP::imuFrequency)--------
  if (sensors.imuUpdate(loop_start_time)) {
    // Obtain RAW measurements
    std::array<float,6> imuMeasRaw = sensors.getIMUMeas();
    // Process Measurement
    std::array<float,6> imuMeas = sensors.processIMUMeas(imuMeasRaw);
    //Predict forward state using IMU
    eskf.predict(imuMeas, loop_start_time);
    // Update State Estimate with Tilt if magnitude of acceleration is small enough
    eskf.updateTiltMeas(std::array<float,3> {imuMeas[0], imuMeas[1], imuMeas[2]}); 
    //Set up Data Samples for logging
    imuData imuSample (current_time, imuMeas); //Raw imu measurement isn't super helpful
    //Push to Buffer (Frequency determined by SETUP::logIMUDataFrequency)
    logger.logIMU(loop_start_time, imuSample); //For now, the buffer is ONLY used for logging so its logging at a slower frequency than sensor itself. Will revisit to buffer every measurement if buffer has other use (like back propagating for missed measurements)
  }

  //-------Magnetometer Loop (Frequency Determined by SETUP::magFrequency)--------
  if (sensors.magUpdate(loop_start_time)) {
    // Obtain RAW measurements
    std::array<float,3> magMeasRaw = sensors.getMagMeas();
    // Process Measurement
    std::array<float,3> magMeas = sensors.processMagMeas(magMeasRaw);
    // Update State Estimate with Magnetometer (tecnically should have a very tiny dt to predict between previous loop to this one, but that should be negliable)
    eskf.updateMagMeas(magMeas);
    //Set up Data Samples for logging
    magData magSample (current_time, magMeas); // For mag, raw measurement isn't super helpful
    //Push to Buffer (Frequency determined by SETUP::logMagDataFrequency)
    logger.logMag(loop_start_time, magSample); //For now, the buffer is ONLY used for logging so its logging at a slower frequency than sensor itself. Will revisit to buffer every measurement if buffer has other use (like back propagating for missed measurements)
  }

  //-------Altimeter Loop (Frequency Determined by SETUP::altFrequency)--------
  if (sensors.altUpdate(loop_start_time)) {
    // Obtain RAW measurements
    float altMeasRaw = sensors.getAltMeas();
    // Process Measurement
    float altMeas = sensors.processAltMeas(altMeasRaw);
    // Update State Estimate with Altimeter (tecnically should have a very tiny dt to predict between previous loop to this one, but that should be negliable)
    eskf.updateAltMeas(altMeas);
    //Set up Data Samples for logging
    altData altSample (current_time, altMeasRaw, altMeas); 
    //Push to Buffer (Frequency determined by SETUP::logAltDataFrequency)
    logger.logAlt(loop_start_time, altSample); //For now, the buffer is ONLY used for logging so its logging at a slower frequency than sensor itself. Will revisit to buffer every measurement if buffer has other use (like back propagating for missed measurements)
  }

  //-------GPS Loop (Frequency Determined by SETUP::gpsFrequency)--------
  // TODO:: Comeback once gps is set up
  if (SETUP::gpsFlag) {
    if (sensors.gpsUpdate(loop_start_time)) {
      std::array<double,5> gpsMeasRaw = sensors.getGPSMeas();
      std::array<float,4> gpsMeas = sensors.processGPSMeas(gpsMeasRaw,-eskf.getPosition()[2]); //Get the best estimate of altitude, which is just our UP direction (negative of Z axis)
      //Tecnically should have a very tiny dt to predict between previous loop to this one, but that should be negliable. 
      eskf.updateGPSMeas(gpsMeas);
      //Push Meas to buffer / ram
      gpsData gpsSample; 

    }
  }
  //-------Update Estimated State (Not really via loops)--------
  eskf.injectError(); // Inject Error of the eskf if there were any updates (use a flag here)


  // Control
  // Call the state directly from the filter inside this loop (like how logger is doing)
  // That way, each time the state is needed, it'll be the most up to date without having to form a seperate nav loop to pass information. 
  // Even with FSM, it can extract the state itself.
  



  // Save down buffer for guidance control and navigation all at the (Frequency determined by SETUP::logGNCDataFrequency)
  logger.logGNC(loop_start_time, current_time, 
                eskf.getPosition(), eskf.getVelocity(), eskf.getQuaternion(), eskf.getBodyRates(), eskf.getAccelBias(), eskf.getGyroBias(), eskf.getMagBias());


  // Flush log (Frequency determined by SETUP::logFlushFrequency)
  logger.flush(loop_start_time);
}




