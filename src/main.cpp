//pio run -t upload
//pio device monitor --baud 115200
//===== Ardiuno / Standard =====
#include <Arduino.h> //Platformio doesn't insert this at compile time like Ardiuno does
#include <Wire.h> //Communicate w/ I2C devices
#include <SD.h> //SD card
#include <Servo.h> //Motor PWM command generation
//===== User / Custom =====
#include <Mathpk.h>
#include "FiniteStateMachine.h"
#include "Sensors.h"
#include "ErrorStateKalmanFilter.h"
#include "Controller.h"
#include "Motors.h"
#include "SetUp.h"
#include "Logger.h" //Includes RingBuffer and DataTypes

#define SD_CS BUILTIN_SDCARD  // Teensy 4.1 internal SD slot

// ============================ Set up Containers for Global Variables ============================
//Init Flags (eventually, will switch this to states/ modes so that it can check the state.)
//FiniteStateMachine fsm;

// ============================SENSORS============================
Sensors sensors(SETUP::imuFrequency, SETUP::magFrequency, SETUP::altFrequency, SETUP::gpsFrequency, SETUP::gpsSerial); //GPS connected to port 8

//============================ESKF============================
// ESKFBias eskf(SETUP::p0, SETUP::v0, SETUP::q0, SETUP::ba0, SETUP::bg0, SETUP::bm0, 
//             SETUP::P0, 
//             SETUP::sig_acc, SETUP::sig_gyro, SETUP::eta_acc, SETUP::eta_gyro, SETUP::eta_mag,
//             SETUP::sig_mag, SETUP::sig_tilt, SETUP::sig_alt, SETUP::sig_gps_pos, SETUP::sig_gps_vel);

ErrorStateKalmanFilter eskf(SETUP::p0, SETUP::v0, SETUP::q0,
                          SETUP::P0, 
                          SETUP::sig_acc, SETUP::sig_gyro, 
                          SETUP::sig_mag, SETUP::sig_tilt, SETUP::sig_alt, SETUP::sig_gps_pos, SETUP::sig_gps_vel, SETUP::nis_gating_flag);

//============================CONTROLLER============================
Controller controller(SETUP::controlFrequency, SETUP::pRef, SETUP::vRef, SETUP::qRef, SETUP::wRef, SETUP::uRef, SETUP::K, SETUP::horizontalControllerFlag, SETUP::verticalControllerFlag); //Initialize with a constant reference. Will be updated by Guidance if necessary in Loop
//============================MOTOR============================
Motors motors(SETUP::kT, SETUP::kM, SETUP::L,
            SETUP::esc1SignalPin, SETUP::esc2SignalPin, SETUP::esc3SignalPin, SETUP::esc4SignalPin,
            SETUP::M1StartPWM, SETUP::M2StartPWM, SETUP::M3StartPWM, SETUP::M4StartPWM, 
            SETUP::minPWM, SETUP::maxPWM, SETUP::saturationPWM, SETUP::maxSpinSquare); 

//============================FINITE STATE MACHINE============================
FiniteStateMachine fsm(SETUP::imuFrequency); //Defaults to BOOT Mode

//============================LOGGERS/BUFFERS============================
//Set up Ringbuffers
RingBuffer<imuData, SETUP::imuRingBufferSize> imuBuffer; 
RingBuffer<tiltData, SETUP::tiltRingBufferSize> tiltBuffer; 
RingBuffer<magData, SETUP::magRingBufferSize> magBuffer; 
RingBuffer<altData, SETUP::altRingBufferSize> altBuffer; 
RingBuffer<gpsData, SETUP::gpsRingBufferSize> gpsBuffer; 
RingBuffer<eskfStateData, SETUP::eskfStateRingBufferSize> eskfStateBuffer; 
RingBuffer<eskfCovarianceData, SETUP::eskfStateRingBufferSize> eskfCovarianceBuffer; 

//Set up Logger(Templated on size of buffers)
Logger<SETUP::imuRingBufferSize, 
      SETUP::magRingBufferSize, 
      SETUP::altRingBufferSize, 
      SETUP::gpsRingBufferSize, 
      SETUP::eskfStateRingBufferSize> logger(SETUP::logFlushFrequency,SETUP::logIMUDataFrequency, SETUP::logMagDataFrequency, SETUP::logAltDataFrequency, SETUP::logGPSDataFrequency, SETUP::logGNCDataFrequency,
                                        imuBuffer, tiltBuffer, magBuffer, altBuffer, gpsBuffer, eskfStateBuffer, eskfCovarianceBuffer);

//============================TIMING============================
uint32_t initial_time; //Get the time setup() finishes at.  [unit32_t because this calls on millis()]

uint32_t loop_start_time; //Keeps track of time at the start of loop when calling all functions [unit32_t because this calls on millis()]

float current_time; //This is what is saved for loggers when post processsing.  [float because this converts milli-seconds into seconds for logging]
//Debug
uint16_t lastPrint = 0;
float printFreq = 20.0;


int counter = 0;
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

  sensors.calibrateSensors(5); // Get IMU Bias, checks GPS lock

  // Pass Magnetometer Reference to ESKF. Either user defined, or from sensor calibration
  eskf.setMagRef(sensors.getMagRefForFilter());

  // for (int i=0; i<5; i++) { //Rapid blinks to tell us this is done

  //   digitalWrite(LED_BUILTIN, HIGH);
  //   delay(500);
  //   digitalWrite(LED_BUILTIN, LOW);
  //   delay(500);
  // }

  //motors.setUp();
  //motors.arm(); //Eventually move this to be the very last thing to occur after checking through
  // // Do some delay before the start



  logger.begin(); //Start logger
  initial_time = millis(); //Set initial Time before starting.


  fsm.preflightCheckStatus(millis(), true); //Transitions into Idle mode
}


// ============================ Actual Setup and Loop Calls ============================


void setup() {
  //Setup helper functions (mostly for calibration and testing)
  //SETUP::SETUP_Offboard_Calibration(sensors);
  //SETUP::SETUP_Calibrate_Motor_Individual(motors,1);
  //SETUP::SETUP_Calibrate_Motor_Individual(motors,2);
  //SETUP::SETUP_Calibrate_Motor_Individual(motors,3);
  //SETUP::SETUP_Calibrate_Motor_Individual(motors,4);
  
  //SETUP::SETUP_Find_Motors_Start(motors);
  //SETUP::SETUP_Find_Motor_Directions(motors);
  //SETUP::SETUP_Test_Sensors_Motors(sensors, motors);
  //SETUP::SETUP_AllanVariance(sensors);
  //SETUP::SETUP_Find_Motor_Max_Spin(motors);
  
  //SETUP::SETUP_Find_Thrust_Constant(motors,1573);
  preflightCheck();


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
    //Set up Data Samples for logging
    imuData imuSample (current_time, imuMeas); //Raw imu measurement isn't super helpful

    // Update State Estimate with Tilt if magnitude of acceleration is small enough
    tiltData tiltSample = eskf.updateTiltMeas(std::array<float,3> {imuMeas[0], imuMeas[1], imuMeas[2]});
    tiltSample.tagData(current_time); //Tag the current time here

    //Push to Buffer (Frequency determined by SETUP::logIMUDataFrequency)
    logger.logIMU(loop_start_time, imuSample, tiltSample); //For now, the buffer is ONLY used for logging so its logging at a slower frequency than sensor itself. Will revisit to buffer every measurement if buffer has other use (like back propagating for missed measurements)

  }

  //-------Magnetometer Loop (Frequency Determined by SETUP::magFrequency)--------
  if (sensors.magUpdate(loop_start_time)) {
    // Obtain RAW measurements
    std::array<float,3> magMeasRaw = sensors.getMagMeas();
    // Process Measurement
    std::array<float,3> magMeas = sensors.processMagMeas(magMeasRaw);
    // Update State Estimate with Magnetometer (tecnically should have a very tiny dt to predict between previous loop to this one, but that should be negliable)
    // if (counter == 0) {
    //   Serial.print(magMeas[0]); Serial.print(",");
    //   Serial.print(magMeas[1]); Serial.print(",");
    //   Serial.print(magMeas[2]); Serial.print(",");
    //   Vector3f testMagRef = sensors.getMagRefForFilter();
    //   Serial.print(testMagRef[0]); Serial.print(",");
    //   Serial.print(testMagRef[1]); Serial.print(",");
    //   Serial.print(testMagRef[2]); Serial.print(",");
    //   counter +=1;
    // };
    magData magSample = eskf.updateMagMeas(magMeas);
    magSample.tagData(current_time); //Tag the current time here

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
    altData altSample = eskf.updateAltMeas(altMeas);

    altSample.setPressureData(altMeasRaw); //Save down the pressure
    altSample.tagData(current_time); //Tag the current time here

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
      gpsData gpsSample = eskf.updateGPSMeas(gpsMeas);
      //Push Meas to buffer / ram
    }
  }
  //-------Update Estimated State (Not really via loops)--------
  eskf.injectError(); // Inject Error of the eskf if there were any updates (uses a flag internally here)
  // if (static_cast<float>(loop_start_time - lastPrint) >= CONSTANTS::seconds2milli/printFreq) {
  //   lastPrint = loop_start_time;
  //   eskf.printStates();
  // }

  //-------- Run Finite State Machine ---------
  fsm.update(loop_start_time, eskf.getPosition(), eskf.getVelocity(), eskf.getQuaternion(), eskf.getBodyRates());

  // GUID Runs here. FOr now, not really running anything


  // Control / Motor
  if (controller.updateControl(loop_start_time, eskf.getPosition(), eskf.getVelocity(), eskf.getQuaternion(), eskf.getBodyRates()) && fsm.getControlFlag()) {
    std::array<float,4> uCMD = controller.getControl();
    // Serial.print(uCMD[0]);
    // Serial.print(", ");
    // Serial.print(uCMD[1]);
    // Serial.print(", ");
    // Serial.print(uCMD[2]);
    // Serial.print(", ");
    // Serial.println(uCMD[3]);

    motors.commandControl(uCMD); //uCMD is in terms of spinrate squared, so motor just needs to map this to a PWM
    motors.printPWMCMD(); //Debugging

  }


  // if (!fsm.getMotorFlag()) { //Disarm motors
  //   motors.disarm();
  // }

  // Save down buffer for guidance control and navigation all at the (Frequency determined by SETUP::logGNCDataFrequency)
  // logger.logGNC(loop_start_time, current_time, 
  //              eskf.getPosition(), eskf.getVelocity(), eskf.getQuaternion(), eskf.getBodyRates(), eskf.getAccelBias(), eskf.getGyroBias(), eskf.getMagBias(), eskf.getCovariance().getDiagonal());
  logger.logGNC(loop_start_time, current_time, 
               eskf.getPosition(), eskf.getVelocity(), eskf.getQuaternion(), eskf.getBodyRates(), eskf.getCovariance().getDiagonal());

  // Flush log (Frequency determined by SETUP::logFlushFrequency)
  logger.flush(loop_start_time);
}




