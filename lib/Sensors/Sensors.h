// Really a class wrapper for all Adafruit sensors. 
// Will initialize all sensors 
#ifndef _SENSORS_H
#define _SENSORS_H



#include <SD.h>

//===== Adafruit =====
#include <Adafruit_Sensor.h>
// IMU
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
// Magnetometer
#include <Adafruit_LIS2MDL.h>
// Altimeter
#include <Adafruit_BMP3XX.h>
// GPS
#include <Adafruit_GPS.h>

#include <Mathpk.h>
#include <Constants.h>

class Sensors {
public:
    // // Conversions
    // static constexpr float deg2rad = PI/180;

    // // Time constants
    // static const unsigned long seconds = 1000000UL;
    // // GPS Constants
    // static constexpr float WGS84_a = 6378137; // m, semi-major axis of ellpsiod
    // static constexpr float WGS84_e2 = 0.006694379990140; // n.d., eccentricity^2 of ellipsoid
    // // Altimeter Constants
    // static constexpr float seaLevelPressure = 1013.25; //Hardcode this for altimeter [hPa]
    // // IMU Constants
    // static constexpr float g0 = 9.80665; //m/s^2
    enum class SensorErrors{
        InitError
    };

    /**
     * @brief Init Sensors object. Make sure when setting the ranges, it matches with this. 
     * There are no checks to ensure they match.
     * 
     * @param freqIMU Frequency of IMU (Hz)
     * @param freqMag Frequency of Magnetometer (Hz)
     * @param freqAlt Frequency of Altimeter (Hz)
     * @param freqGPS Frequency of GPS (Hz)
     * @param serialGPS Serial wire of GPS
     *
     * This function configures all the frequency in which a sensor will be checked to see if 
     * there is a measurement. If there is a mismatch between this value and the actual output rate, 
     * there may be the risk of returning the same measurement due to the pin not being updated.
     */
    Sensors(float freqIMU, float freqMag, float freqAlt, float freqGPS, HardwareSerial &serialGPS); 

    // -------------------------- All Sensors Functions ------------------------------
    /**
    * @brief Updates ALL RAW measurements at the current time.
    * 
    * Goes through each sensor and calls their respective getMeas functions.
    * Updates members that contains the current measurements for the sensors.
    */
    void updateMeasurements();

    /**
    * @brief Return ALL sensor readings at the current time for NAV Filter.
    * 
    * Need to call updateMeasurements before this to ensure all measurements are up to date.
    * Will perform any calibration and rotations needed here before giving this to the NAV Filter. 
    */
    std::array<float,14> getMeasurements();

    /**
    * @brief Print ALL sensor readings at the current time.
    * 
    * Print All measurements that is currently part of the parameters
    */
    void printMeasurements();

    /** 
    * @brief Start up all sensors to ensure sensors are operational
    * 
    * Initialize all sensors, ensures they are wired properly for I2C and checks reponses for UART.
    * Set all starting rates, operational modes, etc.
    * Hardcode all sensor orientations
     */
    void startUpSensors();

    /** 
    * @brief Start up all sensors to their desired operational modes
    * 
    * Set all starting rates, operational modes, etc.
    * Expect array inputs so they can be inside SetUp.h as const expr
     */
    void setUpSensors(std::array<float,3> magHardIronArray, std::array<float,9> magSoftIronArray, std::array<float,9> magNE2TrueNEArray,
                            std::array<float,9> rotBody2IMUArray, std::array<float,9> rotBody2MagArray);

    /**
    * @brief Final Checkup on Health of sensors
    * 
    * Can vary from temperature, to steady stream of measurements, to gps fixes
     */
    void finalCheckOut();
    /**
    * @brief Static Calibration for all sensors beside Magnetometer
    * 
    * Statically collect data for IMU / Altimeter / GPS 
    * Calculate bias / ref height / ref Lat and Long
    * Checks for GPS lock.
     */
    void calibrateSensors();

    /**
    * @brief Checks that all sensor parameters have been set properly. Runs checkoutSensors one last time.
    * 
    * @return Boolean if sensors parameters set properly and sensors are healthy.
     */
    bool finalArmingCheck();

    // -------------------------- IMU Functions ------------------------------
    /**
    * @brief Sets the accelerometer and gyroscope measurement range
    * 
    * @param accelRange The accelerometer range (enum: icm20649_accel_range_t)
    * @param gyroRange The gyroscope range (enum: icm20649_gyro_range_t)
    *
    * This function configures the IMU member (imu_) with the
    * requested measurement ranges.
    */
    void setIMUDataRange(icm20649_accel_range_t accelRange,icm20649_gyro_range_t gyroRange);

    /**
    * @brief Sets the accelerometer and gyroscope output divisor rate (ODR)
    * 
    * @param imuRateDivisor The accelerometer/gyroscope ODR divisor
    * 
    * Internal ODR is 1125Hz, which is the rate the register gets an update. 
    * Set divisor to lower this: Effective ODR = ODR / (1 + rate_divisor)
     */
    void setIMUUpdateRate(unsigned int imuRateDivisor);

    /**
    * @brief Sets the accelerometer and gyroscope bias
    * 
    * @param imuDataSum Summation of IMU data collected over a period of time
    * @param numIMUMeas Number of IMU data collected to calculate sum
    * 
    * Collect X number of IMU data and sum them up. Average to get the bias.
     */
    void setIMUCalibration(std::array<float,6>& imuDataSum, int numIMUMeas);

    /**
    * @brief Returns Calibrated Start-up bias for the accelerometer 
    * 
    * @return Accelerometer Bias in Body frame
    */
    Vector3f getAccelBias();

    /**
    * @brief Returns Calibrated Start-up bias for the gyro
    * 
    * @return Gyro Bias in Body frame
    */
    Vector3f getGyroBias();

    /**
    * @brief Returns accelerometer/gyroscope measurement if the time since the last reading is greater 
    * than the frequency set at initialization.
    * 
    * @param now Current time in microseconds
    * 
    * @return Checks register and returns measurement if (time_now - time_since_last_measurement) > sensor_frequency. 
    * Else, assume register has the same value and return NaN array
    */
    std::array<float,6> getIMUMeas(unsigned long now);

    // -------------------------- Magnetometer Functions ------------------------------
    /**
    * @brief Sets the magnetometer output divisor rate (ODR)
    * 
    * @param magRate The magnetometer ODR (enum: lis2mdl_rate_t)
    * 
    * Directly set ODR using enum
     */
    void setMagUpdateRate(lis2mdl_rate_t magRate);
    
    /**
    * @brief User provided calibration values for the magnetometer
    * 
    * @param hardIronCalibration Calibration value for hard iron interferance
    * @param softIronCalibration Calibration value for soft iron interferance
    * 
    * Expected to obtain these values OFF-BOARD. Collect data, run through least squares,
    * then plug back in. 
    * Formula: mag_cal = S (mag_uncal - b)
    * Where S is the 3x3 soft iron calibration matrix and b is the 3x1 hard iron calibration vector
     */
    void setMagCalibration(Vector3f& hardIronCalibration, Rotation & softIronCalibration);

    /**
    * @brief Collect data for Magnetometer Calibration
    * 
    * Collect data for 60 seconds. 
    * Rotate slowly in all directions to obtain as many directions to form a sphere.
    * Saves all data onto file on SD card
     */
    void calibrateMagnetometer();

    /**
    * @brief Returns magnetometer measurement if the time since the last reading is greater 
    * than the frequency set at initialization.
    * 
    * @param now Current time in microseconds
    * 
    * @return Checks register and returns measurement if (time_now - time_since_last_measurement) > sensor_frequency. 
    * Else, assume register has the same value and return NaN array
     */
    std::array<float,3> getMagMeas(unsigned long now);

    // -------------------------- Altimeter Functions ------------------------------
    /**
    * @brief Sets the altimeter oversampling settings. Options are:
    * BMP3_NO_OVERSAMPLING,BMP3_OVERSAMPLING_2X, BMP3_OVERSAMPLING_4X, 
    * BMP3_OVERSAMPLING_8X,BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_32X
    * 
    * @param altOversamplingFactor The altimeter oversample factor 
    * 
    * For now, set temperature to no oversampling since there is no use for it.
    * From Datasheet, 8x oversampling will require ~53Hz-42Hz, so keep in mind when setting ODR
    * Datasheet recommends BMP3_OVERSAMPLING_8X for drones
     */
    void setAltPressureOversampling(uint8_t altOversamplingFactor);

    /**
    * @brief Sets the altimeter Low pass filter coefficent. Might replace with a
    * software base filter rather than use the default.
    * Options are:
    * BMP3_IIR_FILTER_DISABLE (no filtering), BMP3_IIR_FILTER_COEFF_1,
    * BMP3_IIR_FILTER_COEFF_3, BMP3_IIR_FILTER_COEFF_7, BMP3_IIR_FILTER_COEFF_15,
    * BMP3_IIR_FILTER_COEFF_31, BMP3_IIR_FILTER_COEFF_63, BMP3_IIR_FILTER_COEFF_127
    * 
    * @param altFilterCoeff The altimeter filter coefficent
    * 
    * Datasheet recommends 2 for drone
     */
    void setAltFilterCoefficent(uint8_t altFilterCoeff);

    /**
    * @brief Sets the altimeter ODR
    * Options are:
    * BMP3_ODR_200_HZ, BMP3_ODR_100_HZ, BMP3_ODR_50_HZ, BMP3_ODR_25_HZ, 
    * BMP3_ODR_12_5_HZ, BMP3_ODR_6_25_HZ, BMP3_ODR_3_1_HZ, BMP3_ODR_1_5_HZ, 
    * BMP3_ODR_0_78_HZ, BMP3_ODR_0_39_HZ, BMP3_ODR_0_2_HZ, BMP3_ODR_0_1_HZ, 
    * BMP3_ODR_0_05_HZ, BMP3_ODR_0_02_HZ, BMP3_ODR_0_01_HZ, BMP3_ODR_0_006_HZ, 
    * BMP3_ODR_0_003_HZ, or BMP3_ODR_0_001_HZ
    * 
    * @param altRate The altimeter filter coefficent
    * 
    * Datasheet recommends 2 for drone
     */
    void setAltUpdateRate(uint8_t altRate);

    /**
    * @brief Sets the Altimeter reference height
    * 
    * @param altDataSum Summation of Altimeter data collected over a period of time
    * @param numAltMeas Number of Altimeter data collected to calculate sum
    * 
    * Collect X number of Altimeter data and sum them up. Average to get the bias.
     */
    void setAltCalibration(float altDataSum, int numAltMeas);

    /**
    * @brief Returns altimeter measurement if the time since the last reading is greater 
    * than the frequency set at initialization.
    * 
    * @param now Current time in microseconds
    * 
    * @return Checks register and returns measurement if (time_now - time_since_last_measurement) > sensor_frequency. 
    * Else, assume register has the same value and return NaN array
     */
    float getAltMeas(unsigned long now);

    // -------------------------- GPS Functions ------------------------------
    /**
    * @brief Sets the gps NMEA sentence format
    * Predefined options are under Adafruit_PMTK.h
    * 
    * @param gpsNMEAFormat The gps NMEA Output format
    * 
    * Will be going with the RMC format in order to get the Latitude, Longitude, SOG, and COG.
    * Adafruit provides RMC + GMA format so use that.
    * https://cdn-shop.adafruit.com/product-files/5186/5186_PA1616D_Datasheet.pdf#page=20
     */
    void setGPSNMEAFormat(const char* gpsNMEAFormat);

    /**
    * @brief Sets the gps NMEA Update rate. This is how often the NMEA sentences gets sent over.
    * Does NOT impact how often positions are calculated
    * Predefined options are under Adafruit_PMTK.h
    * 
    * @param gpsNMEAUpdateRate The gps NMEA update rate 
     */
    void setGPSNMEAUpdateRate(const char* gpsNMEAUpdateRate);

    /**
    * @brief Sets the gps Position Update rate. This is how often the position fix are calculated
    * Even if NMEA rate is faster, the gps cannot produce new results past this rate 
    * Predefined options are under Adafruit_PMTK.h
    * 
    * @param gpsPositionUpdateRate The gps position update rate 
     */
    void setGPSPositionUpdateRate(const char* gpsPositionUpdateRate);

    /**
    * @brief Sets the gps Baud rate (BPS). This is many bits per second can be sent between the
    * GPS and computer over serial. Estimate each character as ~10 bits. 9600 should be enough for
    * 1Hz sampling rate.
    * Predefined options are under Adafruit_PMTK.h
    * 
    * @param gpsBaudRate The gps position update rate 
     */
    void setGPSBaudRate(const char* gpsBaudRate);

    /**
    * @brief Computes the ECEF Position vector given the (lat,long,alt) from GPS
    * 
    * @param latitude Lattitude from GPS in Degrees
    * @param longitude Longitude from GPS in Degrees
    * @param altitude Height from Mean Sea Level in meters
    * 
    * @return ECEF Position vector. Using double for full precision for large rECEF values
    */
    std::array<double,3> LLA2ECEF(double latitude, double longitude, double altitude);


    /**
    * @brief Returns the local NED Position vector given the (lat,long,alt) from GPS
    * 
    * @param latitude Lattitude from GPS in Degrees
    * @param longitude Longitude from GPS in Degrees
    * @param altitude Height from Mean Sea Level in meters
    * 
    * @return NED Position Vector. Using double for full precision for large rECEF values
    * ONLY USE THIS WHEN CALIBRATED SENSORS AND HAVE A REFERENCE ECEF POSITION AND ECEF2NED MATRIX
    */
    Vector3f LLA2NED(double latitude, double longitude, double altitude);

    /**
    * @brief Sets the reference Lattitude and Longitude. Using reference altitude, find reference ECEF position
    * 
    * @param gpsDataSum Summation of IMU data collected over a period of time
    * @param numGPSMeas Number of IMU data collected to calculate sum
    * 
    * Collect X number of GPS Lat/Long data and sum them up. Average to get the Reference.
    * Requires a reference altitude to be set    prior to this. This means setAltCalibration() must 
    * be successful.
     */
    void setGPSCalibration(std::array<double,2>& gpsDataSum, int numGPSMeas);

    /**
    * @brief Returns GPS measurement if the time since the last reading is greater 
    * than the frequency set at initialization.
    * 
    * @param now Current time in microseconds
    * 
    * @return Checks register and returns measurement if (time_now - time_since_last_measurement) > sensor_frequency. 
    * Else, assume register has the same value and return NaN array
     */
    std::array<double,5> getGPSMeas(unsigned long now);


private:
    //---------- Initialization ------------
    //Adafruit sensors
    Adafruit_ICM20649 imu_;
    Adafruit_LIS2MDL mag_;
    Adafruit_BMP3XX alt_;
    Adafruit_GPS gps_; 

    //Frequencies (micro-seconds)
    float freqIMU_; 
    float freqMag_;
    float freqAlt_;
    float freqGPS_;

    //Rotations
    Rotation rotBody2IMU_;
    Rotation rotBody2Mag_;
    Vector3f magHardIron_ {NAN,NAN,NAN}; 
    Rotation magSoftIron_ = std::array<float,9>{NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN}; 
    Rotation magNE2TrueNE_ =std::array<float,9>{NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN}; //Adjusts for inclination and declination

    //Time since last measurements ( Initialize w/ zero so start with a reading )
    unsigned long lastIMU_ = 0; 
    unsigned long lastMag_ = 0; 
    unsigned long lastAlt_ = 0;
    unsigned long lastGPS_ = 0;

    //------------ Calibrated ----------------
    // Return for NAV Init
    Vector3f startUpAccelBiasBody_ {NAN,NAN,NAN}; 
    Vector3f startUpGyroBiasBody_ {NAN,NAN,NAN}; 

    // GPS Stuff
    std::array<double,3> referenceECEFPosition_ {NAN,NAN,NAN}; //Double to handle large value
    Rotation ECEF2NED_ = std::array<float,9>{NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN}; //Replace with rotation matrix class
    double referenceAltitude_ {NAN}; //From Alitmeter calibration
    double referenceLatitude_ {NAN}; //From GPS Calibration
    double referenceLongitude_ {NAN}; //From GPS Calibration

    //Bools / Flags
    bool startUpBool_ = false;
    
    bool calibratedSensors_ = false; //Once calibrated, use calibrated parameters
    bool gpsFix_ = false; 

    //------------- Current Measurements ----------------
    //Used to get referenceECEFPosition, but not used afterwards.
    //Store measurements from sensors, will always be overwritten.
    std::array<float,6> currIMUMeas_; // m/s^2 and rad/s
    std::array<float,3> currMagMeas_; // Unitless / Guass
    float currAltMeas_; // m
    std::array<double,5> currGPSMeas_; //m, m/s



};
#endif // SENSORS_H