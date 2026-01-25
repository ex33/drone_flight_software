#include "Sensors.h"

Sensors::Sensors(float freqIMU, float freqMag, float freqAlt, float freqGPS, HardwareSerial &serialGPS):
gps_(&serialGPS) //Initializer list, initializes at same time as sensor object
{
    // Convert from Hz to Milli-Seconds
    freqIMU_ = CONSTANTS::seconds2milli/freqIMU; 
    freqMag_ = CONSTANTS::seconds2milli/freqMag;
    freqAlt_ = CONSTANTS::seconds2milli/freqMag;
    freqGPS_ = CONSTANTS::seconds2milli/freqGPS;
    
};
// -------------------------- All Sensors Functions ------------------------------

void Sensors::startUpSensors() {
    //Given default I2C address and I2C bus, check if initilized properly
    if (!this->imu_.begin_I2C()) { 
        Serial.println("IMU not found at default address... float check wiring");
        while (1) { //Eventually replace with error
        delay(10); //Infinite loop catches IMU not initized 
        };
    } else {
        Serial.println("IMU Detected");
    }

    if (!this->mag_.begin()) {
        Serial.println("Magnetometer not found at default address... float check wiring");
        while (1) { //Eventually replace with error
        delay(10); //Infinite loop catches Magnetometer not initized 
        };
    } else {
        Serial.println("Magnetometer Detected");
    }

    if (!this->alt_.begin_I2C()) {
        Serial.println("Altimeter not found at default address... float check wiring");
        while (1) { //Eventually replace with error
        delay(10); //Infinite loop catches Magnetometer not initized 
        };
    } else {
        Serial.println("Altimeter Detected");
    }

    //UNBLOCK FOR GPS
    // Checks for GPS Lock
    // Check if we are running GPS
    if (this->gpsFlag_) {
        this->gps_.begin(9600); //Initializes gps communication on provided serial

        //Checks to see if we can read a message. This doesn't mean we have a fix
        unsigned long timeout = 10 * CONSTANTS::seconds2milli; //Will check gps for 5 seconds2milli to see if we recieve a test message
        unsigned long startGPSTest = millis();
        bool gpsCheckoutBool = false;
        Serial.println ("Checking for GPS Message . . .");
        while (millis() - startGPSTest < timeout && !gpsCheckoutBool) { //Checks for response without a fix
            this->gps_.read(); // Get each char from buffer until we have the full sentence
            if (this->gps_.newNMEAreceived()) { //Tells you full sentence aquired
                this->gps_.parse(this->gps_.lastNMEA()); //Parse through
                Serial.println(this->gps_.lastNMEA());
                gpsCheckoutBool = true;
            }
        }
        if (gpsCheckoutBool) {
            Serial.println("GPS Message Recieved");
        } else {
            Serial.println("GPS Not Detected, check wiring");
            Serial.println("Proceed without GPS");
            this->gpsFlag_ = 0;
        }
    }

    Serial.println("Sensor Start-Up completed");
    

}

void Sensors :: setUpSensors(std::array<float,3> magHardIronArray, std::array<float,9> magSoftIronArray,
                            std::array<float,9> rotBody2IMUArray, std::array<float,9> rotBody2MagArray) {

    // Convert into Vector and Rotations
    Vector3f magHardIron (magHardIronArray);
    Rotation magSoftIron (magSoftIronArray);
    Rotation rotBody2IMU (rotBody2IMUArray);
    Rotation rotBody2Mag (rotBody2MagArray);

    // ---Set up IMU ----
    Serial.println("Setting up IMU");
    this->setIMUDataRange(icm20649_accel_range_t::ICM20649_ACCEL_RANGE_16_G, icm20649_gyro_range_t::ICM20649_GYRO_RANGE_2000_DPS);
    this->setIMUUpdateRate(10); //102.3 Hz
    this->rotBody2IMU_ = rotBody2IMU;
  
    // --- Set up Magnetometer  ---
    Serial.println("Setting up Magnetometer");
    this->setMagUpdateRate(lis2mdl_rate_t::LIS2MDL_RATE_50_HZ); 
    this->setMagCalibration(magHardIron, magSoftIron);
    this->rotBody2Mag_ = rotBody2Mag;
    // --- Set up Altimeter ---
    Serial.println("Setting up Altimeter");
    this->setAltPressureOversampling(BMP3_OVERSAMPLING_16X); //Datasheet recommended for drones to oversample 8x
    this->setAltUpdateRate(BMP3_ODR_25_HZ); //Datasheet recommended for drones to have ODR at 50Hz
    this->setAltFilterCoefficent(BMP3_IIR_FILTER_COEFF_15); //Datasheet recommended for drones to have IIR filter bit be 2 (Coeff 3, see table 4.3.21)
    // For indoor application, there is a different set of recoomended:
    // Oversampling to 16x
    // ODR to 25 Hz
    // IIR Fitler bit be 4 (Coeff 15)
    
    // --- Set up GPS ---
    if (this->gpsFlag_) {
        Serial.println("Setting up GPS");
        this->setGPSNMEAFormat(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Formats to output RMC (lat,long,SOG,COG) + GGA (Altitude, sat id, etc). Options found in Adafruit_PMTK.h
        this->setGPSNMEAUpdateRate(PMTK_SET_NMEA_UPDATE_1HZ); //1 Hz
        this->setGPSPositionUpdateRate(PMTK_API_SET_FIX_CTL_1HZ); //1Hz
        this->setGPSBaudRate(PMTK_SET_BAUD_9600); //9600 Bits per second limit. Should match with above 
    }
}

void Sensors::finalCheckOut() {

}

void Sensors::calibrateSensors(int total_seconds_for_calibration) {
    // Use this to calibrate...
    // IMU --> Start up biases for accelerometer / gyro
    // Altimeter --> Starting altitude for reference
    // Magnetometer --> Reference Vector for ESKF
    // GPS --> Starting Latitude and Longitude. 

    if (SD.exists("imu_calibration.csv")) {
        SD.remove("imu_calibration.csv");
    }
    if (SD.exists("mag_static_calibration.csv")) {
        SD.remove("mag_static_calibration.csv");
    }
    if (SD.exists("alt_calibration.csv")) {
        SD.remove("alt_calibration.csv");
    }
    if (SD.exists("gps_calibration.csv")) {
        SD.remove("gps_calibration.csv");
    }

    // Open .csv to save down IMU and Altimeter data
    File imuCalFile = SD.open("imu_calibration.csv", FILE_WRITE); //imu .csv
    if (!imuCalFile) {
        Serial.println("Failed to open IMU Calibration file");
        return;
    }
    imuCalFile.println("t,ax,ay,az,gx,gy,gz"); 

    File magCalFile = SD.open("mag_static_calibration.csv", FILE_WRITE); //Altimeter .csv
    if (!magCalFile) {
        Serial.println("Failed to open Magnetometer Static Calibration file");
        return;
    }
    magCalFile.println("t,mx, my, mz"); 

    File altCalFile = SD.open("alt_calibration.csv", FILE_WRITE); //Altimeter .csv
    if (!altCalFile) {
        Serial.println("Failed to open Altimeter Calibration file");
        return;
    }
    altCalFile.println("t,h"); 


    File gpsCalFile = SD.open("gps_calibration.csv", FILE_WRITE); //GPS .csv
    if (!gpsCalFile) {
        Serial.println("Failed to open GPS Calibration file");
        return;
    }
    gpsCalFile.println("t,lat,long,sog,cog"); 


    
    unsigned long next_print = 0; // Keeps track of when to print out progress
    //Static calibrations (get offsets)
    Serial.println("Starting Static Calibrations ...");
    std::array<float,6> imuMeasSum {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // For IMU Bias
    std::array<float,3> magMeasSum {0.0, 0.0, 0.0};
    float altMeasSum = 0.0; // For reference Pressure
    std::array<double,2> gpsMeasSum {0.0, 0.0}; // For reference Lat/Long
    int numIMUMeas = 0; 
    int numMagMeas = 0;
    int numAltMeas = 0;
    int numGPSMeas = 0;

    //Collect Data for 60 seconds2milli
    // ~ 6000 IMU data
    // ~ 3000 Altimeter data
    // ~ 60 GPS data
    std::array<float,6> imuMeas;
    std::array<float,3> magMeas;
    float altMeas;
    std::array<double,5> gpsMeas;

    // Counter to keep track of how many altimeter measurements to ignore. 
    int ignoreAltMeas = 0; 
    unsigned long static_calibration_start = millis();
    unsigned long static_calibration_now = millis();
    unsigned long time_in_static_calibration = static_calibration_now - static_calibration_start;
    while (time_in_static_calibration < total_seconds_for_calibration * CONSTANTS::seconds2milli) {
        static_calibration_now = millis();
        time_in_static_calibration = static_calibration_now - static_calibration_start;
        // IMU Loop
        if (this->imuUpdate(static_calibration_now)) {
            imuMeas = this->getIMUMeas();

            imuMeasSum[0] += imuMeas[0];
            imuMeasSum[1] += imuMeas[1];
            imuMeasSum[2] += imuMeas[2];
            imuMeasSum[3] += imuMeas[3];
            imuMeasSum[4] += imuMeas[4];
            imuMeasSum[5] += imuMeas[5];
            numIMUMeas += 1;

            //Write to file
            imuCalFile.print(time_in_static_calibration);
            imuCalFile.print(",");
            imuCalFile.print(imuMeas[0],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
            imuCalFile.print(",");
            imuCalFile.print(imuMeas[1],6);
            imuCalFile.print(",");
            imuCalFile.print(imuMeas[2],6); 
            imuCalFile.print(",");
            imuCalFile.print(imuMeas[3],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
            imuCalFile.print(",");
            imuCalFile.print(imuMeas[4],6);
            imuCalFile.print(",");
            imuCalFile.println(imuMeas[5],6); 
        };

        // Unlike the other sensors, we MUST use soft/hard iron calibration for the Magnetometer prior to the static calibration step.
        if (this->magUpdate(static_calibration_now)) {
            magMeas = this->getMagMeas();

            // Make Measurement into a vector
            Vector3f magMeasVector (magMeas);
            magMeasVector = this->magSoftIron_ * (magMeasVector - this->magHardIron_);
            magMeasSum[0] += magMeasVector[0];
            magMeasSum[1] += magMeasVector[1];
            magMeasSum[2] += magMeasVector[2];

            numMagMeas += 1;
            //Write to file
            magCalFile.print(time_in_static_calibration);
            magCalFile.print(",");
            magCalFile.print(magMeasVector[0],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
            magCalFile.print(",");
            magCalFile.print(magMeasVector[1],6);
            magCalFile.print(",");
            magCalFile.println(magMeasVector[2],6); 
        };

        if (this->altUpdate(static_calibration_now)) {
            if (ignoreAltMeas > 100) { //Ignore the first 500 altimeter measurement to let pressure / temperature stabilize
                altMeas = this->getAltMeas();
                altMeasSum += altMeas;
                numAltMeas ++ ;
                //Write to file
                altCalFile.print(time_in_static_calibration);
                altCalFile.print(",");
                altCalFile.println(altMeas,6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
            } else {
                ignoreAltMeas++;
            };
        };

        
        if (this->gpsFlag_) {
            if (this->gpsUpdate(static_calibration_now)) {
                gpsMeas = this->getGPSMeas();
  
                //Only need to sum up Lat/Long. We know speed is zero, and we get altitude from altimeter.
                gpsMeasSum[0] += gpsMeas[0];
                gpsMeasSum[1] += gpsMeas[1]; 
                numGPSMeas++;

                //Write to file
                gpsCalFile.print(time_in_static_calibration);
                gpsCalFile.print(",");
                gpsCalFile.print(gpsMeas[0],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
                gpsCalFile.print(",");
                gpsCalFile.println(gpsMeas[1],6);
                };
        };

        if (time_in_static_calibration >= next_print) {
            Serial.print("Static Calibration: ");
            Serial.print(time_in_static_calibration / (CONSTANTS::seconds2milli));
            Serial.print(" out of ");
            Serial.print(total_seconds_for_calibration);
            Serial.println(" seconds");
            next_print += 10*CONSTANTS::seconds2milli;
        };
    };

    imuCalFile.close(); //Close File
    altCalFile.close(); 
    magCalFile.close();
    gpsCalFile.close();
    this -> setIMUCalibration(imuMeasSum, numIMUMeas);
    this -> setAltCalibration(altMeasSum, numAltMeas);
    this -> setMagCalibration(magMeasSum, numMagMeas);

    if (this->gpsFlag_ ) {
        this -> setGPSCalibration(gpsMeasSum, numGPSMeas);
    }

};

bool Sensors::finalArmingCheck() {
    bool calIMU = true;
    bool calMag = true;
    bool calGPS = true;

    //Can make this more efficent, but focus on readability since this is during the setup loop
    //------Check IMU--------
    for (unsigned int i = 0; i<3; i++) {
        if (isnan(this->startUpAccelBiasSensor_[i]) || isnan(this->startUpGyroBiasSensor_[i])) {
            calIMU = false;
            break;
        }
    }

    //------Check Magnetometer--------
    for (unsigned int i = 0; i<3; i++) {
        if (isnan(this->magHardIron_[i])) {
            calMag = false;
            break;
        }
    }
    for (unsigned int i = 0; i<9; i++) {
        if (isnan(this->magSoftIron_[i])) {
            calMag = false;
            break;
        }
    }

    //------Check GPS / Altimeter--------
    for (unsigned int i = 0; i<3; i++) { 
        //If this is not nan, that means we successfully set the refence altitude along with lat/long, so no need for seperate altimeter check
        if (isnan(this->referenceECEFPosition_[i])) { 
            calGPS = false;
            break;
        }
    }

    for (unsigned int i = 0; i<9; i++) { 
        if (isnan(this->ECEF2NED_[i])) { 
            calGPS = false;
            break;
        }
    }

    //Only need to do the final checkout if we calibrated the sensors correctly. 
    if (calIMU && calMag && calGPS) {
        this->finalCheckOut();
        this -> calibratedSensors_ = true; //Set flag to true, ready to use calibrated measurements from sensors

        //If we got past checkoutSensors without triggering a while loop, we have successfully calibrated and checked all sensors
        return true;
    } else {
        return false;
    }

};


// -------------------------- IMU Functions ------------------------------
void Sensors::setIMUDataRange(icm20649_accel_range_t accelRange,icm20649_gyro_range_t gyroRange) {
    this -> imu_.setAccelRange(accelRange);
    this -> imu_.setGyroRange(gyroRange);
};

void Sensors::setIMUUpdateRate(unsigned int imuRateDivisor) {
    // Set up Internal Output data rate (this differs from the 400kHz communication speed. ODR is when the register gets a new value)
    // Gyro ODR is 1125Hz, Accelerometer is 1125Hz. Make them both run at ~100Hz.
    // Gyro and Accelerometer has two modes for ODR, the one with a lower ODR typically has less bandwidth and more filtering, while the higher one trades off higher bandwidth for little filtering and more noise (for faster dynamics)
    // Formula is.. Effective ODR = ODR / (1 + rate_divisor)
    this -> imu_.setAccelRateDivisor(imuRateDivisor);
    this -> imu_.setGyroRateDivisor(imuRateDivisor);
}

void Sensors::setIMUCalibration(std::array<float,6>& imuData, int numIMUMeas) {

    //Get Bias
    this -> startUpAccelBiasSensor_ =  Vector3f(imuData[0]/numIMUMeas,imuData[1]/numIMUMeas,imuData[2]/numIMUMeas - CONSTANTS::g0); //Offset by gravity for z
    this -> startUpGyroBiasSensor_ =  Vector3f(imuData[3]/numIMUMeas,imuData[4]/numIMUMeas,imuData[5]/numIMUMeas);

    //Do some other checks in here if necessary
    Serial.print("IMU Calibrated Successfully with ");
    Serial.print(numIMUMeas);
    Serial.println(" data.");
    Serial.print("B_ax : ");
    Serial.print(this->startUpAccelBiasSensor_[0],6);
    Serial.println(" m/s^2");
    Serial.print("B_ay : ");
    Serial.print(this->startUpAccelBiasSensor_[1],6);
    Serial.println(" m/s^2");
    Serial.print("B_az : ");
    Serial.print(this->startUpAccelBiasSensor_[2],6);
    Serial.println(" m/s^2");
    Serial.print("B_gx : ");
    Serial.print(this->startUpGyroBiasSensor_[0],6);
    Serial.println(" rad/s");
    Serial.print("B_gy : ");
    Serial.print(this->startUpGyroBiasSensor_[1],6);
    Serial.println(" rad/s");
    Serial.print("B_gz : ");
    Serial.print(this->startUpGyroBiasSensor_[2],6);
    Serial.println(" rad/s");

}


bool Sensors::imuUpdate(uint32_t now) {
    if (static_cast<float>(now - lastIMU_) >= this->freqIMU_) {
        lastIMU_ = now;

        imu_.getEvent(&last_imu_accel_meas,
                      &last_imu_gyro_meas,
                      &last_imu_temp_meas);
        return true;
    }

    return false;
}


//This gets the RAW IMU Measurements
std::array<float,6> Sensors::getIMUMeas(){
    return std::array<float,6> {
        this->last_imu_accel_meas.acceleration.x,
        this->last_imu_accel_meas.acceleration.y,
        this->last_imu_accel_meas.acceleration.z,
        this->last_imu_gyro_meas.gyro.x,
        this->last_imu_gyro_meas.gyro.y,
        this->last_imu_gyro_meas.gyro.z
    };
};

std::array<float,6> Sensors::processIMUMeas(std::array<float,6> imuMeas) {

    Vector3f accelBody = this->rotBody2IMU_.transpose() * (Vector3f(imuMeas[0],imuMeas[1],imuMeas[2]) - this->startUpAccelBiasSensor_);
    Vector3f gyroBody = this->rotBody2IMU_.transpose() * (Vector3f(imuMeas[3],imuMeas[4],imuMeas[5]) - this->startUpGyroBiasSensor_);

    return std::array<float,6> {accelBody[0], 
                                accelBody[1],
                                accelBody[2],
                                gyroBody[0],
                                gyroBody[1],
                                gyroBody[2]};
}; 

// -------------------------- Magnetometer Functions ------------------------------
void Sensors::setMagUpdateRate(lis2mdl_rate_t magRate) {
    // Header file sets odr for you by specifying lis2mdl_rate_t
    mag_.setDataRate(magRate);
};

void Sensors::setMagCalibration(Vector3f& hardIronCalibration, Rotation & softIronCalibration) {
    this -> magHardIron_ = hardIronCalibration;
    this -> magSoftIron_ = softIronCalibration;
};

void Sensors::setMagCalibration(std::array<float,3>& magData, float numData) {
    this -> magRef_ = Vector3f(magData[0]/numData, magData[1]/numData, magData[2]/numData);
    Serial.println("Mag Ref is...");
    Serial.println(this->magRef_[0]);
    Serial.println(this->magRef_[1]);
    Serial.println(this->magRef_[2]);
};




void Sensors::calibrateMagnetometer() {
    // This should be done off-board. Collect data, run Least Squares, then input calibrated values.
    // Values shouldn't change much if environment / magnetic interferance doesn't change. 
    std::vector<std::array<float,3>> magMeasCollection; 

    unsigned long dynamic_calibration_start = millis();
    unsigned long dynamic_calibration_now = millis();
    //bool mag_calibration_quality_check = false;
    
    // Open Magnetometer Calibration File
    SD.remove("mag_dynamic_calibration.csv"); //Make sure its clean
    File magCalFile = SD.open("mag_dynamic_calibration.csv", FILE_WRITE);
    if (!magCalFile) {
        Serial.println("Failed to open Magnetometer Calibration file");
        return;
    }
    magCalFile.println("time, mx, my, mz"); //Header
    Serial.println("Starting Dynamic Calibrations ...");
    unsigned long time_in_dynamic_calibration = dynamic_calibration_now - dynamic_calibration_start;
    //unsigned long next_print = 0; // Keeps track of when to print out progress
    while (time_in_dynamic_calibration < 5 * 60 * CONSTANTS::seconds2milli) { // Collect Data for 5 minutes
        if (this->magUpdate(dynamic_calibration_now)) {
            std::array<float,3> magMeas = this->getMagMeas();

            if (!isnan(magMeas[0])) { //Only need to check the first index since they should all be NaN

                magMeasCollection.push_back(magMeas);
                
                //Write to file
                magCalFile.print(time_in_dynamic_calibration);
                magCalFile.print(",");
                magCalFile.print(magMeas[0],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
                magCalFile.print(",");
                magCalFile.print(magMeas[1],6);
                magCalFile.print(",");
                magCalFile.println(magMeas[2],6); 

            }
        }
        dynamic_calibration_now = millis();
        time_in_dynamic_calibration = dynamic_calibration_now - dynamic_calibration_start;
        // if (time_in_dynamic_calibration >= next_print) {
        //     Serial.print("Dynamic Calibration: ");
        //     Serial.print(time_in_dynamic_calibration / CONSTANTS::seconds2milli);
        //     Serial.print(" out of ");
        //     Serial.print(60);
        //     Serial.println(" seconds");
        //     next_print += 10*CONSTANTS::seconds2milli;
        // }
    };
    magCalFile.close(); //Close File
    Serial.println("Finished Collecting Data. Please unplug and perform post processing . . . ");
    
}


bool Sensors::magUpdate(uint32_t now) {
    if (static_cast<float>(now - lastMag_) >= this->freqMag_) {
        lastMag_ = now;

        mag_.getEvent(&this->last_mag_meas);
        return true;
    }

    return false;
}

std::array<float,3> Sensors::getMagMeas() {
    return std::array<float,3> {
        this->last_mag_meas.magnetic.x,
        this->last_mag_meas.magnetic.y,
        -this->last_mag_meas.magnetic.z //Negative here since magnetometer is Left Handed Coordinate system, where X-Y are aligned properly, but Z is Up instead of down
    };
};

std::array<float,3> Sensors::processMagMeas(std::array<float,3> magMeas){
    Vector3f magBody = this->rotBody2Mag_.transpose() * this->magSoftIron_*(Vector3f(magMeas) - this->magHardIron_); // Adjust for Soft/Hard Iron calibration, then rotate into body frame
    return std::array<float,3> {magBody[0], magBody[1], magBody[2]};
}

Vector3f Sensors::getMagRefForFilter() {
    return this->rotBody2Mag_.transpose() * this->magRef_; //Filter expects this in the body frame
};

// -------------------------- Altimeter Functions ------------------------------
void Sensors::setAltPressureOversampling(uint8_t altOversamplingFactor) {
    this->alt_.setPressureOversampling(altOversamplingFactor);
    this->alt_.setTemperatureOversampling(altOversamplingFactor); //Hardcode to no oversample for temperature
};

void Sensors::setAltFilterCoefficent(uint8_t altFilterCoeff) {
    this ->alt_.setIIRFilterCoeff(altFilterCoeff);
};

void Sensors::setAltUpdateRate(uint8_t altRate) {
    this -> alt_.setOutputDataRate(altRate);
};

void Sensors::setAltCalibration(float altDataSum, int numAltMeas) {
    //Get IMU Bias
    //this -> referenceAltitude_ = altDataSum / numAltMeas;
    this -> referencePressure_ = altDataSum/numAltMeas;


    //Do some other checks in here if necessary
    Serial.print("Altimeter Calibrated Successfully with ");
    Serial.print(numAltMeas);
    Serial.println(" data.");
    Serial.print("Reference Pressure : ");
    Serial.print(this->referencePressure_,6);
    Serial.println(" hpa");
}

bool Sensors::altUpdate(uint32_t now) {

    if (static_cast<float>(now - lastAlt_) >= this->freqAlt_) {
        lastAlt_ = now;

        this->last_alt_meas = alt_.readPressure() / 100.0f; // hPa
        return true;
    }

    return false;
}

float Sensors::getAltMeas() {
    return this->last_alt_meas;
};

float Sensors::processAltMeas(float altMeas){
    // // If our current Altimeter Measurement is within the expected absolute error for the pressure measurement, then update it
    // // Causes an issue where if we are hovering within this region, it'll try to adjust for that hover state. Should pass in a flag here for when we are idling (i.e. on the ground, vs in mission mode)
    // if (fabs(this->currAltMeas_-this->referencePressure_)<0.5) { //Absolute pressure up to 0.5 hPA

    //     this->referencePressure_ = (0.995) * this->referencePressure_ + 0.005 * this->currAltMeas_;
    // }

    float altHeight = 44330 * (1- pow(this->last_alt_meas/this->referencePressure_,0.1903));

    // Gate it to account for difference in reference Pressure at the start. If we are getting negative values, we are basically at rest
    if (altHeight < 0) {
        altHeight = 0; 
    }

    return altHeight; //Will be signed within ESKF to match NED
}; 

// -------------------------- GPS Functions ------------------------------
void Sensors::setGPSFlag(const bool gpsFlag) {
    this->gpsFlag_ = gpsFlag;
}


void Sensors::setGPSNMEAFormat(const char* gpsNMEAFormat) {
    this->gps_.sendCommand(gpsNMEAFormat);
};

void Sensors::setGPSNMEAUpdateRate(const char* gpsNMEAUpdateRate) {
    this->gps_.sendCommand(gpsNMEAUpdateRate);
}

void Sensors::setGPSPositionUpdateRate(const char* gpsPositionUpdateRate) {
    this->gps_.sendCommand(gpsPositionUpdateRate);
};

void Sensors::setGPSBaudRate(const char* gpsBaudRate) {
    this->gps_.sendCommand(gpsBaudRate);
};

std::array<double,3> Sensors::LLA2ECEF(double latitude, double longitude, double altitude) {
    double sinLat = sin(latitude * PI/180.0);
    double cosLat = cos(latitude * PI/180.0);
    double cosLong = cos(longitude * PI/180.0);
    double sinLong = sin(longitude * PI/180.0);
    //Radius of curvature in the prime vertical, N= a/sqrt(1-e^2*sin(lat)^2)
    double N = CONSTANTS::WGS84_a / sqrt(1-CONSTANTS::WGS84_e2 * sinLat*sinLat); 

    double x = (N + altitude) * cosLat;
    double z = (N * (1-CONSTANTS::WGS84_e2) + altitude) * sinLat;

    return std::array<double,3>{x * cosLong, x * sinLong, z};
};

Vector3f Sensors::LLA2NED(double latitude, double longitude, double altitude) {
    // Get ECEF position
    std::array<double,3>  r_ECEF = this->LLA2ECEF(latitude, longitude, altitude);

    // Get Delta ECEF
    Vector3f del_r_ECEF {static_cast<float>(r_ECEF[0] - this->referenceECEFPosition_[0]),
                         static_cast<float>(r_ECEF[1] - this->referenceECEFPosition_[1]),
                         static_cast<float>(r_ECEF[2] - this->referenceECEFPosition_[2])};  //Can static cast to float here since the difference between two ECEF vectors are no longer large

    //Rotate into NED, will replace this with RotationMatrix
    Vector3f NED = this->ECEF2NED_ * del_r_ECEF;
    
    return NED;
};

void Sensors::setGPSCalibration(std::array<double,2>& gpsDataSum, int numGPSMeas) {
    //Set ref Lat/Long
    this->referenceLatitude_ = gpsDataSum[0] / numGPSMeas;
    this->referenceLongitude_ = gpsDataSum[1] / numGPSMeas;


    //Set ref ECEF Position
    // Assume we start at 0 Altitude. This is probably fine since the difference is a couple meters, which shouldn't pose too much of an issue
    // Considering the magnitude of the other terms
    this->referenceECEFPosition_ = this->LLA2ECEF(this->referenceLatitude_, this->referenceLongitude_, this->referenceAltitude_); 

    //Set ECEF2NED_
    //hardcode the formula R2(-pi/2)R2(-Lat)R3(Long) = R2(-Lat-pi/2)R3(Long) [Passive convention]
    // Lat --> phi, Long --> theta
    //Don't need high precision for rotation so can use float here.
    float sp = sin(this->referenceLatitude_ * CONSTANTS::deg2rad);
    float cp = cos(this->referenceLatitude_ * CONSTANTS::deg2rad);
    float st = sin(this->referenceLongitude_ * CONSTANTS::deg2rad);
    float ct= cos(this->referenceLongitude_ * CONSTANTS::deg2rad);
    this->ECEF2NED_ = Rotation(std::array<float,9> {-sp * ct, -sp*st, cp,
                                                    -st,      ct,     0,
                                                    -cp*ct,   -cp*st, -sp});

    Serial.print("GPS Calibrated Successfully with ");
    Serial.print(numGPSMeas);
    Serial.println(" data.");
    Serial.println("Reference ECEF Position : ");
    Serial.print("X : ");
    Serial.print(this->referenceECEFPosition_[0],6); 
    Serial.println(" m");
    Serial.print("Y : ");
    Serial.print(this->referenceECEFPosition_[1],6); 
    Serial.println(" m");
    Serial.print("Z : ");
    Serial.print(this->referenceECEFPosition_[2],6); 
    Serial.println(" m");
    Serial.print("Reference Latitude : ");
    Serial.print(this->referenceLatitude_,6);
    Serial.println(" deg");
    Serial.print("Reference Longitude : ");
    Serial.print(this->referenceLongitude_,6);
    Serial.println(" deg");

}


bool Sensors::gpsUpdate(uint32_t now) {
    if (static_cast<float>(now - lastGPS_) >= this->freqGPS_) {
        lastGPS_ = now;

        while(this -> gps_.read()); //Reads from serial buffer until there is no characters left, returns 0 when all characters read
         // Check for sentence
        if (this-> gps_.newNMEAreceived()) {
            this->gps_.parse(this->gps_.lastNMEA());
            this->last_gps_meas = {this-> gps_.latitudeDegrees, this-> gps_.longitudeDegrees, this->gps_.altitude,  this->gps_.speed,this->gps_.angle};
            return true;
        } 
        return false;
    } 
    return false;
};


std::array<double,5> Sensors::getGPSMeas() {
    return this->last_gps_meas;
};


std::array<float,4> Sensors::processGPSMeas(std::array<double,5> gpsMeas, float currentAlt){

    float v_N = this->currGPSMeas_[2] * cos(this->currGPSMeas_[3] * CONSTANTS::deg2rad);
    float v_E = this->currGPSMeas_[2] * sin(this->currGPSMeas_[3] * CONSTANTS::deg2rad);
    Vector3f gpsNEDPos;
    
    gpsNEDPos = this->LLA2NED(this-> currGPSMeas_[0],  //Note that adafruit handles N/S and E/W by returning +/-
                            this-> currGPSMeas_[1],
                            currentAlt); //When we are processing GPS measurement, use the altitude from the kalman filter since its basically a less noisy altimeter

    return std::array<float,4>{gpsNEDPos[0],gpsNEDPos[1], v_N, v_E};
}; 


void Sensors::setGPSReferenceAltitude(double refAlt){
    this->referenceAltitude_=refAlt;
};