#include "Sensors.h"

Sensors::Sensors(double freqIMU, double freqMag, double freqAlt, double freqGPS, HardwareSerial &serialGPS):
gps_(&serialGPS) //Initializer list, initializes at same time as sensor object
{
    // Convert from Hz to Micro-seconds
    freqIMU_ = this->seconds/freqIMU; 
    freqMag_ = this->seconds/freqMag;
    freqAlt_ = this->seconds/freqMag;
    freqGPS_ = this->seconds/freqGPS;
};
// -------------------------- All Sensors Functions ------------------------------
void Sensors::getMeasurements () {
    //Get current Time
    unsigned long now = micros(); 

    //Update each sensor measurements
    this->currIMUMeas = getIMUMeas(now);
    this->currMagMeas = getMagMeas(now);
    this->currAltMeas = getAltMeas(now);
    this->currGPSMeas = getGPSMeas(now);
};

void Sensors::printMeasurements() {
    Serial.println("Printing All Measurements . . . ");
    // Accelerometer
    Serial.print("Accel (m/s^2): X=");
    Serial.print(this->currIMUMeas[0], 6);
    Serial.print(" Y=");
    Serial.print(this->currIMUMeas[1], 6);
    Serial.print(" Z=");
    Serial.println(this->currIMUMeas[2], 6);

    // Gyroscope
    Serial.print("Gyro (rad/s): X=");
    Serial.print(this->currIMUMeas[3], 6);
    Serial.print(" Y=");
    Serial.print(this->currIMUMeas[4], 6);
    Serial.print(" Z=");
    Serial.println(this->currIMUMeas[5], 6);

    // Magnetometer
    Serial.print("Mag (G): X=");
    Serial.print(this->currMagMeas[0], 6);
    Serial.print(" Y=");
    Serial.print(this->currMagMeas[1], 6);
    Serial.print(" Z=");
    Serial.println(this->currMagMeas[2], 6);

    // Altimeter
    Serial.print("Alt (m): Z=");
    Serial.println(this->currAltMeas, 6);

    // GPS 
    Serial.print("GPS (m, m/s): N=");
    Serial.print(this->currGPSMeas[0], 6);
    Serial.print(" E=");
    Serial.print(this->currGPSMeas[1], 6);
    Serial.print(" Vn=");
    Serial.print(this->currGPSMeas[2], 6);
    Serial.print(" Ve=");
    Serial.println(this->currGPSMeas[3], 6);
    Serial.println("------------------------");


};

void Sensors::startUpSensors() {
    //Given default I2C address and I2C bus, check if initilized properly
    if (!this->imu_.begin_I2C()) { 
        Serial.println("IMU not found at default address... double check wiring");
        while (1) { //Eventually replace with error
        delay(10); //Infinite loop catches IMU not initized 
        };
    } else {
        Serial.println("IMU Detected");
    }

    if (!this->mag_.begin()) {
        Serial.println("Magnetometer not found at default address... double check wiring");
        while (1) { //Eventually replace with error
        delay(10); //Infinite loop catches Magnetometer not initized 
        };
    } else {
        Serial.println("Magnetometer Detected");
    }

    if (!this->alt_.begin_I2C()) {
        Serial.println("Altimeter not found at default address... double check wiring");
        while (1) { //Eventually replace with error
        delay(10); //Infinite loop catches Magnetometer not initized 
        };
    } else {
        Serial.println("Altimeter Detected");
    }

    // Checks for GPS Lock
    this->gps_.begin(9600); //Initializes gps communication on provided serial

    //Checks to see if we can read a message. This doesn't mean we have a fix
    unsigned long timeout = 10 * this->seconds; //Will check gps for 5 seconds to see if we recieve a test message
    unsigned long startGPSTest = micros();
    bool gpsCheckoutBool = false;
    Serial.println ("Checking for GPS Message . . .");
    while (micros() - startGPSTest < timeout && !gpsCheckoutBool) { //Checks for response without a fix
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
        Serial.println("GPS Not Detected, double check wiring");
        while (1) { //Eventually replace with error
            delay(10); //Infinite loop catches Magnetometer not initized 
        };
    }

    Serial.println("Sensor Start-Up completed");
    

}

void Sensors :: setUpSensors(std::array<double,3> magHardIron, std::array<double,9> magSoftIron, 
                            std::array<double,9> rotIMU2Body, std::array<double,9> rotMag2Body) {

    // ---Set up IMU ----
    Serial.println("Setting up IMU");
    this->setIMUDataRange(icm20649_accel_range_t::ICM20649_ACCEL_RANGE_16_G, icm20649_gyro_range_t::ICM20649_GYRO_RANGE_2000_DPS);
    this->setIMUUpdateRate(10); //102.3 Hz
    this->rotIMU2Body_ = rotIMU2Body;
  
    // --- Set up Magnetometer  ---
    Serial.println("Setting up Magnetometer");
    this->setMagUpdateRate(lis2mdl_rate_t::LIS2MDL_RATE_50_HZ); 
    this->setMagCalibration(magHardIron, magSoftIron);
    this->rotMag2Body_ = rotMag2Body;
    // --- Set up Altimeter ---
    Serial.println("Setting up Altimeter");
    this->setAltPressureOversampling(BMP3_OVERSAMPLING_8X); //Datasheet recommended for drones to oversample 8x
    this->setAltUpdateRate(BMP3_ODR_50_HZ); //Datasheet recommended for drones to have ODR at 50Hz
    this->setAltFilterCoefficent(BMP3_IIR_FILTER_COEFF_3); //Datasheet recommended for drones to have IIR filter bit be 2 (Coeff 3, see table 4.3.21)
    
    // --- Set up GPS ---
    Serial.println("Setting up GPS");
    this->setGPSNMEAFormat(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Formats to output RMC (lat,long,SOG,COG) + GGA (Altitude, sat id, etc). Options found in Adafruit_PMTK.h
    this->setGPSNMEAUpdateRate(PMTK_SET_NMEA_UPDATE_1HZ); //1 Hz
    this->setGPSPositionUpdateRate(PMTK_API_SET_FIX_CTL_1HZ); //1Hz
    this->setGPSBaudRate(PMTK_SET_BAUD_9600); //9600 Bits per second limit. Should match with above 
}

void Sensors::finalCheckOut() {

}

void Sensors::calibrateSensors() {
    // Use this to calibrate...
    // IMU --> Start up biases for accelerometer / gyro
    // Altimeter --> Starting altitude for reference
    // GPS --> Starting Latitude and Longitude. 

    // Open .csv to save down IMU and Altimeter data
    File imuCalFile = SD.open("imu_calibration.csv", FILE_WRITE); //imu .csv
    if (!imuCalFile) {
        Serial.println("Failed to open IMU Calibration file");
        return;
    }

    File altCalFile = SD.open("alt_calibration.csv", FILE_WRITE); //Altimeter .csv
    if (!altCalFile) {
        Serial.println("Failed to open Altimeter Calibration file");
        return;
    }

    File gpsCalFile = SD.open("gps_calibration.csv", FILE_WRITE); //GPS .csv
    if (!altCalFile) {
        Serial.println("Failed to open GPS Calibration file");
        return;
    }

    // Write header line to files
    imuCalFile.println("t,ax,ay,az,gx,gy,gz"); 
    altCalFile.println("t,h"); 
    gpsCalFile.println("t,lat,long,sog,cog"); 

    unsigned long next_print = 0; // Keeps track of when to print out progress
    //Static calibrations (get offsets)
    Serial.println("Starting Static Calibrations ...");
    unsigned long static_calibration_start = micros();
    unsigned long static_calibration_now = micros();
    unsigned long time_in_static_calibration = static_calibration_now - static_calibration_start;
    std::array<double,6> imuMeasSum {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // For IMU Bias
    double altMeasSum {0.0}; // For reference Height
    std::array<double,2> gpsMeasSum {0.0, 0.0}; // For reference Lat/Long
    int numIMUMeas = 0; 
    int numAltMeas = 0;
    int numGPSMeas = 0;

    //Collect Data for 60 Seconds
    // ~ 6000 IMU data
    // ~ 3000 Altimeter data
    // ~ 60 GPS data
    std::array<double,6> imuMeas;
    double altMeas;
    std::array<double,4> gpsMeas;

    while (time_in_static_calibration < 60 * this->seconds) {
        imuMeas = getIMUMeas(static_calibration_now);
        altMeas = getAltMeas(static_calibration_now);
        gpsMeas = getGPSMeas(static_calibration_now);
        //This if statement is unncessary for IMU but keep here just in case
        if (!isnan(imuMeas[0])) { //Only need to check the first index since they should all be NaN
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
        }
        if (!isnan(altMeas)) { 
            altMeasSum += altMeas;
            numAltMeas += 1;

            //Write to file
            altCalFile.print(time_in_static_calibration);
            altCalFile.print(",");
            altCalFile.println(altMeasSum,6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
        }
        if (!isnan(gpsMeas[0])) {
            //Only need to sum up Lat/Long. We know speed is zero.
            gpsMeasSum[0] += gpsMeas[0];
            gpsMeasSum[1] += gpsMeas[1]; 
            numGPSMeas += 1;

            //Write to file
            //Write to file
            gpsCalFile.print(time_in_static_calibration);
            gpsCalFile.print(",");
            gpsCalFile.print(gpsMeas[0],6); //Print 6 decimals. Can be up to 10 bits (-XX.XXXXXX), including symbols 
            gpsCalFile.print(",");
            gpsCalFile.print(gpsMeas[1],6);
            gpsCalFile.print(",");
            gpsCalFile.print(gpsMeas[2],6); 
            gpsCalFile.print(",");
            gpsCalFile.println(gpsMeas[3],6); 
        }

        //Run this at the fastest rate of sensors, so ~100Hz for IMU
        delayMicroseconds(this->freqIMU_);
        static_calibration_now = micros();
        time_in_static_calibration = static_calibration_now - static_calibration_start;

        if (time_in_static_calibration >= next_print) {
            Serial.print("Static Calibration: ");
            Serial.print(time_in_static_calibration / (this->seconds));
            Serial.print(" out of ");
            Serial.print(60);
            Serial.println(" seconds");
            next_print += 10*this->seconds;
        }
    };
    imuCalFile.close(); //Close File
    altCalFile.close(); 
    gpsCalFile.close(); 
    this -> setIMUCalibration(imuMeasSum, numIMUMeas);
    this -> setAltCalibration(altMeasSum, numAltMeas);
    this -> setGPSCalibration(gpsMeasSum, numGPSMeas);

};

bool Sensors::finalArmingCheck() {
    bool calIMU = true;
    bool calMag = true;
    bool calGPS = true;

    //Can make this more efficent, but focus on readability since this is during the setup loop
    //------Check IMU--------
    for (unsigned int i = 0; i<this->accelBias.size(); i++) {
        if (isnan(this->accelBias[i]) || isnan(this->gyroBias[i])) {
            calIMU = false;
            break;
        }
    }

    //------Check Magnetometer--------
    for (unsigned int i = 0; i<this->magHardIron.size(); i++) {
        if (isnan(this->magHardIron[i])) {
            calMag = false;
            break;
        }
    }
    for (unsigned int i = 0; i<this->magSoftIron.size(); i++) {
        if (isnan(this->magSoftIron[i])) {
            calMag = false;
            break;
        }
    }

    //------Check GPS / Altimeter--------
    for (unsigned int i = 0; i<this->referenceECEFPosition.size(); i++) { 
        //If this is not nan, that means we successfully set the refence altitude along with lat/long, so no need for seperate altimeter check
        if (isnan(this->referenceECEFPosition[i])) { 
            calGPS = false;
            break;
        }
    }

    for (unsigned int i = 0; i<this->ECEF2NED.size(); i++) { 
        if (isnan(this->ECEF2NED[i])) { 
            calGPS = false;
            break;
        }
    }

    //Only need to do the final checkout if we calibrated the sensors correctly. 
    if (calIMU && calMag && calGPS) {
        this->finalCheckOut();
        this -> calibratedSensors = true; //Set flag to true, ready to use calibrated measurements from sensors

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

void Sensors::setIMUCalibration(std::array<double,6>& imuData, int numIMUMeas) {
    //Get IMU Bias
    this -> accelBias = {imuData[0]/numIMUMeas,imuData[1]/numIMUMeas,imuData[2]/numIMUMeas};
    this -> gyroBias = {imuData[3]/numIMUMeas,imuData[4]/numIMUMeas,imuData[5]/numIMUMeas};

    //Do some other checks in here if necessary
    Serial.print("IMU Calibrated Successfully with ");
    Serial.print(numIMUMeas);
    Serial.println(" data.");
    Serial.print("B_ax : ");
    Serial.print(this->accelBias[0]);
    Serial.println(" m/s^2");
    Serial.print("B_ay : ");
    Serial.print(this->accelBias[1]);
    Serial.println(" m/s^2");
    Serial.print("B_az : ");
    Serial.print(this->accelBias[2]);
    Serial.println(" m/s^2");
    Serial.print("B_gx : ");
    Serial.print(this->gyroBias[0]);
    Serial.println(" /s^2");
    Serial.print("B_gy : ");
    Serial.print(this->gyroBias[1]);
    Serial.println(" m/s^2");
    Serial.print("B_gz : ");
    Serial.print(this->gyroBias[2]);
    Serial.println(" m/s^2");
}
std::array<double,6> Sensors::getIMUMeas(unsigned long now) {
    std::array<double,6> IMUMeas;
    if (now - this->lastIMU_ >= this->freqIMU_) {

        //Get IMU Reading
        sensors_event_t imu_a, imu_g, imu_temp; //Don't do anything with temp yet
        imu_.getEvent(&imu_a, &imu_g, &imu_temp);

        IMUMeas[0] = imu_a.acceleration.x;
        IMUMeas[1] = imu_a.acceleration.y;
        IMUMeas[2] = imu_a.acceleration.z;
        IMUMeas[3] = imu_g.gyro.x;
        IMUMeas[4] = imu_g.gyro.y;
        IMUMeas[5] = imu_g.gyro.z;

        if (this->calibratedSensors) {
            IMUMeas[0] -=  this->accelBias[0];
            IMUMeas[1] -=  this->accelBias[1];
            IMUMeas[2] -=  this->accelBias[2];
            IMUMeas[3] -= this->gyroBias[0];
            IMUMeas[4] -= this->gyroBias[1];
            IMUMeas[5] -= this->gyroBias[2];
        }

        this->lastIMU_ = now;
    } else {
        IMUMeas = {NAN, NAN, NAN, NAN, NAN, NAN};
    };
    return IMUMeas;
}



// -------------------------- Magnetometer Functions ------------------------------
void Sensors::setMagUpdateRate(lis2mdl_rate_t magRate) {
    // Header file sets odr for you by specifying lis2mdl_rate_t
    mag_.setDataRate(magRate);
};

void Sensors::setMagCalibration(std::array<double,3>& hardIronCalibration, std::array<double,9> & softIronCalibration) {
    this -> magHardIron = hardIronCalibration;
    this -> magSoftIron = softIronCalibration;
};

void Sensors::calibrateMagnetometer() {
    // This should be done off-board. Collect data, run Least Squares, then input calibrated values.
    // Values shouldn't change much if environment / magnetic interferance doesn't change. 
    std::vector<std::array<double,3>> magMeasCollection; 

    unsigned long dynamic_calibration_start = micros();
    unsigned long dynamic_calibration_now = micros();
    //bool mag_calibration_quality_check = false;
    
    // Open Magnetometer Calibration File
    File magCalFile = SD.open("mag_calibration.csv", FILE_WRITE);
    if (!magCalFile) {
        Serial.println("Failed to open Magnetometer Calibration file");
        return;
    }
    magCalFile.println("time, mx, my, mz"); //Header
    Serial.println("Starting Dynamic Calibrations ...");
    unsigned long time_in_dynamic_calibration = dynamic_calibration_now - dynamic_calibration_start;
    unsigned long next_print = 0; // Keeps track of when to print out progress
    while (time_in_dynamic_calibration < 60 * this->seconds) {

        std::array<double,3> magMeas = this->getMagMeas(dynamic_calibration_now);

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

        delayMicroseconds(this->freqMag_); //Run this at frequency of Magnetometer since its the only sensor here
        dynamic_calibration_now = micros();
        time_in_dynamic_calibration = dynamic_calibration_now - dynamic_calibration_start;
        if (time_in_dynamic_calibration >= next_print) {
            Serial.print("Dynamic Calibration: ");
            Serial.print(time_in_dynamic_calibration / this->seconds);
            Serial.print(" out of ");
            Serial.print(60);
            Serial.println(" seconds");
            next_print += 10*this->seconds;
        }
    };
    magCalFile.close(); //Close File
    Serial.println("Finished Collecting Data. Please unplug and perform post processing . . . ");
    while (1); //Infinite loop here. Shouldn't proceed if calibrating mangeometer.
    
}

std::array<double,3> Sensors::getMagMeas(unsigned long now) {
    std::array<double,3> magMeas;
    if (now - this -> lastMag_ >= this -> freqMag_) {

        //Get Magnetometer Reading
        sensors_event_t mag_m; //Don't do anything with temp yet
        mag_.getEvent(&mag_m);

        magMeas[0] = mag_m.magnetic.x;
        magMeas[1] = mag_m.magnetic.y;
        magMeas[2] = mag_m.magnetic.z;


        if (this->calibratedSensors) {
            // Apply Soft/Hard Iron Calibration!
            // Initialize with 0 bias and identity, since calibration happens first.
        }

        this -> lastMag_ = now;
    } else {
        magMeas = {NAN, NAN, NAN};
    };
    return magMeas;
}


// -------------------------- Altimeter Functions ------------------------------
void Sensors::setAltPressureOversampling(uint8_t altOversamplingFactor) {
    this->alt_.setPressureOversampling(altOversamplingFactor);
    this->alt_.setTemperatureOversampling(BMP3_NO_OVERSAMPLING); //Hardcode to no oversample for temperature
};

void Sensors::setAltFilterCoefficent(uint8_t altFilterCoeff) {
    this ->alt_.setIIRFilterCoeff(altFilterCoeff);
};

void Sensors::setAltUpdateRate(uint8_t altRate) {
    this -> alt_.setOutputDataRate(altRate);
};

void Sensors::setAltCalibration(double altDataSum, int numAltMeas) {
    //Get IMU Bias
    this -> referenceAltitude = altDataSum / numAltMeas;

    //Do some other checks in here if necessary
    Serial.print("Altimeter Calibrated Successfully with ");
    Serial.print(numAltMeas);
    Serial.println(" data.");
    Serial.print("Reference Height : ");
    Serial.print(this->referenceAltitude);
    Serial.println(" m");
}

double Sensors::getAltMeas(unsigned long now) {
    double AltMeas;
    if (now - this -> lastAlt_ >= this -> freqAlt_) {

        //Get Altimeter Reading
        AltMeas = this->alt_.readAltitude(this-> seaLevelPressure);
        this -> lastAlt_ = now;
    } else {
        AltMeas =  NAN;
    };
    return AltMeas;
}

// -------------------------- GPS Functions ------------------------------
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
    double sinLat = sin(latitude * this->deg2rad);
    double cosLat = cos(latitude * this->deg2rad);
    double cosLong = cos(longitude * this->deg2rad);
    double sinLong = sin(longitude * this->deg2rad);
    //Radius of curvature in the prime vertical, N= a/sqrt(1-e^2*sin(lat)^2)
    double N = this->WGS84_a / sqrt(1-this->WGS84_e2 * sinLat*sinLat); 

    double x = (N + altitude) * cosLat;
    double z = (N * (1-this->WGS84_e2) + altitude) * sinLat;

    return std::array<double,3> {x * cosLong, x * sinLong, z};
};

std::array<double,3> Sensors::LLA2NED(double latitude, double longitude, double altitude) {
    // Get ECEF position
    std::array<double,3> r_ECEF = this->LLA2ECEF(latitude, longitude, altitude);

    // Get Delta ECEF
    std::array<double,3> del_r_ECEF{r_ECEF[0] - this->referenceECEFPosition[0],
                                    r_ECEF[1] - this->referenceECEFPosition[1],
                                    r_ECEF[2] - this->referenceECEFPosition[2]};

    //Rotate into NED, will replace this with RotationMatrix
    std::array<double,3> NED {this->ECEF2NED[0] * del_r_ECEF[0] + this->ECEF2NED[1] * del_r_ECEF[1] + this->ECEF2NED[2] * del_r_ECEF[2],
                              this->ECEF2NED[3] * del_r_ECEF[0] + this->ECEF2NED[4] * del_r_ECEF[1] + this->ECEF2NED[5] * del_r_ECEF[2],
                              this->ECEF2NED[6] * del_r_ECEF[0] + this->ECEF2NED[7] * del_r_ECEF[1] + this->ECEF2NED[8] * del_r_ECEF[2]};
    
    return NED;
};

void Sensors::setGPSCalibration(std::array<double,2>& gpsDataSum, int numGPSMeas) {
    //Set ref Lat/Long
    this->referenceLatitude = gpsDataSum[0] / numGPSMeas;
    this->referenceLongitude = gpsDataSum[1] / numGPSMeas;

    //Set ref ECEF Position
    this->referenceECEFPosition = this->LLA2ECEF(this->referenceLatitude, this->referenceLongitude, this->referenceAltitude);

    //Set ECEF2NED
    //hardcode the formula R2(-pi/2)R2(-Lat)R3(Long) = R2(-Lat-pi/2)R3(Long) [Passive convention]
    // Lat --> phi, Long --> theta
    double sp = sin(this->referenceLatitude * this->deg2rad);
    double cp = cos(this->referenceLatitude * this->deg2rad);
    double st = sin(this->referenceLongitude * this->deg2rad);
    double ct= cos(this->referenceLongitude * this->deg2rad);
    this->ECEF2NED = {-sp * ct, -sp*st, cp,
                      -st,      ct,     0,
                      -cp*ct,   -cp*st, -sp};

    Serial.print("GPS Calibrated Successfully with ");
    Serial.print(numGPSMeas);
    Serial.println(" data.");
    Serial.println("Reference ECEF Position : ");
    Serial.print("X : ");
    Serial.print(this->referenceECEFPosition[0]); 
    Serial.println(" m");
    Serial.print("Y : ");
    Serial.print(this->referenceECEFPosition[1]); 
    Serial.println(" m");
    Serial.print("Z : ");
    Serial.print(this->referenceECEFPosition[2]); 
    Serial.println(" m");
    Serial.print("Reference Latitude : ");
    Serial.print(this->referenceLatitude);
    Serial.println(" deg");
    Serial.print("Reference Longitude : ");
    Serial.print(this->referenceLongitude);
    Serial.println(" deg");

}


std::array<double,4> Sensors::getGPSMeas(unsigned long now) {
    std::array<double,4> gpsMeas;
    if (now - this -> lastGPS_ >= this -> freqGPS_) {
        

        while(this -> gps_.read()); //Reads from serial buffer until there is no characters left, returns 0 when all characters read

        // Check for sentence
        if (this-> gps_.newNMEAreceived()) {
            this->gps_.parse(this->gps_.lastNMEA());
        } else {
            //No new message, don't update lastGPS_ and re-check next cycle.
            gpsMeas = {NAN, NAN, NAN,NAN};
            return gpsMeas;
        }

        double v_N = this->gps_.speed * cos(this->gps_.angle * this->deg2rad);
        double v_E = this->gps_.speed * sin(this->gps_.angle * this->deg2rad);

        //Check which type of measurement we want to return.
        if (this->calibratedSensors) {
            std::array<double,3> gpsNED = this->LLA2NED(this-> gps_.latitudeDegrees,  //Note that adafruit handles N/S and E/W by returning +/-
                                                        this-> gps_.longitudeDegrees,
                                                        this-> gps_.altitude);
            // Only return NE position and velocity for Kalman filter
            gpsMeas = {gpsNED[0], gpsNED[1], v_N,v_E};
        } else {
            //If not calibrated, we are in the process of calibrating so return the lattitude and longitude.
            // Will take altitude from altimeter which is more accurate. 
            // Don't need velocity for calibration.
            gpsMeas = {this-> gps_.latitudeDegrees, this-> gps_.longitudeDegrees, v_N,v_E};
        }

        this -> lastGPS_ = now;
        
    } else {
        gpsMeas = {NAN, NAN, NAN, NAN};
    };
    return gpsMeas;
};