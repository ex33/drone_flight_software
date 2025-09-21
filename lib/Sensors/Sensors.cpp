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
};


void Sensors::checkoutSensors() {
    //Given default I2C address and I2C bus, check if initilized properly
    if (!imu_.begin_I2C()) { 
        Serial.println("IMU not found at default address... double check wiring");
        while (1) { //Eventually replace with error
        delay(10); //Infinite loop catches IMU not initized 
        };
    } else {
        Serial.println("IMU Detected");
    }

    if (!mag_.begin()) {
        Serial.println("Magnetometer not found at default address... double check wiring");
        while (1) { //Eventually replace with error
        delay(10); //Infinite loop catches Magnetometer not initized 
        };
    } else {
        Serial.println("Magnetometer Detected");
    }

    if (!alt_.begin_I2C()) {
        Serial.println("Altimeter not found at default address... double check wiring");
        while (1) { //Eventually replace with error
        delay(10); //Infinite loop catches Magnetometer not initized 
        };
    } else {
        Serial.println("Altimeter Detected");
    }

    // Checks for GPS Lock
    this->gps_.begin(9600); //Initializes gps communication
    char testGPS = this->gps_.read(); //Get a NMEA sentence
    if (GPS.)

    Serial.println("Sensor checkout completed");

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
    gpsCalFile.println("t,x,y,vx,vy"); 

    int next_print = 0; // Keeps track of when to print out progress
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

    while (time_in_static_calibration < 60 * this->seconds) {
        imuMeas = getIMUMeas(static_calibration_now);
        altMeas = getAltMeas(static_calibration_now);
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

        //Run this at the fastest rate of sensors, so ~100Hz for IMU
        delayMicroseconds(this->freqIMU_);
        static_calibration_now = micros();
        time_in_static_calibration = static_calibration_now - static_calibration_start;

        if (time_in_static_calibration >= next_print) {
            Serial.print("Static Calibration: ");
            Serial.print(time_in_static_calibration / (10*this->seconds));
            Serial.print(" out of ");
            Serial.print(60);
            Serial.println(" seconds");
        }
    };
    imuCalFile.close(); //Close File
    altCalFile.close(); 
    this -> setIMUCalibration(imuMeasSum, numIMUMeas);
    this -> setAltCalibration(altMeasSum, numAltMeas);
    
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

        IMUMeas[0] = imu_a.acceleration.x - this->accelBias[0];
        IMUMeas[1] = imu_a.acceleration.y - this->accelBias[1];
        IMUMeas[2] = imu_a.acceleration.z - this->accelBias[2];
        IMUMeas[3] = imu_g.gyro.x - this->gyroBias[0];
        IMUMeas[4] = imu_g.gyro.y - this->gyroBias[1];
        IMUMeas[5] = imu_g.gyro.z - this->gyroBias[2];

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

        // Apply Soft/Hard Iron Calibration!
        // Initialize with 0 bias and identity, since calibration happens first.

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