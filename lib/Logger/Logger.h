#ifndef _LOGGER_H
#define _LOGGER_H

#include <RingBuffer.h>
#include <DataTypes.h>
#include <Constants.h>
#include <SD.h> //SD card

// This class will handle the different RingBuffers for the various Data Types

// Template logger class based on buffer sizes
template <
    uint16_t IMU_N, 
    uint16_t MAG_N, 
    uint16_t ALT_N, 
    uint16_t GPS_N, 
    uint16_t GNC_N
>
class Logger {

public:
  Logger(float logFlushFrequency, float logIMUDataFrequency, float logMagDataFrequency, float logAltDataFrequency, float logGPSDataFrequency, float logGNCDataFrequency,
    RingBuffer<imuData, IMU_N>& imuBuffer, RingBuffer<tiltData, IMU_N>& tiltBuffer, RingBuffer<magData, MAG_N>& magBuffer ,RingBuffer<altData,ALT_N>& altBuffer ,RingBuffer<gpsData, GPS_N>& gpsBuffer,
    RingBuffer<eskfStateData, GNC_N>& eskfStateBuffer, RingBuffer<eskfCovarianceData, GNC_N>& eskfCovarianceBuffer, RingBuffer<controlData, GNC_N>& controlBuffer):
    imuBuffer_(imuBuffer), tiltBuffer_(tiltBuffer), magBuffer_(magBuffer), altBuffer_(altBuffer), gpsBuffer_(gpsBuffer), eskfStateBuffer_(eskfStateBuffer), eskfCovarianceBuffer_(eskfCovarianceBuffer), controlBuffer_(controlBuffer)
  {
    freqFlush_ = CONSTANTS::seconds2micro/logFlushFrequency; 
    freqIMU_ = CONSTANTS::seconds2micro/logIMUDataFrequency; 
    freqMag_ = CONSTANTS::seconds2micro/logMagDataFrequency; 
    freqAlt_ = CONSTANTS::seconds2micro/logAltDataFrequency; 
    freqGPS_ = CONSTANTS::seconds2micro/logGPSDataFrequency; 
    freqGNC_ = CONSTANTS::seconds2micro/logGNCDataFrequency; 
  }


  void begin() {
    SD.begin();

    //Make folder
    if (!SD.exists("FLIGHT_DATA")) {
      SD.mkdir("FLIGHT_DATA");
    }

    imuFile_ = SD.open("FLIGHT_DATA/imu.csv", FILE_WRITE);
    tiltFile_ = SD.open("FLIGHT_DATA/tilt.csv", FILE_WRITE);
    magFile_ = SD.open("FLIGHT_DATA/mag.csv", FILE_WRITE);
    altFile_ = SD.open("FLIGHT_DATA/alt.csv", FILE_WRITE);
    gpsFile_ = SD.open("FLIGHT_DATA/gps.csv", FILE_WRITE);
    eskfStateFile_ = SD.open("FLIGHT_DATA/eskfState.csv", FILE_WRITE);
    eskfCovarianceFile_ = SD.open("FLIGHT_DATA/eskfCovariance.csv", FILE_WRITE);
    controlFile_ = SD.open("FLIGHT_DATA/control.csv", FILE_WRITE);

    // Write Headers
    imuFile_.println("t,aX,aY,aZ,gX,gY,gZ");
    tiltFile_.println("t,vecX,vecX,vecZ,nisX,nisY,nisZ");
    magFile_.println("t,mX,mY,mZ,vecX,vecY,vecZ,nisX,nisY,nisZ");
    altFile_.println("t,p,h,nis");
    gpsFile_.println("t,px,py,vx,vy,nisPX,nisPY,nisVX,nisVY");
    eskfStateFile_.println("t,pX,pY,pZ,vX,vY,vZ,qW,qX,qY,qZ,wX,wY,wZ,baX,baY,baZ,bgX,bgY,bgZ,bmX,bmY,bmZ");
    eskfCovarianceFile_.println("t,P1,P2,P3,P4,P5,P6,P7,P8,P9,P10,P11,P12,P13,P14,P15,P16,P17,P18");
    controlFile_.println("t,u1,u2,u3,u4,m1PWM,m2PWM,m3PWM,m4PWM");
  };

  void logIMU(const uint32_t now, const imuData imuSample, const tiltData tiltSample) {
    if ( (now - lastIMU_) >= freqIMU_) {
      imuBuffer_.push(imuSample);

      if (tiltSample.tiltUsed) {
        tiltBuffer_.push(tiltSample);
      }

      lastIMU_ = now;
    }
  }


  void logMag(const uint32_t now, const magData magSample) {
    if ((now - lastMag_) >= freqMag_) {
      magBuffer_.push(magSample);
      lastMag_ = now;
    }
  }

  void logAlt(const uint32_t now, const altData altSample) {
    if ((now - lastAlt_) >= freqAlt_) {
      altBuffer_.push(altSample);
      lastAlt_ = now;
    }
  }

  void logGPS(const uint32_t now, const gpsData gpsSample) {
    if ((now - lastGPS_) >= freqGPS_) {
      gpsBuffer_.push(gpsSample);
      lastGPS_ = now;
    }
  }


  //No need to log covariance. Use NIS, or use sensor outputs to reconstruct
  void logGNC(const uint32_t now, const float currentTime,
              const Vector3f& p, const Vector3f& v, const Quaternion& q, const Vector3f& w, 
              const std::array<int,4>& motorPWM, 
              const std::array<float,4>& controlCMD
              ) {
    if ((now - lastGNC_) >= freqGNC_) {
      eskfStateData eskfStateSample (currentTime, p, v, q, w);
      eskfStateBuffer_.push(eskfStateSample);

      //guidanceData guidSample
      controlData controlSample(currentTime, controlCMD, motorPWM);
      controlBuffer_.push(controlSample);
      // GuidBuffer.push(target state);
      // ControlBuffer.push(commanded control);

      lastGNC_ = now;
    }
  }

  //Version to log covariance
  void logGNC(const uint32_t now, const float currentTime,
              const Vector3f& p, const Vector3f& v, const Quaternion& q, const Vector3f& w,  const std::array<float,9>& diagCov, 
              const std::array<int,4>& motorPWM, 
              const std::array<float,4>& controlCMD
              ) {
    if ((now - lastGNC_) >= freqGNC_) {
      eskfStateData eskfStateSample (currentTime, p, v, q, w);
      eskfCovarianceData eskfCovarianceSample (currentTime, diagCov);
      eskfStateBuffer_.push(eskfStateSample);
      eskfCovarianceBuffer_.push(eskfCovarianceSample);



      //guidanceData guidSample
      controlData controlSample(currentTime, controlCMD, motorPWM);
      controlBuffer_.push(controlSample);
      // GuidBuffer.push(target state);
      // ControlBuffer.push(commanded control);


      lastGNC_ = now;
    }
  }


  void flushIMU() {
    imuData imuSample;
    static char imuCSVBlock [this->imuCSVSize_];
    // Keep track of current positions inside csvblocks
    uint16_t imuPosition = 0;

    //Gets the latest imuSample and increments the tail
    while (this->imuBuffer_.pop(imuSample)) { 
          // Write to imuCSV (increments imuPosition by number of characters written by snprintf)
          imuPosition += snprintf(imuCSVBlock + imuPosition, //Write starting at the current end
                                  this->imuCSVSize_ - imuPosition, //Number of characters left. Ensures that the characters being written is LESS than this value.
                                  "%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", //6 decimals for time, ax, ay, az, gx, gy, gz
                                  imuSample.time, imuSample.ax, imuSample.ay, imuSample.az, imuSample.gx, imuSample.gy, imuSample.gz);
    }

    //Write to IMU file
    imuFile_.write((uint8_t*)imuCSVBlock, imuPosition); //Write needs pointer to uint8_t data, and size of bytes
    imuFile_.flush();
  }

  void flushTilt() {
    tiltData tiltSample;
    static char tiltCSVBlock [this->tiltCSVSize_];
    // Keep track of current positions inside csvblocks
    uint16_t tiltPosition = 0;

    //Gets the latest imuSample and increments the tail
    while (this->tiltBuffer_.pop(tiltSample)) { 
          // Write to imuCSV (increments imuPosition by number of characters written by snprintf)
          tiltPosition += snprintf(tiltCSVBlock + tiltPosition, //Write starting at the current end
                                  this->tiltCSVSize_ - tiltPosition, //Number of characters left. Ensures that the characters being written is LESS than this value.
                                  "%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", //6 decimals for time, ax, ay, az, gx, gy, gz
                                  tiltSample.time, tiltSample.orientingVector_x, tiltSample.orientingVector_y, tiltSample.orientingVector_z, tiltSample.nis_x, tiltSample.nis_y, tiltSample.nis_z);
    }

    //Write to Tilt file
    tiltFile_.write((uint8_t*)tiltCSVBlock, tiltPosition); //Write needs pointer to uint8_t data, and size of bytes
    tiltFile_.flush();
  }


  void flushMag() {
    magData magSample;
    static char magCSVBlock [this->magCSVSize_]; 
    // Keep track of current positions inside csvblocks
    uint16_t magPosition = 0;

    //Gets the latest imuSample and increments the tail
    while (this->magBuffer_.pop(magSample)) { 
          magPosition += snprintf(magCSVBlock + magPosition, 
                                this->magCSVSize_ - magPosition, 
                                "%.4f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", //6 decimals for time, mx, my, mz
                                magSample.time, magSample.mx, magSample.my, magSample.mz, magSample.orientingVector_x, magSample.orientingVector_y, magSample.orientingVector_z, magSample.nis_x, magSample.nis_y, magSample.nis_z);
    }

    //Write to Mag file
    magFile_.write((uint8_t*)magCSVBlock, magPosition); //Write needs pointer to uint8_t data, and size of bytes
    magFile_.flush();
  }

  void flushAlt() {
    altData altSample;
    static char altCSVBlock [this->altCSVSize_]; 
    // Keep track of current positions inside csvblocks
    uint16_t altPosition = 0;

    //Gets the latest imuSample and increments the tail
    while (this->altBuffer_.pop(altSample)) { 
      altPosition += snprintf(altCSVBlock + altPosition, 
                            this->altCSVSize_ - altPosition, 
                            "%.4f,%.6f,%.6f,%.6f\n", //6 decimals for time, pressure, height
                            altSample.time, altSample.pressure, altSample.height, altSample.nis);
    }

    //Write to Alt file
    altFile_.write((uint8_t*)altCSVBlock, altPosition); //Write needs pointer to uint8_t data, and size of bytes
    altFile_.flush();
  }

  // void flushGPS() {
  //   gpsData gpsSample;
  //   int gpsCSVSize = 5 * 11 * freqGPS_/freqFlush_;
  //   // Assume worst case: num_floats * 11 char *  sensor_freq / logger_freq (or logger_freq / sensor_freq if in seconds)
  //   char gpsCSVBlock [gpsCSVSize]; // 5 * 11  ~= 75 (Since this is slower than logger, each time logger is called there will only be at most 1 sample of GPS)
  //   // Keep track of current positions inside csvblocks
  //   uint16_t gpsPosition = 0;

  //   // TODO:: Log RAW GPS measurements. Figure out what this is. Lat long + SOG/COG + num satellites? Just the raw NMEA string?
  //   while (this->gpsBuffer_.pop(gpsSample)) { 
  //     gpsPosition += snprintf(gpsCSVBlock + gpsPosition, 
  //                           gpsCSVSize - gpsPosition, 
  //                           "%.4f,%.6f,%.6f,%.6f\n" //6 decimals for time, lat,long, 
  //                           gpsSample.time, gpsSample.h);
  //   }

  //   //Write to GPS file
  //   gpsFile_.write((uint8_t*)gpsCSVBlock, gpsPosition); //Write needs pointer to uint8_t data, and size of bytes
  //   gpsFile_.flush();
  // }

  void flushESKF() {
    eskfStateData eskfStateSample;
    static char eskfStateCSVBlock [this->eskfStateCSVSize_]; 
    // Keep track of current positions inside csvblocks
    uint16_t eskfStatePosition = 0;

    //Gets the latest imuSample and increments the tail
    while (this->eskfStateBuffer_.pop(eskfStateSample)) { 
      eskfStatePosition += snprintf(eskfStateCSVBlock + eskfStatePosition, 
                            this->eskfStateCSVSize_ - eskfStatePosition, 
                            "%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", 
                            eskfStateSample.time, 
                            eskfStateSample.px,eskfStateSample.py, eskfStateSample.pz,
                            eskfStateSample.vx, eskfStateSample.vy, eskfStateSample.vz,
                            eskfStateSample.qw, eskfStateSample.qx, eskfStateSample.qy, eskfStateSample.qz,
                            eskfStateSample.wx, eskfStateSample.wy, eskfStateSample.wz,
                            eskfStateSample.ba_x, eskfStateSample.ba_y, eskfStateSample.ba_z,
                            eskfStateSample.bg_x, eskfStateSample.bg_y, eskfStateSample.bg_z,
                            eskfStateSample.bm_x, eskfStateSample.bm_y, eskfStateSample.bm_z);
    }
    eskfStateFile_.write((uint8_t*)eskfStateCSVBlock, eskfStatePosition); //Write needs pointer to uint8_t data, and size of bytes
    eskfStateFile_.flush();

    // eskfCovarianceData eskfCovarianceSample;
    // static constexpr size_t eskfCovarianceCSVSize = 6000;
    // static char eskfCovarianceCSVBlock [eskfCovarianceCSVSize];
    // uint16_t eskfCovariancePosition = 0;
     
    // while (this->eskfCovarianceBuffer_.pop(eskfCovarianceSample)) { 
    //   eskfCovariancePosition += snprintf(eskfCovarianceCSVBlock + eskfCovariancePosition, 
    //                         eskfCovarianceCSVSize - eskfCovariancePosition, 
    //                         "%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", 
    //                         eskfStateSample.time, 
    //                         eskfCovarianceSample.P1,eskfCovarianceSample.P2,eskfCovarianceSample.P3,
    //                         eskfCovarianceSample.P4,eskfCovarianceSample.P5,eskfCovarianceSample.P6,
    //                         eskfCovarianceSample.P7,eskfCovarianceSample.P8,eskfCovarianceSample.P9,
    //                         eskfCovarianceSample.P10,eskfCovarianceSample.P11,eskfCovarianceSample.P12,
    //                         eskfCovarianceSample.P13,eskfCovarianceSample.P14,eskfCovarianceSample.P15,
    //                         eskfCovarianceSample.P16,eskfCovarianceSample.P17,eskfCovarianceSample.P18 );
    // }
    // eskfCovarianceFile_.write((uint8_t*)eskfCovarianceCSVBlock, eskfCovariancePosition); //Write needs pointer to uint8_t data, and size of bytes
    // eskfCovarianceFile_.flush();

  }

void flushControl() {
    controlData controlSample;

    static char controlCSVBlock [this->controlCSVSize_]; 
    // Keep track of current positions inside csvblocks
    uint16_t controlPosition = 0;

    while (this->controlBuffer_.pop(controlSample)) { 
      controlPosition += snprintf(controlCSVBlock + controlPosition, 
                            this->controlCSVSize_ - controlPosition, 
                            "%.4f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%d\n", 
                            controlSample.time, 
                            controlSample.Ft, controlSample.Mx, controlSample.My, controlSample.Mz,
                            controlSample.PWM1, controlSample.PWM2, controlSample.PWM3, controlSample.PWM4);
    }
    controlFile_.write((uint8_t*)controlCSVBlock, controlPosition); //Write needs pointer to uint8_t data, and size of bytes
    controlFile_.flush();
  }


  //Note: Push data to the buffers directly outside. Logger will have access to the buffer via reference.
  void flush(uint32_t now){
    if (this->lastLog_ == UINT32_MAX) { //Initialize lastLog_ the first time this is called
      this->lastLog_ = now;
    } else { //If lastLog_ is initialized...
      if ((now - this->lastLog_) >= this->freqFlush_){
        //Break them up into functions to have more ram on the stack
        flushIMU();
        flushTilt();
        flushMag();
        flushAlt();
        flushESKF();
        flushControl();

        //flushGPS();

        this->lastLog_ = now;
      } ;
    };
  };


private: 
  //Frequencies (in micro-seconds)
  uint32_t freqFlush_; 

  //Used to preallocate csv blocks
  uint32_t freqIMU_;
  uint32_t freqMag_;
  uint32_t freqAlt_;
  uint32_t freqGPS_;
  uint32_t freqGNC_;


  //Buffers
  RingBuffer<imuData, IMU_N>& imuBuffer_;
  RingBuffer<tiltData, IMU_N>& tiltBuffer_;
  RingBuffer<magData,MAG_N>& magBuffer_;
  RingBuffer<altData,ALT_N>& altBuffer_;
  RingBuffer<gpsData,GPS_N>& gpsBuffer_;
  RingBuffer<eskfStateData,GNC_N>& eskfStateBuffer_;
  RingBuffer<eskfCovarianceData,GNC_N>& eskfCovarianceBuffer_;
  RingBuffer<controlData, GNC_N>& controlBuffer_;

  // Files
  File imuFile_;
  File tiltFile_;
  File magFile_;
  File altFile_;
  File gpsFile_;
  File eskfStateFile_;
  File eskfCovarianceFile_;
  File controlFile_;

  //CSV BLock Sizes (Putting down here for eaiser access)
   // # inputs * 11 * Freq Sensor Log / Freq Flush 
  static constexpr size_t imuCSVSize_ = 2000; // 7 * 11 * 200/10 = 1540 characters
  static constexpr size_t tiltCSVSize_ = 2000; // 7 * 11 * 200/10 = 1540 characters
  static constexpr size_t magCSVSize_ = 1000; // 10 * 11 * 50/10 = 550 characters 
  static constexpr size_t altCSVSize_ = 200; // 4 * 11 * 25/10 = 110 characters 
  static constexpr size_t gpsCSVSize_ = 100; // 5 * 11 * 1/10 = 5.5 characters, Only need enough to fit 1 line
  static constexpr size_t eskfStateCSVSize_ = 6000; // 23 * 11 * 200/10 = 5060 characters,
  static constexpr size_t eskfCovarianceCSVSize_ = 4500; //UNUSED for now since we are not logging covariance. 18 * 11 * 200/10 = 3960 characters
  static constexpr size_t controlCSVSize_ = 2500; // 9 * 11 * 200/10 = 1980 characters

  //Timer related 
  uint32_t lastLog_ = 0;
  uint32_t lastIMU_ = 0;
  uint32_t lastMag_ = 0;
  uint32_t lastAlt_ = 0;
  uint32_t lastGPS_ = 0;
  uint32_t lastGNC_ = 0;
};

#endif