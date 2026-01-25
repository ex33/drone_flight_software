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
    uint16_t ESKF_N
>
class Logger {

public:
  Logger(float logFlushFrequency, float logIMUDataFrequency, float logMagDataFrequency, float logAltDataFrequency, float logGPSDataFrequency, float logGNCDataFrequency,
    RingBuffer<imuData, IMU_N>& imuBuffer, RingBuffer<tiltData, IMU_N>& tiltBuffer, RingBuffer<magData, MAG_N>& magBuffer ,RingBuffer<altData,ALT_N>& altBuffer ,RingBuffer<gpsData, GPS_N>& gpsBuffer,
    RingBuffer<eskfStateData, ESKF_N>& eskfStateBuffer, RingBuffer<eskfCovarianceData, ESKF_N>& eskfCovarianceBuffer):
    imuBuffer_(imuBuffer), tiltBuffer_(tiltBuffer), magBuffer_(magBuffer), altBuffer_(altBuffer), gpsBuffer_(gpsBuffer), eskfStateBuffer_(eskfStateBuffer), eskfCovarianceBuffer_(eskfCovarianceBuffer)
  {
    freqFlush_ = CONSTANTS::seconds2milli/logFlushFrequency; 
    freqIMU_ = CONSTANTS::seconds2milli/logIMUDataFrequency; 
    freqMag_ = CONSTANTS::seconds2milli/logMagDataFrequency; 
    freqAlt_ = CONSTANTS::seconds2milli/logAltDataFrequency; 
    freqGPS_ = CONSTANTS::seconds2milli/logGPSDataFrequency; 
    freqGNC_ = CONSTANTS::seconds2milli/logGNCDataFrequency; 
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

    // Write Headers
    imuFile_.println("t,aX,aY,aZ,gX,gY,gZ");
    tiltFile_.println("t,vecX,vecX,vecZ,nisX,nisY,nisZ");
    magFile_.println("t,mX,mY,mZ,vecX,vecY,vecZ,nisX,nisY,nisZ");
    altFile_.println("t,p,h,nis");
    gpsFile_.println("t,px,py,vx,vy,nisPX,nisPY,nisVX,nisVY");
    eskfStateFile_.println("t,pX,pY,pZ,vX,vY,vZ,qW,qX,qY,qZ,wX,wY,wZ,baX,baY,baZ,bgX,bgY,bgZ,bmX,bmY,bmZ");
    eskfCovarianceFile_.println("t,P1,P2,P3,P4,P5,P6,P7,P8,P9,P10,P11,P12,P13,P14,P15,P16,P17,P18");
  };

  void logIMU(const uint32_t now, const imuData imuSample, const tiltData tiltSample) {
    if (static_cast<float>(now - lastIMU_) >= freqIMU_) {
      imuBuffer_.push(imuSample);

      if (tiltSample.tiltUsed) {
        tiltBuffer_.push(tiltSample);
      }

      lastIMU_ = now;
    }
  }


  void logMag(const uint32_t now, const magData magSample) {
    if (static_cast<float> (now - lastMag_) >= freqMag_) {
      magBuffer_.push(magSample);
      lastMag_ = now;
    }
  }

  void logAlt(const uint32_t now, const altData altSample) {
    if (static_cast<float> (now - lastAlt_) >= freqAlt_) {
      altBuffer_.push(altSample);
      lastAlt_ = now;
    }
  }

  void logGPS(const uint32_t now, const gpsData gpsSample) {
    if (static_cast<float> (now - lastGPS_) >= freqGPS_) {
      gpsBuffer_.push(gpsSample);
      lastGPS_ = now;
    }
  }

  //Since Nav doesn't have its own loop, and logging GNC all at same frequency, just directly pass values into here, form the samples locally, and push to buffer
  void logGNC(const uint32_t now, const float currentTime,
              const Vector3f& p, const Vector3f& v, const Quaternion& q, const Vector3f& w, const Vector3f& ba, const Vector3f& bg,const Vector3f& bm, const std::array<float,18>& diagCov) {
    if (static_cast<float> (now - lastGNC_) >= freqGNC_) {
      eskfStateData eskfStateSample (currentTime, p, v, q, w, ba, bg, bm);
      eskfCovarianceData eskfCovarianceSample (currentTime, diagCov);
      eskfStateBuffer_.push(eskfStateSample);
      eskfCovarianceBuffer_.push(eskfCovarianceSample);
      //guidanceData guidSample
      // controlData controlSample
      // GuidBuffer.push(target state);
      // ControlBuffer.push(commanded control);


      lastGNC_ = now;
    }
  }

  void logGNC(const uint32_t now, const float currentTime,
              const Vector3f& p, const Vector3f& v, const Quaternion& q, const Vector3f& w,  const std::array<float,9>& diagCov) {
    if (static_cast<float> (now - lastGNC_) >= freqGNC_) {
      eskfStateData eskfStateSample (currentTime, p, v, q, w);
      eskfCovarianceData eskfCovarianceSample (currentTime, diagCov);
      eskfStateBuffer_.push(eskfStateSample);
      eskfCovarianceBuffer_.push(eskfCovarianceSample);
      //guidanceData guidSample
      // controlData controlSample
      // GuidBuffer.push(target state);
      // ControlBuffer.push(commanded control);


      lastGNC_ = now;
    }
  }


  void flushIMU() {
    imuData imuSample;
    static constexpr size_t imuCSVSize = 1250; //Give extra margin. 
    // Assume worst case: num_floats * 11 char *  sensor_freq / logger_freq (or logger_freq / sensor_freq if in seconds)
    static char imuCSVBlock [imuCSVSize]; // 7 * 11 * 25/2 ~= 1000 characters
    // Keep track of current positions inside csvblocks
    uint16_t imuPosition = 0;

    //Gets the latest imuSample and increments the tail
    while (this->imuBuffer_.pop(imuSample)) { 
          // Write to imuCSV (increments imuPosition by number of characters written by snprintf)
          imuPosition += snprintf(imuCSVBlock + imuPosition, //Write starting at the current end
                                  imuCSVSize -imuPosition, //Number of characters left. Ensures that the characters being written is LESS than this value.
                                  "%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", //6 decimals for time, ax, ay, az, gx, gy, gz
                                  imuSample.time, imuSample.ax, imuSample.ay, imuSample.az, imuSample.gx, imuSample.gy, imuSample.gz);
    }

    //Write to IMU file
    imuFile_.write((uint8_t*)imuCSVBlock, imuPosition); //Write needs pointer to uint8_t data, and size of bytes
    imuFile_.flush();
  }

  void flushTilt() {
    tiltData tiltSample;
    static constexpr size_t tiltCSVSize = 1250; //Give extra margin. 
    // Assume worst case: num_floats * 11 char *  sensor_freq / logger_freq (or logger_freq / sensor_freq if in seconds)
    static char tiltCSVBlock [tiltCSVSize]; // 7 * 11 * 25/2 ~= 1000 characters
    // Keep track of current positions inside csvblocks
    uint16_t tiltPosition = 0;

    //Gets the latest imuSample and increments the tail
    while (this->tiltBuffer_.pop(tiltSample)) { 
          // Write to imuCSV (increments imuPosition by number of characters written by snprintf)
          tiltPosition += snprintf(tiltCSVBlock + tiltPosition, //Write starting at the current end
                                  tiltCSVSize - tiltPosition, //Number of characters left. Ensures that the characters being written is LESS than this value.
                                  "%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", //6 decimals for time, ax, ay, az, gx, gy, gz
                                  tiltSample.time, tiltSample.orientingVector_x, tiltSample.orientingVector_y, tiltSample.orientingVector_z, tiltSample.nis_x, tiltSample.nis_y, tiltSample.nis_z);
    }

    //Write to Tilt file
    tiltFile_.write((uint8_t*)tiltCSVBlock, tiltPosition); //Write needs pointer to uint8_t data, and size of bytes
    tiltFile_.flush();
  }


  void flushMag() {
    magData magSample;
    static constexpr size_t magCSVSize = 1500;
    // Assume worst case: num_floats * 11 char *  sensor_freq / logger_freq (or logger_freq / sensor_freq if in seconds)
    static char magCSVBlock [magCSVSize]; // 10 * 11 * 25/2 ~= 500 characters
    // Keep track of current positions inside csvblocks
    uint16_t magPosition = 0;

    //Gets the latest imuSample and increments the tail
    while (this->magBuffer_.pop(magSample)) { 
          magPosition += snprintf(magCSVBlock + magPosition, 
                                magCSVSize -magPosition, 
                                "%.3f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", //6 decimals for time, mx, my, mz
                                magSample.time, magSample.mx, magSample.my, magSample.mz, magSample.orientingVector_x, magSample.orientingVector_y, magSample.orientingVector_z, magSample.nis_x, magSample.nis_y, magSample.nis_z);
    }

    //Write to Mag file
    magFile_.write((uint8_t*)magCSVBlock, magPosition); //Write needs pointer to uint8_t data, and size of bytes
    magFile_.flush();
  }

  void flushAlt() {
    altData altSample;
    static constexpr size_t altCSVSize = 750; //Give extra margin. 
    // Assume worst case: num_floats * 11 char *  sensor_freq / logger_freq (or logger_freq / sensor_freq if in seconds)
    static char altCSVBlock [altCSVSize]; // 4 * 11  * 25/2 ~= 300 characters
    // Keep track of current positions inside csvblocks
    uint16_t altPosition = 0;

    //Gets the latest imuSample and increments the tail
    while (this->altBuffer_.pop(altSample)) { 
      altPosition += snprintf(altCSVBlock + altPosition, 
                            altCSVSize - altPosition, 
                            "%.3f,%.6f,%.6f,%.6f\n", //6 decimals for time, pressure, height
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
  //                           "%.3f,%.6f,%.6f,%.6f\n" //6 decimals for time, lat,long, 
  //                           gpsSample.time, gpsSample.h);
  //   }

  //   //Write to GPS file
  //   gpsFile_.write((uint8_t*)gpsCSVBlock, gpsPosition); //Write needs pointer to uint8_t data, and size of bytes
  //   gpsFile_.flush();
  // }

  void flushESKF() {
    eskfStateData eskfStateSample;
    eskfCovarianceData eskfCovarianceSample;
    static constexpr size_t eskfStateCSVSize = 1500;
    static constexpr size_t eskfCovarianceCSVSize = 1250;
    // Assume worst case: num_floats * 11 char *  sensor_freq / logger_freq (or logger_freq / sensor_freq if in seconds)
    static char eskfStateCSVBlock [eskfStateCSVSize]; // 23 * 11 * 10/2 ~= 1265 characters (log this SLOWER or at controller frequency since we just need enough to reconstruct trajectory)
    static char eskfCovarianceCSVBlock [eskfCovarianceCSVSize];
    // Keep track of current positions inside csvblocks
    uint16_t eskfStatePosition = 0;
    uint16_t eskfCovariancePosition = 0;

    //Gets the latest imuSample and increments the tail
    while (this->eskfStateBuffer_.pop(eskfStateSample)) { 
      eskfStatePosition += snprintf(eskfStateCSVBlock + eskfStatePosition, 
                            eskfStateCSVSize - eskfStatePosition, 
                            "%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", 
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

    while (this->eskfCovarianceBuffer_.pop(eskfCovarianceSample)) { 
      eskfCovariancePosition += snprintf(eskfCovarianceCSVBlock + eskfCovariancePosition, 
                            eskfCovarianceCSVSize - eskfCovariancePosition, 
                            "%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", 
                            eskfStateSample.time, 
                            eskfCovarianceSample.P1,eskfCovarianceSample.P2,eskfCovarianceSample.P3,
                            eskfCovarianceSample.P4,eskfCovarianceSample.P5,eskfCovarianceSample.P6,
                            eskfCovarianceSample.P7,eskfCovarianceSample.P8,eskfCovarianceSample.P9,
                            eskfCovarianceSample.P10,eskfCovarianceSample.P11,eskfCovarianceSample.P12,
                            eskfCovarianceSample.P13,eskfCovarianceSample.P14,eskfCovarianceSample.P15,
                            eskfCovarianceSample.P16,eskfCovarianceSample.P17,eskfCovarianceSample.P18 );
    }
    eskfCovarianceFile_.write((uint8_t*)eskfCovarianceCSVBlock, eskfCovariancePosition); //Write needs pointer to uint8_t data, and size of bytes
    eskfCovarianceFile_.flush();

  }


  //Note: Push data to the buffers directly outside. Logger will have access to the buffer via reference.
  void flush(uint32_t now){
    if (this->lastLog_ == UINT32_MAX) { //Initialize lastLog_ the first time this is called
      this->lastLog_ = now;
    } else { //If lastLog_ is initialized...
      if (static_cast<float>(now - this->lastLog_) >= this->freqFlush_){
        //Break them up into functions to have more ram on the stack
        flushIMU();
        flushTilt();
        flushMag();
        flushAlt();
        flushESKF();

        //flushGPS();
      } ;
    };
  };


private: 
  //Frequencies (in milli-seconds)
  float freqFlush_; 

  //Used to preallocate csv blocks
  float freqIMU_;
  float freqMag_;
  float freqAlt_;
  float freqGPS_;
  float freqGNC_;


  //Buffers
  RingBuffer<imuData, IMU_N>& imuBuffer_;
  RingBuffer<tiltData, IMU_N>& tiltBuffer_;
  RingBuffer<magData,MAG_N>& magBuffer_;
  RingBuffer<altData,ALT_N>& altBuffer_;
  RingBuffer<gpsData,GPS_N>& gpsBuffer_;
  RingBuffer<eskfStateData,ESKF_N>& eskfStateBuffer_;
  RingBuffer<eskfCovarianceData,ESKF_N>& eskfCovarianceBuffer_;

  // Files
  File imuFile_;
  File tiltFile_;
  File magFile_;
  File altFile_;
  File gpsFile_;
  File eskfStateFile_;
  File eskfCovarianceFile_;


  //Timer related 
  uint32_t lastLog_ = 0;
  uint32_t lastIMU_ = 0;
  uint32_t lastMag_ = 0;
  uint32_t lastAlt_ = 0;
  uint32_t lastGPS_ = 0;
  uint32_t lastGNC_ = 0;
};

#endif