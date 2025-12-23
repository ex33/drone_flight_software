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
    RingBuffer<imuData, IMU_N>& imuBuffer , RingBuffer<magData, MAG_N>& magBuffer ,RingBuffer<altData,ALT_N>& altBuffer ,RingBuffer<gpsData, GPS_N>& gpsBuffer ,RingBuffer<eskfStateData, ESKF_N>& eskfStateBuffer):
    imuBuffer_(imuBuffer), magBuffer_(magBuffer), altBuffer_(altBuffer), gpsBuffer_(gpsBuffer), eskfStateBuffer_(eskfStateBuffer)
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
    magFile_ = SD.open("FLIGHT_DATA/mag.csv", FILE_WRITE);
    altFile_ = SD.open("FLIGHT_DATA/alt.csv", FILE_WRITE);
    gpsFile_ = SD.open("FLIGHT_DATA/gps.csv", FILE_WRITE);
    eskfFile_ = SD.open("FLIGHT_DATA/eskf.csv", FILE_WRITE);

    // Write Headers
    imuFile_.println("t_us,ax,ay,az,gx,gy,gz");
    magFile_.println("t_us,mx,my,mz");
    altFile_.println("t_us,p,h");
    gpsFile_.println("t_us,px,py,vx,vy");
    eskfFile_.println("t_us,px,py,pz,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz,bax,bay,baz,bgx,bgy,bgz,bmx,bmy,bmz");
  };

  void logIMU(const uint32_t now, const imuData imuSample) {
    if (static_cast<float>(now - lastIMU_) >= freqIMU_) {
      imuBuffer_.push(imuSample);
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
              const Vector3f& p, const Vector3f& v, const Quaternion& q, const Vector3f& w, const Vector3f& ba, const Vector3f& bg,const Vector3f& bm) {
    if (static_cast<float> (now - lastGNC_) >= freqGNC_) {
      eskfStateData eskfSample (currentTime, p, v, q, w, ba, bg, bm);
      //guidanceData guidSample
      // controlData controlSample
      // GuidBuffer.push(target state);
      // ControlBuffer.push(commanded control);

      eskfStateBuffer_.push(eskfSample);

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
                                  imuSample.t_us, imuSample.ax, imuSample.ay, imuSample.az, imuSample.gx, imuSample.gy, imuSample.gz);
    }

    //Write to IMU file
    imuFile_.write((uint8_t*)imuCSVBlock, imuPosition); //Write needs pointer to uint8_t data, and size of bytes
    imuFile_.flush();
  }

  void flushMag() {
    magData magSample;
    static constexpr size_t magCSVSize = 750;
    // Assume worst case: num_floats * 11 char *  sensor_freq / logger_freq (or logger_freq / sensor_freq if in seconds)
    static char magCSVBlock [magCSVSize]; // 4 * 11 * 25/2 ~= 500 characters
    // Keep track of current positions inside csvblocks
    uint16_t magPosition = 0;

    //Gets the latest imuSample and increments the tail
    while (this->magBuffer_.pop(magSample)) { 
          magPosition += snprintf(magCSVBlock + magPosition, 
                                magCSVSize -magPosition, 
                                "%.3f,%.6f,%.6f,%.6f\n", //6 decimals for time, mx, my, mz
                                magSample.t_us, magSample.mx, magSample.my, magSample.mz);
    }

    //Write to Mag file
    magFile_.write((uint8_t*)magCSVBlock, magPosition); //Write needs pointer to uint8_t data, and size of bytes
    magFile_.flush();
  }

  void flushAlt() {
    altData altSample;
    static constexpr size_t altCSVSize = 750; //Give extra margin. 
    // Assume worst case: num_floats * 11 char *  sensor_freq / logger_freq (or logger_freq / sensor_freq if in seconds)
    static char altCSVBlock [altCSVSize]; // 2 * 11  * 25/2 ~= 300 characters
    // Keep track of current positions inside csvblocks
    uint16_t altPosition = 0;

    //Gets the latest imuSample and increments the tail
    while (this->altBuffer_.pop(altSample)) { 
      altPosition += snprintf(altCSVBlock + altPosition, 
                            altCSVSize - altPosition, 
                            "%.3f,%.6f,%.6f\n", //6 decimals for time, pressure, height
                            altSample.t_us, altSample.pressure, altSample.height);
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
  //                           gpsSample.t_us, gpsSample.h);
  //   }

  //   //Write to GPS file
  //   gpsFile_.write((uint8_t*)gpsCSVBlock, gpsPosition); //Write needs pointer to uint8_t data, and size of bytes
  //   gpsFile_.flush();
  // }

  void flushESKF() {
    eskfStateData eskfSample;
    static constexpr size_t eskfCSVSize = 1500;
    // Assume worst case: num_floats * 11 char *  sensor_freq / logger_freq (or logger_freq / sensor_freq if in seconds)
    static char eskfCSVBlock [eskfCSVSize]; // 23 * 11 * 10/2 ~= 1265 characters (log this SLOWER or at controller frequency since we just need enough to reconstruct trajectory)
    // Keep track of current positions inside csvblocks
    uint16_t eskfPosition = 0;

    //Gets the latest imuSample and increments the tail
    while (this->eskfStateBuffer_.pop(eskfSample)) { 
      eskfPosition += snprintf(eskfCSVBlock + eskfPosition, 
                            eskfCSVSize - eskfPosition, 
                            "%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", 
                            eskfSample.t_us, 
                            eskfSample.px,eskfSample.py, eskfSample.pz,
                            eskfSample.vx, eskfSample.vy, eskfSample.vz,
                            eskfSample.qw, eskfSample.qx, eskfSample.qy, eskfSample.qz,
                            eskfSample.wx, eskfSample.wy, eskfSample.wz,
                            eskfSample.ba_x, eskfSample.ba_y, eskfSample.ba_z,
                            eskfSample.bg_x, eskfSample.bg_y, eskfSample.bg_z,
                            eskfSample.bm_x, eskfSample.bm_y, eskfSample.bm_z);
    }

    //Write to ESKF file
    // Might need a seperate file for covariance
    eskfFile_.write((uint8_t*)eskfCSVBlock, eskfPosition); //Write needs pointer to uint8_t data, and size of bytes
    eskfFile_.flush();
  }


  //Note: Push data to the buffers directly outside. Logger will have access to the buffer via reference.
  void flush(uint32_t now){
    if (this->lastLog_ == UINT32_MAX) { //Initialize lastLog_ the first time this is called
      this->lastLog_ = now;
    } else { //If lastLog_ is initialized...
      if (static_cast<float>(now - this->lastLog_) >= this->freqFlush_){
        //Break them up into functions to have more ram on the stack
        flushIMU();
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
  RingBuffer<magData,MAG_N>& magBuffer_;
  RingBuffer<altData,ALT_N>& altBuffer_;
  RingBuffer<gpsData,GPS_N>& gpsBuffer_;
  RingBuffer<eskfStateData,ESKF_N>& eskfStateBuffer_;

  // Files
  File imuFile_;
  File magFile_;
  File altFile_;
  File gpsFile_;
  File eskfFile_;


  //Timer related 
  uint32_t lastLog_ = 0;
  uint32_t lastIMU_ = 0;
  uint32_t lastMag_ = 0;
  uint32_t lastAlt_ = 0;
  uint32_t lastGPS_ = 0;
  uint32_t lastGNC_ = 0;
};

#endif