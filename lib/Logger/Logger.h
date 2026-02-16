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

    //imuFile_ = SD.open("FLIGHT_DATA/imu.bin", FILE_WRITE);
    tiltFile_ = SD.open("FLIGHT_DATA/tilt.bin", FILE_WRITE);
    magFile_ = SD.open("FLIGHT_DATA/mag.bin", FILE_WRITE);
    altFile_ = SD.open("FLIGHT_DATA/alt.bin", FILE_WRITE);
    gpsFile_ = SD.open("FLIGHT_DATA/gps.bin", FILE_WRITE);
    eskfStateFile_ = SD.open("FLIGHT_DATA/eskfState.bin", FILE_WRITE);
    eskfCovarianceFile_ = SD.open("FLIGHT_DATA/eskfCovariance.bin", FILE_WRITE);
    controlFile_ = SD.open("FLIGHT_DATA/control.bin", FILE_WRITE);

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
  void logGNC(const uint32_t now, 
              const Vector3f& p, const Vector3f& v, const Quaternion& q, const Vector3f& w, 
              const std::array<int,4>& motorPWM, 
              const std::array<float,4>& controlCMD
              ) {
    if ((now - lastGNC_) >= freqGNC_) {
      eskfStateData eskfStateSample (now, p, v, q, w);
      eskfStateBuffer_.push(eskfStateSample);

      //guidanceData guidSample
      controlData controlSample(now, controlCMD, motorPWM);
      controlBuffer_.push(controlSample);
      // GuidBuffer.push(target state);
      // ControlBuffer.push(commanded control);

      lastGNC_ = now;
    }
  }

  //Version to log covariance
  void logGNC(const uint32_t now, 
              const Vector3f& p, const Vector3f& v, const Quaternion& q, const Vector3f& w,  const std::array<float,9>& diagCov, 
              const std::array<int,4>& motorPWM, 
              const std::array<float,4>& controlCMD
              ) {
    if ((now - lastGNC_) >= freqGNC_) {
      eskfStateData eskfStateSample (now, p, v, q, w);
      eskfCovarianceData eskfCovarianceSample (now, diagCov);
      eskfStateBuffer_.push(eskfStateSample);
      eskfCovarianceBuffer_.push(eskfCovarianceSample);



      //guidanceData guidSample
      controlData controlSample(now, controlCMD, motorPWM);
      controlBuffer_.push(controlSample);
      // GuidBuffer.push(target state);
      // ControlBuffer.push(commanded control);


      lastGNC_ = now;
    }
  }


  void flushIMU() {
    size_t remaining = imuBuffer_.size();
    //uint32_t start = micros();
    while (remaining > 0) { //While there is still data in the buffer, equivalent to tail != head
    uint16_t chunkSize = imuBuffer_.chunkSize(); //Get the size of the chunk starting at tail}

    imuFile_.write(
      (uint8_t*)imuBuffer_.tailPtr(), //Write starting at the tail pointer
      chunkSize * sizeof(imuData) // chunkSize is the number of samples in the chunk, sizeof(altData) is the size of each sample in bytes. This ensures that we are writing the correct number of bytes to the file.
    );
    imuBuffer_.advanceTail(chunkSize); // Move tail to end of chunk we just wrote
    remaining -=chunkSize;
  }
  // uint32_t dt = micros() - start;
  // Serial.println(dt);
  
  imuFile_.flush();
  };

  void flushTilt() {

    while (tiltBuffer_.size() > 0) { //While there is still data in the buffer, equivalent to tail != head
      uint16_t chunkSize = tiltBuffer_.chunkSize(); //Get the size of the chunk starting at tail}

      tiltFile_.write(
        (uint8_t*)tiltBuffer_.tailPtr(), //Write starting at the tail pointer
        chunkSize * sizeof(tiltData) // chunkSize is the number of samples in the chunk, sizeof(altData) is the size of each sample in bytes. This ensures that we are writing the correct number of bytes to the file.
      );
      tiltBuffer_.advanceTail(chunkSize); // Move tail to end of chunk we just wrote
    }

    tiltFile_.flush();
  }


  void flushMag() {

    while (magBuffer_.size() > 0) { //While there is still data in the buffer, equivalent to tail != head
      uint16_t chunkSize = magBuffer_.chunkSize(); //Get the size of the chunk starting at tail}

      magFile_.write(
        (uint8_t*)magBuffer_.tailPtr(), //Write starting at the tail pointer
        chunkSize * sizeof(magData) // chunkSize is the number of samples in the chunk, sizeof(altData) is the size of each sample in bytes. This ensures that we are writing the correct number of bytes to the file.
      );
      magBuffer_.advanceTail(chunkSize); // Move tail to end of chunk we just wrote
    }

    magFile_.flush();
  }

  void flushAlt() {
    //Write as RAW Binary data for faster write and less SD card wear. Can convert to CSV later offline.
    while (altBuffer_.size() > 0) { //While there is still data in the buffer, equivalent to tail != head
      uint16_t chunkSize = altBuffer_.chunkSize(); //Get the size of the chunk starting at tail}

      altFile_.write(
        (uint8_t*)altBuffer_.tailPtr(), //Write starting at the tail pointer
        chunkSize * sizeof(altData) // chunkSize is the number of samples in the chunk, sizeof(altData) is the size of each sample in bytes. This ensures that we are writing the correct number of bytes to the file.
      );
      altBuffer_.advanceTail(chunkSize); // Move tail to end of chunk we just wrote
    }

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
    //Write as RAW Binary data for faster write and less SD card wear. Can convert to CSV later offline.
    while (eskfStateBuffer_.size() > 0) { //While there is still data in the buffer, equivalent to tail != head
      uint16_t chunkSize = eskfStateBuffer_.chunkSize(); //Get the size of the chunk starting at tail}

      eskfStateFile_.write(
        (uint8_t*)eskfStateBuffer_.tailPtr(), //Write starting at the tail pointer
        chunkSize * sizeof(eskfStateData) // chunkSize is the number of samples in the chunk, sizeof(eskfStateData) is the size of each sample in bytes. This ensures that we are writing the correct number of bytes to the file.
      );
      eskfStateBuffer_.advanceTail(chunkSize); // Move tail to end of chunk we just wrote
    }
    eskfStateFile_.flush();

  }

void flushControl() {
   while (controlBuffer_.size() > 0) { //While there is still data in the buffer, equivalent to tail != head
      uint16_t chunkSize = controlBuffer_.chunkSize(); //Get the size of the chunk starting at tail}

      controlFile_.write(
        (uint8_t*)controlBuffer_.tailPtr(), //Write starting at the tail pointer
        chunkSize * sizeof(controlData) // chunkSize is the number of samples in the chunk, sizeof(altData) is the size of each sample in bytes. This ensures that we are writing the correct number of bytes to the file.
      );
      controlBuffer_.advanceTail(chunkSize); // Move tail to end of chunk we just wrote
    }

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