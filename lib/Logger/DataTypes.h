#ifndef _DATATYPES_H
#define _DATATYPES_H

#include <Arduino.h>

// These should be RAW data from sensor. Should have enough information to obtain the calibrated values

//IMU Data
struct imuData {
  float t_us; // Time elasped in seconds
  float ax, ay, az; //Accelerometer
  float gx, gy, gz;  //Gyro

  // Default constructor
  imuData() : t_us(0), ax(0), ay(0), az(0), gx(0), gy(0), gz(0) {}

  // Constructor from time and array
  imuData(float t, const std::array<float,6>& imuMeas) {
      setData(t, imuMeas);
  }

  void setData(float t, const std::array<float,6>& imuMeas) {
    this->t_us = t;
    this->ax = imuMeas[0];
    this->ay = imuMeas[1];
    this->az = imuMeas[2];
    this->gx = imuMeas[3];
    this->gy = imuMeas[4];
    this->gz = imuMeas[5];
  }
};

//Magnetometer Data
struct magData {
  float t_us; // Time elasped in seconds
  float mx, my, mz; //Magnetometer

  // Default constructor
  magData() : t_us(0), mx(0), my(0), mz(0) {}

  // Constructor from time and array
  magData(float t, const std::array<float,3>& magMeas) {
      setData(t, magMeas);
  }

  void setData(float t, const std::array<float,3>& magMeas) {
    this->t_us = t;
    this->mx = magMeas[0];
    this->my = magMeas[1];
    this->mz = magMeas[2];
  }

};


// Altiemter Data
struct altData {
  float t_us; // Time elasped in seconds
  float pressure; 
  float height;

  // Default constructor
  altData() : t_us(0),pressure(0) , height(0){}

  // Constructor from time and array
  altData(float t, const float& pressureMeas, const float& heightMeas) {
      setData(t, pressureMeas, heightMeas);
  }

  void setData(float t, const float& pressureMeas, const float& heightMeas) {
    this->t_us = t;
    this->pressure = pressureMeas;
    this->height = heightMeas;
  }

};

// GPS Data
struct gpsData {
  float t_us;// Time elasped in seconds
  float px, py, vx, vy; //Change this to log RAW Measurements

  // Default constructor
  gpsData() : t_us(0), px(0), py(0), vx(0), vy(0){}

  //TODO:: Implement after settling on what gpsdata should contain
  // // Constructor from time and array
  // gpsData(float t, const std::array<float,4>& gpsMeas) {
  //     setData(t, imuMeas);
  // }

  // void setData(float t, const std::array<float,6>& imuMeas) {
  //   this->t_us = t;
  //   this->ax = imuMeas[0];
  //   this->ay = imuMeas[1];
  //   this->az = imuMeas[2];
  //   this->gx = imuMeas[3];
  //   this->gy = imuMeas[4];
  //   this->gz = imuMeas[5];
  // }

};

// Estimation
struct eskfStateData {
  float t_us; // Time elasped in seconds
  // Position
  float px, py, pz;
  // Velocity
  float vx, vy, vz;
  // Orientation quaternion
  float qw, qx, qy, qz;
  // Angular rates
  float wx, wy, wz;
  // Accelerometer bias
  float ba_x, ba_y, ba_z;
  // Gyro bias
  float bg_x, bg_y, bg_z;
  // Magnetometer bias
  float bm_x, bm_y, bm_z;

  // Default constructor
  eskfStateData()
      : t_us(0), px(0), py(0), pz(0),
        vx(0), vy(0), vz(0),
        qw(1), qx(0), qy(0), qz(0),
        wx(0), wy(0), wz(0),
        ba_x(0), ba_y(0), ba_z(0),
        bg_x(0), bg_y(0), bg_z(0),
        bm_x(0), bm_y(0), bm_z(0) {}

    // Constructor from individual states
    eskfStateData(float t, const Vector3f& p, const Vector3f& v, const Quaternion& q,
             const Vector3f& w, const Vector3f& ba, const Vector3f& bg, const Vector3f& bm)
        : t_us(t),
          px(p[0]), py(p[1]), pz(p[2]),
          vx(v[0]), vy(v[1]), vz(v[2]),
          qw(q.w()), qx(q.x()), qy(q.y()), qz(q.z()),
          wx(w[0]), wy(w[1]), wz(w[2]),
          ba_x(ba[0]), ba_y(ba[1]), ba_z(ba[2]),
          bg_x(bg[0]), bg_y(bg[1]), bg_z(bg[2]),
          bm_x(bm[0]), bm_y(bm[1]), bm_z(bm[2]) {}

    // Optional: setData function
    void setData(float t, const Vector3f& p, const Vector3f& v, const Quaternion& q,
                 const Vector3f& w, const Vector3f& ba, const Vector3f& bg, const Vector3f& bm) {
        t_us = t;
        px = p[0]; py = p[1]; pz = p[2];
        vx = v[0]; vy = v[1]; vz = v[2];
        qw = q.w(); qx = q.x(); qy = q.y(); qz = q.z();
        wx = w[0]; wy = w[1]; wz = w[2];
        ba_x = ba[0]; ba_y = ba[1]; ba_z = ba[2];
        bg_x = bg[0]; bg_y = bg[1]; bg_z = bg[2];
        bm_x = bm[0]; bm_y = bm[1]; bm_z = bm[2];
    }
};

struct eskfCovarianceData {
  float t_us; // Time elasped in seconds
  // Covariance (Only Save Diagonal)
  float P1, P2, P3;
  // Velocity
  float P4, P5, P6;
  // Orientation 
  float P7, P8, P9;
  // Accelerometer bias
  float  P10, P11, P12;
  // Gyro bias
  float P13, P14, P15;
  // Magnetometer bias
  float P16, P17, P18;

  // NIS Data


  // Innovation (Sensor)

  // Other data to save down.

};

#endif