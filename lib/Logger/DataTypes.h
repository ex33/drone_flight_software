#ifndef _DATATYPES_H
#define _DATATYPES_H

#include <Arduino.h>

// These should be RAW data from sensor. Should have enough information to obtain the calibrated values

//IMU Data
struct imuData {
  uint32_t now; // Microseconds since start of flight. 
  float ax, ay, az; //Accelerometer
  float gx, gy, gz;  //Gyro


  // Default constructor
  imuData() : now(0), ax(0), ay(0), az(0), gx(0), gy(0), gz(0){}


  imuData(const uint32_t t, const std::array<float,6>& imuMeas) {
    this->now = t;
    this->ax = imuMeas[0];
    this->ay = imuMeas[1];
    this->az = imuMeas[2];
    this->gx = imuMeas[3];
    this->gy = imuMeas[4];
    this->gz = imuMeas[5];
  }
};


struct tiltData {
  uint32_t now; // Time elasped in seconds
  uint8_t tiltUsed; 
  float orientingVector_x, orientingVector_y, orientingVector_z; //tilt
  float nis_x, nis_y, nis_z;

  // Default constructor
  tiltData() : now(0), tiltUsed(0), orientingVector_x(0), orientingVector_y(0), orientingVector_z(0), nis_x(0), nis_y(0), nis_z(0) {}


  void setData(const std::array<float,3>& orientingVectorUsed, const std::array<float,3>& NIS){
  this->tiltUsed = 1; 
  this->orientingVector_x = orientingVectorUsed[0];
  this->orientingVector_y = orientingVectorUsed[1];
  this->orientingVector_z = orientingVectorUsed[2];
  this->nis_x = NIS[0];
  this->nis_y = NIS[1];
  this->nis_z = NIS[2];
  };

  void tagData(const uint32_t t) {
    this->now = t;
  };

};

//Magnetometer Data
struct magData {
  uint32_t now; // now elasped in seconds
  float mx, my, mz; //Processed Magnetometer
  float orientingVector_x, orientingVector_y, orientingVector_z; //yaw
  float nis_x, nis_y, nis_z;

  // Default constructor
  magData() : now(0), mx(0), my(0), mz(0), orientingVector_x(0), orientingVector_y(0), orientingVector_z(0), nis_x(0), nis_y(0), nis_z(0) {}



  void setData(const std::array<float,3>& magMeas, const std::array<float,3>& orientingVectorUsed, const std::array<float,3>& NIS) {
    this->mx = magMeas[0];
    this->my = magMeas[1];
    this->mz = magMeas[2];
    this->orientingVector_x = orientingVectorUsed[0];
    this->orientingVector_y = orientingVectorUsed[1];
    this->orientingVector_z = orientingVectorUsed[2];
    this->nis_x = NIS[0];
    this->nis_y = NIS[1];
    this->nis_z = NIS[2];
  }

  void tagData(uint32_t t) {
    this->now = t;
  }

};


// Altiemter Data
struct altData {
  uint32_t now; // now elasped in seconds
  float pressure; 
  float height;
  float nis;
  // Default constructor
  altData() : now(0),pressure(0) , height(0), nis(0) {}

  void setPressureData(const float& p) {
    this->pressure = p;
  }

  void setData(const float& h, const float& NIS) {
    this->height = h;
    this->nis = NIS;
  }

  void tagData(uint32_t t) {
    this->now = t;
  }

};

// GPS Data
struct gpsData {
  uint32_t now;// now elasped in seconds
  float px, py, vx, vy; //Change this to log RAW Measurements
  float nis_px, nis_py, nis_vx, nis_vy;

  // Default constructor
  gpsData() : now(0), px(0), py(0), vx(0), vy(0), nis_px(0), nis_py(0), nis_vx(0), nis_vy(0){}

  void setData(const std::array<float,4>& gpsMeas, const std::array<float,4>& NIS) {
    this->px = gpsMeas[0];
    this->py = gpsMeas[1];
    this->vx = gpsMeas[2];
    this->vy = gpsMeas[3];
    this->nis_px = NIS[0];
    this->nis_py = NIS[1];
    this->nis_vx = NIS[2];
    this->nis_vy = NIS[3];
  }

  void tagData(uint32_t t) {
    this->now = t;
  }


};

// Estimation
struct eskfStateData {
  uint32_t now;
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
      : now(0), px(0), py(0), pz(0),
        vx(0), vy(0), vz(0),
        qw(1), qx(0), qy(0), qz(0),
        wx(0), wy(0), wz(0),
        ba_x(0), ba_y(0), ba_z(0),
        bg_x(0), bg_y(0), bg_z(0),
        bm_x(0), bm_y(0), bm_z(0) {}

    // Constructor from individual states
    eskfStateData(uint32_t t, const Vector3f& p, const Vector3f& v, const Quaternion& q,
             const Vector3f& w, const Vector3f& ba, const Vector3f& bg, const Vector3f& bm)
        : now(t),
          px(p[0]), py(p[1]), pz(p[2]),
          vx(v[0]), vy(v[1]), vz(v[2]),
          qw(q.w()), qx(q.x()), qy(q.y()), qz(q.z()),
          wx(w[0]), wy(w[1]), wz(w[2]),
          ba_x(ba[0]), ba_y(ba[1]), ba_z(ba[2]),
          bg_x(bg[0]), bg_y(bg[1]), bg_z(bg[2]),
          bm_x(bm[0]), bm_y(bm[1]), bm_z(bm[2]) {}
    eskfStateData(uint32_t t, const Vector3f& p, const Vector3f& v, const Quaternion& q,
             const Vector3f& w)
        : now(t),
          px(p[0]), py(p[1]), pz(p[2]),
          vx(v[0]), vy(v[1]), vz(v[2]),
          qw(q.w()), qx(q.x()), qy(q.y()), qz(q.z()),
          wx(w[0]), wy(w[1]), wz(w[2]){}

    void setData(uint32_t t, const Vector3f& p, const Vector3f& v, const Quaternion& q,
                 const Vector3f& w, const Vector3f& ba, const Vector3f& bg, const Vector3f& bm) {
        now = t;
        px = p[0]; py = p[1]; pz = p[2];
        vx = v[0]; vy = v[1]; vz = v[2];
        qw = q.w(); qx = q.x(); qy = q.y(); qz = q.z();
        wx = w[0]; wy = w[1]; wz = w[2];
        ba_x = ba[0]; ba_y = ba[1]; ba_z = ba[2];
        bg_x = bg[0]; bg_y = bg[1]; bg_z = bg[2];
        bm_x = bm[0]; bm_y = bm[1]; bm_z = bm[2];
    }

    void setData(uint32_t t, const Vector3f& p, const Vector3f& v, const Quaternion& q,
                 const Vector3f& w) {
        now = t;
        px = p[0]; py = p[1]; pz = p[2];
        vx = v[0]; vy = v[1]; vz = v[2];
        qw = q.w(); qx = q.x(); qy = q.y(); qz = q.z();
        wx = w[0]; wy = w[1]; wz = w[2];
    }

};

struct eskfCovarianceData {
  uint32_t now; // now elasped in seconds
  // Covariance (Only Save Diagonal)
  float P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15, P16, P17, P18;


  // Default constructor
  eskfCovarianceData()
      : now(0), P1(0), P2(0), P3(0),
        P4(0), P5(0), P6(0),
        P7(0), P8(0), P9(0), 
        P10(0),P11(0), P12(0), 
        P13(0),P14(0), P15(0),
        P16(0),P17(0), P18(0){};
        
    eskfCovarianceData(uint32_t t, const std::array<float,18>& P_diag)
        : now(t),
        P1(P_diag[0]), P2(P_diag[1]), P3(P_diag[2]),
        P4(P_diag[3]), P5(P_diag[4]), P6(P_diag[5]),
        P7(P_diag[6]), P8(P_diag[7]), P9(P_diag[8]), 
        P10(P_diag[9]),P11(P_diag[10]), P12(P_diag[11]), 
        P13(P_diag[12]),P14(P_diag[13]), P15(P_diag[14]),
        P16(P_diag[15]),P17(P_diag[16]), P18(P_diag[17]){};


    eskfCovarianceData(uint32_t t, const std::array<float,9>& P_diag)
        : now(t),
        P1(P_diag[0]), P2(P_diag[1]), P3(P_diag[2]),
        P4(P_diag[3]), P5(P_diag[4]), P6(P_diag[5]),
        P7(P_diag[6]), P8(P_diag[7]), P9(P_diag[8]){};
};


// Motor and Control Data
struct controlData {
  uint32_t now; // now elasped in seconds
  float Ft, Mx, My, Mz; //Control Inputs (Th
  int PWM1, PWM2, PWM3, PWM4;
  // Default constructor
  controlData() : now(0), Ft(0), Mx(0), My(0), Mz(0), PWM1(0), PWM2(0), PWM3(0), PWM4(0) {};

  controlData(uint32_t t, const std::array<float,4>& uCMD, const std::array<int,4>& PWM) 
    : now(t), Ft(uCMD[0]), Mx(uCMD[1]), My(uCMD[2]), Mz(uCMD[3]), PWM1(PWM[0]), PWM2(PWM[1]), PWM3(PWM[2]), PWM4(PWM[3]) {};
};

#endif