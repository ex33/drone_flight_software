#ifndef _MATHPK_H
#define _MATHPK_H

#include "Quaternion.h"
#include "Rotation.h"
#include "Vector3f.h"
#include <array>
#include <cmath>
#include "Matrix9f.h"
#include "Matrix18f.h"
#include <cstdint> //Gives uint32_t. Basically all modules imports Mathpk so put this here for convinence
//============== Vector3f ===================
inline float dot(const Vector3f& a, const Vector3f& b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

inline Vector3f cross(const Vector3f& a, const Vector3f& b) {
    return Vector3f(std::array<float,3>{ a[1]*b[2] - a[2]*b[1],
                                            a[2]*b[0] - a[0]*b[2],
                                            a[0]*b[1] - a[1]*b[0]});
}
inline Vector3f operator*(const float scalar, const Vector3f& v) {
    return v * scalar;
};


//============== Quaternion ===================
inline Quaternion operator*(float scalar, const Quaternion& q) { //Allows for scalar*q
    return q * scalar;  // calls the member operator
}
inline Quaternion operator/(float scalar, const Quaternion& q) { //Allows for scalar/q
    return q / scalar;  // calls the member operator
}

inline Quaternion quatMult(const Quaternion& p, const Quaternion& q) {
    float pw = p.w();
    float px = p.x();
    float py = p.y();
    float pz = p.z();

    float qw = q.w();
    float qx = q.x();
    float qy = q.y();
    float qz = q.z();

    // Formula is...
    // p c-times q = [pw * qw - dot(p_xyz, q_xyz); qw * p_xyz + pw * q_xyz + cross(p_xyz, q_xyz)]
    // See Wie. Bong
    // Used to apply quaternion p to quatenrion q
    
    float w = pw * qw - (px*qx + py*qy + pz*qz);
    float x = qw*px + pw*qx + (py*qz - pz*qy); // cross product x
    float y = qw*py + pw*qy + (pz*qx - px*qz); // cross product y
    float z = qw*pz + pw*qz + (px*qy - py*qx); // cross product z

    return Quaternion(w,x,y,z);
};


inline Quaternion quatProp(const Vector3f& w, const Quaternion& q) {
    float wx = w[0];
    float wy = w[1];
    float wz = w[2];

    float qw = q.w();
    float qx = q.x();
    float qy = q.y();
    float qz = q.z();

    // Formula is...
    // p c-dot q = [pw * qw - dot(p_xyz, q_xyz); qw * p_xyz + pw * q_xyz - cross(p_xyz, q_xyz)]
    // Used to propagate quaternion q by w
    float scalar =  -(wx*qx + wy*qy + wz*qz);
    float x = qw*wx  - (wy*qz - wz*qy); // cross product x
    float y = qw*wy  - (wz*qx - wx*qz); // cross product y
    float z = qw*wz  - (wx*qy - wy*qx); // cross product z

    return Quaternion(scalar,x,y,z);
}


//============== Rotation ===================

//Quaternion Initialization. Follows Passive convention.
//See Wie, Bong (5.36) and Markley, Crassidis (2.125)
inline Rotation q2R(const Quaternion& q) {
    float qx = q.x();
    float qy = q.y();
    float qz = q.z();
    float qw = q.w();

    Rotation R (std::array<float,9>{1 - 2*(qy*qy + qz*qz), 2*(qx*qy + qw*qz), 2*(qx*qz - qw*qy),
                                    2*(qx*qy - qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz + qw*qx),
                                    2*(qx*qz + qw*qy), 2*(qy*qz - qw*qx), 1-2*(qx*qx+qy*qy) });
    return R;
}

//Passive rotations arounds the 3 axis
inline Rotation R1(float angle) {
    return Rotation(std::array<float,9> {1 , 0 , 0,
                                        0, cosf(angle) , sinf(angle),
                                        0, -sinf(angle), cosf(angle)});
}
inline Rotation R2(float angle) {
    return Rotation(std::array<float,9> {cosf(angle) , 0 , -sinf(angle),
                                          0, 1 , 0,
                                          sinf(angle),0,cosf(angle) });
}
inline Rotation R3(float angle) {
    return Rotation(std::array<float,9> {cosf(angle) , sinf(angle) , 0,
                                        -sinf(angle), cosf(angle) , 0,
                                        0,0,1 });
}

inline Rotation skew(const Vector3f& vec) {
    return Rotation(std::array<float,9> {0.0f, -vec[2], vec[1],
                                         vec[2], 0.0f, -vec[0],
                                        -vec[1], vec[0], 0.0f});
}



#endif