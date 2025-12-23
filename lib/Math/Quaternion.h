#ifndef _QUATERNION_H
#define _QUATERNION_H

#include<array>
#include<cmath>
#include"Vector3f.h"
// Hamiltonian convention of scalar first q = [qw; qx; qy; qz]
class Quaternion {
private:
    float w_;
    float x_;
    float y_;
    float z_;
public:
    //Default constructor
    Quaternion() {
        w_ = 1.0;
        x_ = 0.0;
        y_ = 0.0;
        z_ = 0.0;
    };

    Quaternion(float qw, float qx, float qy, float qz) {
        w_ = qw;
        x_ = qx;
        y_ = qy;
        z_ = qz;
    }
    Quaternion(std::array<float,4> quatArray) {
        w_ = quatArray[0];
        x_ = quatArray[1];
        y_ = quatArray[2];
        z_ = quatArray[3];
    }

    Quaternion(float scalar, Vector3f vector) {
        w_ = scalar;
        x_ = vector[0];
        y_ = vector[1];
        z_ = vector[2];
    }

    float w() const {
        return w_;
    }
    float x() const {
        return x_;
    }
    float y() const {
        return y_;
    }
    float z() const {
        return z_;
    }


    //Overload
    inline Quaternion operator*(float scalar) const {
        return Quaternion(w_ * scalar, x_ * scalar, y_*scalar, z_*scalar);
    }
    inline Quaternion operator/(float scalar) const {
        return Quaternion(w_ / scalar, x_ / scalar, y_/scalar, z_/scalar);
    }
    inline Quaternion& operator+=(const Quaternion& other)  { //Mostly for ESKF discrete propagation
        w_ += other.w_; 
        x_ += other.x_; 
        y_ += other.y_;
        z_ += other.z_;
        return *this;
    }

    inline Quaternion operator+(Quaternion other) const { //Mostly for ESKF discrete propagation
        return Quaternion(w_  + other.w_, x_ + other.x_, y_ + other.y_, z_ + other.z_);
    }

    // Const to ensure old quaternion doesn't get overridden unless reassigned
    Quaternion getConjugate() const {
        return Quaternion(w_, -x_, -y_, -z_);
    }


    float getMag() const {
        return sqrt(x_*x_ + y_*y_ + z_*z_ + w_*w_);
    }

    void normalize() { //Can safely let normalize() quaternion overwrite parameters.
        // Normalize Quaterion
        float mag = getMag();
        w_ /= mag;
        x_ /= mag;
        y_ /= mag;
        z_ /= mag;

    }
};


#endif