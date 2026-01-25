#ifndef _VECTOR3F_H
#define _VECTOR3F_H

#include<array>
#include<cmath>

class Vector3f {
private:
    std::array<float,3> data_;
public:
    //Default constructor
    Vector3f(): data_{0.0, 0.0, 0.0} {};
    Vector3f(std::array<float,3> data): data_(data){};
    Vector3f(float x, float y, float z): data_{x,y,z} {};

    // Copy assignment operator
    Vector3f& operator=(const Vector3f& other) {
        // element-wise copy
        data_ = other.data_;   
        return *this; //Returning reference allows chaining, A= B=C
    }
    Vector3f& operator=(const std::array<float,3>& otherData) {
        data_ = otherData;
        return *this; //Returning reference 
    }
    //Getter
    const float& operator[](unsigned int i) const {
        return data_[i];
    }
    float& operator[](unsigned int i) { //Non const version to modify
        return data_[i];
    }
    //Math Operations
    inline Vector3f operator*(float scalar) const {
        Vector3f result {data_[0]* scalar, data_[1]*scalar, data_[2]* scalar}; //Return a copy, don't want to overwrite our old one
        return result;
    };

    inline Vector3f operator/(float scalar) const {
        Vector3f result {data_[0]/ scalar, data_[1]/scalar, data_[2]/scalar}; //Return a copy, don't want to overwrite our old one
        return result;
    };

    inline Vector3f operator+(const Vector3f& otherVec) const {
        Vector3f result {data_[0] + otherVec.data_[0], data_[1] + otherVec.data_[1], data_[2] + otherVec.data_[2]};
        return result;
    }

    inline Vector3f& operator += (const Vector3f& otherVec) {
        data_[0] += otherVec.data_[0];
        data_[1] += otherVec.data_[1];
        data_[2] += otherVec.data_[2];
        return *this;
    }

    inline Vector3f operator-(const Vector3f& otherVec) const {
        Vector3f result {data_[0] - otherVec.data_[0], data_[1] - otherVec.data_[1], data_[2] - otherVec.data_[2]};
        return result;
    }
    inline Vector3f& operator-=(const Vector3f& otherVec)  {
        data_[0] -= otherVec.data_[0];
        data_[1] -= otherVec.data_[1];
        data_[2] -= otherVec.data_[2];
        return *this;
    }

    float getMag() const {
        return sqrt(data_[0]*data_[0] + data_[1]*data_[1] + data_[2]*data_[2]);
    }

    Vector3f normalize() const {
        float mag = getMag();
        Vector3f result = *this/mag;
        
        return result;
    }

    std::array<float,3> getArray() const {
        return data_;
    }
};


#endif