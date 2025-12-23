#ifndef _ROTATION_H
#define _ROTATION_H

#include "Quaternion.h"
#include "Vector3f.h"
#include <array>
class Rotation {
private: 
    std::array<float,9> elements_; //Row-Major storage, (row1, row2, row3)

public:
    Rotation(){ //Default constructor to identity
        elements_ = std::array<float,9> {1,0,0,0,1,0,0,0,1};
    };
    Rotation(std::array<float,9> elements): elements_(elements) {};
    Rotation(float x11, float x12, float x13, float x21, float x22, float x23 ,float x31, float x32 ,float x33): elements_(std::array<float,9> {x11, x12, x13, x21, x22, x23, x31, x32, x33}) {};


    // Access (row,col)
    float& operator()(int row, int col) {  //Lets you modify the matrix directly
        return elements_[row*3 + col]; 
    }
    float  operator()(int row, int col) const { //Lets you read the matrix since this is const
         return elements_[row*3 + col]; 
    }
    std::array<float,9> getElements() {
        return elements_; //Mostly for unit testing
    }

    const float& operator[](unsigned int i) const {
        return elements_[i];
    }
    // Copy assignment operator
    Rotation& operator=(const Rotation& other) {
        elements_ = other.elements_;      // element-wise copy
        return *this; //Returning reference allows chaining, A= B=C
    }
    Rotation& operator=(const std::array<float,9>& otherElements) {
        elements_ = otherElements;      
        return *this; //Returning reference 
    }

    //Matrix specific operations
    Rotation transpose() const { //Const ensures that original matrix doesn't get overidden
        Rotation R_T;
        for (int i = 0; i<3; i++) {
            for (int j=0; j<3; j++){
                R_T(i,j) = (*this)(j,i); //Swap the row and column
            }
        }
        return R_T;
    }

    // Matrix Multiplication
    Rotation operator*(const Rotation& other) const { 
        Rotation multRot;
        for (int i = 0; i < 3; ++i) {       // row of this
            for (int j = 0; j < 3; ++j) {   // column of other
                multRot.elements_[i*3 + j] = 0;
                for (int k = 0; k < 3; ++k) {
                    multRot.elements_[i*3 + j] += elements_[i*3 + k] * other.elements_[k*3 + j];
                }
            }
        }
        return multRot;
    }

    // Vector Multiplication
    Vector3f operator*(const Vector3f& vector) const { 
        Vector3f rotVec;
        for (int i = 0; i < 3; ++i) {
            rotVec[i] = 0;
            for (int j = 0; j < 3; ++j)
                rotVec[i] += elements_[i*3 + j] * vector[j];
        }
        return rotVec;
    }

};
#endif