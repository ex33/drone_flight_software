#ifndef _MATRIX18f_H
#define _MATRIX18f_H


#include <array>
class Matrix18f {
private: 
    std::array<float,324> elements_; //Row-Major storage, (row1, row2, row3,... row18)

public:
    Matrix18f() : elements_{} {}  // all elements = 0
    Matrix18f(std::array<float,324> elements): elements_(elements) {};

    // Access (row,col)
    inline float& operator()(int row, int col) {  //Lets you modify the matrix directly
        return elements_[row*18 + col]; 
    }
    inline float  operator()(int row, int col) const { //Lets you read the matrix since this is const
         return elements_[row*18 + col]; 
    }
    const std::array<float,324>& getElements() const{ //Return by reference
        return elements_; //Mostly for unit testing
    }

    inline const float& operator[](unsigned int i) const {
        return elements_[i];
    }

    // Set matrix to identity
    inline void setIdentity() {
        for (int i = 0; i < 18; ++i) {
            for (int j = 0; j < 18; ++j) {
                elements_[i*18 + j] = (i == j) ? 1.0f : 0.0f;
            }
        }
    }

    // zero out all elements first
    inline void setZero() {
        for(int i=0;i<18;i++)
            for(int j=0;j<18;j++)
                elements_[i*18+j] = 0.0f;
    }

    // Copy assignment operator
    Matrix18f& operator=(const Matrix18f& other) {
        elements_ = other.elements_;      // element-wise copy
        return *this; //Returning reference allows chaining, A= B=C
    }
    Matrix18f& operator=(const std::array<float,324>& otherElements) {
        elements_ = otherElements;      
        return *this; //Returning reference 
    }

    //Matrix specific operations
    Matrix18f transpose() const { //Const ensures that original matrix doesn't get overidden
        Matrix18f R_T;
        for (int i = 0; i<18; i++) {
            for (int j=0; j<18; j++){
                R_T(i,j) = (*this)(j,i); //Swap the row and column
            }
        }
        return R_T;
    }

    //Matrix Addition
    // ----- Addition operator (returns a new matrix) -----
    inline Matrix18f operator+(const Matrix18f& other) const {
        Matrix18f result;
        for (int i = 0; i < 324; ++i) {
            result.elements_[i] = elements_[i] + other.elements_[i];
        }
        return result;
    }

    inline Matrix18f operator-(const Matrix18f& other) const {
        Matrix18f result;
        for (int i = 0; i < 324; ++i) {
            result.elements_[i] = elements_[i] - other.elements_[i];
        }
        return result;
    }

    inline Matrix18f& operator-=(const Matrix18f& other) {
        for (int i = 0; i < 324; ++i) {
            elements_[i] -= other.elements_[i];
        }
        return *this;
    }

    Matrix18f operator*(const Matrix18f& other) const {
        Matrix18f result;

        for (int i = 0; i < 18; ++i) {
            float row[18];
            for (int k = 0; k < 18; ++k) row[k] = elements_[i*18 + k]; // copy row from *this

            for (int j = 0; j < 18; ++j) {
                float col[18];
                for (int k = 0; k < 18; ++k) col[k] = other.elements_[k*18 + j]; // <-- other, not elements_

                float sum = 0.0f;
                for (int k = 0; k < 18; ++k) sum += row[k] * col[k];

                result.elements_[i*18 + j] = sum;
            }
        }
        return result;
    }

    std::array<float, 18> getDiagonal() const {
        std::array<float, 18> diag{};
        for (int i = 0; i < 18; ++i)
            diag[i] = elements_[i * 18 + i];
        return diag;
    }

};
#endif