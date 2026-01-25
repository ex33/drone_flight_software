#ifndef _MATRIX9f_H
#define _MATRIX9f_H


#include <array>
class Matrix9f {
private: 
    std::array<float,81> elements_; //Row-Major storage, (row1, row2, row3,... row18)

public:
    Matrix9f() : elements_{} {}  // all elements = 0
    Matrix9f(std::array<float,81> elements): elements_(elements) {};

    // Access (row,col)
    inline float& operator()(int row, int col) {  //Lets you modify the matrix directly
        return elements_[row*9 + col]; 
    }
    inline float  operator()(int row, int col) const { //Lets you read the matrix since this is const
         return elements_[row*9 + col]; 
    }
    const std::array<float,81>& getElements() const{ //Return by reference
        return elements_; //Mostly for unit testing
    }

    inline const float& operator[](unsigned int i) const {
        return elements_[i];
    }

    // Set matrix to identity
    inline void setIdentity() {
        for (int i = 0; i < 9; ++i) {
            for (int j = 0; j < 9; ++j) {
                elements_[i*9 + j] = (i == j) ? 1.0f : 0.0f;
            }
        }
    }

    // zero out all elements first
    inline void setZero() {
        for(int i=0;i<9;i++)
            for(int j=0;j<9;j++)
                elements_[i*9+j] = 0.0f;
    }

    // Copy assignment operator
    Matrix9f& operator=(const Matrix9f& other) {
        elements_ = other.elements_;      // element-wise copy
        return *this; //Returning reference allows chaining, A= B=C
    }
    Matrix9f& operator=(const std::array<float,81>& otherElements) {
        elements_ = otherElements;      
        return *this; //Returning reference 
    }

    //Matrix specific operations
    Matrix9f transpose() const { //Const ensures that original matrix doesn't get overidden
        Matrix9f R_T;
        for (int i = 0; i<9; i++) {
            for (int j=0; j<9; j++){
                R_T(i,j) = (*this)(j,i); //Swap the row and column
            }
        }
        return R_T;
    }

    //Matrix Addition
    // ----- Addition operator (returns a new matrix) -----
    inline Matrix9f operator+(const Matrix9f& other) const {
        Matrix9f result;
        for (int i = 0; i < 81; ++i) {
            result.elements_[i] = elements_[i] + other.elements_[i];
        }
        return result;
    }

    inline Matrix9f operator-(const Matrix9f& other) const {
        Matrix9f result;
        for (int i = 0; i < 81; ++i) {
            result.elements_[i] = elements_[i] - other.elements_[i];
        }
        return result;
    }

    inline Matrix9f& operator-=(const Matrix9f& other) {
        for (int i = 0; i < 81; ++i) {
            elements_[i] -= other.elements_[i];
        }
        return *this;
    }

    Matrix9f operator*(const Matrix9f& other) const {
        Matrix9f result;

        for (int i = 0; i < 9; ++i) {
            float row[9];
            for (int k = 0; k < 9; ++k) row[k] = elements_[i*9 + k]; // copy row from *this

            for (int j = 0; j < 9; ++j) {
                float col[9];
                for (int k = 0; k < 9; ++k) col[k] = other.elements_[k*9 + j]; // <-- other, not elements_

                float sum = 0.0f;
                for (int k = 0; k < 9; ++k) sum += row[k] * col[k];

                result.elements_[i*9 + j] = sum;
            }
        }
        return result;
    }

    std::array<float, 9> getDiagonal() const {
        std::array<float, 9> diag{};
        for (int i = 0; i < 9; ++i)
            diag[i] = elements_[i * 9 + i];
        return diag;
    }

};
#endif