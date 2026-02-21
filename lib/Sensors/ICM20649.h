// Starting to make own IMU class using Wire to extract the raw measurements.
// Eventually will also use the interrupt
// IMU class from Adafruit adds additional overhead resulting in ~3000 micro seconds per measurement, which is ~ 333 Hz. This is slower than the controller loop of 500hz, so need this to be faster.
#ifndef _ICM20649_H
#define _ICM20649_H


//===== Ardiuno =====
#include <Wire.h>
#include <Constants.h>

class ICM20649 {
public:


    /**
     * @brief Init IMU Object. This will use the default I2C address.
     */
    ICM20649();

    bool begin(); //Initialize the Sensor


    void setAccelODR();
    void setGyroODR();


    void setAccelRange(); 
    void setGyroRange(); 

    std::array<float,6> readIMU(); // Returns [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z] in m/s^2 and rad/s respectively

private:
    //---------- Initialization ------------
    TwoWire* wire_; //Two wire is the class version of Wire. Allows for selection of Bus
    uint8_t i2cAddress_;

    // ------------ Low-level helpers ------------
    uint8_t readReg(uint8_t reg);
    void writeReg(uint8_t reg, uint8_t data);
    void readRegs(uint8_t reg, uint8_t *buffer, size_t length);

    // ------------ Conversion Factors ------------
    float accelLFBSensitivity_;
    float gyroLFBSensitivity_;    


    // ------------- Relevant Registers -------------
    // Bank selection register and values
    uint8_t REG_BANK_SEL_ = 0x7F; // Register to select bank, set to 0x20 (Bank 2) for accel ODR, 0x00 (Bank 1) for gyro ODR
    uint8_t BANK_2_ = 0x20;
    uint8_t BANK_0_ = 0x00;


    uint8_t ACCEL_XOUT_H_ADDR_ = 0x2D; // Bank 0, this is where the accel measurements start (page 36/61)
    uint8_t ACCEL_XOUT_L_ADDR_ = 0x2E;
    uint8_t ACCEL_YOUT_H_ADDR_ = 0x2F;
    uint8_t ACCEL_YOUT_L_ADDR_ = 0x30;
    uint8_t ACCEL_ZOUT_H_ADDR_ = 0x31;
    uint8_t ACCEL_ZOUT_L_ADDR_ = 0x32;
    uint8_t GYRO_XOUT_H_ADDR_ = 0x33; //This is the starting register for the gyro measurements, which are sequentially stored across 6 registers (page 36/61)
    uint8_t GYRO_XOUT_L_ADDR_ = 0x34;
    uint8_t GYRO_YOUT_H_ADDR_ = 0x35;
    uint8_t GYRO_YOUT_L_ADDR_ = 0x36;
    uint8_t GYRO_ZOUT_H_ADDR_ = 0x37;
    uint8_t GYRO_ZOUT_L_ADDR_ = 0x38;

};
#endif // IMU_h