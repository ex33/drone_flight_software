#include "ICM20649.h"

ICM20649::ICM20649() : 
  wire_(&Wire), i2cAddress_(0x68)
{
  //Default to using Wire, but can be changed if needed
  //Default I2C (see page 15, AD0=0 0110 1000)
};



bool ICM20649::begin() {
  const uint8_t PWR_MGMT_1_ADDR = 0x06; // Bank 0, this is the power management register, which we will use to reset the device and wake it up (page 36/61)

  // Reset device
  writeReg(PWR_MGMT_1_ADDR, 0x80);  // Set Interal registers and restores the default setting. 1000 0000 -> 0x80 (page 39)
  delay(100);

  // Wake up and select clock source
  //Need bit 0-2 to be 1 to auto select a clock. Also set bit 3 to be high to disable temperature sensor, since we won't be using it, and it can save some power.
  writeReg(PWR_MGMT_1_ADDR, 0x05);  // 0000 0101 -> 0x05

  //Set Up Ranges
  this->setAccelRange();
  this->setGyroRange();

  //Set Up ODRs
  this->setAccelODR();
  this->setGyroODR();

  // Switch to regiter bank 0 to read the accel and gyro measurements
  writeReg(REG_BANK_SEL_, BANK_0_);


  return true;
}


void ICM20649::setAccelODR() {
  const uint8_t ACCEL_SMPLRT_DIV_1_ADDR = 0x10; // Bank 2, high 4 bits of 12-bit divider (page 37/67)
  const uint8_t ACCEL_SMPLRT_DIV_2_ADDR = 0x11; // Bank 2, low 8 bits of 12-bit divider (page 37/67)
  const uint8_t ACCEL_CONFIG_2_ADDR = 0x15; // Bank 2, this is where the DEC3_CFG bits are (page 37/67)

  writeReg(REG_BANK_SEL_, BANK_2_); //Switch to Bank 2 to write to accel ODR register

  // Now write to the ACCEL_CONFIG2 register to set the ODR. This is using the digital low-pass filter

  writeReg(ACCEL_CONFIG_2_ADDR, 0x00); // This sets all Self-Test off, and doesn't apply any extra averging for low power mode. This is the same as the default, but just to be explicit.(page 37/67)

  //Set ACCEL_SMPLRT_DIV
  // ODR = 1.125kHz / (1 + ACCEL_SMPLRT_DIV)
  // Note, ACCEL_SMPLRT_DIV is 12 bits, so the btyes are split across two registers
  // Set all of them to 0 to get 1kHz
  writeReg(ACCEL_SMPLRT_DIV_1_ADDR, 0x00);
  writeReg(ACCEL_SMPLRT_DIV_2_ADDR, 0x00); 
}

void ICM20649::setGyroODR() {
  const uint8_t GYRO_SMPLRT_DIV_ADDR = 0x00; // Bank 2, Only 7 bits (page 36/61)
  const uint8_t GYRO_CONFIG_2_ADDR = 0x02; // Bank 2, this is where the DEC3_CFG bits are (page 36/67)

  writeReg(REG_BANK_SEL_, BANK_2_); //Switch to Bank 2 to write to accel ODR register

  writeReg(GYRO_CONFIG_2_ADDR, 0x00); // This sets all Self-Test off, and doesn't apply any extra averging for low power mode. This is the same as the default, but just to be explicit.

  //Set GYRO_SMPLRT_DIV
  // ODR = 1.125kHz / (1 + GYRO_SMPLRT_DIV)
  // Note, GYRO_SMPLRT_DIV is 7 bits
  writeReg(GYRO_SMPLRT_DIV_ADDR, 0x00);
}

void ICM20649::setAccelRange() {
  // NOTE: By default, uses the DLPF. Can bypass this to get higher ODR (~4.5kHZ)
  const uint8_t ACCEL_CONFIG_ADDR = 0x14; // Bank 2, this is where the accel range is configured (page 37 / 66)

  writeReg(REG_BANK_SEL_, BANK_2_); //Switch to Bank 2 to write to accel Range register

  // Set the accel range to +/- 16g.
  // This also sets the DLPF Configuration
  // TODO:: Come pack and consider using your own DLPF, the 3db BW is only 246Hz, while the output is 1.125kHz, so while we are sampling faster, we are oversampling and not getting
  // better information. 
  writeReg(ACCEL_CONFIG_ADDR, 0x05); // Need to set this byte to 0000 0101. Which is 0x05. This sets the range to +/-16g, and keeps the DLPF Enabled (and defaults everything else)

  //Set up the LSB Sensitivity for the accel (counts to g)
  // Page 13, ACCEL_FS = 2 is +/-16g, which corresponds to 2048 LSB/g, 
  this->accelLFBSensitivity_ = 1/2048.0; //Multiplication operator is faster

}

void ICM20649::setGyroRange() {
  // NOTE: By default, uses the DLPF. Can bypass this to get higher ODR (~9kHZ)
  const uint8_t GYRO_CONFIG_1_ADDR = 0x01; // Bank 2, this is where the gyro range is configured (page 36 / 66)

  writeReg(REG_BANK_SEL_, BANK_2_); //Switch to Bank 2 to write to gyro Range register

  // Set the gyro range to +/- 2000dps.
  // This also sets the DLPF Configuration
  // TODO:: Come pack and consider using your own DLPF, the 3db BW is only 196.6, while the output is 1.125kHz, so while we are sampling faster, we are oversampling and not getting
  // better information. 
  writeReg(GYRO_CONFIG_1_ADDR, 0x05); // Need to set this byte to 0000 0101. Which is 0x05. This sets the range to +/-2000dps, and keeps the DLPF Enabled (and defaults everything else)

  //Set up the LSB Sensitivity for the gyro (counts to deg/s)
  // Page 12, GYRO_FS = 2 is +/-2000dps, which corresponds to 16.4 LSB/dps, 
  this->gyroLFBSensitivity_ = 1/16.4;
}


std::array<float,6> ICM20649::readIMU() { // Returns [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z] in m/s^2 and rad/s respectively

  // Read the registers of the IMU directly in an attempt to speed things up and reduce number of calls to the wire.
  uint8_t buf[12]; //6 for accel, 6 for gyro. I2C only transmit 8 bits, so we have to read the high and low byte separately and combine them later
  this->readRegs(this->ACCEL_XOUT_H_ADDR_, buf, 12); // Bytes for ACCEL and GYRO are sequential, so we can read them together. This will read the 6 accel and gyro registers starting from ACCEL_XOUT_H_ADDR_
  
  // Form Accel and Gyro measurements by combining the high and low bytes, and dividing by the sensitivity to get the measurement in g's and deg/s
  //Shift the HIGH byte by 8 bits, then bitwise OR combine the two
  // accel_H: 1111 0000 << 8 --> 1111 0000 0000 0000
  // accel_L = 0000 1111
  // accel = accel_H | accel_L --> 1111 0000 0000 0000 | 0000 0000 0000 1111 --> 1111 0000 0000 1111
  int16_t ax = (buf[0] << 8) | buf[1];
  int16_t ay = (buf[2] << 8) | buf[3];
  int16_t az = (buf[4] << 8) | buf[5];

  int16_t gx = (buf[6] << 8) | buf[7];
  int16_t gy = (buf[8] << 8) | buf[9];
  int16_t gz = (buf[10] << 8) | buf[11];

  return {
    ax * this->accelLFBSensitivity_ * CONSTANTS::g0,
    ay * this->accelLFBSensitivity_ * CONSTANTS::g0,
    az * this->accelLFBSensitivity_ * CONSTANTS::g0,
    gx * this->gyroLFBSensitivity_ * CONSTANTS::deg2rad,
    gy * this->gyroLFBSensitivity_ * CONSTANTS::deg2rad,
    gz * this->gyroLFBSensitivity_ * CONSTANTS::deg2rad
  };
}



//Private

void ICM20649::writeReg(uint8_t reg, uint8_t data) {
    wire_->beginTransmission(this->i2cAddress_); // start I2C communication
    wire_->write(reg);                  // tell device which register
    wire_->write(data);                 // send the value
    wire_->endTransmission();           // finish communication
}


// Used for reading multiple sequential registers. This is more efficient and faster.
void ICM20649::readRegs(uint8_t startReg, uint8_t *buffer, size_t length) {
    wire_->beginTransmission(this->i2cAddress_); // Start I2C communication
    wire_->write(startReg);                        // Send starting register address
    wire_->endTransmission(false);            // Send repeated start

    wire_->requestFrom(this->i2cAddress_, (uint8_t)length);
    for (size_t i = 0; i < length; i++) {
        if (wire_->available()) {
            buffer[i] = wire_->read();       // Read each byte
        } else {
            buffer[i] = 0;                 // Default to 0 if not available
        }
    }
}



//Standard Ardiuno I2C communication functions. 
uint8_t ICM20649::readReg(uint8_t reg) {
    uint8_t val = 0; //Holds the byte read from the register. Default to 0 if there is no value
   
    wire_->beginTransmission(this->i2cAddress_); //Starts communication with the device
    wire_->write(reg); //Sends the register address to the device
    wire_->endTransmission(false); //End the transmission, but DON"T release the I2C bus (false)
    wire_->requestFrom(this->i2cAddress_, (uint8_t)1); //Send 1 btye from the previous register

    if (wire_->available()) { //Non blocking check to see if the device has sent a byte back. If it has, read it. If not, return 0.
      val = wire_->read();
    };
    return val;
};