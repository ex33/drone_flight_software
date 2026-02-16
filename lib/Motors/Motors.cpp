#include "Motors.h"

Motors::Motors(const float kT, const float kM, const float L,
              const int esc1SignalPin, const int esc2SignalPin, const int esc3SignalPin, const int esc4SignalPin,
              const int M1StartPWM, const int M2StartPWM, const int M3StartPWM, const int M4StartPWM, 
              const int minPWM, const int maxPWM, const int saturationPWM, const float maxSpinSquare):
   kT_(kT), kM_(kM), L_(L),
   minPWM_(minPWM), maxPWM_(maxPWM), maxSpinSquare_(maxSpinSquare), saturationPWM_(saturationPWM)
  {
    // Form Control Allocation Matrix
    this->pInvM_ = {1.0f/(4.0f*kT),  1.0f/(4.0f*L*kT),  1.0f/(4.0f*L*kT),  -1.0f/(4.0f*kM),
                    1.0f/(4.0f*kT), -1.0f/(4.0f*L*kT),  1.0f/(4.0f*L*kT),  1.0f/(4.0f*kM),
                    1.0f/(4.0f*kT), -1.0f/(4.0f*L*kT), -1.0f/(4.0f*L*kT),  -1.0f/(4.0f*kM),
                    1.0f/(4.0f*kT),  1.0f/(4.0f*L*kT), -1.0f/(4.0f*L*kT), 1.0f/(4.0f*kM)};

    // Set up ESC Signal Pins and Start PWMs
    this->escSignalPins_ = {esc1SignalPin, esc2SignalPin, esc3SignalPin, esc4SignalPin};
    this->motorStartPWMs_ = {M1StartPWM, M2StartPWM, M3StartPWM, M4StartPWM};
    this->currMotorPWM_ = {minPWM, minPWM, minPWM, minPWM};

  };


void Motors::setUp(){
  if (!this->setUp_) {
  //1. Set up the PWM resolution
  analogWriteResolution(this->pwmResolution_); // Set PWM resolution to 16 bits (0-65535). This is needed to get the full range of PWM values for finer control.

  //2. Set up the desired Frequency / update rates
  for (int i = 0; i < 4; i++){
    analogWriteFrequency(this->escSignalPins_[i], this->pwmFrequency_); // Set PWM frequency to 500, which is upper limit of recommended for Hobbywing ESCs
  }

  this->setUp_ = true;
  }
}


void Motors::arm() {

  writeESC(std::array<int,4>{this->minPWM_, this->minPWM_, this->minPWM_, this->minPWM_});
  this->armed_ = true;

}

void Motors::writeESC(const std::array<int,4>& PWMArray) {
  // PWM Array will be a value in between the min and max PWM. 
  // Need to do conversion to account for duty sycle and then mapped to the correct PWM range for the ESCs.
  // Write to ESCs
  for (int i = 0; i < 4; i++){
    float duty = PWMArray[i] / this->periodUS_; //Fraction of the total pulse that is on
    uint16_t pwmValue = duty * this->pwmRange_; //Maps the commanded PWM onto the 16-bit resolution range (0-65535)
    analogWrite(this->escSignalPins_[i], pwmValue);
  }

  // Update Motor Commands
  this->currMotorPWM_ = PWMArray;
}

void Motors::writeESC(const int& PWM, const int& idx) {
  // PWM will be a value in between the min and max PWM. 
  // Need to do conversion to account for duty sycle and then mapped to the correct PWM range for the ESCs.
  // Write to ESCs
  float duty = PWM / this->periodUS_; //Fraction of the total pulse that is on
  uint16_t pwmValue = duty * this->pwmRange_; //Maps the commanded PWM onto the 16-bit resolution range (0-65535)
  analogWrite(this->escSignalPins_[idx], pwmValue);

  // Update Motor Commands
  this->currMotorPWM_[idx] = PWM;
}

//Only need to disarm once, dont need to repeatedly send the same command
void Motors::disarm() {

  this->writeESC(std::array<int,4>{this->minPWM_, this->minPWM_, this->minPWM_, this->minPWM_});

  if (this->armed_) {
    Serial.println("Motors Disarmed");
    this->armed_ = false;
  }
}


std::array<float,4> Motors::allocateControl(const std::array<float,4> & u_Requested) {

  std::array<float,4> motorSpinRateSquare = {pInvM_[0]*u_Requested[0] + pInvM_[1]*u_Requested[1] + pInvM_[2]*u_Requested[2] + pInvM_[3]*u_Requested[3],
                                            pInvM_[4]*u_Requested[0] + pInvM_[5]*u_Requested[1] + pInvM_[6]*u_Requested[2] + pInvM_[7]*u_Requested[3],
                                            pInvM_[8]*u_Requested[0] + pInvM_[9]*u_Requested[1] + pInvM_[10]*u_Requested[2] + pInvM_[11]*u_Requested[3],
                                            pInvM_[12]*u_Requested[0] + pInvM_[13]*u_Requested[1] + pInvM_[14]*u_Requested[2] + pInvM_[15]*u_Requested[3]};

  return motorSpinRateSquare;
}




void Motors::commandControl(const std::array<float,4>& u_Requested) {

    //Convert into spin rates
    std::array<float,4> motorSpinRateSquare = this->allocateControl(u_Requested);

    // Convert Requested Spin rates into PWM
    for (int i = 0; i < 4; i++){
      float rawPWM = (motorSpinRateSquare[i])/ this->maxSpinSquare_ * (this->maxPWM_ - this->motorStartPWMs_[i]) + this->motorStartPWMs_[i];  
      // Clamp Motor commands and Convert to Int (In reality, should never exceed min and max PWMs with the way things are designed via LQR)
      this->currMotorPWM_[i] = (rawPWM > this->saturationPWM_) ? this->saturationPWM_ : ( (rawPWM < this->motorStartPWMs_[i]) ? this->motorStartPWMs_[i]  : (static_cast<int>(rawPWM + 0.5f)));
    }

    // //Command Motors. Have the if statement down here for regression testing, such that currM#PWMInt_ still updates without commadning motors
    // if (this->armed_) {
    //   this->writeESC(this->currMotorPWM_);
    // } else {
    //   Serial.println("NOT ARMED!!!");
    // }
};

void Motors::commandMotors(const int PWM, const uint32_t durationMillis,  const int idx) {
  if (this->armed_) {

      this->writeESC(PWM, idx);
      delay(durationMillis);
      this->writeESC(this->minPWM_, idx);

  } else {
    Serial.println("NOT ARMED!!!");
  }
};

void Motors::commandMotors(const std::array<int,4> PWMArray, const uint32_t durationMillis) {
  if (this->armed_) {
    this->writeESC(PWMArray);
    delay(durationMillis);
    this->writeESC(std::array<int,4>{this->minPWM_, this->minPWM_, this->minPWM_, this->minPWM_});
  } else {
    Serial.println("NOT ARMED!!!");
  }
};

void Motors::printPWMCMD() {
    Serial.print(this->currMotorPWM_[0]); Serial.print(",");
    Serial.print(this->currMotorPWM_[1]); Serial.print(",");
    Serial.print(this->currMotorPWM_[2]); Serial.print(",");
    Serial.println(this->currMotorPWM_[3]);
};

