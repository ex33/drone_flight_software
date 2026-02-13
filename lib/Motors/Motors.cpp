#include "Motors.h"

Motors::Motors(const float kT, const float kM, const float L,
              const int esc1SignalPin, const int esc2SignalPin, const int esc3SignalPin, const int esc4SignalPin,
              const int M1StartPWM, const int M2StartPWM, const int M3StartPWM, const int M4StartPWM, 
              const int minPWM, const int maxPWM, const int saturationPWM, const float maxSpinSquare):
   kT_(kT), kM_(kM), L_(L),
   esc1SignalPin_(esc1SignalPin), esc2SignalPin_(esc2SignalPin), esc3SignalPin_(esc3SignalPin), esc4SignalPin_(esc4SignalPin),
   M1StartPWM_(M1StartPWM), M2StartPWM_(M2StartPWM), M3StartPWM_(M3StartPWM), M4StartPWM_(M4StartPWM), 
   minPWM_(minPWM), maxPWM_(maxPWM), maxSpinSquare_(maxSpinSquare), saturationPWM_(saturationPWM)
  {
    // Form Control Allocation Matrix
    this->pInvM_ = {1.0f/(4.0f*kT),  1.0f/(4.0f*L*kT),  1.0f/(4.0f*L*kT),  -1.0f/(4.0f*kM),
                    1.0f/(4.0f*kT), -1.0f/(4.0f*L*kT),  1.0f/(4.0f*L*kT),  1.0f/(4.0f*kM),
                    1.0f/(4.0f*kT), -1.0f/(4.0f*L*kT), -1.0f/(4.0f*L*kT),  -1.0f/(4.0f*kM),
                    1.0f/(4.0f*kT),  1.0f/(4.0f*L*kT), -1.0f/(4.0f*L*kT), 1.0f/(4.0f*kM)};

    this->currM1PWMInt_ = minPWM;
    this->currM2PWMInt_ = minPWM;
    this->currM3PWMInt_ = minPWM;
    this->currM4PWMInt_ = minPWM;
  };


void Motors::setUp(){
  // Setup Servos
  motor1CW_.attach(this->esc1SignalPin_, this->minPWM_, this->maxPWM_);
  motor2CCW_.attach(this->esc2SignalPin_, this->minPWM_, this->maxPWM_);
  motor3CW_.attach(this->esc3SignalPin_, this->minPWM_, this->maxPWM_);
  motor4CCW_.attach(this->esc4SignalPin_, this->minPWM_, this->maxPWM_);
}


void Motors::arm() {
  motor1CW_.writeMicroseconds(this->minPWM_);
  motor2CCW_.writeMicroseconds(this->minPWM_);
  motor3CW_.writeMicroseconds(this->minPWM_);
  motor4CCW_.writeMicroseconds(this->minPWM_);

  //Update current PWM (Redundant)
  this->currM1PWMInt_ = this->minPWM_;
  this->currM2PWMInt_ = this->minPWM_;
  this->currM3PWMInt_ = this->minPWM_;
  this->currM4PWMInt_ = this->minPWM_;

  this->armed_ = true;
}

//Only need to disarm once, dont need to repeatedly send the same command
void Motors::disarm() {
  if (currM1PWMInt_ != minPWM_) {
    motor1CW_.writeMicroseconds(this->minPWM_);
    this->currM1PWMInt_ = this->minPWM_;
  }
  if (currM2PWMInt_ != minPWM_) {
    motor2CCW_.writeMicroseconds(this->minPWM_);
    this->currM2PWMInt_ = this->minPWM_;
  }
  if (currM3PWMInt_ != minPWM_) {
    motor3CW_.writeMicroseconds(this->minPWM_);
    this->currM3PWMInt_ = this->minPWM_;
  }
  if (currM4PWMInt_ != minPWM_) {
    motor4CCW_.writeMicroseconds(this->minPWM_);
    this->currM4PWMInt_ = this->minPWM_;
  }

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
    // Mapping must be done with motor Square. Can do a test case:
    // ~8N for kT = 6.58e-5 will result in ~30000 (rev/s)^2. Assume the max is 80000 (rev/s)^2.
    // Looking at the square ratio, this is ~0.375. But if you squareroot both of them and compare it to rev/s,
    // the ratio is ~0.6 (sqrt(30000) / sqrt(80000)) != 30000/80000.
    // So the ACTUAL mapping needed here is rotational rate square. 
    float m1PWM = (motorSpinRateSquare[0])/ this->maxSpinSquare_ * (this->maxPWM_ - this->M1StartPWM_) + this->M1StartPWM_;  
    float m2PWM = (motorSpinRateSquare[1])/ this->maxSpinSquare_ * (this->maxPWM_ - this->M2StartPWM_) + this->M2StartPWM_;  
    float m3PWM = (motorSpinRateSquare[2])/ this->maxSpinSquare_ * (this->maxPWM_ - this->M3StartPWM_) + this->M3StartPWM_;  
    float m4PWM = (motorSpinRateSquare[3])/ this->maxSpinSquare_ * (this->maxPWM_ - this->M4StartPWM_) + this->M4StartPWM_;  

    // Clamp Motor commands and Convert to Int (In reality, should never exceed min and max PWMs with the way things are designed via LQR)
    // Note: I think there is a better way to do this, such that if one motor exceeds max PWM by X amount, you lower all motors equally
    // such that its balanced. But that is more for racing / open loop control. Should be designed to not exceed max PWM to begin with
    // Using Trinary in C++ (condition ? if_true : if_false)
    // So this should read... if PWM > max PWM, set it to max PWM. Else, if PWM < min PWM, set it to min PWM, else, static cast it to int (add 0.5f so  0.5- 1 rounds up)
    this->currM1PWMInt_ = (m1PWM > this->saturationPWM_) ? this->saturationPWM_ : ( (m1PWM < this->M1StartPWM_) ? this->M1StartPWM_  : (static_cast<int>(m1PWM + 0.5f)));
    this->currM2PWMInt_ = (m2PWM > this->saturationPWM_) ? this->saturationPWM_ : ( (m2PWM < this->M2StartPWM_) ? this->M2StartPWM_  : (static_cast<int>(m2PWM + 0.5f)));
    this->currM3PWMInt_ = (m3PWM > this->saturationPWM_) ? this->saturationPWM_ : ( (m3PWM < this->M3StartPWM_) ? this->M3StartPWM_  : (static_cast<int>(m3PWM + 0.5f)));
    this->currM4PWMInt_ = (m4PWM > this->saturationPWM_) ? this->saturationPWM_ : ( (m4PWM < this->M4StartPWM_) ? this->M4StartPWM_  : (static_cast<int>(m4PWM + 0.5f)));

    // // Command Motors. Have the if statement down here for regression testing, such that currM#PWMInt_ still updates without commadning motors
    if (this->armed_) {
      motor1CW_.writeMicroseconds(this->currM1PWMInt_);
      motor2CCW_.writeMicroseconds(this->currM2PWMInt_);
      motor3CW_.writeMicroseconds(this->currM3PWMInt_);
      motor4CCW_.writeMicroseconds(this->currM4PWMInt_);
    } else {
      Serial.println("NOT ARMED!!!");
    }
};

void Motors::commandMotors(const int PWM, const uint32_t durationMillis,  const int num) {
  if (this->armed_) {
    switch (num) {
      case 1:
        motor1CW_.writeMicroseconds(PWM);
        delay(durationMillis);
        motor1CW_.writeMicroseconds(this->minPWM_);
        break;

      case 2:
        motor2CCW_.writeMicroseconds(PWM);
        delay(durationMillis);
        motor2CCW_.writeMicroseconds(this->minPWM_);
        break;

      case 3:
        motor3CW_.writeMicroseconds(PWM);
        delay(durationMillis);
        motor3CW_.writeMicroseconds(this->minPWM_);
        break;
      case 4:
        motor4CCW_.writeMicroseconds(PWM);
        delay(durationMillis);
        motor4CCW_.writeMicroseconds(this->minPWM_);
        break;

      default:
        Serial.println("Invalid Motor Number...");
        break;
    } ;
  } else {
    Serial.println("NOT ARMED!!!");
  }
};


void Motors::commandMotors(const std::array<int,4> PWMArray, const uint32_t durationMillis) {
  if (this->armed_) {
    motor1CW_.writeMicroseconds(PWMArray[0]);
    motor2CCW_.writeMicroseconds(PWMArray[1]);
    motor3CW_.writeMicroseconds(PWMArray[2]);
    motor4CCW_.writeMicroseconds(PWMArray[3]);
    delay(durationMillis);
    motor1CW_.writeMicroseconds(this->minPWM_);
    motor2CCW_.writeMicroseconds(this->minPWM_);
    motor3CW_.writeMicroseconds(this->minPWM_);
    motor4CCW_.writeMicroseconds(this->minPWM_);
  } else {
    Serial.println("NOT ARMED!!!");
  }
};

void Motors::printPWMCMD() {
    Serial.print(currM1PWMInt_); Serial.print(",");
    Serial.print(currM2PWMInt_); Serial.print(",");
    Serial.print(currM3PWMInt_); Serial.print(",");
    Serial.println(currM4PWMInt_);
};

