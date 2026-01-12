#include "Motors.h"

Motors::Motors(const int esc1SignalPin, const int esc2SignalPin, const int esc3SignalPin, const int esc4SignalPin,
              const int M1StartPWM, const int M2StartPWM, const int M3StartPWM, const int M4StartPWM, 
              const int minPWM, const int maxPWM, const float maxSpin):
   esc1SignalPin_(esc1SignalPin), esc2SignalPin_(esc2SignalPin), esc3SignalPin_(esc3SignalPin), esc4SignalPin_(esc4SignalPin),
   M1StartPWM_(M1StartPWM), M2StartPWM_(M2StartPWM), M3StartPWM_(M3StartPWM), M4StartPWM_(M4StartPWM), 
   minPWM_(minPWM), maxPWM_(maxPWM), maxSpin_(maxSpin)
  {
    // Setup Servos
    motor1CW_.attach(this->esc1SignalPin_, this->minPWM_, this->maxPWM_);
    motor2CCW_.attach(this->esc2SignalPin_, this->minPWM_, this->maxPWM_);
    motor3CW_.attach(this->esc3SignalPin_, this->minPWM_, this->maxPWM_);
    motor4CCW_.attach(this->esc4SignalPin_, this->minPWM_, this->maxPWM_);
  };

void Motors::armMotors() {
  motor1CW_.writeMicroseconds(this->minPWM_);
  motor2CCW_.writeMicroseconds(this->minPWM_);
  motor3CW_.writeMicroseconds(this->minPWM_);
  motor4CCW_.writeMicroseconds(this->minPWM_);
}

void Motors::commandControl(const std::array<float,4>& motorSpinRateSquare) {
  int deltaPWM = this->maxPWM_ - this->minPWM_; 
  // Convert Requested Spin rates into PWM
  float m1PWM = (sqrtf(motorSpinRateSquare[0])/ this->maxSpin_) * (deltaPWM) + minPWM_;  
  float m2PWM = (sqrtf(motorSpinRateSquare[1])/ this->maxSpin_) * (deltaPWM) + minPWM_;
  float m3PWM = (sqrtf(motorSpinRateSquare[2])/ this->maxSpin_) * (deltaPWM) + minPWM_;
  float m4PWM = (sqrtf(motorSpinRateSquare[3])/ this->maxSpin_) * (deltaPWM) + minPWM_;

  // Clamp Motor commands and Convert to Int (In reality, should never exceed min and max PWMs with the way things are designed via LQR)
  // Note: I think there is a better way to do this, such that if one motor exceeds max PWM by X amount, you lower all motors equally
  // such that its balanced. But that is more for racing / open loop control. Should be designed to not exceed max PWM to begin with
  // Using Trinary in C++ (condition ? if_true : if_false)
  // So this should read... if PWM > max PWM, set it to max PWM. Else, if PWM < min PWM, set it to min PWM, else, static cast it to int (add 0.5f so  0.5- 1 rounds up)
  int m1PWMInt = (m1PWM > this->maxPWM_) ? this->maxPWM_ : ( (m1PWM < this->minPWM_) ? this->minPWM_  : (static_cast<int>(m1PWM + 0.5f)));
  int m2PWMInt = (m2PWM > this->maxPWM_) ? this->maxPWM_ : ( (m2PWM < this->minPWM_) ? this->minPWM_  : (static_cast<int>(m2PWM + 0.5f)));
  int m3PWMInt = (m3PWM > this->maxPWM_) ? this->maxPWM_ : ( (m3PWM < this->minPWM_) ? this->minPWM_  : (static_cast<int>(m3PWM + 0.5f)));
  int m4PWMInt = (m4PWM > this->maxPWM_) ? this->maxPWM_ : ( (m4PWM < this->minPWM_) ? this->minPWM_  : (static_cast<int>(m4PWM + 0.5f)));

  // Command Motors
  motor1CW_.writeMicroseconds(m1PWMInt);
  motor2CCW_.writeMicroseconds(m2PWMInt);
  motor3CW_.writeMicroseconds(m3PWMInt);
  motor4CCW_.writeMicroseconds(m4PWMInt);
};
