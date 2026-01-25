// Motor Class
#ifndef _MOTORS_H
#define _MOTORS_H

#include <Arduino.h>
#include "Mathpk.h"
#include "Servo.h"
// Motor class 
// Note that Servo typically runs at 50Hz.
// This means 1 period is 1/50s --> ~20ms. 
// So setting a pulse width of 2000 micro s results in off time of ~18ms.
// For ESC, under 500 Hz signal is recommended (satisfied by defulat of servo), and typically has pulse width of 1150 micros - 1950 micro-s
class Motors {
public:

    /**
   * @brief Inits Motors. Servos constructed by Motor class. Will be initialized based on inputs. 
   * 
   * @param kT Thrust Constant of Motor
   * @param kM Torque Constant of Motor
   * @param L Moment Arm (need to distingush pitch vs roll if asymmetric airframe)
   * @param esc1SignalPin Pin on Teensy for Signal Wire of ESC 1
   * @param esc2SignalPin Pin on Teensy for Signal Wire of ESC 2
   * @param esc3SignalPin Pin on Teensy for Signal Wire of ESC 3
   * @param esc4SignalPin Pin on Teensy for Signal Wire of ESC 4
   * @param M1StartPWM Recorded PWM in which Motor 1 starts to Spin. Think of this as lower bound for motor. 
   * @param M2StartPWM Recorded PWM in which Motor 2 starts to Spin. Think of this as lower bound for motor. 
   * @param M3StartPWM Recorded PWM in which Motor 3 starts to Spin. Think of this as lower bound for motor. 
   * @param M4StartPWM Recorded PWM in which Motor 4 starts to Spin. Think of this as lower bound for motor. 
   * @param minPWM  Minimum PWM range for ALL ESCs. This is consistently 0 spin rates.
   * @param maxPWM Maximum PWM range for ALL ESCs. This is the PWM that corresponds with max spin
   * @param saturationPWM PWM at which to saturate the commands of each motors
   * @param maxSpinSquare Maximum Spin rate in rev^2/s^2. This anchors mapping for control coming in to the PWM range. Without this, no idea what maxPWM corresponds to in terms of true maxSpin under loading
   */    
  Motors( const float kT, const float kM, const float L,
          const int esc1SignalPin, const int esc2SignalPin, const int esc3SignalPin, const int esc4SignalPin,
          const int M1StartPWM, const int M2StartPWM, const int M3StartPWM, const int M4StartPWM, 
          const int minPWM, const int maxPWM, const int saturationPWM, const float maxSpinSquare);

    /**
   * @brief Attach all Pin, Min and Max PWM. Call this within setup() to avoid failing from timers not being intialized outside of it
   * 
   */    
  void setUp();

    /**
   * @brief Calculate Motor speeds based on controller ["Motor Mixing"]
   * 
   * @param u Commanded Force and Torques from Controller
   * 
   * @return Array of (Spin rates)^2 of each motor
   * Using the thrust and torque constant, psuedo inverse the allocation matrix 
   */    
  std::array<float,4> allocateControl(const std::array<float,4>& u);


    /**
   * @brief Given Desired Control from Controller, command the associated spin rate 
   * 
   * @param u Commanded Force and Torques from Controller
   * 
   * Computes State error u_des = u_bar - K*(x_hat - x_bar)
   * Control is the Spin Rates ^2 of each Motor
   */    
  void commandControl(const std::array<float,4>& u);


    /**
   * @brief Command one motor given PWM
   * 
   * @param PWM PWM Signal
   * @param durationMillis duration to run motor for
   * @param num Motor Number to command (1-4)
   * 
   */    
  void commandMotors(const int PWM, const uint32_t durationMillis, const int num);

    /**
   * @brief Command ALL motors given PWM
   * 
   * @param PWMArray Array of PWM Signal
   * 
   */    
  void commandMotors(const std::array<int,4> PWMArray, const uint32_t durationMillis);

    /**
   * @brief Command minimum PWM for all motors at the start to arm ESCs
   * 
   */    
  void arm();

    /**
   * @brief Command minimum PWM to turn off motors
   * 
   */    
  void disarm();
  
  void printPWMCMD();


  //Getter functions (for unit testing)
  inline float getKT() const {
      return this->kT_;
  };
  inline float getKM() const {
      return this->kM_;
  };
  inline float getL() const {
      return this->L_;
  };
  inline std::array<float,16> getPInvM() const {
      return this->pInvM_;
  };

  inline int getESC1SignalPin() const {
      return this->esc1SignalPin_;
  };
  inline int getESC2SignalPin() const {
      return this->esc2SignalPin_;
  };
  inline int getESC3SignalPin() const {
      return this->esc3SignalPin_;
  };
  inline int getESC4SignalPin() const {
      return this->esc4SignalPin_;
  };

  inline int getM1StartPWM() const {
      return this->M1StartPWM_;
  };
  inline int getM2StartPWM() const {
      return this->M2StartPWM_;
  };
  inline int getM3StartPWM() const {
      return this->M3StartPWM_;
  };
  inline int getM4StartPWM() const {
      return this->M4StartPWM_;
  };

  inline int getMinPWM() const {
    return this->minPWM_;
  }
  inline int getMaxPWM() const {
    return this->maxPWM_;
  }

  inline float getMaxSpinSquare() const {
    return this->maxSpinSquare_;
  }

  inline bool getArmedBool() const {
    return this->armed_;
  }

  inline std::array<int,4> getCurrentMotorPWN() const {
    return std::array<int,4> {this->currM1PWMInt_, this->currM2PWMInt_, this->currM3PWMInt_, this->currM4PWMInt_};
  }

private:
    // Servos 
    Servo motor1CW_;
    Servo motor2CCW_;
    Servo motor3CW_;
    Servo motor4CCW_;

    // Thrust and Torque conversions
    float kT_;
    float kM_;
    float L_;
    std::array<float, 16> pInvM_; // Psuedo Inverse of Allocation Matrix. u = M * n^2 --> n^2 = pinv(M) * u

    // Servo parameters
    int esc1SignalPin_;
    int esc2SignalPin_;
    int esc3SignalPin_;
    int esc4SignalPin_; 

    int M1StartPWM_;
    int M2StartPWM_;
    int M3StartPWM_;
    int M4StartPWM_;

    int minPWM_;
    int maxPWM_;
    int saturationPWM_;

    float maxSpinSquare_;
 
    bool armed_ = false; //Todo:: for all functions, if this bool is false, should it automatically set PWM to min?

    //Servo Commands (for logging)
    int currM1PWMInt_;
    int currM2PWMInt_;
    int currM3PWMInt_;
    int currM4PWMInt_;


};

#endif 
