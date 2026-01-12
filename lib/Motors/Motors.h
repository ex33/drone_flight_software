// Motor Class
#ifndef _MOTORS_H
#define _MOTORS_H

#include "Mathpk.h"
#include "SetUp.h"

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
   * @param maxSpin Maximum Spin rate in rev/s. This anchors mapping for control coming in to the PWM range. Without this, no idea what maxPWM corresponds to in terms of true maxSpin under loading
   * 
   * Computes State error u_des = u_bar - K*(x_hat - x_bar)
   * Control is the Spin Rates ^2 of each Motor
   */    
  Motors( const int esc1SignalPin, const int esc2SignalPin, const int esc3SignalPin, const int esc4SignalPin,
          const int M1StartPWM, const int M2StartPWM, const int M3StartPWM, const int M4StartPWM, 
          const int minPWM, const int maxPWM, const float maxSpin);

    /**
   * @brief Given Desired Control from Controller, command the associated spin rate 
   * 
   * @param motorSpinRateSquare Commanded w^2 from Controller
   * 
   * Computes State error u_des = u_bar - K*(x_hat - x_bar)
   * Control is the Spin Rates ^2 of each Motor
   */    
  void commandControl(const std::array<float,4>& motorSpinRateSquare);

    /**
   * @brief Command minimum PWM for all motors at the start to arm ESCs
   * 
   */    
  void armMotors();



private:
    // Servos 
    Servo motor1CW_;
    Servo motor2CCW_;
    Servo motor3CW_;
    Servo motor4CCW_;

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

    float maxSpin_;
 



};

#endif 
