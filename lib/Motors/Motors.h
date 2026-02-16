// Motor Class
#ifndef _MOTORS_H
#define _MOTORS_H

#include <Arduino.h>
#include "Mathpk.h"
// Motor class 

// For ESC, under 500 Hz signal is recommended (satisfied by defulat of servo), and typically has pulse width of 1150 micros - 1950 micro-s
// BYPASSING SERVO CLASS. LIMITED TO 50HZ, WHICH IS REALLY SLOW 

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
   * @brief Given 4 PWM commands, send the corresponding command
   * 
   * @param PWMArray Commanded PWM values for each motor
   * 
   * Will compute the PWM value, analogWrite each motor, and update the currentPWM variable for logging.
   */    
  void writeESC(const std::array<int,4>& PWMArray);

    /**
   * @brief Given 1 PWM and the idx of the motor to command, send the corresponding command
   * 
   * @param PWM Commanded PWM value for a specific motor
   * @param idx Index of the motor to command (0-3)
   * 
   * Will compute the PWM value, analogWrite the motor, and update the currentPWM variable for logging.
   */    
  void writeESC(const int& PWM, const int& idx) ;



    /**
   * @brief Command ALL motors given PWM
   * 
   * @param PWM  PWM Signal
   * @param durationMillis Duration to command the motors for (in milliseconds)
   * @param idx Index of the motor to command (0-3)
   * 
   */    
  void commandMotors(const int PWM, const uint32_t durationMillis, const int idx);

    /**
   * @brief Command ALL motors given PWM
   * 
   * @param PWMArray Array of PWM Signal
   * @param durationMillis Duration to command the motors for (in milliseconds)
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

  inline int getESCSignalPin(int idx) const {
      return this->escSignalPins_[idx];
  };


  inline int getMotorStartPWM(int idx) const {
      return this->motorStartPWMs_[idx];
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
    return currMotorPWM_;
  }

private:
  // Motor Parameters
  constexpr static uint8_t pwmResolution_ = 16; // 16-bit resolution for PWM
  constexpr static uint16_t pwmRange_ = 65535; // 0 to 65535 for 16-bit resolution
  constexpr static float pwmFrequency_ = 500.0f; // 500 Hz PWM frequency, which is upper limit of recommended for Hobbywing ESCs
  constexpr static float periodUS_ = 2000.0f; // Period of PWM signal in microseconds (500 Hz frequency corresponds to 2000 microseconds period)

    // Thrust and Torque conversions
    float kT_;
    float kM_;
    float L_;
    std::array<float, 16> pInvM_; // Psuedo Inverse of Allocation Matrix. u = M * n^2 --> n^2 = pinv(M) * u

    // parameters
    std::array<int,4> escSignalPins_;
    std::array<int,4> motorStartPWMs_;


    int minPWM_;
    int maxPWM_;
    int saturationPWM_;

    float maxSpinSquare_;
 
    bool armed_ = false; //Todo:: for all functions, if this bool is false, should it automatically set PWM to min?
    bool setUp_ = false; //Flag to make sure we only set up once. 
    //Servo Commands (for logging)
    std::array<int,4> currMotorPWM_;


};

#endif 
