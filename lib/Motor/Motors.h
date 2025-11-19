// Motor Class
#ifndef _MOTORS_H
#define _MOTORS_H

#include "Mathpk.h"
#include "SetUp.h"
class Motors {
public:

  //Some init function

    /**
   * @brief Given Desired Control from Controller, command the associated spin rate 
   * 
   * @param motorSpinRateSquare Commanded w^2 from Controller
   * 
   * Computes State error u_des = u_bar - K*(x_hat - x_bar)
   * Control is the Spin Rates ^2 of each Motor
   */    
  void commandMotor(std::array<float,4>& motorSpinRateSquare);


  std::array<float,4> getThrustTorques(std::array<float,4>& motorSpinRateSquare);

private:
  float minSpin;
  float maxSpin;
  float deadBand; // If the difference between the current and previous command is TINY, ignore it
  float prevCommand;

};

#endif // CONTROLLER_H
