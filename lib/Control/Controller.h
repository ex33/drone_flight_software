// Main Controller for tracking Translational and Rotational States
#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "Mathpk.h"
#include "Constants.h"
class Controller {
public:

  /**
   * @brief Init Controller object. 
   * 
   * @param freq Frequency to update Controller Action
   * @param positionReference Position Reference
   * @param velocityReference Velocity Reference
   * @param quaternionReference Quaternion Reference
   * @param rateReference Rate Reference
   * @param nominalControl Control for making reference an equilbrium point
   * @param K LQR Gain Matrix [Calculated on ground]
   * @param horizontalControllerFlag Boolean to ignore X-Y Position error
   * @param verticalControllerFlag Boolean to ignore Z Position Error 
   * 
   * Initializes Controller. NOTE, KALMAN GAIN HAS MOTOR SPIN DIRECTION EMBEDED
   */
  Controller(const float & freq, const std::array<float,3>& positionReference,const std::array<float,3>& velocityReference,const std::array<float,4>& quaternionReference,const std::array<float,3>& rateReference, const std::array<float,4>& controlReference, const std::array<float,48>& K, const bool horizontalControllerFlag, const bool verticalControllerFlag);

  /**
   * @brief Updates References 
   * 
   * @param positionReference Position Reference
   * @param velocityReference Velocity Reference
   * @param quaternionReference Quaternion Reference
   * @param rateReference Rate Reference
   * @param nominalControl Control for making reference an equilbrium point
   * @param K LQR Gain Matrix
   * 
   * When more complicated guidance is added and need time-varying references
   */  
  void updateRef(const Vector3f& positionReference,const Vector3f& velocityReference,const Quaternion& quaternionReference,const Vector3f& rateReference);
  void updateRef(const Vector3f& positionReference,const Vector3f& velocityReference,const Quaternion& quaternionReference,const Vector3f& rateReference, const std::array<float,4>& controlReference);
  void updateRef(const Vector3f& positionReference,const Vector3f& velocityReference,const Quaternion& quaternionReference,const Vector3f& rateReference, const std::array<float,4>& controlReference, const std::array<float,48> K);


  /**
   * @brief Update State Error 
   * 
   * @param p_hat Position Estimate from Navigation
   * @param v_hat Velocity Estimate from Navigation
   * @param q_hat Quaternion Estimate from Navigation
   * @param w_hat Rate Estimate from Navigation
   * 
   * Computes State error (x_hat - x_bar)
   */    
  void updateError(const Vector3f& p_hat, const Vector3f& v_hat, const Quaternion& q_hat, const Vector3f& w_hat); 


  /**
   * @brief Checks internally if enough time has passed to udpate control requested
   * 
   * @param now Time to check for Controller Update
   * @param p_hat Position Estimate from Navigation
   * @param v_hat Velocity Estimate from Navigation
   * @param q_hat Quaternion Estimate from Navigation
   * @param w_hat Rate Estimate from Navigation
   * 
  * @return If (time_now - time_last_update) > frequency, then this will update the member containing the 
    * control and return TRUE to signal a new control request is avaliable.
   * Computes State error u_des = u_bar - K*(x_hat - x_bar)
   * Control is the Spin Rates ^2 of each Motor
   */    
  bool updateControl(uint32_t now,  const Vector3f& p_hat, const Vector3f& v_hat, const Quaternion& q_hat, const Vector3f& w_hat); 
    
  /**
    * @brief Returns the Current computed Control
    * 
    * @return Current Computed Control
    */
  std::array<float,4> getControl ();

  // Getter functions for testing
  inline Vector3f getPosRef() const {
      return this->posRef_;
  };
  inline Vector3f getVelRef() const {
      return this->velRef_;
  };
  inline Quaternion getQuatRef() const {
      return this->quatRef_;
  };
  inline Vector3f getRateRef() const {
      return this->rateRef_;
  };
  inline std::array<float,4> getNominalControl() const {
    return this->uRef_;
  };
  inline Vector3f getPosErr() const {
      return this->posErr_;
  };
  inline Vector3f getVelErr() const {
      return this->velErr_;
  };
  inline Vector3f getAlpha() const {
      return this->alpha_;
  };
  inline Vector3f getRateErr() const {
      return this->rateErr_;
  };
  inline std::array<float,48> getGainMatrix() const {
    return this->K_;
  }


private:
  Vector3f posRef_;
  Vector3f velRef_;
  Quaternion quatRef_;
  Vector3f rateRef_;

  std::array<float,4> uRef_;


  //Save down error for telemetry
  Vector3f posErr_;
  Vector3f velErr_;
  Vector3f alpha_; 
  Vector3f rateErr_;

  //Frequency to run LQR Controller at
  float freq_;

  // Time since last controller ran
  uint32_t lastUpdate_ = 0;

  // Current Control
  std::array<float,4> currControl_ = {0.0,0.0,0.0,0.0}; ;

  std::array<float,48> K_ ; //4 x 12

  //Flags 
  bool verticalControllerFlag_;
  bool horizontalControllerFlag_;
};
#endif // CONTROLLER_H
