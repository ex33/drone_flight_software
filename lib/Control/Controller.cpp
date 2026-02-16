#include "Controller.h"


Controller::Controller(const float& freq,
                      const std::array<float,3>& positionReference,
                      const std::array<float,3>& velocityReference,
                      const std::array<float,4>& quaternionReference,
                      const std::array<float,3>& rateReference, 
                      const std::array<float,4>& controlReference, 
                      const std::array<float,48>& K,
                      const bool horizontalControllerFlag, const bool verticalControllerFlag): 
  posRef_(positionReference), velRef_(velocityReference), quatRef_(quaternionReference), rateRef_(rateReference), uRef_(controlReference), K_(K), horizontalControllerFlag_(horizontalControllerFlag), verticalControllerFlag_(verticalControllerFlag) {
    //Convert from Hz to Milli-Seconds
    freq_ = CONSTANTS::seconds2milli/freq;
  };


void Controller::updateRef(const Vector3f& positionReference,const Vector3f& velocityReference,const Quaternion& quaternionReference,const Vector3f& rateReference){
  this->posRef_ = positionReference;
  this->velRef_ = velocityReference;
  this->quatRef_ = quaternionReference;
  this->rateRef_ = rateReference;
};


void Controller::updateRef(const Vector3f& positionReference,const Vector3f& velocityReference,const Quaternion& quaternionReference,const Vector3f& rateReference, const std::array<float,4>& controlReference){
  this->posRef_ = positionReference;
  this->velRef_ = velocityReference;
  this->quatRef_ = quaternionReference;
  this->rateRef_ = rateReference;
  this->uRef_ = controlReference;
};

void Controller::updateRef(const Vector3f& positionReference,const Vector3f& velocityReference,const Quaternion& quaternionReference,const Vector3f& rateReference, const std::array<float,4>& controlReference,const std::array<float,48> K){
  this->posRef_ = positionReference;
  this->velRef_ = velocityReference;
  this->quatRef_ = quaternionReference;
  this->rateRef_ = rateReference;
  this->uRef_ = controlReference;
  this->K_ = K;
};


void Controller::updateError(const Vector3f& p_hat, const Vector3f& v_hat, const Quaternion& q_hat, const Vector3f& w_hat) {
  this->posErr_ = p_hat - this->posRef_;
  this->velErr_ = v_hat - this->velRef_;
  
  Quaternion qErr = quatMult(q_hat, this->quatRef_.getConjugate()); //ToDO: Figure out if this should be normalized

  this->alpha_ = std::array<float,3> {qErr.x(), qErr.y(), qErr.z()}; //Alpha here defined as vector component of Quaternion. 

  this->rateErr_ = w_hat - this->rateRef_;
};

bool Controller::updateControl( uint32_t now, const Vector3f& p_hat, const Vector3f& v_hat, const Quaternion& q_hat, const Vector3f& w_hat) {
  //Check if enough time has passed.
  if (now - lastUpdate_ >= this->freq_) {
    lastUpdate_ = now;
    //1. Update the Error 
    this->updateError(p_hat, v_hat, q_hat, w_hat);

    //2. Make copies of variables. This is purely for ease of disabling states for testing. Otherwise can just use the class members directly
    std::array<float,12> xErr {posErr_[0], posErr_[1], posErr_[2],
                              velErr_[0],velErr_[1],velErr_[2],
                              alpha_[0],alpha_[1],alpha_[2],
                              rateErr_[0], rateErr_[1], rateErr_[2]}; 
    std::array<float,4> uRef = uRef_;

    //. Check if we are running any position controller
    if (!this->verticalControllerFlag_) { //If Distabled
      xErr[2] = 0.0f;
      xErr[5] = 0.0f;
    };

    if (!this->horizontalControllerFlag_) { //If Distabled
      xErr[0] = 0.0f; //X Position
      xErr[1] = 0.0f; //Y Position
      xErr[3] = 0.0f; //X Velocity
      xErr[4] = 0.0f; //Y Velocity
    };

    //3. Calculate the control effort: u = uRef - K*(xHat - xRef)
    this-> currControl_ = {uRef[0] - ( K_[0]*xErr[0]+K_[1]*xErr[1]+K_[2]*xErr[2]     +  K_[3]*xErr[3]+K_[4]*xErr[4]+K_[5]*xErr[5]     +  K_[6]*xErr[6]+K_[7]*xErr[7]+K_[8]*xErr[8]     +  K_[9]*xErr[9]+K_[10]*xErr[10]+K_[11]*xErr[11] ),
                          uRef[1] - ( K_[12]*xErr[0]+K_[13]*xErr[1]+K_[14]*xErr[2]  +  K_[15]*xErr[3]+K_[16]*xErr[4]+K_[17]*xErr[5]  +  K_[18]*xErr[6]+K_[19]*xErr[7]+K_[20]*xErr[8]  +  K_[21]*xErr[9]+K_[22]*xErr[10]+K_[23]*xErr[11] ),
                          uRef[2] - ( K_[24]*xErr[0]+K_[25]*xErr[1]+K_[26]*xErr[2]  +  K_[27]*xErr[3]+K_[28]*xErr[4]+K_[29]*xErr[5]  +  K_[30]*xErr[6]+K_[31]*xErr[7]+K_[32]*xErr[8]  +  K_[33]*xErr[9]+K_[34]*xErr[10]+K_[35]*xErr[11] ),
                          uRef[3] - ( K_[36]*xErr[0]+K_[37]*xErr[1]+K_[38]*xErr[2]  +  K_[39]*xErr[3]+K_[40]*xErr[4]+K_[41]*xErr[5]  +  K_[42]*xErr[6]+K_[43]*xErr[7]+K_[44]*xErr[8]  +  K_[45]*xErr[9]+K_[46]*xErr[10]+K_[47]*xErr[11] )};
    return true;

  }
  return false; 
};

std::array<float,4> Controller::getControl () {
  return this->currControl_;
};