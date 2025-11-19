#include "Controller.h"


Controller::Controller(const Vector3f& positionReference,const Vector3f& velocityReference,const Quaternion& quaternionReference,const Vector3f& rateReference, const std::array<float,4>& controlReference, const std::array<float,48> K): 
  posRef_(positionReference), velRef_(velocityReference), quatRef_(quaternionReference), rateRef_(rateReference), uRef_(controlReference), K_(K) {};

void Controller::updateRef(const Vector3f& positionReference,const Vector3f& velocityReference,const Quaternion& quaternionReference,const Vector3f& rateReference, const std::array<float,4>& controlReference,const std::array<float,48> K){
  this->posRef_ = positionReference;
  this->velRef_ = velocityReference;
  this->quatRef_ = quaternionReference;
  this->rateRef_ = rateReference;
  this->uRef_ = controlReference;
};


void Controller::updateError(const Vector3f& p_hat, const Vector3f& v_hat, const Quaternion& q_hat, const Vector3f& w_hat) {
  this->posErr_ = p_hat - this->posRef_;
  this->velErr_ = v_hat - this->velRef_;
  
  Quaternion qErr = quatMult(q_hat, this->quatRef_.getConjugate()); //ToDO: Figure out if this should be normalized

  this->alpha_ = std::array<float,3> {2.0f * qErr.x(), 2.0f * qErr.y(), 2.0f * qErr.z()};

  this->rateErr_ = w_hat - this->rateRef_;
};

std::array<float,4> Controller::getControl(const Vector3f& p_hat, const Vector3f& v_hat, const Quaternion& q_hat, const Vector3f& w_hat) {
  //1. Update the Error 
  this->updateError(p_hat, v_hat, q_hat, w_hat);

  //2. Calculate the control effort: u = uRef - K*(xHat - xRef)
  std::array<float,4> u_des {uRef_[0] - ( K_[0]*posErr_[0]+K_[1]*posErr_[1]+K_[2]*posErr_[2]     +  K_[3]*velErr_[0]+K_[4]*velErr_[1]+K_[2]*velErr_[2]     +  K_[6]*alpha_[0]+K_[7]*alpha_[1]+K_[8]*alpha_[2]     +  K_[9]*rateErr_[0]+K_[10]*rateErr_[1]+K_[11]*rateErr_[2] ),
                             uRef_[1] - ( K_[12]*posErr_[0]+K_[13]*posErr_[1]+K_[14]*posErr_[2]  +  K_[15]*velErr_[0]+K_[16]*velErr_[1]+K_[17]*velErr_[2]  +  K_[18]*alpha_[0]+K_[19]*alpha_[1]+K_[20]*alpha_[2]  +  K_[21]*rateErr_[0]+K_[22]*rateErr_[1]+K_[23]*rateErr_[2] ),
                             uRef_[2] - ( K_[24]*posErr_[0]+K_[25]*posErr_[1]+K_[26]*posErr_[2]  +  K_[27]*velErr_[0]+K_[28]*velErr_[1]+K_[29]*velErr_[2]  +  K_[30]*alpha_[0]+K_[31]*alpha_[1]+K_[32]*alpha_[2]  +  K_[33]*rateErr_[0]+K_[34]*rateErr_[1]+K_[35]*rateErr_[2] ),
                             uRef_[3] - ( K_[36]*posErr_[0]+K_[37]*posErr_[1]+K_[38]*posErr_[2]  +  K_[39]*velErr_[0]+K_[40]*velErr_[1]+K_[41]*velErr_[2]  +  K_[42]*alpha_[0]+K_[43]*alpha_[1]+K_[44]*alpha_[2]  +  K_[45]*rateErr_[0]+K_[46]*rateErr_[1]+K_[47]*rateErr_[2] )};

};