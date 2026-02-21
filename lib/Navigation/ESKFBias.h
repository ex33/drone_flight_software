// // Error State Kalman Filter for Translational State, Attitude, IMU Biases, and Magnetometer Bias Estiamtion
// #ifndef _ESKFBIAS_H
// #define _ESKFBIAS_H

// #include "Mathpk.h"
// #include "Sensors.h"
// #include "Constants.h"
// #include "DataTypes.h"
// class ESKFBias {
// public:

//     /**
//      * @brief Init ESKFBias object. 
//      * 
//      * @param p0 Initial NED Position [NOMINAL STATE]
//      * @param v0 Initial NED Velocity [NOMINAL STATE]
//      * @param q0 Initial NED2Body Quaternion [NOMINAL STATE]
//      * @param ba0 Initial Accelerometer Bias [NOMINAL STATE]
//      * @param bg0 Initial Gyro Bias [NOMINAL STATE]
//      * @param bm0 Initial Magnetometer Bias [NOMINAL STATE]
//      * 
//      * @param P0 Initial ERROR State Covariance for Kalman filter [324 x 1] (18x18)
//      * 
//      * @param sig_acc 1-Sigma Random Disturbance from Accelerometer [PROCESS NOISE]
//      * @param sig_gyro 1-Sigma Random Disturbance from Gyro [PROCESS NOISE]
//      * @param eta_acc 1-Sigma Random Disturbance to Accelerometer Bias [PROCESS NOISE]
//      * @param eta_gyro 1-Sigma Random Disturbance to Gyro Bias [PROCESS NOISE]
//      * @param eta_mag 1-Sigma Random Disturbance to Magnetometer Bias [PROCESS NOISE]
//      * 
//      * @param sig_m 1-Sigma Noise from Magnetometer Sensor [MEASUREMENT NOISE]
//      * @param sig_tilt 1-Sigma Noise from Tilt Sensor [MEASUREMENT NOISE]
//      * @param sig_alt 1-Sigma Noise from Altimeter Sensor [MEASUREMENT NOISE]
//      * @param sig_gps_pos 1-Sigma Noise from GPS Sensor (Position) [MEASUREMENT NOISE]
//      * @param sig_gps_vel 1-sigma Noise from GPS Sensor (Velocity) [MEASUREMENT NOISE ]
//      *
//      * Initializes Filter
//      * Expect array inputs so they can be inside SetUp.h as const expr
//      */
//     ESKFBias(std::array<float,3> p0, std::array<float,3> v0, std::array<float,4> q0, std::array<float,3> ba0, std::array<float,3> bg0, std::array<float,3> bm0,     //Initial Nominal States
//         std::array<float,324> P0,                                                                   //Error State Covariance
//         float sig_acc, float sig_gyro, float eta_acc, float eta_gyro, float eta_mag,       //Process Noise
//         float sig_m, float sig_tilt, float sig_alt, float sig_gps_pos, float sig_gps_vel); //Measurement Noise

//     /**
//      * @brief Init Discrete Process Noise Matrix
//      * 
//      * @param dt Timestep for Prediction
//      *
//      * Follows Marley's version of the Process Noise Matrix. Qd = int ( STM * Q * STM'), where Q is E[(Gw) * (Gw)'], where Gw is x_k1 = Fxk + Gw from the dynamics
//      * STM is linearized about the identity quaternion and 0 acceleration / rates, which works out to our hover state
//      */
//     Matrix18f getQd(float dt); //Might be able to calculate this every timestep if not too computationally expensive.

//     /**
//      * @brief Return the Discrete Time State Transition Matrix
//      * 
//      * @param accelMeas Raw Accelerometer Reading
//      * @param gyroMeas Raw Gyro Reading
//      *
//      * First order linearization of this via Taylor Expansion of Matrix Exponential. Other forms includes higher order expansions, or RK4
//      * Since we already calculate the inertial acceleration prior to calling this during the predict step, just pass that in directly
//      */
//     Matrix18f getSTM(const Quaternion& q, const Vector3f& accelBias,  const Vector3f& gyroBias, const Vector3f& accelMeas, const Vector3f& gyroMeas, const float dt ) const; 
    

//     /**
//      * @brief Propagates the covariance matrix forward
//      * 
//      * @param q Current Estiamte of Quaternion
//      * @param accelBias Current Estimate of Accelerometer Bias in the body frame
//      * @param gyroBias Current Estimate of the Gyro Bias in the body frame
//      * @param accelMeas Raw Accelerometer Reading
//      * @param gyroMeas Raw Gyro Reading
//      * @param dt Step Size
//      *
//      * First order linearization of this via Taylor Expansion of Matrix Exponential. Other forms includes higher order expansions, or RK4
//      * Since we already calculate the inertial acceleration prior to calling this during the predict step, just pass that in directly
//      */
//     void propagateCovariance(const Quaternion& q, const Vector3f& accelBias,  const Vector3f& gyroBias, const Vector3f& accelMeas, const Vector3f& gyroMeas, const float dt );

//     /**
//      * @brief Calls on propagateCovaraince along with propagating state estimate forward via Euler Integration
//      * 

//      * @param imuMeas
//      * @param now 
//      *
//      */
//     void predict(const std::array<float,6> imuMeas, uint32_t now);

//     /**
//      * @brief Overload for providing a specific timestep of propagation (mostly for regression testing)
//      * 

//      * @param imuMeas
//      * @param dt 
//      *
//      */
//     void predictRegressionTest(const std::array<float,6> imuMeas, float dt);

//     /**
//      * @brief Given accelerometer measurement, form a tilt measurement and perform update step.
//      * 
//      * @param accelMeas
//      *
//      * @return Struct containing relevant IMU data from this update
//      */
//     tiltData updateTiltMeas(const std::array<float,3> accelMeas);

//     /**
//      * @brief Given magnetometer measurement, perform update step.
//      * 
//      * @param magMeas
//      *
//      * @return Struct containing relevant Magnetometer data from this update
//      */
//     magData updateMagMeas(const std::array<float,3> magMeas);

//         /**
//      * @brief Given altimeter measurement, perform update step.
//      * 
//      * @param altMeas
//      *
//      * @return Struct containing relevant Altimeter data from this update
//      */
//     altData updateAltMeas(const float altMeas);

//         /**
//      * @brief Given gps measurement, perform update step.
//      * 
//      * @param gpsMeas
//      *
//      * @return Struct containing relevant GPS data from this update
//      */
//     gpsData updateGPSMeas(const std::array<float,4> gpsMeas);
//      /**
//      * @brief Sequentially updates filter with ONE element of the tilt / magnetometer reading 
//      * 
//      * @param idxBias Index of the bias state being partitioned by H matrix
//      * @param c1 First index of skew(q2R(qk)* orientationVec), where the row is the associated meas index being updated
//      * @param c2 Second index of skew(q2R(qk)* orientationVec), where the row is the associated meas index being updated
//      * @param c3 Third index of skew(q2R(qk)* orientationVec), where the row is the associated meas index being updated
//      * @param z Scalar measurement being used for update
//      * @param hx Measurement model for the updated state
//      * @param R2 Square of the 1-sigma noise of the measurement
//      * @param NIS Reference to the component of the NIS parameter to be updated
//      * @param P Pass by Reference of the covariance matrix (should be this->P_k)
//      * @param del_xk Pass by Reference of the error state for the current update step. 
//      * @param K Variable to be overwritten with the Kalman gain
//      * @param P_row Variable to be overwritten with the row of the covariance.
//      *
//      * Scalar update the magnetometer and tilt, which includes the bias states
//      */
//     inline void scalarMagTiltUpdate(const int& idxBias, const float& c1, const float& c2, const float&c3, const float& z, const float& hx,const float& R2, float& NIS,  Matrix18f& P, std::array<float,18>& del_xk, std::array<float,18>& K, std::array<float,18>& P_row) {
//         // a. Innovation Covariance (S = H * P * H' + R)
//         // R = sig_alt_vel^2
//         // H * P * H' just returns P(idx,idx)
//         // Because H partitions the covariance
//         // This will return... (For H = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, skew_RVec[0], skew_RVec[1], skew_RVec[2], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f }; )
//         // H * P -> {X,X,X,X,X,X, P(6,6)*c1 + P(7,6)*c2 + P(8,6)*c3 + P(idxBias,6), P(6,7)*c1 + P(7,7)*c2 + P(8,7)*c3 + P(idxBias,7), P(6,8)*c1 + P(7,8)*c2 + P(8,8)*c3 + P(idxBias,8), X, X, X ,X ,X , X , P(6,15)*c1 + P(7,15)*c2 + P(8,15)*c3 + P(idxBias,idxBias), X, X}
//         // (H*P) * H' -->  0+0+0+0+0+0+ c1*(P(6,6)*c1 + P(7,6)*c2 + P(8,6)*c3 + P(15,6)) + c2(P(6,7)*c1 + P(7,7)*c2 + P(8,7)*c3 + P(15,7)) + c3*(P(6,8)*c1 + P(7,8)*c2 + P(8,8)*c3 + P(15,8)) +0+0+0+0+0+0+0 + 1*(P(6,15)*c1 + P(7,15)*c2 + P(8,15)*c3 + P(15,15)) + 0+0
//         // Where c1 c2 c3 are the values of skew_RVec.
//         float S = c1*(P(6,6)*c1 + P(7,6)*c2 + P(8,6)*c3 + P(idxBias,6)) + c2*(P(6,7)*c1 + P(7,7)*c2 + P(8,7)*c3 + P(idxBias,7)) + c3*(P(6,8)*c1 + P(7,8)*c2 + P(8,8)*c3 + P(idxBias,8)) + (P(6,idxBias)*c1 + P(7,idxBias)*c2 + P(8,idxBias)*c3 + P(idxBias,idxBias)) + R2;

//         // b. Kalman Gain (P*H' * inv(S))
//         // P*H' returns 18x1 which returns  P(i,6)*c1 + P(i,7)*c2 + P(i,8)*c3  + P(i,idx_bias)
//         for (unsigned int i = 0; i<18; i++) {
//             K[i] = (P(i,6)*c1 + P(i,7)*c2 + P(i,8)*c3 + P(i,idxBias)) / S; 
//         }

//         // c. Residual del_z = z - h(x)
//         float del_z = z - hx;
        
//         // d. Calculate NIS
//         NIS = (del_z*del_z) / S;
//         // Can employ gating here. Since this is scalar, just do 1DOF. This means can provide 1 p-gate value for ALL measuremente
//         // For 1 DOF, if the value is <6.635, we have a 99 confidence that its okay
//         if (NIS < 6.635) {

//             // e. Update Error State
//             // del_x += K*del_z, which is just multiplying K by del_z and adding it to the indices of del_xk
//             for (unsigned int i = 0; i<18; i++) {
//                 del_xk[i] += K[i] * del_z; 
//             }

//             // f. Update Covariance 
//             // P -= K * H * P
//             // Hardcode H * P, taking advantage of sparity of H
//             P_row = {P(6,0)*c1 + P(7,0)*c2 + P(8,0)*c3 + P(idxBias, 0), 
//                     P(6,1)*c1 + P(7,1)*c2 + P(8,1)*c3 + P(idxBias, 1),
//                     P(6,2)*c1 + P(7,2)*c2 + P(8,2)*c3 + P(idxBias, 2),
//                     P(6,3)*c1 + P(7,3)*c2 + P(8,3)*c3 + P(idxBias, 3),
//                     P(6,4)*c1 + P(7,4)*c2 + P(8,4)*c3 + P(idxBias, 4),
//                     P(6,5)*c1 + P(7,5)*c2 + P(8,5)*c3 + P(idxBias, 5),
//                     P(6,6)*c1 + P(7,6)*c2 + P(8,6)*c3 + P(idxBias, 6),
//                     P(6,7)*c1 + P(7,7)*c2 + P(8,7)*c3 + P(idxBias, 7),
//                     P(6,8)*c1 + P(7,8)*c2 + P(8,8)*c3 + P(idxBias, 8),
//                     P(6,9)*c1 + P(7,9)*c2 + P(8,9)*c3 + P(idxBias, 9),
//                     P(6,10)*c1 + P(7,10)*c2 + P(8,10)*c3 + P(idxBias, 10),
//                     P(6,11)*c1 + P(7,11)*c2 + P(8,11)*c3 + P(idxBias, 11),
//                     P(6,12)*c1 + P(7,12)*c2 + P(8,12)*c3 + P(idxBias, 12),
//                     P(6,13)*c1 + P(7,13)*c2 + P(8,13)*c3 + P(idxBias, 13),
//                     P(6,14)*c1 + P(7,14)*c2 + P(8,14)*c3 + P(idxBias, 14),
//                     P(6,15)*c1 + P(7,15)*c2 + P(8,15)*c3 + P(idxBias, 15),
//                     P(6,16)*c1 + P(7,16)*c2 + P(8,16)*c3 + P(idxBias, 16),
//                     P(6,17)*c1 + P(7,17)*c2 + P(8,17)*c3 + P(idxBias, 17)};
                
//             for (unsigned int i = 0; i<18; i++) { 
//                 for (unsigned int j = 0; j<18; j++) {
//                     //Taking advantage of sparity of K*H, only need to access row idx of P 
//                     P(i,j) -= K[i] * P_row[j];
//                 }
//             }
//         }

//     }

//      /**
//      * @brief Sequentially updates filter with ONE element of the gps / altimeter reading 
//      * 
//      * @param idx Index of the error state being partitioned by H matrix
//      * @param z Scalar measurement being used for update
//      * @param hx Measurement model for the updated state
//      * @param R2 Square of the 1-sigma noise of the measurement
//      * @param NIS Reference to the component of the NIS parameter to be updated
//      * @param P Pass by Reference of the covariance matrix (should be this->P_k)
//      * @param del_xk Pass by Reference of the error state for the current update step. 
//      * @param K Variable to be overwritten with the Kalman gain
//      * @param P_row Variable to be overwritten with the row of the covariance.
//      *
//      * Scalar update the GPS and altimeter, which doesnt include bias states
//      */
//     inline void scalarGPSAltUpdate(const int& idx, const float& z, const float& hx,const float& R2, float& NIS,  Matrix18f& P, std::array<float,18>& del_xk, std::array<float,18>& K, std::array<float,18>& P_row) {
//         // a. Innovation Covariance (S = H * P * H' + R)
//         // R = sig_alt_vel^2
//         // H * P * H' just returns P(idx,idx)
//         // Because H partitions the covariance
//         float S = P(idx,idx) + R2;

//         // b. Kalman Gain (P*H' * inv(S))
//         // P*H' returns 18x11 which returns the idx column of P
//         for (unsigned int i = 0; i<18; i++) {
//             K[i] = P(i,idx) / S; 
//         }
        
//         // c. Residual del_z = z - h(x)
//         float del_z = z - hx;
        
//         // d. Calculate NIS
//         NIS = (del_z*del_z) / S;
//         // Can employ gating here. Since this is scalar, just do 1DOF. This means can provide 1 p-gate value for ALL measuremente
//         if (NIS < 6.635) {
//             // e. Update Error State
//             // del_x += K*del_z, which is just multiplying K by del_z and adding it to the indices of del_xk
//             for (unsigned int i = 0; i<18; i++) {
//                 del_xk[i] += K[i] * del_z; 
//             }

//             // f. Update Covariance 
//             // P -= K * H * P
//             // Taking advantage of sparity of H*P, only need to access row idx of P . Make a copy of the row so we aren't updating the value we need
//             for (int j = 0; j < 18; j++) {
//                 P_row[j] = P(idx, j);
//             }

//             for (unsigned int i = 0; i<18; i++) { 
//                 for (unsigned int j = 0; j<18; j++) {
//                     P(i,j) -= K[i] * P_row[j];
//                 }
//             }
//         }
//     }

//      /**
//      * @brief Inject Error to Update Nominal States if there is at least one update. If not, do nothing
//      * 
//      * This is what takes the estimate from priori to posteriori
//      */
//     inline void injectError() {
//         if (this->updateFlag) {
//             // Unpack Errors
//             Vector3f del_p (this->del_xk[0], this->del_xk[1], this->del_xk[2]);
//             Vector3f del_v (this->del_xk[3], this->del_xk[4], this->del_xk[5]);
//             Quaternion del_q (1, 0.5f*this->del_xk[6],  0.5f*this->del_xk[7],  0.5f*this->del_xk[8]); //alpha = 2 * del_q_xyz
//             Vector3f del_ba (this->del_xk[9], this->del_xk[10], this->del_xk[11]);
//             Vector3f del_bg (this->del_xk[12], this->del_xk[13], this->del_xk[14]);
//             Vector3f del_bm (this->del_xk[15], this->del_xk[16], this->del_xk[17]);
            
//             //Update States
//             this->p_k += del_p;
//             this->v_k += del_v;
//             this->q_k = quatMult(del_q, q_k);
//             this->q_k.normalize();
//             this->ba_k += del_ba;
//             this->bg_k += del_bg;
//             this->bm_k += del_bm;
            
//             this->w_k -= del_bg; // Update for body rates: gyro_meas - (bias_updated) = gyro_meas - (bias_old + del_bg) = old_w_k - del_bg


//             //Reset error states
//             this->del_xk = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f };
//             updateFlag = false;
//         }

//     };


    
//     /**
//      * @brief Set the Magnetometer Reference Vector
//      * 

//      * @param magRef
//      *
//      */
//     void setMagRef(Vector3f magRef);


//     // Getter functions
//     inline Vector3f getPosition() const {
//         return this->p_k;
//     };
//     inline Vector3f getVelocity() const {
//         return this->v_k;
//     };
//     inline Quaternion getQuaternion() const {
//         return this->q_k;
//     };
//     inline Vector3f getBodyRates() const {
//         return this->w_k;
//     };
//     inline Vector3f getAccelBias() const {
//         return this->ba_k;
//     };
//     inline Vector3f getGyroBias() const {
//         return this->bg_k;
//     };
//     inline Vector3f getMagBias() const {
//         return this->bm_k;
//     };
//     inline Matrix18f getCovariance() const {
//         return this->P_k;
//     };
//     inline Matrix18f getQMatrix() const {
//         return this->Qd_;
//     };

//     inline float getSigAcc() const {
//         return this -> sigAcc_;
//     }
//     inline float getSigGyro() const {
//         return this -> sigGyro_;
//     }
//     inline float getEtaAcc() const {
//         return this -> etaAcc_;
//     }
//     inline float getEtaGyro() const {
//         return this -> etaGyro_;
//     }
//     inline float getEtaMag() const {
//         return this -> etaMag_;
//     }

//     inline float getSigMag() const {
//         return this -> sigMag_;
//     }
//     inline float getSigTilt() const {
//         return this -> sigTilt_;
//     }
//     inline float getSigAlt() const {
//         return this -> sigAlt_;
//     }
//     inline float getSigGPSPos() const {
//         return this -> sigGPSPos_;
//     }
//     inline float getSigGPSVel() const {
//         return this -> sigGPSVel_;
//     }

//     //DEBUGGING
//     inline float getDT() const {
//         return this -> dt_;
//     }

//     inline void printStates() const {
        
//         // Serial.print(this->getPosition()[2]); Serial.print(",");  //Print out Height
//         // Serial.print(this->getQuaternion().w()); Serial.print(",");  //Print out qw
//         // Serial.print(this->getQuaternion().x()); Serial.print(",");  //Print out qx
//         // Serial.print(this->getQuaternion().y()); Serial.print(",");  //Print out qy
//         // Serial.print(this->getQuaternion().z()); Serial.print(",");  //Print out qz
//         // Serial.print(this->getBodyRates()[0]); Serial.print(",");  //Print out wx
//         // Serial.print(this->getBodyRates()[1]); Serial.print(",");  //Print out wy
//         // Serial.print(this->getBodyRates()[2]); Serial.print(",");  //Print out wz
//         // Serial.print(this->getGyroBias()[0]); Serial.print(",");  //Print out bgx
//         // Serial.print(this->getGyroBias()[1]); Serial.print(",");  //Print out bgy
//         // Serial.print(this->getGyroBias()[2]); Serial.print(",");  //Print out bgz
//         // Serial.print(this->getMagBias()[0]); Serial.print(",");  //Print out bmx
//         // Serial.print(this->getMagBias()[1]); Serial.print(",");  //Print out bmy
//         // Serial.println(this->getMagBias()[2]);  //Print out bmz

//         Serial.print(this->getQuaternion().w()); Serial.print(",");  //Print out qw
//         Serial.print(this->getQuaternion().x()); Serial.print(",");  //Print out qx
//         Serial.print(this->getQuaternion().y()); Serial.print(",");  //Print out qy
//         Serial.print(this->getQuaternion().z()); Serial.print(",");  //Print out qz
//         Serial.print(this->getAccelBias()[0]); Serial.print(",");  //Print out bax
//         Serial.print(this->getAccelBias()[1]); Serial.print(",");  //Print out bay
//         Serial.print(this->getAccelBias()[2]); Serial.print(",");  //Print out baz
//         Serial.print(this->getGyroBias()[0]); Serial.print(",");  //Print out bgx
//         Serial.print(this->getGyroBias()[1]); Serial.print(",");  //Print out bgy
//         Serial.print(this->getGyroBias()[2]); Serial.print(",");  //Print out bgz
//         Serial.print(this->getMagBias()[0]); Serial.print(",");  //Print out bmx
//         Serial.print(this->getMagBias()[1]); Serial.print(",");  //Print out bmy
//         Serial.print(this->getMagBias()[2]);  Serial.print(",");//Print out bmz

//         std::array<float,18> P_diag = this->P_k.getDiagonal();
//         Serial.print(P_diag[9]); Serial.print(",");  //Print out bax cov
//         Serial.print(P_diag[10]); Serial.print(",");  //Print out bay cov
//         Serial.print(P_diag[11]); Serial.print(",");  //Print out baz cov
//         Serial.print(P_diag[15]); Serial.print(",");  //Print out bmx cov
//         Serial.print(P_diag[16]); Serial.print(",");  //Print out bmy cov
//         Serial.print(P_diag[17]);  Serial.println(",");//Print out bmz cov
//     }


// private:
//     //unsigned long lastPredict = 0; // Used to keep track of dt
//     float dt_; //DEBUGGING


//     //Nominal States
//     Vector3f p_k;
//     Vector3f v_k;
//     Quaternion q_k;
//     Vector3f w_k;
//     Vector3f ba_k; //Keeps track of the bias here 
//     Vector3f bg_k;
//     Vector3f bm_k;
    
//     //Error States
//     Vector3f del_p_k;
//     Vector3f del_v_k;
//     Vector3f alpha_k;
//     Vector3f del_ba_k;
//     Vector3f del_bg_k;
//     Vector3f del_bm_k;
    
//     Matrix18f P_k; //This is the covariance of the ERROR State

//     // NIS
//     std::array<float,4> nisGPS;
//     std::array<float,3> nisMag;
//     std::array<float,3> nisTilt;
//     float nisAlt;


//     //Process Noise
//     Matrix18f Qd_;

//     float sigAcc_;
//     float sigGyro_;
//     float etaAcc_;
//     float etaGyro_;
//     float etaMag_;

//     float sigMag_;
//     float sigTilt_;
//     float sigAlt_;
//     float sigGPSPos_;
//     float sigGPSVel_;

//     // Delta_Error
//     std::array<float,18> del_xk {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f };

//     //Mag Ref
//     Vector3f magRef_;

//     // Prediction and Update Flags/Tokens
//     // Need a way to now if lastFilterTime is initialized or not to keep prediction timestep properly logged
//     uint32_t lastFilterTime = UINT32_MAX; //General time that gets set after each prediction and update, in case measurements wants to be processed seperately

//     bool updateFlag; 

// };
// #endif // ESKF_H