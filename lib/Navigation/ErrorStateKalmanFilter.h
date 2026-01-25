// Error State Kalman Filter for Translational State, Attitude, IMU Biases, and Magnetometer Bias Estiamtion
#ifndef _ESKF_H
#define _ESKF_H

#include "Mathpk.h"
#include "Sensors.h"
#include "Constants.h"
#include "DataTypes.h"
class ErrorStateKalmanFilter {
public:

    /**
     * @brief Init ErrorStateKalmanFilter object. 
     * 
     * @param p0 Initial NED Position [NOMINAL STATE]
     * @param v0 Initial NED Velocity [NOMINAL STATE]
     * @param q0 Initial NED2Body Quaternion [NOMINAL STATE]
     * 
     * @param P0 Initial ERROR State Covariance for Kalman filter [324 x 1] (18x18)
     * 
     * @param sig_acc 1-Sigma Random Disturbance from Accelerometer [PROCESS NOISE]
     * @param sig_gyro 1-Sigma Random Disturbance from Gyro [PROCESS NOISE]
     * 
     * @param sig_m 1-Sigma Noise from Magnetometer Sensor [MEASUREMENT NOISE]
     * @param sig_tilt 1-Sigma Noise from Tilt Sensor [MEASUREMENT NOISE]
     * @param sig_alt 1-Sigma Noise from Altimeter Sensor [MEASUREMENT NOISE]
     * @param sig_gps_pos 1-Sigma Noise from GPS Sensor (Position) [MEASUREMENT NOISE]
     * @param sig_gps_vel 1-sigma Noise from GPS Sensor (Velocity) [MEASUREMENT NOISE ]
     * 
     * @param nisGatingFlag
     *
     * Initializes Filter
     * Expect array inputs so they can be inside SetUp.h as const expr
     */
    ErrorStateKalmanFilter(std::array<float,3> p0, std::array<float,3> v0, std::array<float,4> q0,     //Initial Nominal States
        std::array<float,81> P0,                                                                   //Error State Covariance
        float sig_acc, float sig_gyro,       //Process Noise
        float sig_m, float sig_tilt, float sig_alt, float sig_gps_pos, float sig_gps_vel,
        bool nisGatingFlag); //Measurement Noise

    /**
     * @brief Init Discrete Process Noise Matrix
     * 
     * @param dt Timestep for Prediction
     *
     * Follows Marley's version of the Process Noise Matrix. Qd = int ( STM * Q * STM'), where Q is E[(Gw) * (Gw)'], where Gw is x_k1 = Fxk + Gw from the dynamics
     * STM is linearized about the identity quaternion and 0 acceleration / rates, which works out to our hover state
     */
    Matrix9f getQd(float dt); //Might be able to calculate this every timestep if not too computationally expensive.

    /**
     * @brief Return the Discrete Time State Transition Matrix
     * 
     * @param accelMeas Raw Accelerometer Reading
     * @param gyroMeas Raw Gyro Reading
     *
     * First order linearization of this via Taylor Expansion of Matrix Exponential. Other forms includes higher order expansions, or RK4
     * Since we already calculate the inertial acceleration prior to calling this during the predict step, just pass that in directly
     */
    Matrix9f getSTM(const Quaternion& q, const Vector3f& accelMeas, const Vector3f& gyroMeas, const float dt ) const; 
    

    /**
     * @brief Propagates the covariance matrix forward
     * 
     * @param q Current Estiamte of Quaternion
     * @param accelMeas Raw Accelerometer Reading
     * @param gyroMeas Raw Gyro Reading
     * @param dt Step Size
     *
     * First order linearization of this via Taylor Expansion of Matrix Exponential. Other forms includes higher order expansions, or RK4
     * Since we already calculate the inertial acceleration prior to calling this during the predict step, just pass that in directly
     */
    void propagateCovariance(const Quaternion& q, const Vector3f& accelMeas, const Vector3f& gyroMeas, const float dt );

    /**
     * @brief Calls on propagateCovaraince along with propagating state estimate forward via Euler Integration
     * 

     * @param imuMeas
     * @param now 
     *
     */
    void predict(const std::array<float,6> imuMeas, uint32_t now);

    /**
     * @brief Overload for providing a specific timestep of propagation (mostly for regression testing)
     * 

     * @param imuMeas
     * @param dt 
     *
     */
    void predictRegressionTest(const std::array<float,6> imuMeas, float dt);

    /**
     * @brief Given accelerometer measurement, form a tilt measurement and perform update step.
     * 
     * @param accelMeas
     *
     * @return Struct containing relevant IMU data from this update
     */
    tiltData updateTiltMeas(const std::array<float,3> accelMeas);

    /**
     * @brief Given magnetometer measurement, perform update step.
     * 
     * @param magMeas
     *
     * @return Struct containing relevant Magnetometer data from this update
     */
    magData updateMagMeas(const std::array<float,3> magMeas);

        /**
     * @brief Given altimeter measurement, perform update step.
     * 
     * @param altMeas
     *
     * @return Struct containing relevant Altimeter data from this update
     */
    altData updateAltMeas(const float altMeas);

        /**
     * @brief Given gps measurement, perform update step.
     * 
     * @param gpsMeas
     *
     * @return Struct containing relevant GPS data from this update
     */
    gpsData updateGPSMeas(const std::array<float,4> gpsMeas);
     /**
     * @brief Sequentially updates filter with ONE element of the tilt / magnetometer reading 
     * 
     * @param c1 First index of skew(q2R(qk)* orientationVec), where the row is the associated meas index being updated
     * @param c2 Second index of skew(q2R(qk)* orientationVec), where the row is the associated meas index being updated
     * @param c3 Third index of skew(q2R(qk)* orientationVec), where the row is the associated meas index being updated
     * @param z Scalar measurement being used for update
     * @param hx Measurement model for the updated state
     * @param R2 Square of the 1-sigma noise of the measurement
     * @param NIS Reference to the component of the NIS parameter to be updated
     * @param P Pass by Reference of the covariance matrix (should be this->P_k)
     * @param del_xk Pass by Reference of the error state for the current update step. 
     * @param K Variable to be overwritten with the Kalman gain
     * @param P_row Variable to be overwritten with the row of the covariance.
     *
     * Scalar update the magnetometer and tilt, which includes the bias states
     */
    inline void scalarMagTiltUpdate(const float& c1, const float& c2, const float&c3, const float& z, const float& hx,const float& R2, float& NIS,  Matrix9f& P, std::array<float,9>& del_xk, std::array<float,9>& K, std::array<float,9>& P_row) {
        // a. Innovation Covariance (S = H * P * H' + R)
        // R = sig_alt_vel^2
        // H * P * H' just returns P(idx,idx)
        // Because H partitions the covariance
        // This will return... (For H = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, skew_RVec[0], skew_RVec[1], skew_RVec[2]}; )
        // H * P -> {X,X,X,X,X,X, P(6,6)*c1 + P(7,6)*c2 + P(8,6)*c3 , P(6,7)*c1 + P(7,7)*c2 + P(8,7)*c3 , P(6,8)*c1 + P(7,8)*c2 + P(8,8)*c3}
        // (H*P) * H' -->  0+0+0+0+0+0+ c1*(P(6,6)*c1 + P(7,6)*c2 + P(8,6)*c3) + c2(P(6,7)*c1 + P(7,7)*c2 + P(8,7)*c3 ) + c3*(P(6,8)*c1 + P(7,8)*c2 + P(8,8)*c3 ) 
        // Where c1 c2 c3 are the values of skew_RVec.
        float S = c1*(P(6,6)*c1 + P(7,6)*c2 + P(8,6)*c3) + c2*(P(6,7)*c1 + P(7,7)*c2 + P(8,7)*c3 ) + c3*(P(6,8)*c1 + P(7,8)*c2 + P(8,8)*c3 )  + R2;

        // b. Kalman Gain (P*H' * inv(S))
        // P*H' returns 9x1 which returns  P(i,6)*c1 + P(i,7)*c2 + P(i,8)*c3  
        for (unsigned int i = 0; i<9; i++) {
            K[i] = (P(i,6)*c1 + P(i,7)*c2 + P(i,8)*c3 ) / S; 
        }

        // c. Residual del_z = z - h(x)
        float del_z = z - hx;
        
        // d. Calculate NIS
        NIS = (del_z*del_z) / S;
        // Can employ gating here. Since this is scalar, just do 1DOF. This means can provide 1 p-gate value for ALL measuremente
        // For 1 DOF, if the value is <6.635, we have a 99 confidence that its okay
        if (static_cast<float>(nisGatingFlag_)*NIS < 6.635) {

            // e. Update Error State
            // del_x += K*del_z, which is just multiplying K by del_z and adding it to the indices of del_xk
            for (unsigned int i = 0; i<9; i++) {
                del_xk[i] += K[i] * del_z; 
            }

            // f. Update Covariance 
            // P -= K * H * P
            // Hardcode H * P, taking advantage of sparity of H
            P_row = {P(6,0)*c1 + P(7,0)*c2 + P(8,0)*c3 ,
                    P(6,1)*c1 + P(7,1)*c2 + P(8,1)*c3 ,
                    P(6,2)*c1 + P(7,2)*c2 + P(8,2)*c3 ,
                    P(6,3)*c1 + P(7,3)*c2 + P(8,3)*c3 ,
                    P(6,4)*c1 + P(7,4)*c2 + P(8,4)*c3 ,
                    P(6,5)*c1 + P(7,5)*c2 + P(8,5)*c3 ,
                    P(6,6)*c1 + P(7,6)*c2 + P(8,6)*c3 ,
                    P(6,7)*c1 + P(7,7)*c2 + P(8,7)*c3 ,
                    P(6,8)*c1 + P(7,8)*c2 + P(8,8)*c3};
                
            for (unsigned int i = 0; i<9; i++) { 
                for (unsigned int j = 0; j<9; j++) {
                    //Taking advantage of sparity of K*H, only need to access row idx of P 
                    P(i,j) -= K[i] * P_row[j];
                }
            }
        }

    }

     /**
     * @brief Sequentially updates filter with ONE element of the gps / altimeter reading 
     * 
     * @param idx Index of the error state being partitioned by H matrix
     * @param z Scalar measurement being used for update
     * @param hx Measurement model for the updated state
     * @param R2 Square of the 1-sigma noise of the measurement
     * @param NIS Reference to the component of the NIS parameter to be updated
     * @param P Pass by Reference of the covariance matrix (should be this->P_k)
     * @param del_xk Pass by Reference of the error state for the current update step. 
     * @param K Variable to be overwritten with the Kalman gain
     * @param P_row Variable to be overwritten with the row of the covariance.
     *
     * Scalar update the GPS and altimeter, which doesnt include bias states
     */
    inline void scalarGPSAltUpdate(const int& idx, const float& z, const float& hx,const float& R2, float& NIS,  Matrix9f& P, std::array<float,9>& del_xk, std::array<float,9>& K, std::array<float,9>& P_row) {
        // a. Innovation Covariance (S = H * P * H' + R)
        // R = sig_alt_vel^2
        // H * P * H' just returns P(idx,idx)
        // Because H partitions the covariance
        float S = P(idx,idx) + R2;

        // b. Kalman Gain (P*H' * inv(S))
        // P*H' returns 18x11 which returns the idx column of P
        for (unsigned int i = 0; i<9; i++) {
            K[i] = P(i,idx) / S; 
        }
        
        // c. Residual del_z = z - h(x)
        float del_z = z - hx;
        
        // d. Calculate NIS
        NIS = (del_z*del_z) / S;
        // Can employ gating here. Since this is scalar, just do 1DOF. This means can provide 1 p-gate value for ALL measuremente
        if (static_cast<float>(nisGatingFlag_)*NIS < 6.635) {
            // e. Update Error State
            // del_x += K*del_z, which is just multiplying K by del_z and adding it to the indices of del_xk
            for (unsigned int i = 0; i<9; i++) {
                del_xk[i] += K[i] * del_z; 
            }

            // f. Update Covariance 
            // P -= K * H * P
            // Taking advantage of sparity of H*P, only need to access row idx of P . Make a copy of the row so we aren't updating the value we need
            for (int j = 0; j < 9; j++) {
                P_row[j] = P(idx, j);
            }

            for (unsigned int i = 0; i<9; i++) { 
                for (unsigned int j = 0; j<9; j++) {
                    P(i,j) -= K[i] * P_row[j];
                }
            }
        }
    }

     /**
     * @brief Inject Error to Update Nominal States if there is at least one update. If not, do nothing
     * 
     * This is what takes the estimate from priori to posteriori
     */
    inline void injectError() {
        if (this->updateFlag) {
            // Unpack Errors
            Vector3f del_p (this->del_xk[0], this->del_xk[1], this->del_xk[2]);
            Vector3f del_v (this->del_xk[3], this->del_xk[4], this->del_xk[5]);
            Quaternion del_q (1, 0.5f*this->del_xk[6],  0.5f*this->del_xk[7],  0.5f*this->del_xk[8]); //alpha = 2 * del_q_xyz
            Vector3f del_ba (this->del_xk[9], this->del_xk[10], this->del_xk[11]);
            Vector3f del_bg (this->del_xk[12], this->del_xk[13], this->del_xk[14]);
            Vector3f del_bm (this->del_xk[15], this->del_xk[16], this->del_xk[17]);
            
            //Update States
            this->p_k += del_p;
            this->v_k += del_v;
            this->q_k = quatMult(del_q, q_k);
            this->q_k.normalize();

            //Reset error states
            this->del_xk = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
            updateFlag = false;
        }

    };


    
    /**
     * @brief Set the Magnetometer Reference Vector
     * 

     * @param magRef
     *
     */
    void setMagRef(Vector3f magRef);


    // Getter functions
    inline Vector3f getPosition() const {
        return this->p_k;
    };
    inline Vector3f getVelocity() const {
        return this->v_k;
    };
    inline Quaternion getQuaternion() const {
        return this->q_k;
    };
    inline Vector3f getBodyRates() const {
        return this->w_k;
    };

    inline Matrix9f getCovariance() const {
        return this->P_k;
    };
    inline Matrix9f getQMatrix() const {
        return this->Qd_;
    };

    inline float getSigAcc() const {
        return this -> sigAcc_;
    }
    inline float getSigGyro() const {
        return this -> sigGyro_;
    }

    inline float getSigMag() const {
        return this -> sigMag_;
    }
    inline float getSigTilt() const {
        return this -> sigTilt_;
    }
    inline float getSigAlt() const {
        return this -> sigAlt_;
    }
    inline float getSigGPSPos() const {
        return this -> sigGPSPos_;
    }
    inline float getSigGPSVel() const {
        return this -> sigGPSVel_;
    }

    //DEBUGGING
    inline float getDT() const {
        return this -> dt_;
    }

    inline void printStates() const {
        
        Serial.print(this->getPosition()[2]); Serial.print(",");  //Print out Height
        Serial.print(this->getVelocity()[2]); Serial.print(",");

        Serial.print(this->getQuaternion().w()); Serial.print(",");  //Print out qw
        Serial.print(this->getQuaternion().x()); Serial.print(",");  //Print out qx
        Serial.print(this->getQuaternion().y()); Serial.print(",");  //Print out qy
        Serial.print(this->getQuaternion().z()); Serial.print(",");  //Print out qz
        Serial.print(this->getBodyRates()[0]); Serial.print(",");  //Print out wx
        Serial.print(this->getBodyRates()[1]); Serial.print(",");  //Print out wy
        Serial.print(this->getBodyRates()[2]); Serial.print(",");  //Print out wz

        std::array<float,9> P_diag = this->P_k.getDiagonal();
        Serial.print(P_diag[3]); Serial.print(",");
        Serial.print(P_diag[5]); Serial.print(",");
        Serial.print(P_diag[6]); Serial.print(",");
        Serial.print(P_diag[7]); Serial.print(",");
        Serial.print(P_diag[8]); Serial.println(",");

    }


private:
    //unsigned long lastPredict = 0; // Used to keep track of dt
    float dt_; //DEBUGGING


    //Nominal States
    Vector3f p_k;
    Vector3f v_k;
    Quaternion q_k;
    Vector3f w_k;
    
    //Error States
    Vector3f del_p_k;
    Vector3f del_v_k;
    Vector3f alpha_k;
    
    Matrix9f P_k; //This is the covariance of the ERROR State

    // NIS
    std::array<float,4> nisGPS;
    std::array<float,3> nisMag;
    std::array<float,3> nisTilt;
    float nisAlt;


    //Process Noise
    Matrix9f Qd_;

    float sigAcc_;
    float sigGyro_;

    float sigMag_;
    float sigTilt_;
    float sigAlt_;
    float sigGPSPos_;
    float sigGPSVel_;

    // Delta_Error
    std::array<float,9> del_xk {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

    //Mag Ref
    Vector3f magRef_;

    // Prediction and Update Flags/Tokens
    // Need a way to now if lastFilterTime is initialized or not to keep prediction timestep properly logged
    uint32_t lastFilterTime = UINT32_MAX; //General time that gets set after each prediction and update, in case measurements wants to be processed seperately

    bool updateFlag; 
    bool nisGatingFlag_;
};
#endif // ESKF_H