// Error State Kalman Filter for Translational State, Attitude, IMU Biases, and Magnetometer Bias Estiamtion
#ifndef _ESKF_H
#define _ESKF_H

#include "Mathpk.h"
#include "Sensors.h"
#include "Constants.h"
class ESKF {
public:

    /**
     * @brief Init ESKF object. 
     * 
     * @param p0 Initial NED Position [NOMINAL STATE]
     * @param v0 Initial NED Velocity [NOMINAL STATE]
     * @param q0 Initial NED2Body Quaternion [NOMINAL STATE]
     * @param ba0 Initial Accelerometer Bias [NOMINAL STATE]
     * @param bg0 Initial Gyro Bias [NOMINAL STATE]
     * @param bm0 Initial Magnetometer Bias [NOMINAL STATE]
     * 
     * @param P0 Initial ERROR State Covariance for Kalman filter [324 x 1] (18x18)
     * 
     * @param dt Ideal Filter Timestep.
     * 
     * @param sig_acc 1-Sigma Random Disturbance from Accelerometer [PROCESS NOISE]
     * @param sig_gyro 1-Sigma Random Disturbance from Gyro [PROCESS NOISE]
     * @param eta_acc 1-Sigma Random Disturbance to Accelerometer Bias [PROCESS NOISE]
     * @param eta_gyro 1-Sigma Random Disturbance to Gyro Bias [PROCESS NOISE]
     * @param eta_mag 1-Sigma Random Disturbance to Magnetometer Bias [PROCESS NOISE]
     * 
     * @param sig_m 1-Sigma Noise from Magnetometer Sensor [MEASUREMENT NOISE]
     * @param sig_tilt 1-Sigma Noise from Tilt Sensor [MEASUREMENT NOISE]
     * @param sig_alt 1-Sigma Noise from Altimeter Sensor [MEASUREMENT NOISE]
     * @param sig_gps_pos 1-Sigma Noise from GPS Sensor (Position) [MEASUREMENT NOISE]
     * @param sig_gps_vel 1-sigma Noise from GPS Sensor (Velocity) [MEASUREMENT NOISE ]
     *
     * Initializes Filter
     */
    ESKF(Vector3f p0, Vector3f v0, Quaternion q0, Vector3f ba0, Vector3f bg0, Vector3f bm0,     //Initial Nominal States
        Matrix18f P0,                                                              //Error State Covariance
        float dt,                                                                              // Filter Timestep
        float sig_acc, float sig_gyro, float eta_acc, float eta_gyro, float eta_mag,       //Process Noise
        float sig_m, float sig_tilt, float sig_alt, float sig_gps_pos, float sig_gps_vel); //Measurement Noise

    /**
     * @brief Init Discrete Process Noise Matrix
     * 
     * @param dt Nominal Timestep for Filter
     *
     * Follows Marley's version of the Process Noise Matrix. Qd = int ( STM * Q * STM'), where Q is E[(Gw) * (Gw)'], where Gw is x_k1 = Fxk + Gw from the dynamics
     * STM is linearized about the identity quaternion and 0 acceleration / rates, which works out to our hover state
     */
    void initQd(float dt); //Might be able to calculate this every timestep if not too computationally expensive.

    /**
     * @brief Return the Discrete Time State Transition Matrix
     * 
     * @param accelMeas Raw Accelerometer Reading
     * @param gyroMeas Raw Gyro Reading
     *
     * First order linearization of this via Taylor Expansion of Matrix Exponential. Other forms includes higher order expansions, or RK4
     * Since we already calculate the inertial acceleration prior to calling this during the predict step, just pass that in directly
     */
    Matrix18f getSTM(const Quaternion& q, const Vector3f& accelBias,  const Vector3f& gyroBias, const Vector3f& accelMeas, const Vector3f& gyroMeas, const float dt ) const; 
    

    /**
     * @brief Propagates the covariance matrix forward
     * 
     * @param accelMeas Raw Accelerometer Reading
     * @param gyroMeas Raw Gyro Reading
     *
     * First order linearization of this via Taylor Expansion of Matrix Exponential. Other forms includes higher order expansions, or RK4
     * Since we already calculate the inertial acceleration prior to calling this during the predict step, just pass that in directly
     */
    void propagateCovariance(const Quaternion& q, const Vector3f& accelBias,  const Vector3f& gyroBias, const Vector3f& accelMeas, const Vector3f& gyroMeas, const float dt );


    void predict(const Vector3f& accelMeas, const Vector3f& gyroMeas);


    void update(const Vector3f& magMeas, const Vector3f& tiltMeas, const float& altMeas,const std::array<float,4>& gpsMeas);

     /**
     * @brief Sequentially updates filter with ONE element of the tilt / magnetometer reading 
     * 
     * @param idxBias Index of the bias state being partitioned by H matrix
     * @param c1 First index of skew(q2R(qk)* orientationVec), where the row is the associated meas index being updated
     * @param c2 Second index of skew(q2R(qk)* orientationVec), where the row is the associated meas index being updated
     * @param c3 Third index of skew(q2R(qk)* orientationVec), where the row is the associated meas index being updated
     * @param z Scalar measurement being used for update
     * @param hx Measurement model for the updated state
     * @param R2 Square of the 1-sigma noise of the measurement
     * @param P Pass by Reference of the covariance matrix (should be this->P_k)
     * @param del_xk Pass by Reference of the error state for the current update step. 
     * @param K Variable to be overwritten with the Kalman gain
     * @param P_row Variable to be overwritten with the row of the covariance.
     *
     * Scalar update the magnetometer and tilt, which includes the bias states
     */
    inline void scalarMagTiltUpdate(const int& idxBias, const float& c1, const float& c2, const float&c3, const float& z, const float& hx,const float& R2, Matrix18f& P, std::array<float,18>& del_xk, std::array<float,18>& K, std::array<float,18>& P_row) {
        // a. Innovation Covariance (S = H * P * H' + R)
        // R = sig_alt_vel^2
        // H * P * H' just returns P(idx,idx)
        // Because H partitions the covariance
        // This will return... (For H = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, skew_RVec[0], skew_RVec[1], skew_RVec[2], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f }; )
        // H * P -> {X,X,X,X,X,X, P(6,6)*c1 + P(7,6)*c2 + P(8,6)*c3 + P(idxBias,6), P(6,7)*c1 + P(7,7)*c2 + P(8,7)*c3 + P(idxBias,7), P(6,8)*c1 + P(7,8)*c2 + P(8,8)*c3 + P(idxBias,8), X, X, X ,X ,X , X , P(6,15)*c1 + P(7,15)*c2 + P(8,15)*c3 + P(idxBias,idxBias), X, X}
        // (H*P) * H' -->  0+0+0+0+0+0+ c1*(P(6,6)*c1 + P(7,6)*c2 + P(8,6)*c3 + P(15,6)) + c2(P(6,7)*c1 + P(7,7)*c2 + P(8,7)*c3 + P(15,7)) + c3*(P(6,8)*c1 + P(7,8)*c2 + P(8,8)*c3 + P(15,8)) +0+0+0+0+0+0+0 + 1*(P(6,15)*c1 + P(7,15)*c2 + P(8,15)*c3 + P(15,15)) + 0+0
        // Where c1 c2 c3 are the values of skew_RVec.
        float S = c1*(P(6,6)*c1 + P(7,6)*c2 + P(8,6)*c3 + P(idxBias,6)) + c2*(P(6,7)*c1 + P(7,7)*c2 + P(8,7)*c3 + P(idxBias,7)) + c3*(P(6,8)*c1 + P(7,8)*c2 + P(8,8)*c3 + P(idxBias,8)) + (P(6,idxBias)*c1 + P(7,idxBias)*c2 + P(8,idxBias)*c3 + P(idxBias,idxBias)) + R2;

        // b. Kalman Gain (P*H' * inv(S))
        // P*H' returns 18x1 which returns  P(i,6)*c1 + P(i,7)*c2 + P(i,8)*c3  + P(i,idx_bias)
        for (unsigned int i = 0; i<18; i++) {
            K[i] = (P(i,6)*c1 + P(i,7)*c2 + P(i,8)*c3 + P(i,idxBias)) / S; 
        }

        // c. Residual del_z = z - h(x)
        float del_z = z - hx;
        
        // Can Calculate NIS here...

        // d. Update Error State
        // del_x += K*del_z, which is just multiplying K by del_z and adding it to the indices of del_xk
        for (unsigned int i = 0; i<18; i++) {
            del_xk[i] += K[i] * del_z; 
        }

        // e. Update Covariance 
        // P -= K * H * P
        // Hardcode H * P, taking advantage of sparity of H
        P_row = {P(6,0)*c1 + P(7,0)*c2 + P(8,0)*c3 + P(idxBias, 0), 
                P(6,1)*c1 + P(7,1)*c2 + P(8,1)*c3 + P(idxBias, 1),
                P(6,2)*c1 + P(7,2)*c2 + P(8,2)*c3 + P(idxBias, 2),
                P(6,3)*c1 + P(7,3)*c2 + P(8,3)*c3 + P(idxBias, 3),
                P(6,4)*c1 + P(7,4)*c2 + P(8,4)*c3 + P(idxBias, 4),
                P(6,5)*c1 + P(7,5)*c2 + P(8,5)*c3 + P(idxBias, 5),
                P(6,6)*c1 + P(7,6)*c2 + P(8,6)*c3 + P(idxBias, 6),
                P(6,7)*c1 + P(7,7)*c2 + P(8,7)*c3 + P(idxBias, 7),
                P(6,8)*c1 + P(7,8)*c2 + P(8,8)*c3 + P(idxBias, 8),
                P(6,9)*c1 + P(7,9)*c2 + P(8,9)*c3 + P(idxBias, 9),
                P(6,10)*c1 + P(7,10)*c2 + P(8,10)*c3 + P(idxBias, 10),
                P(6,11)*c1 + P(7,11)*c2 + P(8,11)*c3 + P(idxBias, 11),
                P(6,12)*c1 + P(7,12)*c2 + P(8,12)*c3 + P(idxBias, 12),
                P(6,13)*c1 + P(7,13)*c2 + P(8,13)*c3 + P(idxBias, 13),
                P(6,14)*c1 + P(7,14)*c2 + P(8,14)*c3 + P(idxBias, 14),
                P(6,15)*c1 + P(7,15)*c2 + P(8,15)*c3 + P(idxBias, 15),
                P(6,16)*c1 + P(7,16)*c2 + P(8,16)*c3 + P(idxBias, 16),
                P(6,17)*c1 + P(7,17)*c2 + P(8,17)*c3 + P(idxBias, 17)};
            
        for (unsigned int i = 0; i<18; i++) { 
            for (unsigned int j = 0; j<18; j++) {
                //Taking advantage of sparity of K*H, only need to access row idx of P 
                P(i,j) -= K[i] * P_row[j];
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
     * @param P Pass by Reference of the covariance matrix (should be this->P_k)
     * @param del_xk Pass by Reference of the error state for the current update step. 
     * @param K Variable to be overwritten with the Kalman gain
     * @param P_row Variable to be overwritten with the row of the covariance.
     *
     * Scalar update the GPS and altimeter, which doesnt include bias states
     */
    inline void scalarGPSAltUpdate(const int& idx, const float& z, const float& hx,const float& R2, Matrix18f& P, std::array<float,18>& del_xk, std::array<float,18>& K, std::array<float,18>& P_row) {
        // a. Innovation Covariance (S = H * P * H' + R)
        // R = sig_alt_vel^2
        // H * P * H' just returns P(idx,idx)
        // Because H partitions the covariance
        float S = P(idx,idx) + R2;

        // b. Kalman Gain (P*H' * inv(S))
        // P*H' returns 18x11 which returns the idx column of P
        for (unsigned int i = 0; i<18; i++) {
            K[i] = P(i,idx) / S; 
        }
        
        // c. Residual del_z = z - h(x)
        float del_z = z - hx;
        
        // Can Calculate NIS here...

        // d. Update Error State
        // del_x += K*del_z, which is just multiplying K by del_z and adding it to the indices of del_xk
        for (unsigned int i = 0; i<18; i++) {
            del_xk[i] += K[i] * del_z; 
        }

        // e. Update Covariance 
        // P -= K * H * P
        // Taking advantage of sparity of H*P, only need to access row idx of P . Make a copy of the row so we aren't updating the value we need
        for (int j = 0; j < 18; j++) {
            P_row[j] = P(idx, j);
        }

        for (unsigned int i = 0; i<18; i++) { 
            for (unsigned int j = 0; j<18; j++) {
                P(i,j) -= K[i] * P_row[j];
            }
        }
    }

     /**
     * @brief Inject Error to Update Nominal States
     * 
     * @param del_x Error State after using all avaliable measurements
     * 
     * This is what takes the estimate from priori to posteriori
     */
    inline void injectError(const std::array<float,18>& del_x) {
        // Unpack Errors
        Vector3f del_p (del_x[0], del_x[1], del_x[2]);
        Vector3f del_v (del_x[3], del_x[4], del_x[5]);
        Quaternion del_q (1, 0.5f*del_x[6],  0.5f*del_x[7],  0.5f*del_x[8]); //alpha = 2 * del_q_xyz
        Vector3f del_ba (del_x[9], del_x[10], del_x[11]);
        Vector3f del_bg (del_x[12], del_x[13], del_x[14]);
        Vector3f del_bm (del_x[15], del_x[16], del_x[17]);
 
        this->p_k += del_p;
        this->v_k += del_v;
        this->q_k = quatMult(del_q, q_k);
        this->q_k.normalize();
        this->ba_k += del_ba;
        this->bg_k += del_bg;
        this->bm_k += del_bm;

    };

    void step(std::array<float,14> z);

    
    // Getter functions. More for regression testing
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
    inline Vector3f getAccelBias() const {
        return this->ba_k;
    };
    inline Vector3f getGyroBias() const {
        return this->bg_k;
    };
    inline Vector3f getMagBias() const {
        return this->bm_k;
    };
    inline Matrix18f getCovariance() const {
        return this->P_k;
    };
    inline Matrix18f getQMatrix() const {
        return this->Qd_;
    };

    inline float getDt() const {
        return this -> dt_;
    }

    inline float getSigAcc() const {
        return this -> sigAcc_;
    }
    inline float getSigGyro() const {
        return this -> sigGyro_;
    }
    inline float getEtaAcc() const {
        return this -> etaAcc_;
    }
    inline float getEtaGyro() const {
        return this -> etaGyro_;
    }
    inline float getEtaMag() const {
        return this -> etaMag_;
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

private:
    //unsigned long lastPredict = 0; // Used to keep track of dt

    //Nominal States
    Vector3f p_k;
    Vector3f v_k;
    Quaternion q_k;
    Vector3f w_k;
    Vector3f ba_k; //Keeps track of the bias here 
    Vector3f bg_k;
    Vector3f bm_k;
    
    //Error States
    Vector3f del_p_k;
    Vector3f del_v_k;
    Vector3f alpha_k;
    Vector3f del_ba_k;
    Vector3f del_bg_k;
    Vector3f del_bm_k;
    
    Matrix18f P_k; //This is the covariance of the ERROR State

    float dt_;

    //Process Noise
    Matrix18f Qd_;

    float sigAcc_;
    float sigGyro_;
    float etaAcc_;
    float etaGyro_;
    float etaMag_;

    float sigMag_;
    float sigTilt_;
    float sigAlt_;
    float sigGPSPos_;
    float sigGPSVel_;



};
#endif // ESKF_H