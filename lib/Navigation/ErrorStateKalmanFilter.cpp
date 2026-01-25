#include "ErrorStateKalmanFilter.h"


ErrorStateKalmanFilter::ErrorStateKalmanFilter(std::array<float,3> p0, std::array<float,3> v0, std::array<float,4> q0,//Initial Nominal States
            std::array<float,81> P0,                                                              //Error State Covariance
            float sig_acc, float sig_gyro,  //Process Noise
            float sig_m, float sig_tilt, float sig_alt, float sig_gps_pos, float sig_gps_vel, bool nisGatingFlag):
            p_k(p0), v_k(v0), q_k(q0), w_k (std::array<float,3>{0,0,0}), 
            P_k(P0),
            sigAcc_(sig_acc), sigGyro_(sig_gyro),
            sigMag_(sig_m), sigTilt_(sig_tilt), sigAlt_(sig_alt), sigGPSPos_(sig_gps_pos), sigGPSVel_(sig_gps_vel),
            nisGatingFlag_(nisGatingFlag)
            {}; 


//Drone Sim has this in a symbolic form. Verify with that. Can also verify with Marley Paper
Matrix9f ErrorStateKalmanFilter::getQd(float dt) {
    float sigAcc2 = this->sigAcc_ * this->sigAcc_;
    float sigGyro2 = this->sigGyro_ * this->sigGyro_;
    float t2 = dt*dt;
    float t3 = t2*dt;

    //1. Zero out Qd
    Matrix9f Qd;
    Qd.setZero();

    // --- Position & velocity blocks ---
    Qd(0,0) = sigAcc2*t3/3.0f;   Qd(0,3) = sigAcc2*t2/2.0f;        
    Qd(1,1) = sigAcc2*t3/3.0f;   Qd(1,4) = sigAcc2*t2/2.0f;      
    Qd(2,2) = sigAcc2*t3/3.0f;   Qd(2,5) = sigAcc2*t2/2.0f;      
    Qd(3,0) = sigAcc2*t2/2.0f;   Qd(3,3) = sigAcc2*dt;  
    Qd(4,1) = sigAcc2*t2/2.0f;   Qd(4,4) = sigAcc2*dt;  
    Qd(5,2) = sigAcc2*t2/2.0f;   Qd(5,5) = sigAcc2*dt;  

    // --- Alpha blocks ---
    Qd(6,6) = sigGyro2*dt;  
    Qd(7,7) = sigGyro2*dt;   
    Qd(8,8) = sigGyro2*dt;   
    
    return Qd;


}

//TODO:: FIgure out if we need f in body can we leave it in inertial frame. revisit derivation
Matrix9f ErrorStateKalmanFilter::getSTM(const Quaternion& q, const Vector3f& accelMeas, const Vector3f& gyroMeas, const float dt ) const{
    //0. Get all necessary variables
    //a. Get the total acceleration in the BODY FRAME. Can probabaly reformulate this to be in the inertial frame and skip the rotation.
    Vector3f aB = (accelMeas ) + (q2R(q) * Vector3f(0.0f,0.0f, CONSTANTS::g0)); 
    Vector3f wB = gyroMeas ;
    //b. Unpack variables 
    float qw = q.w();
    float qx = q.x();
    float qy = q.y();
    float qz = q.z();

    float fx = aB[0];
    float fy = aB[1];
    float fz = aB[2];

    float wx = wB[0];
    float wy = wB[1];
    float wz = wB[2];

    // 2. Precompute repeated quaternion terms
    float two_qw_qx = 2.0f * qw * qx;
    float two_qw_qy = 2.0f * qw * qy;
    float two_qw_qz = 2.0f * qw * qz;
    float two_qx_qy = 2.0f * qx * qy;
    float two_qx_qz = 2.0f * qx * qz;
    float two_qy_qz = 2.0f * qy * qz;
    float two_qx2 = 2.0f * qx * qx;
    float two_qy2 = 2.0f * qy * qy;
    float two_qz2 = 2.0f * qz * qz;

    Matrix9f STM;
    STM.setIdentity(); // fill diagonal with 1

    // TODO:: To speed this up, make STM a member. THen just update the index that are not just dt, since we can keep dt
    // 3. Fill position blocks (dp/dx)
    STM(0,3) = dt;
    STM(1,4) = dt;
    STM(2,5) = dt;

    // 4. Fill velocity block (dv / dx)
    STM(3,6) =  (fy*(two_qw_qy + two_qx_qz) + fz*(two_qw_qz - two_qx_qy)) * dt; 
    STM(3,7) = -(fz*(two_qy2 + two_qz2 -1) + fx*(two_qw_qy + two_qx_qz)) * dt;
    STM(3,8) =  (fy*(two_qy2 + two_qz2 -1) - fx*(two_qw_qz - two_qx_qy)) * dt;

    STM(4,6) =  (fz*(two_qx2 + two_qz2 -1) - fy*(two_qw_qx - two_qy_qz)) * dt;
    STM(4,7) = (fx*(two_qw_qx - two_qy_qz) + fz*(two_qw_qz + two_qx_qy)) * dt;
    STM(4,8) = -(fx*(two_qx2 + two_qz2 -1) + fy*(two_qw_qz + two_qx_qy)) * dt;

    STM(5,6) = -(fy*(two_qx2 + two_qy2 -1) + fz*(two_qw_qx + two_qy_qz)) * dt;
    STM(5,7) = (fx*(two_qx2 + two_qy2 -1) - fz*(two_qw_qy - two_qx_qz)) * dt;
    STM(5,8) =  (fx*(two_qw_qx + two_qy_qz) + fy*(two_qw_qy - two_qx_qz)) * dt;

    // 5. Fill in alpha block (d alpha / dx) 
    //Fill angular rate skew blocks (alpha_dot = -skew(wB)*alpha)
    STM(6,7) =  wz*dt; STM(6,8) = -wy*dt; 
    STM(7,6) = -wz*dt; STM(7,8) =  wx*dt; 
    STM(8,6) =  wy*dt; STM(8,7) = -wx*dt;

    return STM;

}


void ErrorStateKalmanFilter::propagateCovariance(const Quaternion& q, const Vector3f& accelMeas, const Vector3f& gyroMeas, const float dt ) {
    Matrix9f STM = this->getSTM(q, accelMeas,gyroMeas, dt);
    Matrix9f Qd = this-> getQd(dt);
    //Can probably take advantage of the sparity of the matrices here to speed things up significantly (see update step)
    this->P_k = (STM * this -> P_k) * STM.transpose() + Qd; 
}


void ErrorStateKalmanFilter::predict(const std::array<float,6> imuMeas, uint32_t now) {

    if (lastFilterTime == UINT32_MAX) { 
        lastFilterTime = now;  // set the initial timestamp
    } else {
        Vector3f accelMeas (imuMeas[0], imuMeas[1], imuMeas[2]);
        Vector3f gyroMeas (imuMeas[3], imuMeas[4], imuMeas[5]);
        //Extract out all previous values
        Quaternion q_temp = this->q_k;
        Vector3f v_temp = this->v_k;

        float dt = (now - this->lastFilterTime) / 1000.0f; // Convert milliseconds into seconds here
        this->dt_ = dt; //DEBUGGING. REMOVE THIS 

        //1. Propagate the Error State Covariance Forward in time 
        // Propagate this first since STMs are dependent on state, so want to calculate this BEFORE prediction
        this->propagateCovariance(q_temp, accelMeas, gyroMeas, dt); 

        //2. Propagate Nominal State forward in time using IMU measurements (NO UPDATES TO BIAS)
        Vector3f estAccelMeas = q2R(q_temp).transpose() * (accelMeas ) + Vector3f(0.0f,0.0f,CONSTANTS::g0); // Rotate Accelerometer reading into Inertial frame, add back in Inertial gravity in NED to get Coordinate acceleration
        Vector3f estGyroMeas = gyroMeas ; // This is w_est

        this->p_k += (v_temp*dt) + (0.5f * estAccelMeas) * (dt*dt); //pk = pk + vk*dt * 0.5* (ak) *dt^2     
        this->v_k += estAccelMeas * dt;                           //vk = vk + ak*dt
        this->q_k += 0.5f * quatProp(estGyroMeas, q_temp) * dt; //qk = q_k + 0.5 * w x qk * dt = q_k + q_dot*dt
        this->q_k.normalize();

        //3. Update best estimate of body rates
        this->w_k = gyroMeas;


        lastFilterTime = now;
    }

}

void ErrorStateKalmanFilter::predictRegressionTest(const std::array<float,6> imuMeas, float dt){

    Vector3f accelMeas (imuMeas[0], imuMeas[1], imuMeas[2]);
    Vector3f gyroMeas (imuMeas[3], imuMeas[4], imuMeas[5]);
    //Extract out all previous values
    Quaternion q_temp = this->q_k;
    Vector3f v_temp = this->v_k;

    //1. Propagate the Error State Covariance Forward in time 
    // Propagate this first since STMs are dependent on state, so want to calculate this BEFORE prediction
    this->propagateCovariance(q_temp, accelMeas, gyroMeas, dt); 

    //2. Propagate Nominal State forward in time using IMU measurements (NO UPDATES TO BIAS)
    Vector3f estAccelMeas = q2R(q_temp).transpose() * (accelMeas ) + Vector3f(0.0f,0.0f,CONSTANTS::g0); // Rotate Accelerometer reading into Inertial frame, add back in Inertial gravity in NED to get Coordinate acceleration
    Vector3f estGyroMeas = gyroMeas ; // This is w_est

    this->p_k += (v_temp*dt) + (0.5f * estAccelMeas) * (dt*dt); //pk = pk + vk*dt * 0.5* (ak) *dt^2     
    this->v_k += estAccelMeas * dt;                           //vk = vk + ak*dt
    this->q_k += 0.5f * quatProp(estGyroMeas, q_temp) * dt; //qk = q_k + 0.5 * w x qk * dt = q_k + q_dot*dt
    this->q_k.normalize();

    //3. Update best estimate of body rates
    this->w_k = gyroMeas ;
    
}



// See Marley EKF Paper for Nonspinning Guided Missles. If all measurements are uncorrelated (which is the assumption here, the R matrix is diagonal for everything)
// Then the measurements can be processed one at a time with identical results to processing them all at once. Reduces computational speed since there isn't a need for matrix inversion
tiltData ErrorStateKalmanFilter::updateTiltMeas(const std::array<float,3> accelMeas) {
    // Calculate Tilt

    tiltData tiltSample; //Initialize bool to be false
    //Decide if we have low enough acceleration to use the accelerometer as a tilt sensor
    Vector3f accelMeasVec(accelMeas);
    if (std::fabs( (accelMeasVec).getMag() - CONSTANTS::g0)<0.5) { //Raw Accelerometer measurement should be measuring -g in NED / body frame if we don't have acceleration. Threshold is only twice that of the noise ceiling. May need a low pas filter 
        std::array<float,9> K; //Kalman Gain
        std::array<float,9> P_row; //Row of Covariance for this update
        
        if (!this->updateFlag) {
            updateFlag = true; //Keeps track of whether this is the first update of the loop
        }

        //Start Processing 
        // 0. Store relevant variable 
        float sigTilt2 = this->sigTilt_ * this->sigTilt_; 

        // Normalize measurement such that it is a unit vector
        Vector3f measOrientation = accelMeasVec; //Note that this is the TILT vector

        // 1. Create Observation Matrix H and observation model h(x). Suppose to be...
        // H = [zeros(3,6), skew(q2R(q_k)*[0;0;1]), eye(3), zeros(3,6)]
        // Split it up into 3 vectors to perform sequential updates 
        Vector3f hx = q2R(this->q_k) * Vector3f(0.0f, 0.0f, -CONSTANTS::g0); //If on the ground, Accelerometer would read +g in the UP direction, so this is negative
        Rotation skew_RVec = skew(hx);

        // 2. Sequentially update each component
        // Update Row 1: H1 {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, skew_RVec[0], skew_RVec[1], skew_RVec[2]}; 
        this->scalarMagTiltUpdate(skew_RVec[0], skew_RVec[1], skew_RVec[2], measOrientation[0], hx[0],sigTilt2, this->nisTilt[0], this->P_k, this->del_xk, K, P_row);

        // Update Row 2: H2 {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, skew_RVec[0], skew_RVec[1], skew_RVec[2] }; 
        this->scalarMagTiltUpdate( skew_RVec[3], skew_RVec[4], skew_RVec[5], measOrientation[1], hx[1],sigTilt2, this->nisTilt[1], this->P_k, this->del_xk, K, P_row);

        // Update Row 3: H3 {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, skew_RVec[0], skew_RVec[1], skew_RVec[2]}; 
        this->scalarMagTiltUpdate( skew_RVec[6], skew_RVec[7], skew_RVec[8], measOrientation[2], hx[2],sigTilt2, this->nisTilt[2], this->P_k, this->del_xk, K, P_row);

        //Update Data
        tiltSample.setData(measOrientation.getArray(), this->nisTilt); //Will update flag to let us know tilt sample was used
    }
    return tiltSample;

}


magData ErrorStateKalmanFilter::updateMagMeas(const std::array<float,3> magMeas) {
    magData magSample;

    Vector3f magMeasVec(magMeas);
 
    std::array<float,9> K; //Kalman Gain
    std::array<float,9> P_row; //Row of Covariance for this update
    
    if (!this->updateFlag) {
        updateFlag = true; //Keeps track of whether this is the first update of the loop
    }

    //Start Processing 
    // 0. Store relevant variable 
    float sigMag2 = this->sigMag_ * this->sigMag_; 

    // Form Orientation Vector
    Vector3f measOrientation = magMeasVec;


    // 1. Create Observation Matrix H and observation model h(x). Suppose to be...
    // H = [zeros(3,6), skew(q2R(q_k)*refVec), zeros(3,6), eye(3)]
    // Split it up into 3 vectors to perform sequential updates 
    Vector3f hx = q2R(this->q_k) * this->magRef_;
    Rotation skew_RVec = skew(hx);

    // 2. Sequentially update each component
    // Update Row 1: H1 {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, skew_RVec[0], skew_RVec[1], skew_RVec[2],}; 
    this->scalarMagTiltUpdate(skew_RVec[0], skew_RVec[1], skew_RVec[2], measOrientation[0], hx[0],sigMag2, this->nisMag[0], this->P_k, this->del_xk, K, P_row);

    // Update Row 2: H2 {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, skew_RVec[3], skew_RVec[4], skew_RVec[5]}; 
    this->scalarMagTiltUpdate(skew_RVec[3], skew_RVec[4], skew_RVec[5], measOrientation[1], hx[1],sigMag2, this->nisMag[1], this->P_k, this->del_xk, K, P_row);

    // Update Row 3: H3 {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, skew_RVec[6], skew_RVec[7], skew_RVec[8]}; 
    this->scalarMagTiltUpdate(skew_RVec[6], skew_RVec[7], skew_RVec[8], measOrientation[2], hx[2],sigMag2, this->nisMag[2], this->P_k, this->del_xk, K, P_row);

    magSample.setData(magMeas, measOrientation.getArray(), this->nisMag);

    return magSample;
};

altData ErrorStateKalmanFilter::updateAltMeas(const float altMeas) {
    altData altSample; 

    std::array<float,9> K; //Kalman Gain
    std::array<float,9> P_row; //Row of Covariance for this update
    
    if (!this->updateFlag) {
        updateFlag = true; //Keeps track of whether this is the first update of the loop
    }

    //Start Processing 
    // 0. Store relevant variables
    float sigAlt2 = this->sigAlt_ * this->sigAlt_;

    // 1. Update Down Position
    // H = [0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] --> Paritions the third row / col --> idx = 2
    this->scalarGPSAltUpdate( 2, -altMeas, this->p_k[2], sigAlt2, this->nisAlt, this->P_k, this->del_xk, K, P_row);  //Negative alt meas for DOWN 


    altSample.setData(altMeas, this->nisAlt);
    return altSample;
}

gpsData ErrorStateKalmanFilter::updateGPSMeas(const std::array<float,4> gpsMeas) {
    gpsData gpsSample;

    std::array<float,9> K; //Kalman Gain
    std::array<float,9> P_row; //Row of Covariance for this update
    
    if (!this->updateFlag) {
        updateFlag = true; //Keeps track of whether this is the first update of the loop
    }

    //Start Processing 
    // 0. Store relevant variables
    float sigGPSPos2 = this->sigGPSPos_ * this->sigGPSPos_;
    float sigGPSVel2 = this->sigGPSVel_ * this->sigGPSVel_;

    // 1. Update North Position
    // H = [1,0,0,0,0,0,0,0,0] --> Paritions the first row / col --> idx = 0
    this->scalarGPSAltUpdate( 0, gpsMeas[0], this->p_k[0], sigGPSPos2, this->nisGPS[0],  this->P_k, this->del_xk, K, P_row); // Update 

    // 2. Update East Position
    // H = [0,1,0,0,0,0,0,0,0] --> Paritions the second row / col --> idx = 1
    this->scalarGPSAltUpdate( 1, gpsMeas[1], this->p_k[1], sigGPSPos2, this->nisGPS[1],  this->P_k, this->del_xk, K, P_row); // Update 

    // 3. Update North Velocity
    // H = [0,0,0,1,0,0,0,0,0] --> Paritions the fourth row / col --> idx = 3
    this->scalarGPSAltUpdate( 3, gpsMeas[2], this->v_k[0], sigGPSVel2, this->nisGPS[2], this->P_k, this->del_xk, K, P_row);  

    // 4. Update East Velocity
    // H = [0,0,0,0,1,0,0,0,0] --> Paritions the fifth row / col --> idx = 4
    this->scalarGPSAltUpdate( 4, gpsMeas[3], this->v_k[1], sigGPSVel2, this->nisGPS[3], this->P_k, this->del_xk, K, P_row); 

    //Implement once tested
    gpsSample.setData(gpsMeas, this->nisGPS);

    return gpsSample;
}

void ErrorStateKalmanFilter::setMagRef(Vector3f magRef) {
    this->magRef_ = magRef;
};

