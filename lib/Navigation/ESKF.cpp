#include "ESKF.h"


ESKF::ESKF(Vector3f p0, Vector3f v0, Quaternion q0, Vector3f ba0, Vector3f bg0, Vector3f bm0,     //Initial Nominal States
            Matrix18f P0,                                                              //Error State Covariance
            float dt,
            float sig_acc, float sig_gyro, float eta_acc, float eta_gyro, float eta_mag,       //Process Noise
            float sig_m, float sig_tilt, float sig_alt, float sig_gps_pos, float sig_gps_vel):
            p_k(p0), v_k(v0), q_k(q0), w_k (std::array<float,3>{0,0,0}), ba_k(ba0), bg_k(bg0), bm_k(bm0),
            P_k(P0), dt_(dt),
            sigAcc_(sig_acc), sigGyro_(sig_gyro), etaAcc_(eta_acc), etaGyro_(eta_gyro), etaMag_(eta_mag),
            sigMag_(sig_m), sigTilt_(sig_tilt), sigAlt_(sig_alt), sigGPSPos_(sig_gps_pos), sigGPSVel_(sig_gps_vel)
            {
                this->initQd(this->dt_); //Get the discrete Process Noise covariance
            }; 


//Drone Sim has this in a symbolic form. Verify with that. Can also verify with Marley Paper
void ESKF::initQd(float dt) {
    float sigAcc2 = this->sigAcc_ * this->sigAcc_;
    float sigGyro2 = this->sigGyro_ * this->sigGyro_;
    float etaAcc2 = this->etaAcc_ * this -> etaAcc_;
    float etaGyro2 = this->etaGyro_ * this->etaGyro_;
    float etaMag2 = this->etaMag_ * this->etaMag_;
    float t = this->dt_;
    float t2 = t*t;
    float t3 = t2*t;

    //1. Zero out Qd
    this->Qd_.setZero();


    // --- Position & velocity blocks ---
    Qd_(0,0) = sigAcc2*t3/3.0f;   Qd_(0,3) = sigAcc2*t2/2.0f;        
    Qd_(1,1) = sigAcc2*t3/3.0f;   Qd_(1,4) = sigAcc2*t2/2.0f;      
    Qd_(2,2) = sigAcc2*t3/3.0f;   Qd_(2,5) = sigAcc2*t2/2.0f;      
    Qd_(3,0) = sigAcc2*t2/2.0f;   Qd_(3,3) = sigAcc2*dt + etaAcc2*t3/3.0f;   Qd_(3,12) = -etaAcc2*t2/2.0f; Qd_(3,12) = -etaAcc2*t2/2.0f; 
    Qd_(4,1) = sigAcc2*t2/2.0f;   Qd_(4,4) = sigAcc2*dt + etaAcc2*t3/3.0f;   Qd_(4,13) = -etaAcc2*t2/2.0f;
    Qd_(5,2) = sigAcc2*t2/2.0f;   Qd_(5,5) = sigAcc2*dt + etaAcc2*t3/3.0f;   Qd_(5,14) = -etaAcc2*t2/2.0f;

    // --- Alpha blocks ---
    Qd_(6,6) = sigGyro2*dt + etaGyro2*t3/3.0f;   Qd_(6,15) = -etaGyro2*t2/2.0f;   
    Qd_(7,7) = sigGyro2*dt + etaGyro2*t3/3.0f;   Qd_(7,16) = -etaGyro2*t2/2.0f;  
    Qd_(8,8) = sigGyro2*dt + etaGyro2*t3/3.0f;   Qd_(8,17) = -etaGyro2*t2/2.0f;   
    
    // Accelerometer Bias block --
    Qd_(9,3) = -etaAcc2*t2/2.0f;   Qd_(9,9) = etaAcc2*dt;
    Qd_(10,4) = -etaAcc2*t2/2.0f;   Qd_(10,10) = etaAcc2*dt;
    Qd_(11,5) = -etaAcc2*t2/2.0f;   Qd_(11,11) = etaAcc2*dt;

    // Gyro Bias Block
    Qd_(12,6) = -etaGyro2*t2/2.0f;   Qd_(12,12) = etaGyro2*dt;
    Qd_(13,7) = -etaGyro2*t2/2.0f;   Qd_(13,13) = etaGyro2*dt;
    Qd_(14,8) = -etaGyro2*t2/2.0f;   Qd_(14,14) = etaGyro2*dt;

    // --- Magnetometer bias block ---
    Qd_(15,15) = etaMag2*dt;
    Qd_(16,16) = etaMag2*dt;
    Qd_(17,17) = etaMag2*dt;

    // //Using the Process Noise 1-sigma, initialize the 18 x 18 covariance 
    // this->Qd_ = std::array<float,324> {sigAcc2*t3/3,            0,            0,               sigAcc2*t2/2,                          0,                          0,                            0,                            0,                            0,             0,             0,             0,              0,              0,               0,         0,         0,         0,
    //                                                0, sigAcc2*t3/3,            0,                          0,               sigAcc2*t2/2,                          0,                            0,                            0,                            0,             0,             0,             0,              0,              0,               0,         0,         0,         0,     
    //                                                0,            0, sigAcc2*t3/3,                          0,                          0,               sigAcc2*t2/2,                            0,                            0,                            0,             0,             0,             0,              0,              0,               0,         0,         0,         0,
    //                                     sigAcc2*t2/2,            0,            0, sigAcc2*t + (etaAcc2*t3)/3,                          0,                          0,                            0,                            0,                            0, -etaAcc2*t2/2,             0,             0,              0,              0,               0,         0,         0,         0,
    //                                                0, sigAcc2*t2/2,            0,                          0, sigAcc2*t + (etaAcc2*t3)/3,                          0,                            0,                            0,                            0,             0, -etaAcc2*t2/2,             0,              0,              0,               0,         0,         0,         0,     
    //                                                0,            0, sigAcc2*t2/2,                          0,                          0, sigAcc2*t + (etaAcc2*t3)/3,                            0,                            0,                            0,             0,             0, -etaAcc2*t2/2,              0,              0,               0,         0,         0,         0,
    //                                                0,            0,            0,                          0,                          0,                          0, sigGyro2*t + (etaGyro2)*t3/3,                            0,                            0,             0,             0,             0, -etaGyro2*t2/2,              0,               0,         0,         0,         0,
    //                                                0,            0,            0,                          0,                          0,                          0,                            0, sigGyro2*t + (etaGyro2)*t3/3,                            0,             0,             0,             0,              0, -etaGyro2*t2/2,               0,         0,         0,         0,     
    //                                                0,            0,            0,                          0,                          0,                          0,                            0,                            0, sigGyro2*t + (etaGyro2)*t3/3,             0,             0,             0,              0,              0,  -etaGyro2*t2/2,         0,         0,         0,
    //                                                0,            0,            0,              -etaAcc2*t2/2,                          0,                          0,                            0,                            0,                            0,     etaAcc2*t,             0,             0,              0,              0,               0,         0,         0,         0,
    //                                                0,            0,            0,                          0,              -etaAcc2*t2/2,                          0,                            0,                            0,                            0,             0,     etaAcc2*t,             0,              0,              0,               0,         0,         0,         0,     
    //                                                0,            0,            0,                          0,                          0,              -etaAcc2*t2/2,                            0,                            0,                            0,             0,             0,     etaAcc2*t,              0,              0,               0,         0,         0,         0,
    //                                                0,            0,            0,                          0,                          0,                          0,               -etaGyro2*t2/2,                            0,                            0,             0,             0,             0,     etaGyro2*t,              0,               0,         0,         0,         0,
    //                                                0,            0,            0,                          0,                          0,                          0,                            0,               -etaGyro2*t2/2,                            0,             0,             0,             0,              0,     etaGyro2*t,               0,         0,         0,         0,     
    //                                                0,            0,            0,                          0,                          0,                          0,                            0,                            0,               -etaGyro2*t2/2,             0,             0,             0,              0,              0,      etaGyro2*t,         0,         0,         0,
    //                                                0,            0,            0,                          0,                          0,                          0,                            0,                            0,                            0,             0,             0,             0,              0,              0,               0, etaMag2*t,         0,         0,
    //                                                0,            0,            0,                          0,                          0,                          0,                            0,                            0,                            0,             0,             0,             0,              0,              0,               0,         0, etaMag2*t,         0,     
    //                                                0,            0,            0,                          0,                          0,                          0,                            0,                            0,                            0,             0,             0,             0,              0,              0,               0,         0,         0, etaMag2*t
    //                                             };

}

//TODO:: FIgure out if we need f in body can we leave it in inertial frame. revisit derivation
Matrix18f ESKF::getSTM(const Quaternion& q, const Vector3f& accelBias,  const Vector3f& gyroBias, const Vector3f& accelMeas, const Vector3f& gyroMeas, const float dt ) const{
    //0. Get all necessary variables
    //a. Get the total acceleration in the BODY FRAME. Can probabaly reformulate this to be in the inertial frame and skip the rotation.
    Vector3f aB = (accelMeas - accelBias) + (q2R(q) * Vector3f(0.0f,0.0f, Constants::g0)); 
    Vector3f wB = gyroMeas - gyroBias;
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

    Matrix18f STM;
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
    STM(3,9) =  (two_qy2 + two_qz2 -1)*dt; 
    STM(3,10) =  (two_qw_qz - two_qx_qy)*dt; 
    STM(3,11) =  -(two_qw_qy + two_qx_qz)*dt; 

    STM(4,6) =  (fz*(two_qx2 + two_qz2 -1) - fy*(two_qw_qx - two_qy_qz)) * dt;
    STM(4,7) = (fx*(two_qw_qx - two_qy_qz) + fz*(two_qw_qz + two_qx_qy)) * dt;
    STM(4,8) = -(fx*(two_qx2 + two_qz2 -1) + fy*(two_qw_qz + two_qx_qy)) * dt;
    STM(4,9) =  -(two_qw_qz + two_qx_qy)*dt; 
    STM(4,10) =  (two_qx2 + two_qz2 -1)*dt; 
    STM(4,11) =  (two_qw_qx - two_qy_qz)*dt; 

    STM(5,6) = -(fy*(two_qx2 + two_qy2 -1) + fz*(two_qw_qx + two_qy_qz)) * dt;
    STM(5,7) = (fx*(two_qx2 + two_qy2 -1) - fz*(two_qw_qy - two_qx_qz)) * dt;
    STM(5,8) =  (fx*(two_qw_qx + two_qy_qz) + fy*(two_qw_qy - two_qx_qz)) * dt;
    STM(5,9) =  (two_qw_qy - two_qx_qz)*dt; 
    STM(5,10) =  -(two_qw_qx + two_qy_qz)*dt; 
    STM(5,11) =  (two_qx2 + two_qy2 -1 )*dt; 

    // 5. Fill in alpha block (d alpha / dx) 
    //Fill angular rate skew blocks (alpha_dot = -skew(wB)*alpha)
    STM(6,7) =  wz*dt; STM(6,8) = -wy*dt; STM(6,12) = -dt;
    STM(7,6) = -wz*dt; STM(7,8) =  wx*dt; STM(7,13) = -dt;
    STM(8,6) =  wy*dt; STM(8,7) = -wx*dt; STM(8,14) = -dt;

    // 6. All Remaining rows with bias are just filled by identity
    return STM;

    // STM = I + F * dt
    // delx_dot = |  dp_dot    |   |  0 , I ,             0       ,     0    , 0  , 0| [ dp    ]   |      0   ,  0 , 0, 0, 0 | [ a_n   ]         
    //            |  dv_dot    |   |  0 , 0 , -R(q)^T*skew(aB)    ,  -R(q)^T , 0  , 0| [ dv    ]   |  -R(q)^T ,  0 , 0, 0, 0 | [ w_n   ]
    //            |  alpha_dot | = |  0 , 0 ,    -skew([wB])      ,     0    , 0  , 0| [ alpha ] + |      0   , -I , 0, 0, 0 | [ eta_a ]
    //            |  dba_dot   |   |  0 , 0 ,             0       ,     0    , 0  , 0| [ dba   ]   |      0   ,  0 , I, 0, 0 | [ eta_g ]
    //            |  dbg_dot   |   |  0 , 0 ,             0       ,     0    , 0  , 0| [ dbg   ]   |      0   ,  0 , 0, I, 0 | [ eta_m ]
    //            |  dbm_dot   |   |  0 , 0 ,             0       ,     0    , 0  , 0| [ dbm   ]   |      0   ,  0 , 0, 0, I | 
    //              del_x_dot    =                      F                                del_x   +              G                 w



    // Matrix18f STM = Matrix18f(std::array<float,324>{1, 0, 0, dt,  0,  0,                                                               0,                                                               0,                                                               0,                                                                                0,   0,   0,   0, 0, 0, 0, 
    //                                                 0, 1, 0,  0, dt,  0,                                                               0,                                                               0,                                                               0,                                                                                0,   0,   0,   0, 0, 0, 0, 
    //                                                 0, 0, 1,  0,  0, dt,                                                               0,                                                               0,                                                               0,                                                                                0,   0,   0,   0, 0, 0, 0, 

    //                                                 0, 0, 0,  1,  0,  0,      (aB[1]*(2*qw*qy + 2*qx*qz) + aB[2]*(2*qw*qz - 2*qx*qy))*dt, -(aB[2]*(2*qy*qy + 2*qz*qz - 1) + aB[0]*(2*qw*qy + 2*qx*qz))*dt,  (aB[1]*(2*qy*qy + 2*qz*qz - 1) - aB[0]*(2*qw*qz - 2*qx*qy))*dt, (2*qy*qy + 2*qz*qz - 1)*dt,    (2*qw*qz - 2*qx*qy)*dt,   -(2*qw*qy + 2*qx*qz)*dt,   0,   0,   0, 0, 0, 0,
    //                                                 0, 0, 0,  0,  1,  0,  (aB[2]*(2*qx*qx + 2*qz*qz - 1) - aB[1]*(2*qw*qx - 2*qy*qz))*dt,      (aB[0]*(2*qw*qx - 2*qy*qz) + aB[2]*(2*qw*qz + 2*qx*qy))*dt, -(aB[0]*(2*qx*qx + 2*qz*qz - 1) + aB[1]*(2*qw*qz + 2*qx*qy))*dt,    -(2*qw*qz + 2*qx*qy)*dt, (2*qx*qx + 2*qz*qz -1)*dt,    (2*qw*qx - 2*qy*qz)*dt,   0,   0,   0, 0, 0, 0, 
    //                                                 0, 0, 0,  0,  0,  1, -(aB[1]*(2*qx*qx + 2*qy*qy - 1) + aB[2]*(2*qw*qx + 2*qy*qz))*dt,  (aB[0]*(2*qx*qx + 2*qy*qy - 1) - aB[2]*(2*qw*qy - 2*qx*qz))*dt,  (aB[0]*(2*qw*qx + 2*qy*qz - 1) + aB[1]*(2*qw*qy - 2*qx*qz))*dt,     (2*qw*qy - 2*qx*qz)*dt,   -(2*qw*qx + 2*qy*qz)*dt, (2*qx*qx + 2*qy*qy -1)*dt,   0,   0,   0, 0, 0, 0, 

    //                                                 0, 0, 0,  0,  0,  0,                                                               1,                                                        wB[2]*dt,                                                       -wB[1]*dt,                          0,                         0,                         0, -dt,   0,   0, 0, 0, 0,
    //                                                 0, 0, 0,  0,  0,  0,                                                       -wB[2]*dt,                                                               1,                                                        wB[0]*dt,                          0,                         0,                         0,   0, -dt,   0, 0, 0, 0, 
    //                                                 0, 0, 0,  0,  0,  0,                                                        wB[1]*dt,                                                       -wB[0]*dt,                                                               1,                          0,                         0,                         0,   0,   0, -dt, 0, 0, 0, 

    //                                                 0, 0, 0,  0,  0,  0,                                                               0,                                                               0,                                                               0,                          1,                         0,                         0,   0,   0,   0, 0, 0, 0, 
    //                                                 0, 0, 0,  0,  0,  0,                                                               0,                                                               0,                                                               0,                          0,                         1,                         0,   0,   0,   0, 0, 0, 0, 
    //                                                 0, 0, 0,  0,  0,  0,                                                               0,                                                               0,                                                               0,                          0,                         0,                         1,   0,   0,   0, 0, 0, 0,

    //                                                 0, 0, 0,  0,  0,  0,                                                               0,                                                               0,                                                               0,                          0,                         0,                         0,   1,   0,   0, 0, 0, 0,
    //                                                 0, 0, 0,  0,  0,  0,                                                               0,                                                               0,                                                               0,                          0,                         0,                         0,   0,   1,   0, 0, 0, 0,
    //                                                 0, 0, 0,  0,  0,  0,                                                               0,                                                               0,                                                               0,                          0,                         0,                         0,   0,   0,   1, 0, 0, 0,

    //                                                 0, 0, 0,  0,  0,  0,                                                               0,                                                               0,                                                               0,                          0,                         0,                         0,   0,   0,   0, 1, 0, 0,
    //                                                 0, 0, 0,  0,  0,  0,                                                               0,                                                               0,                                                               0,                          0,                         0,                         0,   0,   0,   0, 0, 1, 0,
    //                                                 0, 0, 0,  0,  0,  0,                                                               0,                                                               0,                                                               0,                          0,                         0,                         0,   0,   0,   0, 0, 0, 1

    // });


}


void ESKF::propagateCovariance(const Quaternion& q, const Vector3f& accelBias,  const Vector3f& gyroBias, const Vector3f& accelMeas, const Vector3f& gyroMeas, const float dt ) {
    Matrix18f STM = this->getSTM(q, accelBias, gyroBias, accelMeas,gyroMeas, dt);

    //Can probably take advantage of the sparity of the matrices here to speed things up significantly (see update step)
    this->P_k = (STM * this -> P_k) * STM.transpose() + this->Qd_; 
}


void ESKF::step(std::array<float,14> z) {
    //Unpack Measurements
    Vector3f accelMeas(z[0],z[1],z[2]); //Raw measurement, Bias Uncompensated, Body Frame
    Vector3f gyroMeas(z[3], z[4], z[5]); //Raw Measurement, Bias Uncompensated, Body Frame
    Vector3f magMeas(z[6], z[7], z[8]); //Raw Measurement, Soft / Hard Iron calibrated, Reference to True North, Body Frame
    float altMeas = z[9];              //Raw Measuremen, height in D
    std::array<float,4> gpsMeas {z[10],z[11],z[12],z[13]}; // Raw Measurement, Postion and Velocity in NE
    Vector3f tiltMeas (NAN, NAN, NAN);

    //Decide if we have low enough acceleration to use the accelerometer as a tilt sensor
    if (std::fabs( (accelMeas-this->ba_k).getMag() - Constants::g0)<0.5) { //Raw Accelerometer measurement should be measuring -g in NED / body frame if we don't have acceleration. Threshold is only twice that of the noise ceiling. May need a low pas filter 
        tiltMeas = (accelMeas-this->ba_k).normalize();
    }


    // If both IMU meas has values, proceed
    // Else something has gone wrong. Skip this step part and increment some sort of counter. If this happens X times, go into some sort of safety mode
    this->predict(accelMeas, gyroMeas);


    //Perform Update only if there is at least one measurement avaliable.
    if (!isnan(magMeas[0]) || !isnan(tiltMeas[0]) || !isnan(altMeas) || !isnan(gpsMeas[0])) {
        this->update(magMeas,tiltMeas, altMeas, gpsMeas);
    };

    //Update the body rate estimate
    this->w_k = gyroMeas - this->bg_k;
}

void ESKF::predict(const Vector3f& accelMeas, const Vector3f& gyroMeas) {
    //Extract out all previous values
    Quaternion q_temp = this->q_k;
    Vector3f v_temp = this->v_k;
    Vector3f ba_temp = this->ba_k;
    Vector3f bg_temp = this->bg_k;
    float dt_temp = this->dt_;

    //1. Propagate the Error State Covariance Forward in time 
    // Propagate this first since STMs are dependent on state, so want to calculate this BEFORE prediction
    this->propagateCovariance(q_temp, ba_temp, bg_temp, accelMeas, gyroMeas, dt_temp); 

    //2. Propagate Nominal State forward in time using IMU measurements (NO UPDATES TO BIAS)
    Vector3f estAccelMeas = q2R(q_temp).transpose() * (accelMeas - ba_temp) + Vector3f(0.0f,0.0f,Constants::g0); // Rotate Accelerometer reading into Inertial frame, add back in Inertial gravity in NED to get Coordinate acceleration
    Vector3f estGyroMeas = gyroMeas - bg_temp; // This is w_est

    this->p_k += (v_temp*dt_temp) + (0.5f * estAccelMeas) * (dt_temp*dt_temp); //pk = pk + vk*dt * 0.5* (ak) *dt^2     
    this->v_k += estAccelMeas * dt_temp;                           //vk = vk + ak*dt
    this->q_k += 0.5f * quatProp(estGyroMeas, q_temp) * dt_temp; //qk = q_k + 0.5 * w x qk * dt = q_k + q_dot*dt
    this->q_k.normalize();

}

// See Marley EKF Paper for Nonspinning Guided Missles. If all measurements are uncorrelated (which is the assumption here, the R matrix is diagonal for everything)
// Then the measurements can be processed one at a time with identical results to processing them all at once. Reduces computational speed since there isn't a need for matrix inversion
void ESKF::update(const Vector3f& magMeas, const Vector3f& tiltMeas, const float& altMeas,const std::array<float,4>& gpsMeas) {
    std::array<float,18> del_xk {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f };
    Matrix18f&P_temp = this->P_k;
    // WIll pass these by reference so each function overwrites the same exisiting address
    std::array<float,18> K;
    std::array<float,18> P_row;

    //Check if we have a valid magnetometer update (Yaw)
    if (!isnan(magMeas[0])) {
        // 0. Store relevant variable 
        float sigMag2 = this->sigMag_ * this->sigMag_; 

        // Normalize measurement such that it is a unit vector
        Vector3f measOrientation = magMeas.normalize();


        // 1. Create Observation Matrix H and observation model h(x). Suppose to be...
        // H = [zeros(3,6), skew(q2R(q_k)*[1;0;0]), zeros(3,6), eye(3)]
        // Split it up into 3 vectors to perform sequential updates 
        Vector3f hx = q2R(this->q_k) * Vector3f(1.0f, 0.0f, 0.0f);
        Rotation skew_RVec = skew(hx);

        // 2. Sequentially update each component
        // Update Row 1: H1 {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, skew_RVec[0], skew_RVec[1], skew_RVec[2], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f }; 
        // Index bias -> 15
        this->scalarMagTiltUpdate(15, skew_RVec[0], skew_RVec[1], skew_RVec[2], measOrientation[0], hx[0],sigMag2, P_temp, del_xk, K, P_row);

        // Update Row 2: H2 {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, skew_RVec[3], skew_RVec[4], skew_RVec[5], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f }; 
        // Index bias -> 16
        this->scalarMagTiltUpdate(16, skew_RVec[3], skew_RVec[4], skew_RVec[5], measOrientation[1], hx[1],sigMag2, P_temp, del_xk, K, P_row);

        // Update Row 3: H3 {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, skew_RVec[6], skew_RVec[7], skew_RVec[8], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f }; 
        // Index bias -> 17
        this->scalarMagTiltUpdate(17, skew_RVec[6], skew_RVec[7], skew_RVec[8], measOrientation[2], hx[2],sigMag2, P_temp, del_xk, K, P_row);
    }
    
    // Check if we have a valid tilt measurement (roll / pitch)
    if (!isnan(tiltMeas[0])) {
        // 0. Store relevant variable 
        float sigTilt2 = this->sigTilt_ * this->sigTilt_; 

        // Normalize measurement such that it is a unit vector
        Vector3f measOrientation = tiltMeas;

        // 1. Create Observation Matrix H and observation model h(x). Suppose to be...
        // H = [zeros(3,6), skew(q2R(q_k)*[0;0;1]), eye(3), zeros(3,6)]
        // Split it up into 3 vectors to perform sequential updates 
        Vector3f hx = q2R(this->q_k) * Vector3f(0.0f, 0.0f, -1.0f); //If on the ground, Accelerometer would read +g in the UP direction, so this is negative
        Rotation skew_RVec = skew(hx);

        // 2. Sequentially update each component
        // Update Row 1: H1 {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, skew_RVec[0], skew_RVec[1], skew_RVec[2], 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }; 
        // Index bias -> 9
        this->scalarMagTiltUpdate(9, skew_RVec[0], skew_RVec[1], skew_RVec[2], measOrientation[0], hx[0],sigTilt2, P_temp, del_xk, K, P_row);

        // Update Row 2: H2 {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, skew_RVec[0], skew_RVec[1], skew_RVec[2], 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }; 
        // Index bias -> 10
        this->scalarMagTiltUpdate(10, skew_RVec[3], skew_RVec[4], skew_RVec[5], measOrientation[1], hx[1],sigTilt2, P_temp, del_xk, K, P_row);

        // Update Row 3: H3 {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, skew_RVec[0], skew_RVec[1], skew_RVec[2], 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }; 
        // Index bias -> 11
        this->scalarMagTiltUpdate(11, skew_RVec[6], skew_RVec[7], skew_RVec[8], measOrientation[2], hx[2],sigTilt2, P_temp, del_xk, K, P_row);

    }

    // Check if we have a valid GPS measurement
    if (!isnan(gpsMeas[0])) {
        
        // 0. Store relevant variables
        float sigGPSPos2 = this->sigGPSPos_ * this->sigGPSPos_;
        float sigGPSVel2 = this->sigGPSVel_ * this->sigGPSVel_;

        // 1. Update North Position
        // H = [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] --> Paritions the first row / col --> idx = 0
        this->scalarGPSAltUpdate( 0, gpsMeas[0], this->p_k[0], sigGPSPos2,  P_temp, del_xk, K, P_row); // Update 

        // 2. Update East Position
        // H = [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] --> Paritions the second row / col --> idx = 1
        this->scalarGPSAltUpdate( 1, gpsMeas[1], this->p_k[1], sigGPSPos2,  P_temp, del_xk, K, P_row); // Update 

        // 3. Update North Velocity
        // H = [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0] --> Paritions the fourth row / col --> idx = 3
        this->scalarGPSAltUpdate( 3, gpsMeas[2], this->v_k[0], sigGPSVel2,  P_temp, del_xk, K, P_row);  

        // 4. Update East Velocity
        // H = [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0] --> Paritions the fifth row / col --> idx = 4
        this->scalarGPSAltUpdate( 4, gpsMeas[3], this->v_k[1], sigGPSVel2,  P_temp, del_xk, K, P_row); 

    }

    //Check if we have a valid altimeter measurement 
    if (!isnan(altMeas)) {
        // 0. Store relevant variables
        float sigAlt2 = this->sigAlt_ * this->sigAlt_;

        // 1. Update Down Position
        // H = [0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] --> Paritions the third row / col --> idx = 2
        this->scalarGPSAltUpdate( 2, altMeas, this->p_k[2], sigAlt2,  P_temp, del_xk, K, P_row); 
    }



    // After going through each measurement, inject the error state
    this->injectError(del_xk);

    // Update Covariance
    this->P_k = P_temp;

}



