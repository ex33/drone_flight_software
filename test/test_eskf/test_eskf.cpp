#include <ErrorStateKalmanFilter.h>
#include <Mathpk.h>
#include<unity.h>
#include <string>
#include <DataTypes.h>


float tol = 1e-4;

std::array<float,3> p0 {0.0f,0.0f,0.0f};
std::array<float,3>  v0 {0.0f,0.0f,0.0f};
std::array<float,4> q0 {1.0f,0.0f,0.0f,0.0f};


std::array<float,81> P0 {
    3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f
};

float dt = 0.02; //50Hz

//Set Process Noise
float sig_acc(0.003f);
float sig_gyro(0.5f);
//Set Measurement Noise
float sig_mag(0.003f);
float sig_tilt(0.08f);
float sig_alt(0.1f);
float sig_gps_pos(5.0f);
float sig_gps_vel(0.1f);

ErrorStateKalmanFilter ErrorStateKalmanFilter(p0, v0, q0, P0,
          sig_acc, sig_gyro, 
          sig_mag, sig_tilt, sig_alt, sig_gps_pos, sig_gps_vel,false);




//Make these bool so that the actual test assert returns what lines.
inline bool compare_vector(const std::array<float,3>& TruthVec, const Vector3f& TestVec, float tol, char* msg) {
  msg[0] = '\0'; //Clear previous msg
  for (size_t i = 0; i < 3; ++i) {
      if (std::fabs(TruthVec[i] - TestVec[i]) > tol) {
          std::snprintf(msg, 100,
                        "Vector mismatch at index %u: expected %.9f, got %.9f",
                          i, TruthVec[i], TestVec[i]);
          return false;
      }
  }
  return true;
}


inline bool compare_quaternion(const std::array<float, 4>& TruthQuat,
                               const Quaternion& TestQuat,
                               float tol,
                               char* msg) {
                                
    std::array<float, 4> test = {TestQuat.w(), TestQuat.x(), TestQuat.y(), TestQuat.z()};

    msg[0] = '\0'; //Clear previous msg
    for (size_t i = 0; i < 4; ++i) {
        float diff = std::fabs(TruthQuat[i] - test[i]);
        if (diff > tol) {
            std::snprintf(msg, 100,
                          "Quaternion mismatch at index %u: expected %.9f, got %.9f",
                           i, TruthQuat[i], test[i]);
            return false;
        }
    }
    
    return true;
}


inline bool compare_matrix(const Matrix9f& TruthMatrix, const Matrix9f& TestMatrix, float tol,char* msg) {
  // Compare covariance
  msg[0] = '\0'; //Clear previous msg

  for (unsigned int i = 0; i<81; i++ ){
    if (fabs(TruthMatrix[i] - TestMatrix[i]) > tol) {
        snprintf(msg, 100, 
                  "Matrix mismatch at index %u: expected %.9f, got %.9f", 
                  i, TruthMatrix[i], TestMatrix[i]);
        return false; 
    }
  }

  return true;
}

// // Individual test functions
void test_init() {
  TEST_ASSERT_FLOAT_WITHIN(tol, sig_acc, ErrorStateKalmanFilter.getSigAcc());
  TEST_ASSERT_FLOAT_WITHIN(tol, sig_gyro, ErrorStateKalmanFilter.getSigGyro());

  TEST_ASSERT_FLOAT_WITHIN(tol, sig_mag, ErrorStateKalmanFilter.getSigMag());
  TEST_ASSERT_FLOAT_WITHIN(tol, sig_tilt, ErrorStateKalmanFilter.getSigTilt());
  TEST_ASSERT_FLOAT_WITHIN(tol, sig_alt, ErrorStateKalmanFilter.getSigAlt());
  TEST_ASSERT_FLOAT_WITHIN(tol, sig_gps_pos, ErrorStateKalmanFilter.getSigGPSPos());
  TEST_ASSERT_FLOAT_WITHIN(tol, sig_gps_vel, ErrorStateKalmanFilter.getSigGPSVel());

  // Compare covariance
  Matrix9f testP0 = ErrorStateKalmanFilter.getCovariance();

  char msg [100];
  bool P0_test_result = compare_matrix(P0, testP0, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(P0_test_result, msg);

  
}



void test_functions() {
  // Test that Qd_ and STM are obtained / set properly

  Matrix9f Qd = ErrorStateKalmanFilter.getQd(dt); //Dependent upon noise
  Matrix9f STM = ErrorStateKalmanFilter.getSTM(Quaternion(0.714844396765618f, 0.295237747760709f ,0.560951720745348f, 0.295237747760709f), Vector3f(0.35f, 0.5f,-10.5f), Vector3f(0.25f, 1.0f, 0.5f),dt); //Make sure its all different numbers
  //Truth
std::array<float, 81> QdTrue = {
    0.000000000024f, 0.0f, 0.0f, 0.0000000018f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.000000000024f, 0.0f, 0.0f, 0.0000000018f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.000000000024f, 0.0f, 0.0f, 0.0000000018f, 0.0f, 0.0f, 0.0f, 
    0.0000000018f, 0.0f, 0.0f, 0.00000018f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0000000018f, 0.0f, 0.0f, 0.00000018f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 0.0000000018f, 0.0f, 0.0f, 0.00000018f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0050f, 0.0f, 0.0f, 
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.005f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.005f
};

  std::array<float, 81> STMTrue = {
    1.0f, 0.0f, 0.0f, 0.02f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 1.0f, 0.0f, 0.0f, 0.02f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.02f, 0.0f, 0.0f, 0.0f, 

    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.138432657034807f,0.079684299314976f,   -0.020422022864827f, 
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.097364520541395f, -0.139739866302007f, -0.194461883465677f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.160161889166834f, 0.130433354275711f, 0.011549841747262f, 

    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.01f, -0.02f, 
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.01f, 1.0f, 0.005f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.02f, -0.005f, 1.0f
};

//Test Q_d
char msg [100];
bool Qd_test_result = compare_matrix(QdTrue, Qd, tol, msg);
TEST_ASSERT_TRUE_MESSAGE(Qd_test_result, msg);


//Test STM
bool STM_test_result = compare_matrix(STMTrue, STM, tol, msg);
TEST_ASSERT_TRUE_MESSAGE(STM_test_result, msg);

}



void test_propagation() {
  // Raw Accelerometer reading, gyro, magnetometer, altimeter, gps
  std::array<float,6> imuMeas {0.35f,0.5f,-10.5f,0.25f,1.0f,0.5f};
  //std::array<float,14> z {0.35f,0.5f,-10.5f,0.25f,1.0f,0.5f,NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN}; // Propagate only

  //Propagate forward 3 timesteps. No Updates
  ErrorStateKalmanFilter.predictRegressionTest(imuMeas,dt);
  ErrorStateKalmanFilter.predictRegressionTest(imuMeas,dt);
  ErrorStateKalmanFilter.predictRegressionTest(imuMeas,dt);

  // True Values (From Sim)
  std::array<float,3>pos_true = {0.000414595877949f,0.000954497829611f,-0.001249323598196f};
  std::array<float,3>vel_true = {0.008071457486999f,0.033252621338523f,-0.041641971420546f};
  std::array<float,4>quat_true = {0.999409484801674f,0.007498195619300f,0.029992782477202f,0.014996391238601f};
  std::array<float,3>rates_true = {0.25f, 1.0f, 0.5f};


  std::array<float,81>P_true =   {
    /* Row 1  */ 3.001800109340763,  -0.000000020823961 ,  0.000000027886328 ,  0.030005617416278,  -0.000000502904312 ,  0.000000674029415 ,  0.000001224846047,  -0.000085636818250,  -0.000062008821865,
    /* Row 2  */ -0.000000020823961,   3.001800081581806,   0.000000052122790,  -0.000001116302760,   0.030003794119454,   0.000002802923825,   0.000082546803206,  -0.000001991678225, 0.000038708245580,
    /* Row 3  */ 0.000000027886328,0.000000052122790,3.001800050749628,0.000001398047668,0.000002621803032,0.030002380822283,0.000062197878398 , -0.000035526368686,0.000004292234378,
    /* Row 4  */ 0.030005617416278,-0.000001116302760,0.000001398047668,0.500292794079878,-0.000026843406003,0.000033667198228,0.000077422264825,-0.004425304699672, -0.003435178946776,
    /* Row 5  */ -0.000000502904312,0.030003794119454,0.000002621803032,-0.000026843406003,0.500185383497570,0.000142615416023,0.004318686783821,-0.000117520057634, 0.001066282163790,
    /* Row 6  */0.000000674029415,0.000002802923825,0.030002380822283,0.000033667198228,0.000142615416023,0.500121151721277,0.003473509916098,-0.000908284150256, 0.000222181068453,
    /* Row 7  */ 0.000001224846047,0.000082546803206,0.000062197878398,0.000077422264825,0.004318686783821,0.003473509916098,0.115157580076281,-0.000031516015256,  -0.000015758007628,
    /* Row 8  */ -0.000085636818250,-0.000001991678225,-0.000035526368686,-0.004425304699672,-0.000117520057634,-0.000908284150256,-0.000031516015256,0.115039395019070,-0.000063032030513,
    /* Row 9  */ -0.000062008821865,0.000038708245580,0.000004292234378,-0.003435178946776,0.001066282163790,0.000222181068453,-0.000015758007628,-0.000063032030512, 0.115133943064839};

  // Estimated Values
  Vector3f pos_est = ErrorStateKalmanFilter.getPosition();
  Vector3f vel_est = ErrorStateKalmanFilter.getVelocity();
  Quaternion quat_est = ErrorStateKalmanFilter.getQuaternion();
  Vector3f rates_est = ErrorStateKalmanFilter.getBodyRates();
  Matrix9f P_est = ErrorStateKalmanFilter.getCovariance();

  // Compare States
  char msg[100];
  bool pos_test_result = compare_vector(pos_true,pos_est, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(pos_test_result, msg);

  bool vel_test_result = compare_vector(vel_true,vel_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(vel_test_result, msg);

  bool quat_test_result = compare_quaternion(quat_true, quat_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(quat_test_result, msg);

  bool rates_test_result = compare_vector(rates_true,rates_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(rates_test_result, msg);

  bool covariance_test_result = compare_matrix(P_true, P_est, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(covariance_test_result, msg);
}

void test_update_mag() {
  // Set mag Ref
  ErrorStateKalmanFilter.setMagRef(Vector3f(1,0,0)); // Assume reference is just pointed North

  // Form IMU and Mag Measurements
  std::array<float,6> imuMeas {0.35f,0.5f,-10.5f,0.25f,1.0f,0.5f};
  std::array<float,3> magMeas {0.996848222532429f, 0.0604830540752986f,-0.0513363555744027f};
  //Propagate and Update Magnetometer. 
  ErrorStateKalmanFilter.predictRegressionTest(imuMeas, dt);
  ErrorStateKalmanFilter.updateMagMeas(magMeas);
  ErrorStateKalmanFilter.injectError();
  //std::array<float,14> z = {0.35f,0.5f,-10.5f,0.25f,1.0f,0.5f,0.996848222532429f, 0.0604830540752986f,-0.0513363555744027f,NAN, NAN, NAN, NAN,NAN}; 

  // True Values (From Sim)
  std::array<float,3> pos_true = {0.000761969963648f,0.001707609376645f,-0.002182262106788f};
  std::array<float,3> vel_true = {0.010964945627520f,0.046314901661304f,-0.055275225395960f};
  std::array<float,4> quat_true = {0.999408348308876f, 0.013498607723885f,-0.006362595660957f,-0.030988034731102f};
  std::array<float,3> rates_true = {0.25, 1, 0.5};

  std::array<float, 81> P_true = {
    /* Row 1  */ 3.0032e+00 , -8.7409e-09 ,  1.1384e-08 ,  4.0001e-02 , -1.1531e-07  , 1.4878e-07 , -3.0359e-08 , -1.1908e-08,  -1.2196e-08,
    /* Row 2  */  -8.7409e-09 ,  3.0032e+00 ,  2.1765e-07  ,-2.3069e-07  , 4.0010e-02  , 7.9235e-06  , 1.7164e-04 , -6.7455e-06,   1.3839e-05,
    /* Row 3  */ 1.1384e-08 ,  2.1765e-07 ,  3.0032e+00 ,  2.8583e-07 ,  7.3056e-06 ,  4.0006e-02  ,1.3297e-04 , -5.2293e-06,   1.0718e-05,
    /* Row 4  */ 4.0001e-02 , -2.3069e-07 ,  2.8583e-07 ,  5.0004e-01 , -2.6312e-06  , 3.2866e-06 , -7.6572e-07  ,-4.2340e-07,  -4.2933e-07,
    /* Row 5  */ -1.1531e-07 ,  4.0010e-02 ,  7.3056e-06,  -2.6312e-06  , 5.0032e-01  , 2.6916e-04 ,  5.9349e-03 , -2.3324e-04,   4.7840e-04,
    /* Row 6  */  1.4878e-07 ,  7.9235e-06 ,  4.0006e-02 ,  3.2866e-06 ,  2.6916e-04  , 5.0023e-01 ,  5.0030e-03 , -1.9663e-04 ,  4.0327e-04,
    /* Row 7  */ 3.0359e-08 ,  1.7164e-04 , 1.3297e-04 , -7.6572e-07  , 5.9349e-03  , 5.0030e-03  , 1.1926e-01 , -4.6864e-03 ,  9.6121e-03,
    /* Row 8  */ -1.1908e-08 , -6.7455e-06,  -5.2293e-06 , -4.2340e-07 , -2.3324e-04 , -1.9663e-04 , -4.6864e-03  , 1.9318e-04 , -3.7776e-04,
    /* Row 9  */  -1.2196e-08 ,  1.3839e-05 ,  1.0718e-05  ,-4.2933e-07 ,  4.7840e-04 , 4.0327e-04 , 9.6121e-03 , -3.7776e-04 ,  7.8380e-04};


  // Estimated Values
  Vector3f pos_est = ErrorStateKalmanFilter.getPosition();
  Vector3f vel_est = ErrorStateKalmanFilter.getVelocity();
  Quaternion quat_est = ErrorStateKalmanFilter.getQuaternion();
  Vector3f rates_est = ErrorStateKalmanFilter.getBodyRates();
  Matrix9f P_est = ErrorStateKalmanFilter.getCovariance();

  // Compare States
  char msg [100];
  bool pos_test_result = compare_vector(pos_true,pos_est, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(pos_test_result, msg);

  bool vel_test_result = compare_vector(vel_true,vel_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(vel_test_result, msg);

  bool quat_test_result = compare_quaternion(quat_true, quat_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(quat_test_result, msg);

  bool rates_test_result = compare_vector(rates_true,rates_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(rates_test_result, msg);

  bool covariance_test_result = compare_matrix(P_true, P_est, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(covariance_test_result, msg);

};

void test_update_alt() {
  // Propagate and Update with Altimeter
  float altMeas = 0.0009644664277875f; //Will flip the sign within the update
  std::array<float,6> imuMeas {0.35f,0.5f,-10.5f,0.25f,1.0f,0.5f};
  ErrorStateKalmanFilter.predictRegressionTest(imuMeas, dt);
  ErrorStateKalmanFilter.updateAltMeas(altMeas);
  ErrorStateKalmanFilter.injectError();
  // True Values (From Sim)
  std::array<float,3>pos_true = {1.0858e-03 ,  2.7852e-03 , -9.7262e-04};
  std::array<float,3>vel_true = {2.1415e-02,   6.1441e-02 , -6.8651e-02};
  std::array<float,4>quat_true = {9.9953e-01 ,  1.6274e-02 ,  3.4863e-03 , -2.5838e-02};
  std::array<float,3>rates_true = {0.25f, 1.0, 0.5};
  std::array<float,81>P_true = {
   3.0050e+00,  -1.6713e-08,   7.0949e-11,   5.0002e-02,  -1.6879e-07,   2.1363e-07,  -4.5463e-08,  -2.0023e-08,  -2.1595e-08,
  -1.6713e-08,   3.0050e+00,   2.0892e-09,  -2.8162e-07,   5.0020e-02,   1.7977e-05,   2.8976e-04,  -1.4197e-05,   2.9271e-05,
   7.0949e-11,   2.0892e-09,   9.9668e-03,   1.1709e-09,   5.3136e-08,   1.6589e-04,   7.7134e-07,  -3.7805e-08,   7.7910e-08,
   5.0002e-02,  -2.8162e-07,   1.1709e-09,   5.0004e-01,  -2.6020e-06,   3.3158e-06,  -3.7562e-08,  -5.7791e-07,  -5.0227e-07,
  -1.6879e-07,   5.0020e-02,   5.3136e-08,  -2.6020e-06,   5.0052e-01,   4.6360e-04,   7.6246e-03,  -3.7355e-04,   7.7022e-04,
   2.1363e-07,   1.7977e-05,   1.6589e-04,   3.3158e-06,   4.6360e-04,   4.9959e-01,   6.9080e-03,  -3.3855e-04,   6.9774e-04,
  -4.5463e-08,   2.8976e-04,   7.7134e-07,  -3.7562e-08,   7.6246e-03,   6.9080e-03,   1.2378e-01,  -5.8191e-03,   1.1996e-02,
  -2.0023e-08,  -1.4197e-05,  -3.7805e-08,  -5.7791e-07,  -3.7355e-04,  -3.3855e-04,  -5.8191e-03,   5.2941e-03,  -5.8777e-04,
  -2.1595e-08,   2.9271e-05,   7.7910e-08,  -5.0227e-07,   7.7022e-04,   6.9774e-04,  1.1996e-02 , -5.8777e-04,   6.2207e-03};

  // Estimated Values
  Vector3f pos_est = ErrorStateKalmanFilter.getPosition();
  Vector3f vel_est = ErrorStateKalmanFilter.getVelocity();
  Quaternion quat_est = ErrorStateKalmanFilter.getQuaternion();
  Vector3f rates_est = ErrorStateKalmanFilter.getBodyRates();
  Matrix9f P_est = ErrorStateKalmanFilter.getCovariance();

  // Compare States
  char msg[100];
  bool pos_test_result = compare_vector(pos_true,pos_est, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(pos_test_result, msg);

  bool vel_test_result = compare_vector(vel_true,vel_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(vel_test_result, msg);

  bool quat_test_result = compare_quaternion(quat_true, quat_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(quat_test_result, msg);

  bool rates_test_result = compare_vector(rates_true,rates_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(rates_test_result, msg);

  bool covariance_test_result = compare_matrix(P_true, P_est, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(covariance_test_result, msg);
}

void test_update_gps() {
  //Predict and Update with GPS
  std::array<float,4> gpsMeas{-0.00117903695028239f, -0.000528094651009709f, 0.0293177068457126f,0.06104907300949f};
  std::array<float,6> imuMeas {0.35f,0.5f,-10.5f,0.25f,1.0f,0.5f};
  ErrorStateKalmanFilter.predictRegressionTest(imuMeas,dt);
  ErrorStateKalmanFilter.updateGPSMeas(gpsMeas);
  ErrorStateKalmanFilter.injectError();
  // True Values (From Sim)
  std::array<float,3>pos_true = {1.4570e-03 ,  1.9017e-03 , -2.4808e-03};
  std::array<float,3>vel_true = {2.9279e-02 ,  6.1370e-02  ,-8.2157e-02};
  std::array<float,4>quat_true = {9.9952e-01 ,  1.8893e-02 ,  1.3340e-02  ,-2.0707e-02};
  std::array<float,3>rates_true = {0.25, 1.0f, 0.5f};

  std::array<float,81>P_true = {
   2.6787e+00,-1.0440e-08,-3.9291e-10, 1.0504e-03, 4.6667e-09,-3.9485e-08, 8.1170e-07, 7.3594e-06, 8.5849e-06,
  -1.0440e-08, 2.6787e+00,-6.9351e-07, 1.8442e-09, 1.0495e-03,-4.4504e-05,-5.8547e-04, 3.3501e-05,-7.3773e-05,
  -3.9291e-10,-6.9351e-07, 1.0173e-02, 1.0230e-09, 2.2071e-07, 1.0160e-02, 1.3838e-04,-8.1161e-06, 1.6820e-05,
   1.0504e-03, 1.8442e-09, 1.0230e-09, 9.8039e-03,-1.5946e-09, 5.3976e-08,-1.5852e-07,-1.3793e-06,-1.6082e-06,
   4.6667e-09, 1.0495e-03, 2.2071e-07,-1.5946e-09, 9.8042e-03, 1.4102e-05, 1.8273e-04,-1.0560e-05, 2.2699e-05,
  -3.9485e-08,-4.4504e-05, 1.0160e-02, 5.3976e-08, 1.4102e-05, 4.9986e-01, 8.9850e-03,-5.4971e-04, 1.0853e-03,
   8.1170e-07,-5.8547e-04, 1.3838e-04,-1.5852e-07, 1.8273e-04, 8.9850e-03, 1.2801e-01,-6.9201e-03, 1.4343e-02,
   7.3594e-06, 3.3501e-05,-8.1161e-06,-1.3793e-06,-1.0560e-05,-5.4971e-04 , -6.9201e-03, 1.0415e-02 , -8.4210e-04,
   8.5849e-06,-7.3773e-05, 1.6820e-05,-1.6082e-06, 2.2699e-05, 1.0853e-03 ,1.4343e-02,-8.4210e-04, 1.1755e-02};
  // Estimated Values
  Vector3f pos_est = ErrorStateKalmanFilter.getPosition();
  Vector3f vel_est = ErrorStateKalmanFilter.getVelocity();
  Quaternion quat_est = ErrorStateKalmanFilter.getQuaternion();
  Vector3f rates_est = ErrorStateKalmanFilter.getBodyRates();

  Matrix9f P_est = ErrorStateKalmanFilter.getCovariance();

  // Compare States
  char msg[100];
  bool pos_test_result = compare_vector(pos_true,pos_est, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(pos_test_result, msg);

  bool vel_test_result = compare_vector(vel_true,vel_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(vel_test_result, msg);

  bool quat_test_result = compare_quaternion(quat_true, quat_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(quat_test_result, msg);

  bool rates_test_result = compare_vector(rates_true,rates_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(rates_test_result, msg);

  bool covariance_test_result = compare_matrix(P_true, P_est, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(covariance_test_result, msg);
}

void test_update_tilt() {
  std::array<float,6>imuMeas = {0.15f,0.25f,-9.8f,0.25f,1.0f,0.5f};
  std::array<float,3> accelMeas {0.15f,0.25f,-9.8f};
  //z = {0.15f,0.25f,-9.8f,0.25f,1.0f,0.5f,NAN,NAN,NAN, NAN, NAN, NAN, NAN, NAN};  
  ErrorStateKalmanFilter.predictRegressionTest(imuMeas,dt);
  ErrorStateKalmanFilter.updateTiltMeas(accelMeas);
  ErrorStateKalmanFilter.injectError();
  // True Values (From Sim)
  std::array<float,3>pos_true = {2.0071e-03,3.4638e-03,  -4.2328e-03};
  std::array<float,3>vel_true = {2.7454e-02,7.3713e-02,  -8.5521e-02};
  std::array<float,4>quat_true = {9.9981e-01,  -3.1296e-03,6.7982e-03 , -1.8229e-02};
  std::array<float,3>rates_true = {0.25f, 1.0f, 0.5f};
  std::array<float,81>P_true = {
   2.6787e+00,  -7.3199e-09,  -2.3760e-09,   1.2464e-03,  -1.2714e-08,  -6.1005e-08,  -4.0871e-07,   4.0351e-07,   8.7287e-06,
  -7.3199e-09,   2.6787e+00,  -1.8123e-07,   1.0890e-07,   1.2464e-03,  -5.1585e-06,   1.1661e-07,  -3.4976e-07,  -8.6218e-06,
  -2.3760e-09,  -1.8123e-07,   1.0779e-02,  -3.6948e-08,   5.4293e-08,   2.0136e-02,   1.0789e-08,   1.1971e-07,   3.1258e-06,
   1.2464e-03,   1.0890e-07,  -3.6948e-08,   9.8057e-03,   2.0832e-07,  -1.0707e-06,   5.9036e-06,  -5.3895e-06,  -1.2676e-04,
  -1.2714e-08,   1.2464e-03,   5.4293e-08,   2.0832e-07,   9.8041e-03,   1.4878e-06,   8.3808e-07,  -7.1685e-07,  -1.6677e-05,
  -6.1005e-08,  -5.1585e-06,   2.0136e-02,  -1.0707e-06,   1.4878e-06,   4.9926e-01,   1.0101e-06,   3.4893e-06,   8.9923e-05,
  -4.0871e-07,   1.1661e-07,   1.0789e-08,   5.9036e-06,   8.3808e-07,   1.0101e-06,   9.9305e-05,  -2.9925e-05,  -7.0136e-04,
   4.0351e-07,  -3.4976e-07,   1.1971e-07,  -5.3895e-06,  -7.1685e-07,   3.4893e-06,  -2.9925e-05,   9.3529e-05,   6.3962e-04,
   8.7287e-06,  -8.6218e-06,   3.1258e-06,  -1.2676e-04,  -1.6677e-05,   8.9923e-05,  -7.0136e-04,   6.3962e-04,   1.5067e-02};
  // Estimated Values
  Vector3f pos_est = ErrorStateKalmanFilter.getPosition();
  Vector3f vel_est = ErrorStateKalmanFilter.getVelocity();
  Quaternion quat_est = ErrorStateKalmanFilter.getQuaternion();
  Vector3f rates_est = ErrorStateKalmanFilter.getBodyRates();
  Matrix9f P_est = ErrorStateKalmanFilter.getCovariance();

  // Compare States
  char msg[100];
  bool pos_test_result = compare_vector(pos_true,pos_est, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(pos_test_result, msg);

  bool vel_test_result = compare_vector(vel_true,vel_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(vel_test_result, msg);

  bool quat_test_result = compare_quaternion(quat_true, quat_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(quat_test_result, msg);

  bool rates_test_result = compare_vector(rates_true,rates_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(rates_test_result, msg);

  bool covariance_test_result = compare_matrix(P_true, P_est, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(covariance_test_result, msg);
}
// Initial state / covariance of ErrorStateKalmanFilter starts after the 3 propagation steps from previous test.
void test_update_all() {
  
  // Update with all 4 measurements
  std::array<float,6>imuMeas = {0.15f,0.25f,-9.8f,0.25f,1.0f,0.5f};
  std::array<float,3>accelMeas = {0.15f,0.25f,-9.8f};
  std::array<float,3>magMeas = {0.996848222532429f, 0.0604830540752986f,-0.0513363555744027f};
  float altMeas = 0.0009644664277875f;
  std::array<float,4>gpsMeas = {-0.00117903695028239f, -0.000528094651009709f, 0.0293177068457126f,0.06104907300949f};
  //Test values were taken with the updates in the following order: Mag --> Tilt --> GPS --> Alt.
  // In reality, order shouldn't matter
  // z = {0.15f,0.25f,-9.8f, //Accel
  //      0.25f,1.0f,0.5f,   //Gyro
  //      0.996848222532429f, 0.0604830540752986f,-0.0513363555744027f, //Mag
  //     -0.0009644664277875f,  //Alt
  //     -0.00117903695028239f, -0.000528094651009709f, 0.0293177068457126f,0.06104907300949f};   //GPS
  ErrorStateKalmanFilter.predictRegressionTest(imuMeas,dt);
  ErrorStateKalmanFilter.updateMagMeas(magMeas);
  ErrorStateKalmanFilter.updateTiltMeas(accelMeas);
  ErrorStateKalmanFilter.updateGPSMeas(gpsMeas);
  ErrorStateKalmanFilter.updateAltMeas(altMeas);
  ErrorStateKalmanFilter.injectError();
  
  // Updates Mag --> Tilt --> GPS --> Alt
  // True Values (From Sim)
  std::array<float,3> pos_true = { 2.2649e-03 ,  3.4025e-03 , -3.2710e-03};
  std::array<float,3>vel_true = { 2.9331e-02,   6.9351e-02 , -7.8946e-02};
  std::array<float,4>quat_true = {9.9801e-01 , -2.3245e-02 , -2.6927e-02,  -5.2041e-02};
  std::array<float,3>rates_true = { 0.25, 1.0f, 0.5};
  std::array<float,81>P_true = {
   2.4194e+00,  -5.3620e-09,  -1.6413e-09,   6.5785e-04,  -4.3805e-09,  -5.6508e-08,  -2.0566e-09,   3.3180e-10,   7.8047e-09,
  -5.3620e-09,   2.4194e+00,  -1.1899e-07,   2.6382e-08,   6.5788e-04,  -4.3418e-06,  -2.4863e-09,  -1.7279e-10,  -3.4412e-09,
  -1.6413e-09,  -1.1899e-07,   5.4095e-03,  -7.3890e-09,   2.0568e-08,   1.3827e-02,   1.2792e-09,   5.7127e-11,   1.0591e-09,
   6.5785e-04,   2.6382e-08,  -7.3890e-09,   4.9508e-03,   2.4408e-08,  -2.7107e-07,   1.0708e-08,  -1.4970e-09,  -4.0882e-08,
  -4.3805e-09,   6.5788e-04,   2.0568e-08,   2.4408e-08,   4.9506e-03,   7.3273e-07,   9.0069e-10,  -7.4677e-11,  -2.0480e-09,
  -5.6508e-08,  -4.3418e-06,   1.3827e-02,  -2.7107e-07,   7.3273e-07,   4.5761e-01,   5.6237e-08,   2.2734e-09,   3.8328e-08,
  -2.0566e-09,  -2.4863e-09,   1.2792e-09,   1.0708e-08,   9.0069e-10,   5.6237e-08,   6.5582e-05,   1.5315e-06,   1.8966e-06,
   3.3180e-10,  -1.7279e-10,   5.7127e-11,  -1.4970e-09,  -7.4677e-11,   2.2734e-09,   1.5315e-06,   7.9562e-06,   5.0481e-08,
   7.8047e-09,  -3.4412e-09,   1.0591e-09,  -4.0882e-08,  -2.0480e-09,   3.8328e-08,   1.8966e-06,   5.0481e-08,   9.0595e-06
};
  // Estimated Values
  Vector3f pos_est = ErrorStateKalmanFilter.getPosition();
  Vector3f vel_est = ErrorStateKalmanFilter.getVelocity();
  Quaternion quat_est = ErrorStateKalmanFilter.getQuaternion();
  Vector3f rates_est = ErrorStateKalmanFilter.getBodyRates();
  Matrix9f P_est = ErrorStateKalmanFilter.getCovariance();

  // Compare States
  char msg[100];
  bool pos_test_result = compare_vector(pos_true,pos_est, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(pos_test_result, msg);

  bool vel_test_result = compare_vector(vel_true,vel_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(vel_test_result, msg);

  bool quat_test_result = compare_quaternion(quat_true, quat_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(quat_test_result, msg);

  bool rates_test_result = compare_vector(rates_true,rates_est, tol,msg);
  TEST_ASSERT_TRUE_MESSAGE(rates_test_result, msg);


  bool covariance_test_result = compare_matrix(P_true, P_est, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(covariance_test_result, msg);



}


// // No setup() or loop()
// // Use UNITY_BEGIN() in special main provided by PlatformIO
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_init);
    RUN_TEST(test_functions);
    RUN_TEST(test_propagation);
    RUN_TEST(test_update_mag);
    RUN_TEST(test_update_alt);
    RUN_TEST(test_update_gps);
    RUN_TEST(test_update_tilt);
    RUN_TEST(test_update_all);
    return UNITY_END();
}