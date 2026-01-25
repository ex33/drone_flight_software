#include "Controller.h"
#include "Constants.h"
#include "Motors.h"
#include<unity.h>
#include <string>

float tol = 1e-6;

std::array<float,3> posRef {1.0f, 1.0f, -10.0f};
std::array<float,3> velRef{0.0f, 0.0f, 0.0f};
std::array<float,4> quatRef{1.0f, 0.0f, 0.0f, 0.0f};
std::array<float,3> rateRef{0.0f, 0.0f, 0.0f};


// Thrust = kT * (u1 + u2 + u3 + u4)
// To cancel out weight, Thrust = m*g.
// To NOT have any roll/pitch/yaw u1=u2=u3=u4
// u = u1 = u2 = u3 = u4 = mg/(4*kT)
float kT = 1.2e-04;
float kM = 7.51e-07;
float L = 0.08f;
float m = 0.83f; 

float Ft = m*CONSTANTS::g0;
std::array<float,4> nominalControl {Ft,0.0f,0.0f,0.0f};

// Gain Matrix.
// Linearization only dependent upon quaternion and rate reference (identity and zero)
// NOT dependent upon kT kM or L
std::array<float,48> K {0.0f , 0.0f, -7.9464090440365f , 0.0f, 0.0f, -22.8071638549271f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                         0.0f, 0.0633491788912569f , 0.0f, 0.0f, 0.216910884742845f, 0.0f, 2.29232028316404f, 0.0f, 0.0f, 0.300548926668003f, 0.0f, 0.0f,
                                        -0.0822366804856256f, 0.0f, 0.0f, -0.281605587633673f, 0.0f, 0.0f, 0.0f, 2.97731024068504f , 0.0f, 0.0f, 0.390567835752269f, 0.0f, 
                                         0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.311508647946585f};

//Frequency 
float freq = 50; // Hz

Controller controller(freq, posRef, velRef, quatRef, rateRef, nominalControl, K, true, true); 

// Motor stuff
// Pins are 28 29 37 36 (top to bottom), left to right
int esc1SignalPin = 28;
int esc2SignalPin = 37;
int esc3SignalPin = 29;
int esc4SignalPin = 36;

int maxPWM = 1950; //Recommended by Hobby Wing
int saturationPWM = 1700;
int minPWM = 1150; //Recommended by Hobby Wing
int M1StartPWM = 1195; 
int M2StartPWM = 1195; 
int M3StartPWM = 1195; 
int M4StartPWM = 1195; 

float maxSpinSquare =  1600000.0f; // [Rev^2 / s^2] Used to Convert Control requested to PWM
Motors motors(kT, kM, L, esc1SignalPin, esc2SignalPin, esc3SignalPin, esc4SignalPin,
              M1StartPWM, M2StartPWM, M3StartPWM, M4StartPWM, 
              minPWM, maxPWM, saturationPWM, maxSpinSquare);


//Make these bool so that the actual test assert returns what lines.
inline bool compare_vector(const Vector3f& TruthVec, const Vector3f& TestVec, float tol, char* msg) {
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

inline bool compare_quaternion(const Quaternion& TruthQuat,
                               const Quaternion& TestQuat,
                               float tol,
                               char* msg) {
                                
    std::array<float, 4> test = {TestQuat.w(), TestQuat.x(), TestQuat.y(), TestQuat.z()};
    std::array<float, 4> truth = {TruthQuat.w(), TruthQuat.x(), TruthQuat.y(), TruthQuat.z()};
    msg[0] = '\0'; //Clear previous msg
    for (size_t i = 0; i < 4; ++i) {
        float diff = std::fabs(truth[i] - test[i]);
        if (diff > tol) {
            std::snprintf(msg, 100,
                          "Quaternion mismatch at index %u: expected %.9f, got %.9f",
                           i, truth[i], test[i]);
            return false;
        }
    }
    
    return true;
}

inline bool compare_matrix(const std::array<float,48>& TruthMatrix, const std::array<float,48>& TestMatrix, float tol,char* msg) {
  // Compare Gain Matrix
  msg[0] = '\0'; //Clear previous msg

  for (unsigned int i = 0; i<48; i++ ){
    if (fabs(TruthMatrix[i] - TestMatrix[i]) > tol) {
        snprintf(msg, 100, 
                  "Matrix mismatch at index %u: expected %.9f, got %.9f", 
                  i, TruthMatrix[i], TestMatrix[i]);
        return false; 
    }
  }

  return true;
}


// Make sure things are initialized properly
void test_controller_init() {
  Vector3f testPosRef = controller.getPosRef();
  Vector3f testVelRef = controller.getVelRef();
  Quaternion testQuatRef = controller.getQuatRef();
  Vector3f testRateRef = controller.getRateRef();

  std::array<float,4> testNominalControl = controller.getNominalControl();
  std::array<float,48> testGainMatrix = controller.getGainMatrix();

  char msg [100];
  bool posRef_test_result = compare_vector(posRef, testPosRef, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(posRef_test_result, msg);

  bool velRef_test_result = compare_vector(velRef, testVelRef, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(velRef_test_result, msg);

  bool quatRef_test_result = compare_quaternion(quatRef, testQuatRef, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(quatRef_test_result, msg);

  bool rateRef_test_result = compare_vector(rateRef, testRateRef, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(rateRef_test_result, msg);

  for (unsigned int i = 0; i < 4; ++i) {
      TEST_ASSERT_FLOAT_WITHIN(tol, nominalControl[i], testNominalControl[i]);
  };

  bool gain_test_result = compare_matrix(K, testGainMatrix, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(gain_test_result, msg);

}

void test_controller_functions() {
  Vector3f p_hat (0.5f, 0.6f, -0.5f);
  Vector3f v_hat (0.1f, 0.3f, 0.4f);
  Quaternion q_hat (0.999986366648586f ,0.00400194643981425f ,  0.0032015571518514f  , 0.00100048660995356f);
  Vector3f w_hat (0.2f, 0.1f, 0.4f);

  // Test Control Calcluated correctly
  std::array<float,4> uk_Test;
  if (controller.updateControl(100, p_hat, v_hat, q_hat, w_hat)) { //For the first call, any time provided to controller will update it
    uk_Test = controller.getControl(); 
  }

  std::array<float,4> uk_True {92.7532709603176, -0.109017122196074, -0.061546593949018 ,-0.124603459178634};
  for (unsigned int i = 0; i < 4; ++i) {
      TEST_ASSERT_FLOAT_WITHIN(1e-6, uk_True[i], uk_Test[i]); 
  };

  // Test Errors updated properly
  Vector3f posErr_Test = controller.getPosErr();
  Vector3f velErr_Test = controller.getVelErr();
  Vector3f alpha_Test = controller.getAlpha();
  Vector3f rateErr_Test = controller.getRateErr();

  Vector3f posErr_True (-0.5f , -0.4f, 9.5f);
  Vector3f velErr_True (0.1f, 0.3f, 0.4f);
  Vector3f alpha_True (0.00400194643981425f,0.0032015571518514f, 0.00100048660995356f);
  Vector3f rateErr_True (0.2f, 0.1f, 0.4f);

  char msg [100];
  bool posErr_test_result = compare_vector(posErr_True, posErr_Test, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(posErr_test_result, msg);

  bool velErr_test_result = compare_vector(velErr_True, velErr_Test, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(velErr_test_result, msg);

  bool alpha_test_result = compare_vector(alpha_True, alpha_Test, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(alpha_test_result, msg);

  bool rateErr_test_result = compare_vector(rateErr_True, rateErr_Test, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(rateErr_test_result, msg);

  // Make sure that the parameters stayed constant
  Vector3f testPosRef = controller.getPosRef();
  Vector3f testVelRef = controller.getVelRef();
  Quaternion testQuatRef = controller.getQuatRef();
  Vector3f testRateRef = controller.getRateRef();

  std::array<float,4> testNominalControl = controller.getNominalControl();
  std::array<float,48> testGainMatrix = controller.getGainMatrix();

  bool posRef_test_result = compare_vector(posRef, testPosRef, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(posRef_test_result, msg);

  bool velRef_test_result = compare_vector(velRef, testVelRef, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(velRef_test_result, msg);

  bool quatRef_test_result = compare_quaternion(quatRef, testQuatRef, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(quatRef_test_result, msg);

  bool rateRef_test_result = compare_vector(rateRef, testRateRef, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(rateRef_test_result, msg);

  for (unsigned int i = 0; i < 4; ++i) {
      TEST_ASSERT_FLOAT_WITHIN(tol, nominalControl[i], testNominalControl[i]);
  };

  bool gain_test_result = compare_matrix(K, testGainMatrix, tol, msg);
  TEST_ASSERT_TRUE_MESSAGE(gain_test_result, msg);
}

// Test motor stuff
void test_motors_init() {
    // Motor constants 
    float testKM = motors.getKM();
    float testKT = motors.getKT();
    float testL = motors.getL();
    std::array<float,16> testPInvM = motors.getPInvM();
    std::array<float,16> pInvM = {2083.333333f,          26041.666666f,          26041.666666f,          -332889.480692409f,
                                  2083.333333f,         -26041.666666f,          26041.666666f,          332889.48069241f,
                                  2083.333333f,         -26041.666666f,        -26041.666666f,          -332889.480692411f,
                                  2083.333333f,          26041.666666f,         -26041.666666f,          332889.48069241f};
    // Servo parameters
    int testESC1SignalPin = motors.getESC1SignalPin();
    int testESC2SignalPin = motors.getESC2SignalPin();
    int testESC3SignalPin = motors.getESC3SignalPin();
    int testESC4SignalPin = motors.getESC4SignalPin(); 

    int testM1StartPWM = motors.getM1StartPWM();
    int testM2StartPWM = motors.getM2StartPWM();
    int testM3StartPWM = motors.getM3StartPWM();
    int testM4StartPWM = motors.getM4StartPWM();

    int testMinPWM = motors.getMinPWM();
    int testMaxPWM = motors.getMaxPWM();

    float testMaxSpinSquare = motors.getMaxSpinSquare();
 
    bool testArmedBool = false; 


    TEST_ASSERT_FLOAT_WITHIN(tol, kM, testKM);
    TEST_ASSERT_FLOAT_WITHIN(tol, kT, testKT);
    TEST_ASSERT_FLOAT_WITHIN(tol, L, testL);
    for (int i = 0; i<16; i++) {
      TEST_ASSERT_FLOAT_WITHIN(1e-2, pInvM[i], testPInvM[i]); //Need looser tolerance here
    }

    TEST_ASSERT_EQUAL_INT(esc1SignalPin, testESC1SignalPin);
    TEST_ASSERT_EQUAL_INT(esc2SignalPin, testESC2SignalPin);
    TEST_ASSERT_EQUAL_INT(esc3SignalPin, testESC3SignalPin);
    TEST_ASSERT_EQUAL_INT(esc4SignalPin, testESC4SignalPin);

    TEST_ASSERT_EQUAL_INT(M1StartPWM, testM1StartPWM);
    TEST_ASSERT_EQUAL_INT(M2StartPWM, testM2StartPWM);
    TEST_ASSERT_EQUAL_INT(M3StartPWM, testM3StartPWM);
    TEST_ASSERT_EQUAL_INT(M4StartPWM, testM4StartPWM);

    TEST_ASSERT_EQUAL_INT(minPWM, testMinPWM);
    TEST_ASSERT_EQUAL_INT(maxPWM, testMaxPWM);  

    TEST_ASSERT_FLOAT_WITHIN(tol , maxSpinSquare, testMaxSpinSquare);

    TEST_ASSERT_EQUAL_INT(0, testArmedBool); //Should start off NOT armed
}


void test_controller_motors_interaction() {
  Vector3f p_hat (0.5f, 0.6f, -0.5f);
  Vector3f v_hat (0.1f, 0.3f, 0.4f);
  Quaternion q_hat (0.999986366648586f ,0.00400194643981425f ,  0.0032015571518514f  , 0.00100048660995356f);
  Vector3f w_hat (0.2f, 0.1f, 0.4f);


  std::array<float,4> uk_Test;
  if (controller.updateControl(30000, p_hat, v_hat, q_hat, w_hat)) { //Provide some large number to get it to update
    uk_Test = controller.getControl(); 
  }

  //Already tested that controller works properly, so no need to reverify.
  motors.commandControl(uk_Test); //Because not armed, this should just update the PWM without doing anything
  std::array<int,4> pwm_Test = motors.getCurrentMotorPWN();

  std::array<int,4> pwm_Truth = {1304, 1268 ,1308, 1267};

  for (int i = 0; i<4; i++) {
    TEST_ASSERT_INT_WITHIN(2,pwm_Truth[i],pwm_Test[i]);
  }

}
// No setup() or loop()
// Use UNITY_BEGIN() in special main provided by PlatformIO
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_controller_init);
    RUN_TEST(test_controller_functions);
    RUN_TEST(test_motors_init);
    RUN_TEST(test_controller_motors_interaction);
    return UNITY_END();
}