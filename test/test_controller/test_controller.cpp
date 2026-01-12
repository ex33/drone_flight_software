#include "Controller.h"
#include "Constants.h"
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
float kT = 5.63e-5f;
float m = 0.83f; 

float u = m*CONSTANTS::g0 / (4*kT);
std::array<float,4> nominalControl {u,u,u,u};

// Gain Matrix.
// For MOI = [0.0023, 0.0023, 0.004]
// L = 0.1750
// kT = 1, kM = 0.0245
// m = 0.5
// Linearization only dependent upon quaternion and rate reference (identity and zero)
std::array<float,48> K {  
-3768.37555821952f,  3094.9351294033f,  -7821.72041357272f, -12912.5526355348f,  10601.5460968904f, -23429.0995571371f,  112281.94027554f,   136993.645715096f,  32351.5594092494f,   14760.9652458602f,  18047.8038528373f,  33254.3231975853f,
-3768.37555821286f, -3094.93512940513f, -7821.72041357484f, -12912.5526355172f, -10601.5460968818f, -23429.0995571414f, -112281.940275284f,  136993.645714991f,  -32351.5594092518f, -14760.9652458586f,  18047.8038528366f, -33254.3231975848f,
 3768.3755582117f,  -3094.93512940343f, -7821.72041357451f,  12912.5526355139f, -10601.5460968996f, -23429.0995571395f, -112281.940275589f, -136993.645714994f,  32351.5594092463f,  -14760.9652458604f, -18047.8038528366f,  33254.3231975843f,
 3768.37555821385f,  3094.93512939963f, -7821.72041357656f,  12912.5526355188f,  10601.5460968693f, -23429.0995571431f,  112281.94027523f,  -136993.645715011f,  -32351.5594092447f,  14760.9652458584f, -18047.8038528367f, -33254.3231975854f
};

//Frequency 
float freq = 50; // Hz

Controller controller(freq, posRef, velRef, quatRef, rateRef, nominalControl, K, true, true); 


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

  std::array<float,4> uk_True {98307.0667997464, 135663.318237072, 114667.736849078, 130647.871735933};
  for (unsigned int i = 0; i < 4; ++i) {
      TEST_ASSERT_FLOAT_WITHIN(1, uk_True[i], uk_Test[i]); // Lower the tolerance here since this is on order of 1e6 already
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


// No setup() or loop()
// Use UNITY_BEGIN() in special main provided by PlatformIO
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_controller_init);
    RUN_TEST(test_controller_functions);

    return UNITY_END();
}