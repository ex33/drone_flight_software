#include "Controller.h"
#include "Constants.h"
#include<unity.h>
#include <string>

float tol = 1e-6;

Vector3f posRef(1.0f, 1.0f, -10.0f);
Vector3f velRef(0.0f, 0.0f, 0.0f);
Quaternion quatRef(1.0f, 0.0f, 0.0f, 0.0f);
Vector3f rateRef(0.0f, 0.0f, 0.0f);


// Thrust = kT * (u1 + u2 + u3 + u4)
// To cancel out weight, Thrust = m*g.
// To NOT have any roll/pitch/yaw u1=u2=u3=u4
// u = u1 = u2 = u3 = u4 = mg/(4*kT)
float kT = 1.0f;
float m = 0.5f; 

float u = m*Constants::g0 / (4*kT);
std::array<float,4> nominalControl {u,u,u,u};

// Gain Matrix.
// For MOI = [0.0023, 0.0023, 0.004]
// L = 0.1750
// kT = 1, kM = 0.0245
// m = 0.5
// Linearization only dependent upon quaternion and rate reference (identity and zero)
std::array<float,48> K {-4.849329729498414e-04f, 4.849329729501528e-04f, -1.577043970534665e-02f, -2.701174473031700e-03f, 2.701174473031881e-03f, -6.481927938667377e-02f, 4.592644411647154e-02f, 4.592644411647229e-02f, 1.575353741974887e-02f, 1.973242999999838e-02f, 1.973242999999882e-02f, 2.989197200035366e-02f,
                        -4.849329729498509e-04f, -4.849329729500159e-04f, -1.577043970535033e-02f, -2.701174473031981e-03f, -2.701174473031971e-03f, -6.481927938665394e-02f, -4.592644411646527e-02f, 4.592644411647453e-02f, -1.575353741974892e-02f, -1.973242999999827e-02f, 1.973242999999937e-02f, -2.989197200035303e-02f,
                        4.849329729498911e-04f, -4.849329729497276e-04f, -1.577043970534945e-02f, 2.701174473031877e-03f, -2.701174473031226e-03f, -6.481927938666136e-02f, -4.592644411646945e-02f, -4.592644411647349e-02f, 1.575353741976178e-02f, -1.973242999999824e-02f, -1.973242999999901e-02f, 2.989197200035791e-02f,
                        4.849329729498709e-04f, 4.849329729497230e-04f, -1.577043970534666e-02f, 2.701174473032187e-03f, 2.701174473032131e-03f, -6.481927938668910e-02f, 4.592644411647674e-02f, -4.592644411647954e-02f, -1.575353741976311e-02f, 1.973242999999941e-02f, -1.973242999999982e-02f, -2.989197200036071e-02f};

Controller controller(posRef, velRef, quatRef, rateRef, nominalControl, K);


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
  std::array<float,4> uk_Test = controller.getControl(p_hat, v_hat, q_hat, w_hat);
  std::array<float,4> uk_True {1.38241970794663f,1.41625724134349f, 1.39675994755104f,1.41087565898074f};
  for (unsigned int i = 0; i < 4; ++i) {
      TEST_ASSERT_FLOAT_WITHIN(tol, uk_True[i], uk_Test[i]);
  };

  // Test Errors updated properly
  Vector3f posErr_Test = controller.getPosErr();
  Vector3f velErr_Test = controller.getVelErr();
  Vector3f alpha_Test = controller.getAlpha();
  Vector3f rateErr_Test = controller.getRateErr();

  Vector3f posErr_True (-0.5f , -0.4f, 9.51f);
  Vector3f velErr_True (0.1f, 0.3f, 0.4f);
  Vector3f alpha_True (0.0080038928796285f, 0.0064031143037028f, 0.00200097321990712f);
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