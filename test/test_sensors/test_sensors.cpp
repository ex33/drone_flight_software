#include "Sensors.h"
#include<unity.h>

//Init Sensors
float imuFrequency {100.0f}; //Hz
float magFrequency {50.0f}; //Hz
float altFrequency {50.0f}; //Hz
float gpsFrequency {1.0f}; //Hz
std::array<float,3> magHardIron{0.0f, 0.0f, 0.0f};
std::array<float,9> magSoftIron{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
std::array<float,9> rotIMU2Body{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
std::array<float,9> rotMag2Body{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
Sensors sensors(imuFrequency, magFrequency, altFrequency, gpsFrequency, Serial6); //GPS connected to port 6

// Set Reference
double refAlt = 93.0L; //m
double refLatitude = 34.05L; //North, deg
double refLongitude = 241.76L; //East, Deg

double testAlt = 90.965286873281L;
double testLatitude = 34.0500134411289L;
double testLongitude = 241.759976543169L;

// Individual test functions
void test_gps_functions() {
    std::array<double,3> rECEF_test=sensors.LLA2ECEF(refLatitude, refLongitude, refAlt);
    std::array<double,3> rECEF_true{-2503157.47768362L,-4660553.3447325L,3551095.242177L}; //From Matlab Sim / Online Calculator
    for (unsigned int i = 0; i < 3; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(1e-6,rECEF_true[i], rECEF_test[i]); //Because we are using floats, we can only get up to 7 digits. 2/1e6 ~ order of 1e-6 accuracy
    }

    //Set up the internal References and ECEF2NED rotation
    std::array<double,2> refLLA = {refLatitude, refLongitude};
    sensors.setAltCalibration(refAlt,1); //Sets reference altitude
    sensors.setGPSCalibration(refLLA, 1); //Sets reference r_ECEF AND ECEF2NED rotation

    //Test that if given the reference Lat,Long,Alt, we should get (0,0,0) for NED Position
    Vector3f rNED_NULL = sensors.LLA2NED(refLatitude, refLongitude, refAlt);
    for (unsigned int i = 0; i < 3; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(1e-4,rNED_NULL[i], 0);
    }
    //Now test that if we give a random Lat Long Alt, we should get the corresponding NED Position given our reference
    Vector3f rNED_test = sensors.LLA2NED(testLatitude, testLongitude, testAlt);
    std::array<float,3> rNED_true { 1.49095581802802 ,-2.16581433006995, 2.03471366871746}; //From Matlab Sim
    for (unsigned int i = 0; i < 3; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(1e-4,rNED_true[i], rNED_test[i]);
    }
}


// No setup() or loop()
// Use UNITY_BEGIN() in special main provided by PlatformIO
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_gps_functions);

    return UNITY_END();
}