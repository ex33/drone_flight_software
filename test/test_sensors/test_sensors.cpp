#include "Sensors.h"
#include<unity.h>

//Init Sensors
double imuFrequency {100}; //Hz
double magFrequency {50}; //Hz
double altFrequency {50}; //Hz
double gpsFrequency {1}; //Hz
std::array<double,3> magHardIron{0.0, 0.0, 0.0};
std::array<double,9> magSoftIron{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
std::array<double,9> rotIMU2Body{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
std::array<double,9> rotMag2Body{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
Sensors sensors(imuFrequency, magFrequency, altFrequency, gpsFrequency, Serial6); //GPS connected to port 6

// Set Reference
double refAlt = 93; //m
double refLatitude = 34.05; //North, deg
double refLongitude = 241.76; //East, Deg

double testAlt = 90.965286873281;
double testLatitude = 34.0500134411289;
double testLongitude = 241.759976543169;

// Individual test functions
void test_gps_functions() {
    std::array<double,3> rECEF_test=sensors.LLA2ECEF(refLatitude, refLongitude, refAlt);
    std::array<double,3> rECEF_true{-2503157.47768362,-4660553.3447325,3551095.242177 }; //From Matlab Sim / Online Calculator
    for (unsigned int i = 0; i < 3; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(1e-6,rECEF_true[i], rECEF_test[i]);
    }

    std::array<double,2> refLLA = {refLatitude, refLongitude};
    sensors.setAltCalibration(refAlt,1); //Sets reference altitude
    sensors.setGPSCalibration(refLLA, 1); //Sets reference r_ECEF AND ECEF2NED rotation

    std::array<double,3> rNED_NULL = sensors.LLA2NED(refLatitude, refLongitude, refAlt);
    for (unsigned int i = 0; i < 3; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(1e-6,rNED_NULL[i], 0);
    }
    std::array<double,3> rNED_test = sensors.LLA2NED(testLatitude, testLongitude, testAlt);
    std::array<double,3> rNED_true { 1.49095581802802 ,-2.16581433006995, 2.03471366871746}; //From Matlab Sim
    for (unsigned int i = 0; i < 3; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(1e-6,rNED_true[i], rNED_test[i]);
    }
}

// No setup() or loop()
// Use UNITY_BEGIN() in special main provided by PlatformIO
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_gps_functions);

    return UNITY_END();
}