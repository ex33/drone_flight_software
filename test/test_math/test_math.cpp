#include "Mathpk.h"
#include<unity.h>
#include <cstdio>
//Init Rotation and Quaternion
//Passive rotation around R1 R2 R3 by 90 degrees
Rotation testR1 = R1(M_PI/2);
Rotation testR2 = R2(M_PI/2);
Rotation testR3 = R3(M_PI/2);

Quaternion testQuat (0.118678165819385, 0.356034497458156, 0.593390829096927, 0.712068994916312);

// Normalized {1,-3,5}
Vector3f vec {0.147441956154897, -0.442325868464691,0.884651736929383};
float tol = 1e-5;


Matrix18f testSTM( std::array<float, 324> {
    1.0f, 0.0f, 0.0f, 0.02f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.02f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.02f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,

    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,0.202133f, -0.004f, -0.02f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -0.202133f, 0.0f,  0.002f, 0.0f, -0.02f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.004f, -0.002f, 0.0f, 0.0f, 0.0f, -0.02f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,

    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.012f, -0.01f, 0.0f, 0.0f, 0.0f, -0.02f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.012f, 1.0f, 0.008f, 0.0f, 0.0f, 0.0f, 0.0f, -0.02f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, -0.008f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.02f, 0.0f, 0.0f, 0.0f,

    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,

    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,

    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
});

Matrix18f test_Qd (std::array<float, 324>{
    0.000000000024f, 0.0f, 0.0f, 0.0000000018f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.000000000024f, 0.0f, 0.0f, 0.0000000018f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.000000000024f, 0.0f, 0.0f, 0.0000000018f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0000000018f, 0.0f, 0.0f, 0.000000180266667f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.00000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0000000018f, 0.0f, 0.0f, 0.000000180266667f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.00000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0000000018f, 0.0f, 0.0f, 0.000000180266667f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.00000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.005000000000027f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.000000000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.005000000000027f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.000000000002f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.005000000000027f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.000000000002f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, -0.00000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, -0.00000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.00000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.000000000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0000000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.000000000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0000000002f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.000000000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0000000002f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.00000002f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.00000002f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.00000002f,
});


Matrix18f test_P(std::array<float,324>{
    3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f
});

//Helper function for comparing arrays



// Individual test functions
    
void test_vector3f_functions() {
    Vector3f vec1 {1,-2,3};
    Vector3f vec2 {-3 ,5 , -12};
    Vector3f vec3 {2, 10, 8};

    //Dot product
    float dotProduct = dot(vec1,vec2);
    TEST_ASSERT_FLOAT_WITHIN(tol , -49, dotProduct);

    //Cross product
    Vector3f crossProduct = cross(vec1, vec2);
    Vector3f crossProduct_true {9,3,-1};
    for (unsigned int i = 0; i<3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(tol, crossProduct_true[i], crossProduct[i]);
    }

    // Normalize Vector
    float vec1Mag = vec1.getMag();
    vec1 = vec1.normalize();

    TEST_ASSERT_FLOAT_WITHIN(tol, 3.741657386773941, vec1Mag);
    TEST_ASSERT_FLOAT_WITHIN(tol, 0.267261241912424, vec1[0]);
    TEST_ASSERT_FLOAT_WITHIN(tol , -0.534522483824849, vec1[1]);
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.801783725737273, vec1[2]);

    //Operators
    Vector3f multVec = vec2 * 2;
    TEST_ASSERT_FLOAT_WITHIN(tol, -6, multVec[0]);
    TEST_ASSERT_FLOAT_WITHIN(tol , 10, multVec[1]);
    TEST_ASSERT_FLOAT_WITHIN(tol , -24, multVec[2]);

    Vector3f divideVec = vec2 / 2;
    TEST_ASSERT_FLOAT_WITHIN(tol, -1.5, divideVec[0]);
    TEST_ASSERT_FLOAT_WITHIN(tol , 2.5, divideVec[1]);
    TEST_ASSERT_FLOAT_WITHIN(tol , -6, divideVec[2]);

    Vector3f addVec = vec2 + vec3;
    TEST_ASSERT_FLOAT_WITHIN(tol, -1, addVec[0]);
    TEST_ASSERT_FLOAT_WITHIN(tol , 15, addVec[1]);
    TEST_ASSERT_FLOAT_WITHIN(tol , -4, addVec[2]);

    Vector3f subVec = vec2 - vec3;
    TEST_ASSERT_FLOAT_WITHIN(tol, -5, subVec[0]);
    TEST_ASSERT_FLOAT_WITHIN(tol , -5, subVec[1]);
    TEST_ASSERT_FLOAT_WITHIN(tol , -20, subVec[2]);
}



void test_rotation_functions() {
    //Test Rotation of Vectors
    Vector3f vec_R1 = testR1 * vec;
    Vector3f vec_R1_true {vec[0], vec[2], -vec[1]}; // Old Z is now Y, Old Y is now -Z
    Vector3f vec_R2 = testR2 * vec;
    Vector3f vec_R2_true {-vec[2],vec[1],vec[0]}; // Old X is now Z. Old Z is now -X
    Vector3f vec_R3 = testR3 * vec; 
    Vector3f vec_R3_true {vec[1],-vec[0],vec[2]}; // Old X is now -Y. Old Y is now X

    
    for (unsigned int i = 0; i<3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(tol, vec_R1_true[i], vec_R1[i]);
    }

    for (unsigned int i = 0; i<3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(tol, vec_R2_true[i], vec_R2[i]);
    }
    for (unsigned int i = 0; i<3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(tol, vec_R3_true[i], vec_R3[i]);
    }

    //Test Rotation Multiplcation
    Rotation R321 = testR1 * testR2 * testR3;
    std::array<float,9> R321Elements = R321.getElements();
    std::array<float,9> R321_true {0,0,-1,0,1,0,1,0,0};
    for (unsigned int i = 0; i<9; i++) {
        TEST_ASSERT_FLOAT_WITHIN(tol, R321_true[i], R321Elements[i]);
    }

    //Quaternion init / q2R
    Rotation RotQuatInit = q2R(testQuat);
    std::array<float,9> RotQuatInitElements = RotQuatInit.getElements();
    std::array<float,9> q2R_true {-0.718309859154929,  0.591549295774648,  0.366197183098591,
                                    0.253521126760563,  -0.267605633802817, 0.929577464788732,
                                    0.647887323943662, 0.76056338028169, 0.0422535211267607}; //Obtained from Drone Sim
    
    for (unsigned int i = 0; i<9; i++) {
        TEST_ASSERT_FLOAT_WITHIN(tol, q2R_true[i], RotQuatInitElements[i]);
    }

    //Test Transpose
    Rotation RotTranspose = RotQuatInit.transpose();
    std::array<float,9> RotTransposeElements = RotTranspose.getElements();
    std::array<float,9> RotTranspose_true {-0.718309859154929,  0.253521126760563,  0.647887323943662,
                                            0.591549295774648,  -0.267605633802817, 0.76056338028169,
                                            0.366197183098591, 0.929577464788732, 0.0422535211267607}; //Obtained from Drone Sim

    for (unsigned int i = 0; i<9; i++) {
        TEST_ASSERT_FLOAT_WITHIN(tol, RotTranspose_true[i], RotTransposeElements[i]);
    }

}



void test_quaternion_functions(){
    Quaternion quatTest(3,7,10,3);

    // Normalize Quaternion
    float quatTestMag = quatTest.getMag();
    quatTest.normalize();

    TEST_ASSERT_FLOAT_WITHIN(tol, 12.9228479833201, quatTestMag);
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.232146969760241, quatTest.w());
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.541676262773896, quatTest.x());
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.773823232534137, quatTest.y());
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.232146969760241, quatTest.z());

    // Test Scalar Multiplication
    Quaternion quatTest2 = quatTest * 2;
    Quaternion quatTest3 = 2 * quatTest;
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.464293939520482, quatTest2.w());
    TEST_ASSERT_FLOAT_WITHIN(tol , 1.083352525547792, quatTest2.x());
    TEST_ASSERT_FLOAT_WITHIN(tol , 1.547646465068274, quatTest2.y());
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.464293939520482, quatTest2.z());

    TEST_ASSERT_FLOAT_WITHIN(tol , 0.464293939520482, quatTest3.w());
    TEST_ASSERT_FLOAT_WITHIN(tol , 1.083352525547792, quatTest3.x());
    TEST_ASSERT_FLOAT_WITHIN(tol , 1.547646465068274, quatTest3.y());
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.464293939520482, quatTest3.z());

    //Test Scalar Divide
    Quaternion quatTest4 = quatTest2 / 2;
    Quaternion quatTest5 = 2 / quatTest3;
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.232146969760241, quatTest4.w());
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.541676262773896, quatTest4.x());
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.773823232534137, quatTest4.y());
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.232146969760241, quatTest4.z());

    TEST_ASSERT_FLOAT_WITHIN(tol , 0.232146969760241, quatTest5.w());
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.541676262773896, quatTest5.x());
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.773823232534137, quatTest5.y());
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.232146969760241, quatTest5.z());

    //Test Prop: q_dot = 1/2 omega cdot q * dt = quatMul(q, 1/2*omega*dt) = quatProp(1/2*omega*dt, q)
    Vector3f omegaTest(1,1,1);

    Quaternion omegaQuat (0 , omegaTest); // Let omega = {1,1,1} and dt = 1
    float dt = 1;
    Quaternion qdotTest = quatMult(quatTest * dt, 0.5 * omegaQuat); //Note that quatMult uses q otimes w
    TEST_ASSERT_FLOAT_WITHIN(tol , -0.773823232534137, qdotTest.w());
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.386911616267068, qdotTest.x());
    TEST_ASSERT_FLOAT_WITHIN(tol , -0.0386911616267068, qdotTest.y());
    TEST_ASSERT_FLOAT_WITHIN(tol ,-2.77555756156289e-17, qdotTest.z());

    //Test overload
    Quaternion qdotOverloadTest = quatProp(0.5*omegaTest, quatTest*dt); //Note that quatProp uses w cdot q
    TEST_ASSERT_FLOAT_WITHIN(tol , -0.773823232534137, qdotTest.w());
    TEST_ASSERT_FLOAT_WITHIN(tol , 0.386911616267068, qdotTest.x());
    TEST_ASSERT_FLOAT_WITHIN(tol , -0.0386911616267068, qdotTest.y());
    TEST_ASSERT_FLOAT_WITHIN(tol ,-2.77555756156289e-17, qdotTest.z());
}


void test_matrix18f_functions () {
    // Test Transpose 
    Matrix18f transpose_test = testSTM.transpose();

    std::array<float, 324> truth_transpose_test = {
        /* Row 1  */ 1.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,     0.0f,     0.0f,     0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row 2  */ 0.0f,    1.0f,    0.0f,    0.0f,    0.0f,    0.0f,     0.0f,     0.0f,     0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row 3  */ 0.0f,    0.0f,    1.0f,    0.0f,    0.0f,    0.0f,     0.0f,     0.0f,     0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row 4  */ 0.02f,   0.0f,    0.0f,    1.0f,    0.0f,    0.0f,     0.0f,     0.0f,     0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row 5  */ 0.0f,    0.02f,   0.0f,    0.0f,    1.0f,    0.0f,     0.0f,     0.0f,     0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row 6  */ 0.0f,    0.0f,    0.02f,   0.0f,    0.0f,    1.0f,     0.0f,     0.0f,     0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row 7  */ 0.0f,    0.0f,    0.0f,    0.0f,   -0.202133f, 0.004f,  1.0f,   -0.012f,   0.01f,   0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row 8  */ 0.0f,    0.0f,    0.0f,    0.202133f, 0.0f,   -0.002f,  0.012f,  1.0f,    -0.008f,  0.0f,    0.0f,    0.0f,    0.0f,   -0.0f, 0.0f,    0.0f,    0.0f,    0.0f,
        /* Row 9  */ 0.0f,    0.0f,    0.0f,   -0.004f,   0.002f,  0.0f,   -0.01f,   0.008f,   1.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,   -0.0f, 0.0f,    0.0f,    0.0f,
        /* Row10  */ 0.0f,    0.0f,    0.0f,   -0.02f,   0.0f,   0.0f,    0.0f,    0.0f,    0.0f,   1.0f,   0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row11  */ 0.0f,    0.0f,    0.0f,    0.0f,   -0.02f,  0.0f,    0.0f,    0.0f,    0.0f,   0.0f,   1.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row12  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,   -0.02f,   0.0f,    0.0f,    0.0f,   0.0f,   0.0f,    1.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row13  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,   -0.02f,   0.0f,    0.0f,   0.0f,   0.0f,    0.0f,    1.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row14  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,   -0.02f,   0.0f,   0.0f,   0.0f,    0.0f,    0.0f,    1.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row15  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,   -0.02f,  0.0f,   0.0f,    0.0f,    0.0f,    0.0f,    1.0f,    0.0f,    0.0f,    0.0f,
        /* Row16  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,   0.0f,   0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    1.0f,    0.0f,    0.0f,
        /* Row17  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,   0.0f,   0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    1.0f,    0.0f,
        /* Row18  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,   0.0f,   0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    1.0f
    };

    char msg[100]; // buffer for the message
    for (unsigned int i = 0; i<324; i++ ){
        if (fabs(truth_transpose_test[i] - transpose_test[i]) > 1e-6f) {
            snprintf(msg, sizeof(msg), 
                    "Matrix mismatch at index %u: expected %.9f, got %.9f", 
                    i, truth_transpose_test[i], transpose_test[i]);
            TEST_FAIL_MESSAGE(msg); // fails the test with the formatted message
        }
    }


    //Mostly just need to test multiplication and addition
    Matrix18f mult_test = testSTM * test_P;
    std::array<float, 324> truth_mult_test = {
        /* Row 1  */ 3.0f,    0.0f,    0.0f,    0.01f,   0.0f,    0.0f,    0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row 2  */ 0.0f,    3.0f,    0.0f,    0.0f,    0.01f,   0.0f,    0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row 3  */ 0.0f,    0.0f,    3.0f,    0.0f,    0.0f,    0.01f,   0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row 4  */ 0.0f,    0.0f,    0.0f,    0.5f,    0.0f,    0.0f,    0.0f,     0.020213f,-0.0004f, -0.0002f, 0.0f,     0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row 5  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.5f,    0.0f,   -0.020213f, 0.0f,     0.0002f,  0.0f,    -0.0002f,  0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row 6  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.5f,    0.0004f,  -0.0002f,  0.0f,     0.0f,     0.0f,   -0.0002f,  0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row 7  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.1f,     0.0012f, -0.0010f,  0.0f,     0.0f,    0.0f,   -0.0002f,  0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row 8  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,   -0.0012f,  0.1f,     0.0008f,  0.0f,     0.0f,    0.0f,    0.0f,   -0.0002f,  0.0f,    0.0f,    0.0f,    0.0f,
        /* Row 9  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0010f, -0.0008f,  0.1f,     0.0f,     0.0f,    0.0f,    0.0f,    0.0f,   -0.0002f,  0.0f,    0.0f,    0.0f,
        /* Row10  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,     0.0f,     0.0f,     0.01f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row11  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,     0.0f,     0.0f,     0.0f,     0.01f,   0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row12  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,     0.0f,     0.0f,     0.0f,     0.0f,    0.01f,   0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row13  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,     0.0f,     0.0f,     0.0f,     0.0f,    0.0f,    0.01f,   0.0f,    0.0f,    0.0f,    0.0f,    0.0f,
        /* Row14  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,     0.0f,     0.0f,     0.0f,     0.0f,    0.0f,    0.0f,    0.01f,   0.0f,    0.0f,    0.0f,    0.0f,
        /* Row15  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,     0.0f,     0.0f,     0.0f,     0.0f,    0.0f,    0.0f,    0.0f,    0.01f,   0.0f,    0.0f,    0.0f,
        /* Row16  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,     0.0f,     0.0f,     0.0f,     0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.01f,   0.0f,    0.0f,
        /* Row17  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,     0.0f,     0.0f,     0.0f,     0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.01f,   0.0f,
        /* Row18  */ 0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,     0.0f,     0.0f,     0.0f,     0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.0f,    0.01f
    };
  // Compare covariance
  for (unsigned int i = 0; i<324; i++ ){
    if (fabs(truth_mult_test[i] - mult_test[i]) > 1e-6f) {
        snprintf(msg, sizeof(msg), 
                  "Matrix mismatch at index %u: expected %.9f, got %.9f", 
                  i, truth_mult_test[i], mult_test[i]);
        TEST_FAIL_MESSAGE(msg); // fails the test with the formatted message
    }
  }

    //Mostly just need to test multiplication and addition
    Matrix18f chain_mult_test = testSTM * test_P * testSTM.transpose();
    std::array<float, 324> truth_chain_mult_test = {
        /* Row 1  */ 3.0002f, 0.0f, 0.0f, 0.01f,          0.0f,            0.0f,            0.0f,            0.0f,            0.0f,            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row 2  */ 0.0f,    3.0002f,0.0f, 0.0f,          0.01f,           0.0f,            0.0f,            0.0f,            0.0f,            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row 3  */ 0.0f,    0.0f,   3.0002f,0.0f,        0.0f,            0.01f,           0.0f,            0.0f,            0.0f,            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row 4  */ 0.01f,   0.0f,   0.0f,   0.50409137f,  -8.0e-07f,      -4.04266e-05f,   2.465596e-04f,   2.02101e-02f,    -5.617064e-04f,  -0.0002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row 5  */ 0.0f,    0.01f,  0.0f,   -8.0e-07f,    0.50409017f,    -8.08532e-05f,   -2.02153e-02f,   2.441596e-04f,   -2.133e-06f,      0.0f, -0.0002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row 6  */ 0.0f,    0.0f,   0.01f,  -4.04266e-05f, -8.08532e-05f,   0.50000600f,     3.9760e-04f,     -2.0480e-04f,    5.6e-06f,         0.0f, 0.0f, -0.0002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row 7  */ 0.0f,    0.0f,   0.0f,   0.0002465596f, -2.02153e-02f,   3.9760e-04f,     0.1000284f,      -8.0e-06f,       -9.6e-06f,        0.0f, 0.0f, 0.0f, -0.0002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row 8  */ 0.0f,    0.0f,   0.0f,   0.0202101f,     0.0002441596f,   -0.0002048f,     -8.0e-06f,       0.1000248f,      -1.2e-05f,        0.0f, 0.0f, 0.0f, 0.0f, -0.0002f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row 9  */ 0.0f,    0.0f,   0.0f,  -0.0005617064f, -2.133e-06f,     5.6e-06f,       -9.6e-06f,       -1.2e-05f,        0.1000204f,       0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.0002f, 0.0f, 0.0f, 0.0f,
        /* Row10  */ 0.0f,    0.0f,   0.0f,   -0.0002f,       0.0f,            0.0f,            0.0f,            0.0f,            0.0f,            0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row11  */ 0.0f,    0.0f,   0.0f,    0.0f,         -0.0002f,        0.0f,            0.0f,            0.0f,            0.0f,             0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row12  */ 0.0f,    0.0f,   0.0f,    0.0f,          0.0f,          -0.0002f,         0.0f,            0.0f,            0.0f,             0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row13  */ 0.0f,    0.0f,   0.0f,    0.0f,          0.0f,           0.0f,          -0.0002f,         0.0f,            0.0f,              0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row14  */ 0.0f,    0.0f,   0.0f,    0.0f,          0.0f,           0.0f,           0.0f,         -0.0002f,         0.0f,                0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row15  */ 0.0f,    0.0f,   0.0f,    0.0f,          0.0f,           0.0f,           0.0f,          0.0f,          -0.0002f,              0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f,
        /* Row16  */ 0.0f,    0.0f,   0.0f,    0.0f,          0.0f,           0.0f,           0.0f,          0.0f,           0.0f,                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f,
        /* Row17  */ 0.0f,    0.0f,   0.0f,    0.0f,          0.0f,           0.0f,           0.0f,          0.0f,           0.0f,                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f,
        /* Row18  */ 0.0f,    0.0f,   0.0f,    0.0f,          0.0f,           0.0f,           0.0f,          0.0f,           0.0f,                 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f
    };

  for (unsigned int i = 0; i<324; i++ ){
    if (fabs(truth_chain_mult_test[i] - chain_mult_test[i]) > 1e-6f) {
        snprintf(msg, sizeof(msg), 
                  "Matrix mismatch at index %u: expected %.9f, got %.9f", 
                  i, truth_chain_mult_test[i], chain_mult_test[i]);
        TEST_FAIL_MESSAGE(msg); // fails the test with the formatted message
    }
  }
  // Addition Test
    //Mostly just need to test multiplication and addition
    Matrix18f add_test = testSTM * test_P * testSTM.transpose() + test_Qd;

    std::array<float, 324> truth_add_test = {
        /* Row 1  */ 3.000200000024f, 0.0f, 0.0f, 0.0100000018f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row 2  */ 0.0f, 3.000200000024f, 0.0f, 0.0f,0.0100000018f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row 3  */ 0.0f, 0.0f, 3.000200000024f, 0.0f, 0.0f, 0.0100000018f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row 4  */ 0.0100000018f, 0.0f, 0.0f, 0.504091555235567f, -8e-7f, -4.04266e-05f, 0.0002465596f, 0.0202101f, -0.0005617064f, -0.00020002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row 5  */ 0.0f, 0.0100000018f, 0.0f, -8e-07f, 0.504090355235567f, -8.08532e-05f, -0.0202153f, 0.0002441596f, -2.133e-06f, 0.0f, -0.00020002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row 6  */ 0.0f, 0.0f, 0.0100000018f, -4.04266e-05f, -8.08532e-5, 0.500006180266667f, 0.0003976f, -0.0002048f, 5.6e-06f, 0.0f, 0.0f, -0.000200000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row 7  */ 0.0f, 0.0f, 0.0f, 0.0002465596f, -0.0202153f, 0.0003976f,  0.105028400000027f, -8e-06f, -9.60000000000002e-06f, 0.0f, 0.0f, 0.0f, -0.000200000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row 8  */ 0.0f, 0.0f, 0.0f, 0.0202101f, 0.0002441596f, -0.0002048f, -8.00000000000002e-06, 0.105024800000027f, -1.19999999999999e-05f, 0.0f, 0.0f, 0.0f, 0.0f, -0.000200000002f, 0.0f, 0.0f, 0.0f, 0.0f, 
        /* Row 9  */ 0.0f, 0.0f, 0.0f, -0.0005617064f, -2.133e-06f, 5.6e-06f, -9.60000000000007e-06f, -1.20000000000001e-05f, 0.105020400000027f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.000200000002f, 0.0f, 0.0f, 0.0f, 
        /* Row10  */ 0.0f, 0.0f, 0.0f, -0.00020002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.010002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row11  */ 0.0f, 0.0f, 0.0f, 0.0f, -0.00020002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.010002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row12  */ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.00020002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.010002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row13  */ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.000200000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0100000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row14  */ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.000200000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0100000002f, 0.0f, 0.0f, 0.0f, 0.0f,
        /* Row15  */ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.000200000002f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0100000002f, 0.0f, 0.0f, 0.0f,
        /* Row16  */ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01000002f, 0.0f, 0.0f,
        /* Row17  */ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01000002f, 0.0f,
        /* Row18  */ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01000002f
    };


  for (unsigned int i = 0; i<324; i++ ){
    if (fabs(truth_add_test[i] - add_test[i]) > 1e-6f) {
        snprintf(msg, sizeof(msg), 
                  "Matrix mismatch at index %u: expected %.9f, got %.9f", 
                  i, truth_add_test[i], add_test[i]);
        TEST_FAIL_MESSAGE(msg); // fails the test with the formatted message
    }
  }
}





// No setup() or loop()
// Use UNITY_BEGIN() in special main provided by PlatformIO
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_vector3f_functions);
    RUN_TEST(test_rotation_functions);
    RUN_TEST(test_quaternion_functions);
    RUN_TEST(test_matrix18f_functions);
    return UNITY_END();
}