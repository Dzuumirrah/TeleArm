#pragma once
// Simulated hardware for the host tests: replaces lib/connections/components.cpp.
// The test sets the TRUE arm orientation (Kurpath P frame) before every sessionStep(); the sim turns it
// into raw accelerometer / gyro / magnetometer readings of the two IMUs, with mounting, noise, bias,
// hard/soft iron, AK8963 axis swap and the lever-arm accelerations of a moving limb.
#include "kinematics.h"

struct SimWorld {
    // truth: segment axes (columns k,u,s) in P
    Mat3 Aup, Afore;
    float heading;                 // operator forward heading in the compass frame [rad]
    Mat3 M9, M6;                   // IMU mounting (columns k,u,s in sensor coordinates)
    float gyroBias9[3], gyroBias6[3];   // [deg/s], sensor axes
    float gyroNoise, accNoise, magNoise;
    xyzFloat magEarth;             // compass frame: x north, y west, z up [uT]
    xyzFloat magDisturb;           // added to magEarth (compass frame) -> interference
    xyzFloat hardIron, softIron;
    bool akSwap;
    bool dynamicAccel;             // lever-arm accelerations on
    float lever1, lever2;          // IMU distance along the segment from the joint [m]
    bool mpuInitOk, magAvailable;
    int failReads;                 // next N readIMUs() fail (-1: always)
    int initCalls, benchCalls, readCalls;
    void reset();
};
extern SimWorld sim;
void simStartMotion(unsigned long nowUs);   // forget the previous-sample history (new sensor continuity)
