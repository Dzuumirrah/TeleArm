#ifndef KINEMATICS_H
#define KINEMATICS_H

struct IMUData{
    float time;
    xyzFloat acc;
    xyzFloat gyr;
    xyzFloat mag;
};

struct quaternion {
    float w;
    float x;
    float y;
    float z;
};

struct eulerAngles {
    float roll;
    float pitch;
    float yaw;
};

struct axisVector {
    float u;
    float k;
    float s;
};

struct joints{
    byte joint_a;
    byte joint_b;
    byte joint_d;
};

// Receive Raw IMU data from MPU6500 and MPU9250
// IMUData will separated into two categories: MPU6500 and MPU9250
IMUData obtainIMUData(MPU6500_WE &mpu);

// calibrate the yaw offset of MPU6500 according to MPU9250 to align 
// the north reference  
IMUData alignYawOffset_6500(IMUData &imuData, float yawOffset);

// Convert the raw IMU data to quaternion representation
// The output of quaternion structured as q = [w, x, y, z]
quaternion convertRawToQuaternion(IMUData &imuData);

// Convert the quaternion to euler angles representation
// The output of eulerAngles structured as [roll, pitch, yaw]
eulerAngles convertQuaternionToEuler(quaternion q);

// Calculate the axis vector from euler angles based on Kurpath method.
// The output of axisVector structured as [u, k, s]
xyzFloat calculateAxisVector(axisVector axis, eulerAngles &euler);

// Calculate the joint angles based on axis vector and joint configuration 
// on Kurpath method. The joint sused only [joint_a, joint_b, joint_d]    
float calculateJointAngle(axisVector axis, joints joint);

// 
void MadgwickAHRSupdate(IMUData &imuData, float deltat, quaternion &q);


#endif