#include "kinematics.h"

IMUData obtainIMUData(MPU6500_WE &mpu){

}

IMUData alignYawOffset_6500(IMUData &imuData, float yawOffset){

}

quaternion convertRawToQuaternion(IMUData &imuData){

}

eulerAngles convertQuaternionToEuler(quaternion q){

}

xyzFloat calculateAxisVector(axisVector axis, eulerAngles &euler){

}

float calculateJointAngle(axisVector axis, joints joint){
    
}
 
void MadgwickAHRSupdate(IMUData &imuData, float deltat, quaternion &q){
    
}