#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include <MPU9250_WE.h>   // MPU6500_WE, MPU9250_WE, xyzFloat

// =============================================================================
//  Frames and conventions (read this first)
//
//  Quaternion q = (w,x,y,z), Hamilton, rotates a vector from the BODY frame to the
//  WORLD frame:  v_world = q * v_body * conj(q).
//
//  Madgwick world frame ("W"): x = north (or arbitrary heading without magnetometer),
//  y = west, z = up.
//
//  Kurpath frame ("P", the global frame of the paper, Fig. 7/8):
//      X = operator's right,  Y = up,  Z = operator's back        (right-handed)
//  Segment frame axes (paper Fig. 7):  k = front (along the limb, shoulder->elbow
//  / elbow->wrist), u = up axis, s = side axis, with k x u = s.
//  Reference "N-pose" (arm hanging straight down, forearm straight):
//      k = (0,-1,0)   u = (0,0,-1)   s = (1,0,0)    expressed in P.
//
//  Units: acc [g], gyr [deg/s], mag [uT], angles in structs below [rad] unless stated,
//  lengths [m], joint angles [deg].
// =============================================================================

struct IMUData{
    uint32_t time;     // micros() at read time (was float: loses precision)
    xyzFloat acc;      // [g]
    xyzFloat gyr;      // [deg/s]
    xyzFloat mag;      // [uT]; (0,0,0) for the MPU6500
};

struct quaternion {
    float w;
    float x;
    float y;
    float z;
};

struct eulerAngles {      // [rad]
    float roll;
    float pitch;
    float yaw;
};

struct axisVector {       // three unit vectors of one segment, expressed in P
    xyzFloat u;
    xyzFloat k;
    xyzFloat s;
};

struct joints{            // Kurpath human joint angles [deg]
    float joint_a;        // shoulder flexion(+)/extension
    float joint_b;        // shoulder abduction(+)/adduction
    float joint_d;        // elbow flexion (0 = straight)
};

struct Mat3 { float m[3][3]; };

// ---- small math helpers -----------------------------------------------------
float    vdot(const xyzFloat &a, const xyzFloat &b);
xyzFloat vcross(const xyzFloat &a, const xyzFloat &b);
float    vnorm(const xyzFloat &a);
xyzFloat vunit(const xyzFloat &a);
Mat3     mat3Identity();
Mat3     mat3Mul(const Mat3 &a, const Mat3 &b);
Mat3     mat3Transpose(const Mat3 &a);
xyzFloat mat3MulVec(const Mat3 &a, const xyzFloat &v);
quaternion qMul(const quaternion &a, const quaternion &b);
quaternion qConj(const quaternion &a);
quaternion qNormalize(const quaternion &a);
Mat3       rotationFromQuaternion(const quaternion &q);   // body -> world
quaternion quaternionFromRotation(const Mat3 &r);

// ---- sensor calibration (persistent) ----------------------------------------
struct SensorCal {
    xyzFloat gyroBias;     // [deg/s], subtracted from gyr
    xyzFloat magOffset;    // [uT] hard iron, AK8963 axes
    xyzFloat magScale;     // soft iron (per-axis), AK8963 axes
    bool     magSwapAxes;  // true: MPU frame = (my, mx, -mz) of the AK8963 frame
    bool     magValid;
};

// Maps one IMU's world frame/sensor frame to the P frame and the segment axes.
//   segment axes in P (columns k,u,s) = T * R(G) * M      (G = Madgwick quaternion)
struct SegmentCal {
    Mat3 T;        // W -> P (heading anchored to the operator's forward direction)
    Mat3 M;        // columns = segment axes k,u,s expressed in SENSOR coordinates
    bool valid;    // M and T usable
    bool anchored; // T usable in the current power cycle (6500 heading is lost at reboot)
};

struct CalibrationData {
    uint32_t   magic;
    uint16_t   version;
    uint8_t    calUsedMag;     // heading frame of the 9250 was north-referenced when captured
    uint8_t    reserved;
    SensorCal  s9250;
    SensorCal  s6500;
    SegmentCal seg9250;
    SegmentCal seg6500;
    uint32_t   checksum;       // over all bytes before this field
};

// ---- sensor data ------------------------------------------------------------
// Read raw data from the IMU. (implemented in components.cpp, needs the hardware)
// The MPU6500 version returns mag = 0. The MPU9250 version also returns the raw
// AK8963 vector in AK8963 axes.
IMUData obtainIMUData(MPU6500_WE &mpu);
IMUData obtainIMUData(MPU9250_WE &mpu);

// Apply gyro bias and magnetometer hard/soft iron + axis alignment.
IMUData applySensorCalibration(const IMUData &raw, const SensorCal &cal);

// Rotate the heading (world yaw) of a Madgwick quaternion by yawOffset [rad]
// (about world up). The 6500 has no magnetometer, so its heading must be pulled
// onto the 9250's: see computeSegmentCalibration() / hingeYawCorrection().
quaternion alignYawOffset_6500(const quaternion &q, float yawOffsetRad);

// ---- orientation ------------------------------------------------------------
// Initial orientation from one still sample (accelerometer, + magnetometer when
// |mag| > 0). Used to seed Madgwick so it does not start from identity.
quaternion convertRawToQuaternion(const IMUData &imuData);

// Kurpath Eq. (1): quaternion -> roll, pitch, yaw [rad]
eulerAngles convertQuaternionToEuler(const quaternion &q);

// Madgwick gradient-descent filter. 9-axis when |mag| > 0, otherwise 6-axis.
// gyr in deg/s (converted inside), deltat in seconds, beta = filter gain.
void MadgwickAHRSupdate(const IMUData &imuData, float deltat, quaternion &q, float beta);
// exposed for unit testing: gradient of the Madgwick objective (not normalised)
void madgwickGradient(const quaternion &q, const xyzFloat &accUnit,
                      const xyzFloat *magUnit, float bx, float bz, float grad[4]);

// ---- Kurpath ROV method -------------------------------------------------------
// Segment orientation (segment frame k,u,s -> P) from a Madgwick quaternion.
quaternion segmentOrientation(const quaternion &g, const SegmentCal &cal);

// Kurpath Eq. (2)-(4): the u, k, s axis vectors of a segment in P.
// Computed from the rotation matrix of the segment quaternion, which is what Eq. (2)-(4)
// are (rows of the rotation matrix) but without the Euler singularity.
axisVector calculateAxisVector(const quaternion &segmentQ);

// Kurpath Eq. (5),(6),(10): joint a (shoulder flexion), b (abduction), d (elbow flexion).
joints calculateJointAngles(const axisVector &upperArm, const axisVector &forearm);

// ---- Cartesian mapping --------------------------------------------------------
// Fingertip position in P, origin at the shoulder, straight wrist assumed:
//    tip = L_upper * k1 + L_elbowToTip * k2
xyzFloat humanFingertip(const axisVector &upperArm, const axisVector &forearm);
// Fingertip (P) -> robot pen target (robot frame: x out, y left, z up)
xyzFloat mapHumanToRobot(const xyzFloat &tipP);
// Analytic IK of the 3-DOF robot, elbow-up. Returns false if the target was out of reach
// (the arm is then stretched/folded towards it). Angles [deg]:
//   [0] base yaw about z, [1] shoulder: link1 angle from vertical-up toward +radial,
//   [2] elbow: link2 angle relative to link1 (0 = straight, + = bends outward/down)
bool robotInverseKinematics(const xyzFloat &target, float jointDeg[3]);
xyzFloat robotForwardKinematics(const float jointDeg[3]);
// Clamp to joint limits and slew-limit against the previous command. Returns flag bits:
// bit0 = a limit clamped the angle, bit1 = slew limit active.
uint8_t limitJointAngles(float jointDeg[3], const float prevDeg[3], float dt);

// ---- calibration maths --------------------------------------------------------
// Two-pose functional calibration for ONE IMU.
//   g1: Madgwick quaternion averaged in pose 1 (arm hanging straight down)
//   g2: same in pose 2 (arm raised forward ~90 deg, straight, in the sagittal plane)
// Gives mounting (M) and heading anchoring (T) without knowing how the PCB sits.
bool computeSegmentCalibration(const quaternion &g1, const quaternion &g2, SegmentCal &out);
// Quick re-anchor of the 6500 heading in pose 1 (arm straight down), using the already
// anchored 9250 and the stored mounting M of the 6500.
bool reanchorSegment6500(const quaternion &g9250, const SegmentCal &seg9250,
                         const quaternion &g6500, SegmentCal &seg6500);

// Optional: yaw correction [rad, about up] for the 6500 from the elbow hinge constraint
// (forearm axis k2 must stay perpendicular to the upper-arm side axis s1). 0 when the
// heading is not observable (forearm near vertical).
float hingeYawCorrection(const axisVector &upperArm, const axisVector &forearm);

// Gyro bias from a still window (accumulated sums). Returns false if it was not still.
bool computeGyroBias(int n, const double sum[3], const double sumsq[3], float maxStdDps,
                     xyzFloat &meanOut);

struct MagSample { float ax, ay, az; float mx, my, mz; };   // acc [g], raw AK8963 mag [uT]
// Hard/soft-iron calibration and detection of the AK8963 axis mapping from rotated samples.
bool computeMagCalibration(const MagSample *s, int n, SensorCal &out);

#endif
