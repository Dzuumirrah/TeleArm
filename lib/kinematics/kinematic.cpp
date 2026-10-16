#include "kinematics.h"
#include "configurations.h"
#include <math.h>

static const float RAD2DEGF = 57.29577951308232f;

// ============================================================================
//  math helpers
// ============================================================================
float vdot(const xyzFloat &a, const xyzFloat &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

xyzFloat vcross(const xyzFloat &a, const xyzFloat &b) {
    return xyzFloat(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

float vnorm(const xyzFloat &a) { return sqrtf(vdot(a, a)); }

xyzFloat vunit(const xyzFloat &a) {
    float n = vnorm(a);
    if (n < 1e-9f) return xyzFloat(0.0f, 0.0f, 0.0f);
    return a / n;
}

Mat3 mat3Identity() {
    Mat3 r;
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) r.m[i][j] = (i == j) ? 1.0f : 0.0f;
    return r;
}

Mat3 mat3Mul(const Mat3 &a, const Mat3 &b) {
    Mat3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
    return r;
}

Mat3 mat3Transpose(const Mat3 &a) {
    Mat3 r;
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) r.m[i][j] = a.m[j][i];
    return r;
}

xyzFloat mat3MulVec(const Mat3 &a, const xyzFloat &v) {
    return xyzFloat(a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
                    a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
                    a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z);
}

quaternion qMul(const quaternion &a, const quaternion &b) {
    quaternion r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return r;
}

quaternion qConj(const quaternion &a) {
    quaternion r = { a.w, -a.x, -a.y, -a.z };
    return r;
}

quaternion qNormalize(const quaternion &a) {
    float n = sqrtf(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
    if (n < 1e-12f) { quaternion id = { 1.0f, 0.0f, 0.0f, 0.0f }; return id; }
    quaternion r = { a.w / n, a.x / n, a.y / n, a.z / n };
    return r;
}

Mat3 rotationFromQuaternion(const quaternion &q) {
    Mat3 r;
    const float w = q.w, x = q.x, y = q.y, z = q.z;
    r.m[0][0] = 1.0f - 2.0f * (y * y + z * z);
    r.m[0][1] = 2.0f * (x * y - w * z);
    r.m[0][2] = 2.0f * (x * z + w * y);
    r.m[1][0] = 2.0f * (x * y + w * z);
    r.m[1][1] = 1.0f - 2.0f * (x * x + z * z);
    r.m[1][2] = 2.0f * (y * z - w * x);
    r.m[2][0] = 2.0f * (x * z - w * y);
    r.m[2][1] = 2.0f * (y * z + w * x);
    r.m[2][2] = 1.0f - 2.0f * (x * x + y * y);
    return r;
}

quaternion quaternionFromRotation(const Mat3 &r) {
    quaternion q;
    const float tr = r.m[0][0] + r.m[1][1] + r.m[2][2];
    if (tr > 0.0f) {
        float s = sqrtf(tr + 1.0f) * 2.0f;
        q.w = 0.25f * s;
        q.x = (r.m[2][1] - r.m[1][2]) / s;
        q.y = (r.m[0][2] - r.m[2][0]) / s;
        q.z = (r.m[1][0] - r.m[0][1]) / s;
    } else if (r.m[0][0] > r.m[1][1] && r.m[0][0] > r.m[2][2]) {
        float s = sqrtf(1.0f + r.m[0][0] - r.m[1][1] - r.m[2][2]) * 2.0f;
        q.w = (r.m[2][1] - r.m[1][2]) / s;
        q.x = 0.25f * s;
        q.y = (r.m[0][1] + r.m[1][0]) / s;
        q.z = (r.m[0][2] + r.m[2][0]) / s;
    } else if (r.m[1][1] > r.m[2][2]) {
        float s = sqrtf(1.0f + r.m[1][1] - r.m[0][0] - r.m[2][2]) * 2.0f;
        q.w = (r.m[0][2] - r.m[2][0]) / s;
        q.x = (r.m[0][1] + r.m[1][0]) / s;
        q.y = 0.25f * s;
        q.z = (r.m[1][2] + r.m[2][1]) / s;
    } else {
        float s = sqrtf(1.0f + r.m[2][2] - r.m[0][0] - r.m[1][1]) * 2.0f;
        q.w = (r.m[1][0] - r.m[0][1]) / s;
        q.x = (r.m[0][2] + r.m[2][0]) / s;
        q.y = (r.m[1][2] + r.m[2][1]) / s;
        q.z = 0.25f * s;
    }
    return qNormalize(q);
}

// ============================================================================
//  sensor conditioning
// ============================================================================
IMUData applySensorCalibration(const IMUData &raw, const SensorCal &cal) {
    IMUData out = raw;
    out.gyr = raw.gyr - cal.gyroBias;

    const float mn = vnorm(raw.mag);
    if (mn > 0.0f && cal.magValid) {
        xyzFloat m((raw.mag.x - cal.magOffset.x) * cal.magScale.x,
                   (raw.mag.y - cal.magOffset.y) * cal.magScale.y,
                   (raw.mag.z - cal.magOffset.z) * cal.magScale.z);
        // AK8963 axes -> MPU accel/gyro axes (datasheet: X<->Y swapped, Z inverted).
        // The swap is verified/decided during the magnetometer calibration.
        out.mag = cal.magSwapAxes ? xyzFloat(m.y, m.x, -m.z) : m;
    } else {
        out.mag = xyzFloat(0.0f, 0.0f, 0.0f);   // uncalibrated mag is never used
    }
    return out;
}

quaternion alignYawOffset_6500(const quaternion &q, float yawOffsetRad) {
    quaternion qz = { cosf(0.5f * yawOffsetRad), 0.0f, 0.0f, sinf(0.5f * yawOffsetRad) };
    return qNormalize(qMul(qz, q));      // rotation about WORLD up, applied after q
}

// ============================================================================
//  orientation
// ============================================================================
quaternion convertRawToQuaternion(const IMUData &d) {
    xyzFloat up = vunit(d.acc);                         // world up, in body coordinates
    if (vnorm(up) < 0.5f) up = xyzFloat(0.0f, 0.0f, 1.0f);

    xyzFloat north(0.0f, 0.0f, 0.0f);
    if (vnorm(d.mag) > 0.0f) {
        north = d.mag - up * vdot(d.mag, up);           // horizontal part of the field
    }
    if (vnorm(north) < 1e-3f) {                         // no usable mag: any horizontal direction
        xyzFloat bx(1.0f, 0.0f, 0.0f);
        north = bx - up * vdot(bx, up);
        if (vnorm(north) < 1e-3f) {
            xyzFloat by(0.0f, 1.0f, 0.0f);
            north = by - up * vdot(by, up);
        }
    }
    north = vunit(north);
    xyzFloat west = vcross(up, north);                  // z x x = y  (north x west = up)

    Mat3 r;                                             // rows = world axes in body coords
    r.m[0][0] = north.x; r.m[0][1] = north.y; r.m[0][2] = north.z;
    r.m[1][0] = west.x;  r.m[1][1] = west.y;  r.m[1][2] = west.z;
    r.m[2][0] = up.x;    r.m[2][1] = up.y;    r.m[2][2] = up.z;
    return quaternionFromRotation(r);
}

// Kurpath Eq. (1)
eulerAngles convertQuaternionToEuler(const quaternion &q) {
    eulerAngles e;
    e.roll = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y));
    float sp = 2.0f * (q.w * q.y - q.z * q.x);
    if (sp > 1.0f) sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;
    e.pitch = asinf(sp);
    e.yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    return e;
}

// ============================================================================
//  Kurpath ROV method
// ============================================================================
quaternion segmentOrientation(const quaternion &g, const SegmentCal &cal) {
    Mat3 a = mat3Mul(cal.T, mat3Mul(rotationFromQuaternion(g), cal.M));
    return quaternionFromRotation(a);
}

// Columns of the segment rotation matrix (segment local x=k, y=u, z=s) are the axes in P.
axisVector calculateAxisVector(const quaternion &segmentQ) {
    Mat3 r = rotationFromQuaternion(segmentQ);
    axisVector a;
    a.k = xyzFloat(r.m[0][0], r.m[1][0], r.m[2][0]);
    a.u = xyzFloat(r.m[0][1], r.m[1][1], r.m[2][1]);
    a.s = xyzFloat(r.m[0][2], r.m[1][2], r.m[2][2]);
    return a;
}

joints calculateJointAngles(const axisVector &up, const axisVector &fo) {
    joints j;
    const xyzFloat &k1 = up.k;
    // Eq. (5): theta_a = -atan2(k1y,k1z)*180/pi - 90. Same value, rewritten as
    // atan2(-k1z,-k1y) so it does not jump by 360 deg when the arm passes horizontal.
    j.joint_a = atan2f(-k1.z, -k1.y) * RAD2DEGF;
    // Eq. (6)
    j.joint_b = atan2f(k1.x, sqrtf(k1.y * k1.y + k1.z * k1.z)) * RAD2DEGF;
    // Eq. (10)
    float c = vdot(k1, fo.k);
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    j.joint_d = acosf(c) * RAD2DEGF;
    return j;
}
