#pragma once
#include "kinematics.h"
#include "configurations.h"
#include <stdio.h>
#include <math.h>
#include <random>
#include <vector>

static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...) do { g_checks++; if (!(cond)) { g_fail++; printf("  FAIL %s:%d  ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define D2R(x) ((x) * 0.017453292519943295f)
#define R2D(x) ((x) * 57.29577951308232f)

static std::mt19937 rng(12345);
inline float urand(float a, float b) { return std::uniform_real_distribution<float>(a, b)(rng); }
inline float nrand(float s) { return std::normal_distribution<float>(0.0f, s)(rng); }

inline Mat3 Rx(float a) { Mat3 r = mat3Identity(); r.m[1][1] = cosf(a); r.m[1][2] = -sinf(a); r.m[2][1] = sinf(a); r.m[2][2] = cosf(a); return r; }
inline Mat3 Ry(float a) { Mat3 r = mat3Identity(); r.m[0][0] = cosf(a); r.m[0][2] = sinf(a); r.m[2][0] = -sinf(a); r.m[2][2] = cosf(a); return r; }
inline Mat3 Rz(float a) { Mat3 r = mat3Identity(); r.m[0][0] = cosf(a); r.m[0][1] = -sinf(a); r.m[1][0] = sinf(a); r.m[1][1] = cosf(a); return r; }

inline Mat3 randomRotation() {
    quaternion q = { nrand(1), nrand(1), nrand(1), nrand(1) };
    return rotationFromQuaternion(qNormalize(q));
}
inline float matErr(const Mat3 &a, const Mat3 &b) {
    float e = 0; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) e = fmaxf(e, fabsf(a.m[i][j] - b.m[i][j]));
    return e;
}
// angle [deg] between two rotations (atan2 form: accurate for tiny angles in float)
inline float rotAngleDeg(const Mat3 &a, const Mat3 &b) {
    Mat3 d = mat3Mul(a, mat3Transpose(b));
    float sx = d.m[2][1] - d.m[1][2], sy = d.m[0][2] - d.m[2][0], sz = d.m[1][0] - d.m[0][1];
    float s = 0.5f * sqrtf(sx * sx + sy * sy + sz * sz);
    float c = 0.5f * (d.m[0][0] + d.m[1][1] + d.m[2][2] - 1.0f);
    return R2D(atan2f(s, c));
}
inline float angDeg(const xyzFloat &a, const xyzFloat &b) {
    return R2D(atan2f(vnorm(vcross(a, b)), vdot(a, b)));
}

// ---- truth arm model (P frame) ----------------------------------------------
inline Mat3 poseN() {            // columns k=(0,-1,0) u=(0,0,-1) s=(1,0,0)
    Mat3 a; float v[3][3] = { {0, 0, 1}, {-1, 0, 0}, {0, -1, 0} };
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) a.m[i][j] = v[i][j];
    return a;
}
// upper arm: flexion a, abduction b, humeral twist about its own axis [rad]
inline Mat3 upperTruth(float a, float b, float twist) { return mat3Mul(mat3Mul(Rx(a), Rz(b)), mat3Mul(poseN(), Rx(twist))); }
// forearm: elbow flexion d about the upper-arm side axis, pronation about its own axis
inline Mat3 foreTruth(const Mat3 &A1, float d, float pron) { return mat3Mul(A1, mat3Mul(Rz(d), Rx(pron))); }

// sensor -> world quaternion that makes  T R(G) M = A
inline quaternion gFromA(const Mat3 &A, const Mat3 &T, const Mat3 &M) {
    return quaternionFromRotation(mat3Mul(mat3Transpose(T), mat3Mul(A, mat3Transpose(M))));
}
// a W->P matrix for a W frame (z up) whose forward heading is given: rows [right; up; back]
inline Mat3 trueT(float forwardHeadingRad) {
    xyzFloat f(cosf(forwardHeadingRad), sinf(forwardHeadingRad), 0), up(0, 0, 1);
    xyzFloat r = vcross(f, up);
    Mat3 t; t.m[0][0] = r.x; t.m[0][1] = r.y; t.m[0][2] = r.z; t.m[1][0] = 0; t.m[1][1] = 0; t.m[1][2] = 1;
    t.m[2][0] = -f.x; t.m[2][1] = -f.y; t.m[2][2] = -f.z; return t;
}
