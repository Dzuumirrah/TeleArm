#include "kinematics.h"
#include "configurations.h"
#include <math.h>

// ============================================================================
//  Two-pose functional calibration (per IMU)
//
//  Pose 1: arm hanging straight down  -> gravity gives the limb axis k in sensor coords.
//  Pose 2: arm raised forward, straight, in the sagittal plane (limb horizontal)
//          -> the horizontal direction of the limb is the operator's FORWARD, in this
//             IMU's own world frame (north-referenced for the 9250, arbitrary for the 6500).
//  From forward + gravity: right = forward x up, back = -forward.
//    T : W -> P      rows [right; up; back]
//    M : columns k, u(=forward in pose), s(=right in pose) in sensor coordinates
//  Nothing about how the PCB is mounted has to be known, and the yaw offset between the
//  two IMUs falls out of the same procedure.
// ============================================================================
bool computeSegmentCalibration(const quaternion &g1, const quaternion &g2, SegmentCal &out) {
    const Mat3 r1 = rotationFromQuaternion(g1);
    const Mat3 r2 = rotationFromQuaternion(g2);
    const xyzFloat up(0.0f, 0.0f, 1.0f);

    // limb axis in sensor coords: world "down" seen from pose 1  (R1^T * (0,0,-1))
    const xyzFloat kS(-r1.m[2][0], -r1.m[2][1], -r1.m[2][2]);

    // limb direction in world during pose 2
    const xyzFloat k2 = mat3MulVec(r2, kS);
    if (fabsf(k2.z) > 0.5f) return false;                 // not within 30 deg of horizontal
    xyzFloat f(k2.x, k2.y, 0.0f);
    if (vnorm(f) < 0.8f) return false;
    f = vunit(f);                                         // forward (world)
    const xyzFloat right = vcross(f, up);                 // forward x up = right

    Mat3 t;
    t.m[0][0] = right.x; t.m[0][1] = right.y; t.m[0][2] = right.z;
    t.m[1][0] = 0.0f;    t.m[1][1] = 0.0f;    t.m[1][2] = 1.0f;
    t.m[2][0] = -f.x;    t.m[2][1] = -f.y;    t.m[2][2] = -f.z;

    const Mat3 r1t = mat3Transpose(r1);
    const xyzFloat uS = mat3MulVec(r1t, f);               // forward as seen in sensor coords
    const xyzFloat sS = mat3MulVec(r1t, right);           // right   as seen in sensor coords

    Mat3 m;
    m.m[0][0] = kS.x; m.m[0][1] = uS.x; m.m[0][2] = sS.x;
    m.m[1][0] = kS.y; m.m[1][1] = uS.y; m.m[1][2] = sS.y;
    m.m[2][0] = kS.z; m.m[2][1] = uS.z; m.m[2][2] = sS.z;

    out.T = t;
    out.M = m;
    out.valid = true;
    out.anchored = true;
    return true;
}

// After a reboot the 6500 starts with an arbitrary heading, the 9250 (with mag) does not.
// With the arm straight down both segment frames coincide, so the 9250 gives the target.
bool reanchorSegment6500(const quaternion &g9250, const SegmentCal &seg9250,
                         const quaternion &g6500, SegmentCal &seg6500) {
    if (!seg9250.valid || !seg9250.anchored || !seg6500.valid) return false;

    const Mat3 a9 = mat3Mul(seg9250.T, mat3Mul(rotationFromQuaternion(g9250), seg9250.M));
    // T6 * R6 * M6 = A9   ->   T6 = A9 * M6^T * R6^T
    Mat3 t = mat3Mul(a9, mat3Mul(mat3Transpose(seg6500.M), mat3Transpose(rotationFromQuaternion(g6500))));
    t = rotationFromQuaternion(quaternionFromRotation(t));   // re-orthonormalise

    // sanity: world up must still map to P up (i.e. the 6500 sees gravity as the 9250 does)
    if (t.m[1][2] < 0.985f) return false;                 // within ~10 deg
    seg6500.T = t;
    seg6500.anchored = true;
    return true;
}

// ============================================================================
//  optional continuous heading correction of the 6500
//  Elbow = hinge: the forearm axis k2 stays in the plane perpendicular to the hinge axis,
//  which is fixed in the upper arm (side axis s1):  e = k2 . s1 = 0.
//  A heading error dpsi about up changes e by  dpsi * d,  d = (Y x k2) . s1.
// ============================================================================
float hingeYawCorrection(const axisVector &up, const axisVector &fo) {
    const float e = vdot(fo.k, up.s);
    const float d = fo.k.z * up.s.x - fo.k.x * up.s.z;      // (Y x k2) . s1, Y = (0,1,0)
    if (fabsf(d) < HINGE_MIN_OBSERVABILITY) return 0.0f;
    float delta = -e / d * HINGE_YAW_GAIN;
    if (delta > HINGE_MAX_STEP_RAD) delta = HINGE_MAX_STEP_RAD;
    if (delta < -HINGE_MAX_STEP_RAD) delta = -HINGE_MAX_STEP_RAD;
    return delta;
}

// ============================================================================
//  gyro bias
// ============================================================================
bool computeGyroBias(int n, const double sum[3], const double sumsq[3], float maxStdDps,
                     xyzFloat &meanOut) {
    if (n < 20) return false;
    double mean[3];
    for (int i = 0; i < 3; i++) {
        mean[i] = sum[i] / n;
        double var = sumsq[i] / n - mean[i] * mean[i];
        if (var < 0.0) var = 0.0;
        if (sqrt(var) > maxStdDps) return false;        // the arm was moving
    }
    meanOut = xyzFloat((float)mean[0], (float)mean[1], (float)mean[2]);
    return true;
}

// ============================================================================
//  magnetometer: hard iron, soft iron (per-axis) and AK8963 axis mapping
// ============================================================================
static xyzFloat applyMagCal(const MagSample &s, const SensorCal &c, bool swap) {
    xyzFloat m((s.mx - c.magOffset.x) * c.magScale.x,
               (s.my - c.magOffset.y) * c.magScale.y,
               (s.mz - c.magOffset.z) * c.magScale.z);
    return swap ? xyzFloat(m.y, m.x, -m.z) : m;
}

// Fit  A x^2 + B y^2 + C z^2 + D x + E y + F z = 1  (x,y,z in units of 30 uT) by linear least squares.
// Returns centre o[] and semi-axes r[] in uT. false if the normal equations are singular.
static bool fitAxisAlignedEllipsoid(const MagSample *s, int n, float o[3], float r[3]) {
    const double SC = 1.0 / 30.0;
    double N[6][7];
    for (int i = 0; i < 6; i++) for (int j = 0; j < 7; j++) N[i][j] = 0.0;
    for (int i = 0; i < n; i++) {
        const double x = s[i].mx * SC, y = s[i].my * SC, z = s[i].mz * SC;
        const double row[6] = { x * x, y * y, z * z, x, y, z };
        for (int a = 0; a < 6; a++) { for (int b = 0; b < 6; b++) N[a][b] += row[a] * row[b]; N[a][6] += row[a]; }
    }
    for (int c = 0; c < 6; c++) {                       // Gauss-Jordan with partial pivoting
        int piv = c;
        for (int rr = c + 1; rr < 6; rr++) if (fabs(N[rr][c]) > fabs(N[piv][c])) piv = rr;
        if (fabs(N[piv][c]) < 1e-9) return false;
        if (piv != c) for (int j = 0; j < 7; j++) { double t = N[c][j]; N[c][j] = N[piv][j]; N[piv][j] = t; }
        for (int rr = 0; rr < 6; rr++) {
            if (rr == c) continue;
            const double f = N[rr][c] / N[c][c];
            for (int j = c; j < 7; j++) N[rr][j] -= f * N[c][j];
        }
    }
    double p[6];
    for (int i = 0; i < 6; i++) p[i] = N[i][6] / N[i][i];
    if (!(p[0] > 0.0 && p[1] > 0.0 && p[2] > 0.0)) return false;
    const double ox = -p[3] / (2.0 * p[0]), oy = -p[4] / (2.0 * p[1]), oz = -p[5] / (2.0 * p[2]);
    const double G = 1.0 + p[0] * ox * ox + p[1] * oy * oy + p[2] * oz * oz;
    if (G <= 0.0) return false;
    o[0] = (float)(ox / SC); o[1] = (float)(oy / SC); o[2] = (float)(oz / SC);
    r[0] = (float)(sqrt(G / p[0]) / SC); r[1] = (float)(sqrt(G / p[1]) / SC); r[2] = (float)(sqrt(G / p[2]) / SC);
    return true;
}

bool computeMagCalibration(const MagSample *s, int n, SensorCal &out) {
    if (n < 50) return false;
    float mn[3] = {  1e9f,  1e9f,  1e9f };
    float mx[3] = { -1e9f, -1e9f, -1e9f };
    for (int i = 0; i < n; i++) {
        const float v[3] = { s[i].mx, s[i].my, s[i].mz };
        for (int j = 0; j < 3; j++) { if (v[j] < mn[j]) mn[j] = v[j]; if (v[j] > mx[j]) mx[j] = v[j]; }
    }
    float rad[3];
    for (int j = 0; j < 3; j++) {
        rad[j] = 0.5f * (mx[j] - mn[j]);
        if (rad[j] < 10.0f) return false;               // sensor was not rotated through this axis
    }
    // 1) min/max estimate: exact only if every direction was visited
    xyzFloat off(0.5f * (mx[0] + mn[0]), 0.5f * (mx[1] + mn[1]), 0.5f * (mx[2] + mn[2]));
    float r[3] = { rad[0], rad[1], rad[2] };
    // 2) least-squares fit of an axis-aligned ellipsoid to ALL samples: tolerant of partial coverage
    {
        float o2[3], r2[3];
        if (fitAxisAlignedEllipsoid(s, n, o2, r2)) {
            bool sane = true;
            for (int j = 0; j < 3; j++) {
                if (!(r2[j] > 10.0f && r2[j] < 200.0f) || fabsf(o2[j]) > 400.0f) sane = false;
            }
            if (sane) { off = xyzFloat(o2[0], o2[1], o2[2]); r[0] = r2[0]; r[1] = r2[1]; r[2] = r2[2]; }
        }
    }
    const float avg = (r[0] + r[1] + r[2]) / 3.0f;
    out.magOffset = off;
    out.magScale  = xyzFloat(avg / r[0], avg / r[1], avg / r[2]);

    // Which axis mapping gives a constant angle between gravity and the field?
    double stdv[2];
    for (int h = 0; h < 2; h++) {
        double mean = 0.0, m2 = 0.0; int cnt = 0;
        for (int i = 0; i < n; i++) {
            const xyzFloat a(s[i].ax, s[i].ay, s[i].az);
            const xyzFloat m = applyMagCal(s[i], out, h == 1);
            const float den = vnorm(a) * vnorm(m);
            if (den < 1e-6f) continue;
            const double c = vdot(a, m) / den;
            cnt++;
            const double dlt = c - mean;
            mean += dlt / cnt;
            m2 += dlt * (c - mean);
        }
        stdv[h] = (cnt > 1) ? sqrt(m2 / (cnt - 1)) : 1e9;
    }
    const int best = (stdv[1] < stdv[0]) ? 1 : 0;
    if (stdv[best] > 0.30) return false;                // data not coherent: redo the rotation
    // the calibrated field magnitude must be nearly constant
    {
        double sum = 0.0, sum2 = 0.0; int cnt = 0;
        for (int i = 0; i < n; i++) { const double v = vnorm(applyMagCal(s[i], out, best == 1)); sum += v; sum2 += v * v; cnt++; }
        const double mean = sum / cnt, sd = sqrt(fmax(0.0, sum2 / cnt - mean * mean));
        if (mean < 5.0 || sd / mean > 0.12) return false;
    }
    out.magSwapAxes = (best == 1);
    out.magValid = true;
    return true;
}
