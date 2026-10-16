#include "testutil.h"

static void test_quat_basics() {
    printf("[quaternion / matrix / euler]\n");
    for (int i = 0; i < 200; i++) {
        Mat3 r = randomRotation();
        Mat3 r2 = rotationFromQuaternion(quaternionFromRotation(r));
        CHECK(matErr(r, r2) < 2e-6f, "matrix->quat->matrix err %g", matErr(r, r2));
    }
    quaternion qz = { cosf(D2R(20) / 2), 0, 0, sinf(D2R(20) / 2) };
    quaternion qy = { cosf(D2R(-15) / 2), 0, sinf(D2R(-15) / 2), 0 };
    quaternion qx = { cosf(D2R(30) / 2), sinf(D2R(30) / 2), 0, 0 };
    quaternion q = qNormalize(qMul(qMul(qz, qy), qx));
    eulerAngles e = convertQuaternionToEuler(q);
    CHECK(fabsf(R2D(e.yaw) - 20) < 1e-3f && fabsf(R2D(e.pitch) + 15) < 1e-3f && fabsf(R2D(e.roll) - 30) < 1e-3f,
          "euler %g %g %g", R2D(e.roll), R2D(e.pitch), R2D(e.yaw));
}

static void test_madgwick_gradient() {
    printf("[madgwick gradient vs finite difference]\n");
    for (int t = 0; t < 50; t++) {
        quaternion q = qNormalize((quaternion){ nrand(1), nrand(1), nrand(1), nrand(1) });
        xyzFloat a = vunit(xyzFloat(nrand(1), nrand(1), nrand(1)));
        xyzFloat m = vunit(xyzFloat(nrand(1), nrand(1), nrand(1)));
        float bx = fabsf(nrand(1)) + 0.1f, bz = nrand(1);
        float g[4]; madgwickGradient(q, a, &m, bx, bz, g);
        auto obj = [&](const quaternion &p) {
            float q0 = p.w, q1 = p.x, q2 = p.y, q3 = p.z;
            float f[6] = {
                2 * (q1 * q3 - q0 * q2) - a.x, 2 * (q0 * q1 + q2 * q3) - a.y, 2 * (0.5f - q1 * q1 - q2 * q2) - a.z,
                2 * bx * (0.5f - q2 * q2 - q3 * q3) + 2 * bz * (q1 * q3 - q0 * q2) - m.x,
                2 * bx * (q1 * q2 - q0 * q3) + 2 * bz * (q0 * q1 + q2 * q3) - m.y,
                2 * bx * (q0 * q2 + q1 * q3) + 2 * bz * (0.5f - q1 * q1 - q2 * q2) - m.z };
            double s = 0; for (int i = 0; i < 6; i++) s += f[i] * f[i]; return 0.5 * s; };
        for (int c = 0; c < 4; c++) {
            quaternion p1 = q, p2 = q; float h = 1e-3f;
            float *a1 = &p1.w + c, *a2 = &p2.w + c; *a1 += h; *a2 -= h;
            double num = (obj(p1) - obj(p2)) / (2 * h);
            CHECK(fabs(num - g[c]) < 2e-3 * (1 + fabs(num)), "grad[%d] analytic %g numeric %g", c, g[c], num);
        }
    }
}

static IMUData synth(const quaternion &q, const xyzFloat &magEarth, bool withMag) {
    Mat3 rt = mat3Transpose(rotationFromQuaternion(q));
    IMUData d; d.time = 0; d.gyr = xyzFloat(0, 0, 0);
    d.acc = mat3MulVec(rt, xyzFloat(0, 0, 1));
    d.mag = withMag ? mat3MulVec(rt, magEarth) : xyzFloat(0, 0, 0);
    return d;
}

static void test_seed_and_madgwick() {
    printf("[convertRawToQuaternion + Madgwick convergence]\n");
    const xyzFloat mE(28.0f, 0.0f, 12.0f);   // NWU field, uT
    for (int t = 0; t < 30; t++) {
        quaternion qt = qNormalize((quaternion){ nrand(1), nrand(1), nrand(1), nrand(1) });
        IMUData d = synth(qt, mE, true);
        quaternion qs = convertRawToQuaternion(d);
        CHECK(rotAngleDeg(rotationFromQuaternion(qs), rotationFromQuaternion(qt)) < 0.01f, "9-axis seed err");
        IMUData d6 = synth(qt, mE, false);
        quaternion q6 = convertRawToQuaternion(d6);
        xyzFloat up1 = mat3MulVec(rotationFromQuaternion(qt), mat3MulVec(mat3Transpose(rotationFromQuaternion(q6)), xyzFloat(0, 0, 1)));
        CHECK(angDeg(up1, xyzFloat(0, 0, 1)) < 0.01f, "6-axis seed gravity err");
    }
    for (int t = 0; t < 10; t++) {
        quaternion qt = qNormalize((quaternion){ nrand(1), nrand(1), nrand(1), nrand(1) });
        IMUData d = synth(qt, mE, true);
        quaternion q = { 1, 0, 0, 0 };
        for (int i = 0; i < 4000; i++) MadgwickAHRSupdate(d, 0.005f, q, 0.5f);
        float e9 = rotAngleDeg(rotationFromQuaternion(q), rotationFromQuaternion(qt));
        CHECK(e9 < 0.5f, "9-axis converged error %g deg", e9);
        IMUData d6 = synth(qt, mE, false);
        quaternion q6 = { 1, 0, 0, 0 };
        for (int i = 0; i < 4000; i++) MadgwickAHRSupdate(d6, 0.005f, q6, 0.5f);
        xyzFloat upT = mat3MulVec(rotationFromQuaternion(q6), mat3MulVec(mat3Transpose(rotationFromQuaternion(qt)), xyzFloat(0, 0, 1)));
        CHECK(angDeg(upT, xyzFloat(0, 0, 1)) < 0.5f, "6-axis tilt error");
    }
    quaternion q = { 1, 0, 0, 0 };
    IMUData g; g.time = 0; g.acc = xyzFloat(0, 0, 0); g.mag = xyzFloat(0, 0, 0); g.gyr = xyzFloat(0, 0, 90);
    for (int i = 0; i < 200; i++) MadgwickAHRSupdate(g, 0.005f, q, 0.0f);
    eulerAngles e = convertQuaternionToEuler(q);
    CHECK(fabsf(R2D(e.yaw) - 90.0f) < 0.1f, "gyro integration yaw %g", R2D(e.yaw));
}

static void test_joint_angles() {
    printf("[Kurpath axis vectors + joint angles a,b,d]\n");
    float maxErr = 0;
    for (int t = 0; t < 2000; t++) {
        float a = urand(-60, 175), b = urand(-60, 80), d = urand(0, 150), tw = urand(-80, 80), pr = urand(-80, 80);
        Mat3 A1 = upperTruth(D2R(a), D2R(b), D2R(tw)), A2 = foreTruth(A1, D2R(d), D2R(pr));
        axisVector u = calculateAxisVector(quaternionFromRotation(A1)), f = calculateAxisVector(quaternionFromRotation(A2));
        joints j = calculateJointAngles(u, f);
        maxErr = fmaxf(maxErr, fmaxf(fabsf(j.joint_a - a), fmaxf(fabsf(j.joint_b - b), fabsf(j.joint_d - d))));
        CHECK(fabsf(j.joint_a - a) < 0.01f && fabsf(j.joint_b - b) < 0.01f && fabsf(j.joint_d - d) < 0.02f,
              "a %g/%g b %g/%g d %g/%g", j.joint_a, a, j.joint_b, b, j.joint_d, d);
        CHECK(fabsf(vdot(f.k, u.s)) < 1e-4f, "hinge constraint of the truth model");
    }
    printf("  max joint-angle error %.4f deg\n", maxErr);
    axisVector n = calculateAxisVector(quaternionFromRotation(poseN()));
    joints j0 = calculateJointAngles(n, n);
    CHECK(fabsf(j0.joint_a) < 1e-4f && fabsf(j0.joint_b) < 1e-4f && fabsf(j0.joint_d) < 1e-3f, "N-pose not zero");
}

static void test_segment_calibration() {
    printf("[two-pose calibration: unknown mounting + unknown heading]\n");
    float worst = 0;
    for (int t = 0; t < 200; t++) {
        Mat3 M = randomRotation(); Mat3 T = trueT(urand(-3.14f, 3.14f));
        Mat3 A1 = poseN(), A2 = mat3Mul(Rx(D2R(90)), poseN());
        SegmentCal c; c.valid = false; c.anchored = false;
        bool ok = computeSegmentCalibration(gFromA(A1, T, M), gFromA(A2, T, M), c);
        CHECK(ok, "calibration rejected");
        CHECK(matErr(c.T, T) < 1e-4f, "T err %g", matErr(c.T, T));
        CHECK(matErr(c.M, M) < 1e-4f, "M err %g", matErr(c.M, M));
        for (int k = 0; k < 20; k++) {
            Mat3 A = upperTruth(D2R(urand(-60, 170)), D2R(urand(-60, 80)), D2R(urand(-80, 80)));
            Mat3 Ar = rotationFromQuaternion(segmentOrientation(gFromA(A, T, M), c));
            worst = fmaxf(worst, matErr(Ar, A));
        }
    }
    printf("  worst segment-orientation error after calibration %.2e\n", worst);
    CHECK(worst < 1e-4f, "segment orientation");
    Mat3 M = randomRotation(), T = trueT(0.3f); SegmentCal c;
    CHECK(!computeSegmentCalibration(gFromA(poseN(), T, M), gFromA(mat3Mul(Rx(D2R(30)), poseN()), T, M), c), "30 deg pose 2 must be rejected");
    float e80 = 0;
    for (int t = 0; t < 100; t++) {
        Mat3 Mt = randomRotation(), Tt = trueT(urand(-3.14f, 3.14f)); SegmentCal cc;
        Mat3 A2 = mat3Mul(mat3Mul(Rx(D2R(80)), Rz(D2R(5))), poseN());
        computeSegmentCalibration(gFromA(poseN(), Tt, Mt), gFromA(A2, Tt, Mt), cc);
        Mat3 A = upperTruth(D2R(45), D2R(0), 0);
        e80 = fmaxf(e80, rotAngleDeg(rotationFromQuaternion(segmentOrientation(gFromA(A, Tt, Mt), cc)), A));
    }
    printf("  sloppy pose 2 (80 deg flexion + 5 deg abduction): max orientation error %.2f deg\n", e80);
}

static void test_reanchor() {
    printf("[6500 re-anchor after reboot]\n");
    for (int t = 0; t < 100; t++) {
        Mat3 M9 = randomRotation(), M6 = randomRotation(), T9 = trueT(urand(-3, 3)), T6 = trueT(urand(-3, 3));
        Mat3 A1 = poseN(), A2 = mat3Mul(Rx(D2R(90)), poseN());
        SegmentCal c9, c6;
        computeSegmentCalibration(gFromA(A1, T9, M9), gFromA(A2, T9, M9), c9);
        computeSegmentCalibration(gFromA(A1, T6, M6), gFromA(A2, T6, M6), c6);
        Mat3 T6n = trueT(urand(-3, 3));
        c6.anchored = false; c6.T = mat3Identity();
        bool ok = reanchorSegment6500(gFromA(A1, T9, M9), c9, gFromA(A1, T6n, M6), c6);
        CHECK(ok, "reanchor rejected");
        CHECK(matErr(c6.T, T6n) < 1e-4f, "T6 err %g", matErr(c6.T, T6n));
    }
}

static void test_mag_calibration() {
    printf("[magnetometer: hard/soft iron + AK8963 axis mapping]\n");
    const xyzFloat mE(28.0f, 0.0f, -14.0f);
    for (int swap = 0; swap < 2; swap++) {
        std::vector<MagSample> v;
        xyzFloat off(nrand(15), nrand(15), nrand(15)), sc(urand(0.8f, 1.2f), urand(0.8f, 1.2f), urand(0.8f, 1.2f));
        for (int i = 0; i < 400; i++) {
            Mat3 r = randomRotation(); Mat3 rt = mat3Transpose(r);
            xyzFloat a = mat3MulVec(rt, xyzFloat(0, 0, 1)) + xyzFloat(nrand(0.02f), nrand(0.02f), nrand(0.02f));
            xyzFloat m = mat3MulVec(rt, mE);
            xyzFloat ak = swap ? xyzFloat(m.y, m.x, -m.z) : m;
            MagSample s = { a.x, a.y, a.z, ak.x * sc.x + off.x + nrand(0.3f), ak.y * sc.y + off.y + nrand(0.3f), ak.z * sc.z + off.z + nrand(0.3f) };
            v.push_back(s);
        }
        SensorCal c = SensorCal();
        bool ok = computeMagCalibration(v.data(), (int)v.size(), c);
        CHECK(ok, "mag cal rejected");
        CHECK(c.magSwapAxes == (swap == 1), "axis mapping detection (truth swap=%d got %d)", swap, c.magSwapAxes);
        CHECK(fabsf(c.magOffset.x - off.x) < 1.5f && fabsf(c.magOffset.y - off.y) < 1.5f && fabsf(c.magOffset.z - off.z) < 1.5f,
              "offset est %g %g %g vs %g %g %g", c.magOffset.x, c.magOffset.y, c.magOffset.z, off.x, off.y, off.z);
        // calibrated |m| must be constant: judge by std-dev (max-min is dominated by sensor noise)
        double sum = 0, sum2 = 0;
        for (size_t i = 0; i < v.size(); i++) {
            IMUData raw; raw.time = 0; raw.acc = xyzFloat(v[i].ax, v[i].ay, v[i].az); raw.gyr = xyzFloat(0, 0, 0); raw.mag = xyzFloat(v[i].mx, v[i].my, v[i].mz);
            double n = vnorm(applySensorCalibration(raw, c).mag); sum += n; sum2 += n * n;
        }
        double mean = sum / v.size(), sd = sqrt(sum2 / v.size() - mean * mean);
        CHECK(sd / mean < 0.03, "calibrated |m| std/mean %.3f", sd / mean);
    }
    std::vector<MagSample> flat; for (int i = 0; i < 100; i++) { MagSample s = { 0, 0, 1, 20.0f + nrand(0.2f), 5.0f, -30.0f }; flat.push_back(s); }
    SensorCal c = SensorCal();
    CHECK(!computeMagCalibration(flat.data(), 100, c), "static data must be rejected");
}

static void test_cartesian() {
    printf("[fingertip -> robot target -> IK/FK]\n");
    Mat3 A1 = poseN(), A2 = foreTruth(A1, D2R(90), 0);
    axisVector u = calculateAxisVector(quaternionFromRotation(A1)), f = calculateAxisVector(quaternionFromRotation(A2));
    xyzFloat tip = humanFingertip(u, f), tgt = mapHumanToRobot(tip);
    CHECK(fabsf(tgt.x - ROBOT_CENTER_X) < 1e-5f && fabsf(tgt.y - ROBOT_CENTER_Y) < 1e-5f && fabsf(tgt.z - ROBOT_CENTER_Z) < 1e-5f,
          "centre pose -> %g %g %g", tgt.x, tgt.y, tgt.z);
    xyzFloat tF = mapHumanToRobot(tip + xyzFloat(0, 0, -0.1f)), tL = mapHumanToRobot(tip + xyzFloat(-0.1f, 0, 0)), tU = mapHumanToRobot(tip + xyzFloat(0, 0.1f, 0));
    CHECK(tF.x > tgt.x && tL.y > tgt.y && tU.z > tgt.z, "axis directions");
    float worst = 0; int n = 0;
    for (int i = 0; i < 5000; i++) {
        float j[3] = { urand(-80, 80), urand(-20, 110), urand(5, 145) };
        xyzFloat p = robotForwardKinematics(j);
        float jr[3] = { 0, 0, 0 };
        bool ok = robotInverseKinematics(p, jr);
        xyzFloat p2 = robotForwardKinematics(jr);
        worst = fmaxf(worst, vnorm(p - p2)); n++;
        CHECK(ok, "reachable point flagged unreachable");
        CHECK(vnorm(p - p2) < 5e-4f, "IK/FK mismatch %g", vnorm(p - p2));   // float acosf near a fully folded elbow
        CHECK(jr[2] >= 0, "elbow branch");
    }
    printf("  %d random poses, worst |FK(IK(p)) - p| = %.2e m\n", n, worst);
    float jj[3] = { 0, 0, 0 };
    CHECK(!robotInverseKinematics(xyzFloat(2.0f, 0, 0.1f), jj), "unreachable not flagged");
    float c[3] = { 200, -100, 10 }, prev[3] = { 0, 0, 0 };
    uint8_t fl = limitJointAngles(c, prev, 0.02f);
    CHECK((fl & 1) && (fl & 2), "limit flags %d", fl);
    CHECK(fabsf(c[0] - MAX_JOINT_RATE_DPS * 0.02f) < 1e-4f && fabsf(c[1] + MAX_JOINT_RATE_DPS * 0.02f) < 1e-4f, "slew");
    float nanj[3] = { NAN, 10, 10 }, pv[3] = { 5, 10, 10 };
    limitJointAngles(nanj, pv, 0.02f);
    CHECK(nanj[0] == 5.0f, "NaN not replaced by previous");
}

static void test_hinge() {
    printf("[hinge-constraint yaw correction: sign and observability]\n");
    Mat3 A1 = poseN(), A2 = foreTruth(A1, D2R(90), 0);
    Mat3 A2e = mat3Mul(Ry(D2R(5)), A2);
    axisVector u = calculateAxisVector(quaternionFromRotation(A1)), f = calculateAxisVector(quaternionFromRotation(A2e));
    float d = hingeYawCorrection(u, f);
    CHECK(d < 0, "correction must oppose the +5 deg error (got %g)", d);
    axisVector fd = calculateAxisVector(quaternionFromRotation(mat3Mul(Ry(D2R(5)), poseN())));
    CHECK(hingeYawCorrection(u, fd) == 0.0f, "must be zero when forearm vertical");
}

int main() {
    test_quat_basics();
    test_madgwick_gradient();
    test_seed_and_madgwick();
    test_joint_angles();
    test_segment_calibration();
    test_reanchor();
    test_mag_calibration();
    test_cartesian();
    test_hinge();
    printf("\n%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
