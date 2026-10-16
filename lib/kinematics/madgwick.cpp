#include "kinematics.h"
#include <math.h>

static const float DEG2RADF = 0.017453292519943295f;

// Gradient of 0.5*|f|^2 for the Madgwick objective (report eq. 25-34).
// accUnit/magUnit are unit vectors in the BODY frame, q rotates body -> world.
// bx,bz: Earth magnetic field reference [bx 0 bz] (north/vertical). magUnit may be NULL.
void madgwickGradient(const quaternion &q, const xyzFloat &a, const xyzFloat *m,
                      float bx, float bz, float grad[4]) {
    const float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;

    // gravity: f_g = [2(q1q3-q0q2); 2(q0q1+q2q3); 2(0.5-q1^2-q2^2)] - a
    const float f0 = 2.0f * (q1 * q3 - q0 * q2) - a.x;
    const float f1 = 2.0f * (q0 * q1 + q2 * q3) - a.y;
    const float f2 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - a.z;

    grad[0] = -2.0f * q2 * f0 + 2.0f * q1 * f1;
    grad[1] =  2.0f * q3 * f0 + 2.0f * q0 * f1 - 4.0f * q1 * f2;
    grad[2] = -2.0f * q0 * f0 + 2.0f * q3 * f1 - 4.0f * q2 * f2;
    grad[3] =  2.0f * q1 * f0 + 2.0f * q2 * f1;

    if (m) {
        const float f3 = 2.0f * bx * (0.5f - q2 * q2 - q3 * q3) + 2.0f * bz * (q1 * q3 - q0 * q2) - m->x;
        const float f4 = 2.0f * bx * (q1 * q2 - q0 * q3) + 2.0f * bz * (q0 * q1 + q2 * q3) - m->y;
        const float f5 = 2.0f * bx * (q0 * q2 + q1 * q3) + 2.0f * bz * (0.5f - q1 * q1 - q2 * q2) - m->z;

        grad[0] += (-2.0f * bz * q2) * f3 + (-2.0f * bx * q3 + 2.0f * bz * q1) * f4 + (2.0f * bx * q2) * f5;
        grad[1] += (2.0f * bz * q3) * f3 + (2.0f * bx * q2 + 2.0f * bz * q0) * f4 + (2.0f * bx * q3 - 4.0f * bz * q1) * f5;
        grad[2] += (-4.0f * bx * q2 - 2.0f * bz * q0) * f3 + (2.0f * bx * q1 + 2.0f * bz * q3) * f4 + (2.0f * bx * q0 - 4.0f * bz * q2) * f5;
        grad[3] += (-4.0f * bx * q3 + 2.0f * bz * q1) * f3 + (-2.0f * bx * q0 + 2.0f * bz * q2) * f4 + (2.0f * bx * q1) * f5;
    }
}

void MadgwickAHRSupdate(const IMUData &d, float dt, quaternion &q, float beta) {
    const float gx = d.gyr.x * DEG2RADF, gy = d.gyr.y * DEG2RADF, gz = d.gyr.z * DEG2RADF;
    const float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;

    // rate of change of the quaternion from the gyroscope: 0.5 * q (x) [0 g]
    float d0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float d1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float d2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float d3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    const float an = vnorm(d.acc);
    if (an > 1e-6f) {                       // skip the correction if the accelerometer is invalid
        const xyzFloat a = d.acc / an;
        float grad[4];
        const float mn = vnorm(d.mag);
        if (mn > 1e-6f) {
            const xyzFloat m = d.mag / mn;
            // reference direction of the Earth field: h = R m ; b = [sqrt(hx^2+hy^2) 0 hz]
            const Mat3 r = rotationFromQuaternion(q);
            const xyzFloat h = mat3MulVec(r, m);
            const float bx = sqrtf(h.x * h.x + h.y * h.y);
            const float bz = h.z;
            madgwickGradient(q, a, &m, bx, bz, grad);
        } else {
            madgwickGradient(q, a, NULL, 0.0f, 0.0f, grad);
        }
        const float gn = sqrtf(grad[0] * grad[0] + grad[1] * grad[1] + grad[2] * grad[2] + grad[3] * grad[3]);
        if (gn > 1e-12f) {
            d0 -= beta * grad[0] / gn;
            d1 -= beta * grad[1] / gn;
            d2 -= beta * grad[2] / gn;
            d3 -= beta * grad[3] / gn;
        }
    }

    quaternion n = { q0 + d0 * dt, q1 + d1 * dt, q2 + d2 * dt, q3 + d3 * dt };
    q = qNormalize(n);
}
