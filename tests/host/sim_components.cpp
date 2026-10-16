#include "sim.h"
#include "connections.h"
#include "configurations.h"
#include <random>
#include <math.h>

SimWorld sim;
static std::mt19937 srng(777);
static float sn(float s) { return std::normal_distribution<float>(0.0f, s)(srng); }

void SimWorld::reset() {
    *this = SimWorld();                       // value-initialised: everything zero
    Aup = mat3Identity(); Afore = mat3Identity(); M9 = mat3Identity(); M6 = mat3Identity();
    heading = 0.0f;
    gyroNoise = 0.06f; accNoise = 0.003f; magNoise = 0.4f;
    magEarth = xyzFloat(28.0f, 0.0f, -14.0f);
    magDisturb = xyzFloat(0, 0, 0);
    hardIron = xyzFloat(0, 0, 0); softIron = xyzFloat(1, 1, 1);
    mpuInitOk = true; magAvailable = true;
    lever1 = 0.14f; lever2 = 0.10f; dynamicAccel = true;
}

// ---- history for gyro (finite difference of the true rotation) and lever-arm acceleration -------
struct Hist { bool valid; unsigned long t; Mat3 R; xyzFloat p[2]; int np; };
static Hist h9, h6;
void simStartMotion(unsigned long) { h9.valid = false; h6.valid = false; h9.np = h6.np = 0; }

static Mat3 sensorRotation(const Mat3 &A, const Mat3 &M, float heading) {
    // A = T R M  ->  R = T^T A M^T       (R: sensor -> compass world)
    xyzFloat f(cosf(heading), sinf(heading), 0), up(0, 0, 1);
    xyzFloat r = vcross(f, up);
    Mat3 T; T.m[0][0] = r.x; T.m[0][1] = r.y; T.m[0][2] = r.z; T.m[1][0] = 0; T.m[1][1] = 0; T.m[1][2] = 1;
    T.m[2][0] = -f.x; T.m[2][1] = -f.y; T.m[2][2] = -f.z;
    return mat3Mul(mat3Transpose(T), mat3Mul(A, mat3Transpose(M)));
}
static Mat3 headingT(float heading) {
    xyzFloat f(cosf(heading), sinf(heading), 0), up(0, 0, 1);
    xyzFloat r = vcross(f, up);
    Mat3 T; T.m[0][0] = r.x; T.m[0][1] = r.y; T.m[0][2] = r.z; T.m[1][0] = 0; T.m[1][1] = 0; T.m[1][2] = 1;
    T.m[2][0] = -f.x; T.m[2][1] = -f.y; T.m[2][2] = -f.z;
    return T;
}
static xyzFloat col(const Mat3 &a, int j) { return xyzFloat(a.m[0][j], a.m[1][j], a.m[2][j]); }

static IMUData produce(Hist &h, const Mat3 &A, const Mat3 &M, const float bias[3], bool hasMag, xyzFloat posP, int idx) {
    const unsigned long now = micros();
    const Mat3 R = sensorRotation(A, M, sim.heading);
    const Mat3 Rt = mat3Transpose(R);
    IMUData d; d.time = (uint32_t)now;

    // gyro: R_prev^T R = exp([w]x dt)
    xyzFloat w(0, 0, 0);
    if (h.valid && now > h.t) {
        const float dt = (float)(now - h.t) * 1e-6f;
        Mat3 D = mat3Mul(mat3Transpose(h.R), R);
        xyzFloat v(D.m[2][1] - D.m[1][2], D.m[0][2] - D.m[2][0], D.m[1][0] - D.m[0][1]);
        float s = 0.5f * vnorm(v), c = 0.5f * (D.m[0][0] + D.m[1][1] + D.m[2][2] - 1.0f);
        float ang = atan2f(s, c);
        xyzFloat axis = (s > 1e-9f) ? v * (0.5f / s) : xyzFloat(0, 0, 0);
        w = axis * (ang / dt);                                  // rad/s, body frame
    }
    // acceleration: specific force = a_world + g, with the lever-arm acceleration of the IMU position
    xyzFloat aW(0, 0, 0);
    if (sim.dynamicAccel && h.valid && h.np >= 2 && now > h.t) {
        const float dt = (float)(now - h.t) * 1e-6f;
        xyzFloat aP = (posP - h.p[0] * 2.0f + h.p[1]) * (1.0f / (dt * dt));
        aW = mat3MulVec(mat3Transpose(headingT(sim.heading)), aP);   // P -> compass world
    }
    xyzFloat fW = aW * (1.0f / 9.81f) + xyzFloat(0, 0, 1);
    xyzFloat f = mat3MulVec(Rt, fW);
    d.acc = xyzFloat(f.x + sn(sim.accNoise), f.y + sn(sim.accNoise), f.z + sn(sim.accNoise));

    const float RAD2DEG = 57.29577951308232f;
    float gx = w.x * RAD2DEG + bias[0] + sn(sim.gyroNoise), gy = w.y * RAD2DEG + bias[1] + sn(sim.gyroNoise), gz = w.z * RAD2DEG + bias[2] + sn(sim.gyroNoise);
    const float lim = 500.0f;
    d.gyr = xyzFloat(fminf(fmaxf(gx, -lim), lim), fminf(fmaxf(gy, -lim), lim), fminf(fmaxf(gz, -lim), lim));

    d.mag = xyzFloat(0, 0, 0);
    if (hasMag && sim.magAvailable) {
        xyzFloat m = mat3MulVec(Rt, sim.magEarth + sim.magDisturb);
        xyzFloat ak = sim.akSwap ? xyzFloat(m.y, m.x, -m.z) : m;
        d.mag = xyzFloat(ak.x * sim.softIron.x + sim.hardIron.x + sn(sim.magNoise),
                         ak.y * sim.softIron.y + sim.hardIron.y + sn(sim.magNoise),
                         ak.z * sim.softIron.z + sim.hardIron.z + sn(sim.magNoise));
    }
    h.p[1] = h.p[0]; h.p[0] = posP; if (h.np < 2) h.np++;
    // the position history above stores previous samples; the current one is pushed after use
    h.valid = true; h.t = now; h.R = R;
    (void)idx;
    return d;
}

bool MPUInit() { sim.initCalls++; return sim.mpuInitOk; }
bool magnetometerAvailable() { return sim.magAvailable; }
bool benchCalibrateIMUs() { sim.benchCalls++; return true; }

bool readIMUs(IMUData &imu9250, IMUData &imu6500) {
    sim.readCalls++;
    if (sim.failReads != 0) { if (sim.failReads > 0) sim.failReads--; return false; }
    const xyzFloat k1 = col(sim.Aup, 0), k2 = col(sim.Afore, 0);
    const xyzFloat p1 = k1 * sim.lever1;
    const xyzFloat p2 = k1 * HUMAN_UPPER_ARM_LEN + k2 * sim.lever2;
    imu9250 = produce(h9, sim.Aup, sim.M9, sim.gyroBias9, true, p1, 0);
    imu6500 = produce(h6, sim.Afore, sim.M6, sim.gyroBias6, false, p2, 1);
    return true;
}
