#include "connections.h"
#include "configurations.h"
#include <SPI.h>
#include <math.h>

// Two independent SPI buses (pins are final, see configurations.cpp)
static SPIClass spi6500(VSPI);
static SPIClass spi9250(HSPI);

MPU6500_WE imu6500 = MPU6500_WE(&spi6500, csPin_6500, mosiPin_6500, misoPin_6500, sckPin_6500, useSPI);
MPU9250_WE imu9250 = MPU9250_WE(&spi9250, csPin_9250, mosiPin_9250, misoPin_9250, sckPin_9250, useSPI);

static bool magOk = false;

// Same settings for both IMUs (MPU9250_WE derives from MPU6500_WE).
// Internal rate 1 kHz + DLPF 4, polled at SAMPLE_RATE_HZ: every poll reads the freshest sample.
// (With a 200 Hz internal rate the poll would sometimes see a sample twice or skip one.)
static void configureIMU(MPU6500_WE &m) {
    m.enableGyrDLPF();
    m.setGyrDLPF(MPU9250_DLPF_4);
    m.setSampleRateDivider(0);
    m.setGyrRange(MPU9250_GYRO_RANGE_500);
    m.setAccRange(MPU9250_ACC_RANGE_2G);
    m.enableAccDLPF(true);
    m.setAccDLPF(MPU9250_DLPF_4);
}

IMUData obtainIMUData(MPU6500_WE &mpu) {
    IMUData d;
    d.time = micros();
    d.acc = mpu.getGValues();
    d.gyr = mpu.getGyrValues();
    d.mag = xyzFloat(0.0f, 0.0f, 0.0f);
    return d;
}

IMUData obtainIMUData(MPU9250_WE &mpu) {
    IMUData d = obtainIMUData(static_cast<MPU6500_WE &>(mpu));
    d.mag = mpu.getMagValues();
    return d;
}

bool MPUInit() {
    bool ok = true;

    if (!imu9250.init()) { Serial.println("[IMU] MPU9250 does not respond"); ok = false; }
    else {
        magOk = imu9250.initMagnetometer();
        if (!magOk) Serial.println("[IMU] AK8963 magnetometer does not respond -> 6-axis only");
    }
    if (!imu6500.init()) { Serial.println("[IMU] MPU6500 does not respond"); ok = false; }
    if (!ok) return false;

    BenchOffsets b;
    if (loadBenchOffsets(b)) {
        imu9250.setAccOffsets(b.acc9250);
        imu9250.setGyrOffsets(b.gyr9250);
        imu6500.setAccOffsets(b.acc6500);
        imu6500.setGyrOffsets(b.gyr6500);
    } else {
        Serial.println("[IMU] no bench offsets stored: hold the button while powering on to create them");
    }

    configureIMU(imu9250);
    configureIMU(imu6500);
    if (magOk) imu9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
    delay(100);
    return true;
}

bool magnetometerAvailable() { return magOk; }

static bool plausible(const IMUData &d) {
    const float a = vnorm(d.acc), g = vnorm(d.gyr);
    return isfinite(a) && isfinite(g) && a > 0.3f && a < 3.0f && g < 2000.0f;
}

bool readIMUs(IMUData &i9250, IMUData &i6500) {
    i9250 = obtainIMUData(imu9250);
    i6500 = obtainIMUData(imu6500);
    return plausible(i9250) && plausible(i6500);
}

bool benchCalibrateIMUs() {
    Serial.println("[BENCH] keep both IMUs flat (z up) and still...");
    imu9250.autoOffsets();
    imu6500.autoOffsets();

    BenchOffsets b;
    b.acc9250 = imu9250.getAccOffsets();  b.gyr9250 = imu9250.getGyrOffsets();
    b.acc6500 = imu6500.getAccOffsets();  b.gyr6500 = imu6500.getGyrOffsets();

    // autoOffsets assumes z = +1 g. An IMU that is not flat gives huge offsets: refuse them.
    const xyzFloat *acc[2] = { &b.acc9250, &b.acc6500 };
    const xyzFloat *gyr[2] = { &b.gyr9250, &b.gyr6500 };
    bool sane = true;
    for (int i = 0; i < 2; i++) {
        if (fabsf(acc[i]->x) > 4000.0f || fabsf(acc[i]->y) > 4000.0f || fabsf(acc[i]->z) > 4000.0f) sane = false;
        if (fabsf(gyr[i]->x) > 2000.0f || fabsf(gyr[i]->y) > 2000.0f || fabsf(gyr[i]->z) > 2000.0f) sane = false;
    }
    configureIMU(imu9250);       // autoOffsets changed the ranges / DLPF
    configureIMU(imu6500);
    if (!sane) {
        Serial.println("[BENCH] offsets implausible (IMU not flat or moving) - NOT saved");
        imu9250.setAccOffsets(xyzFloat(0, 0, 0)); imu9250.setGyrOffsets(xyzFloat(0, 0, 0));
        imu6500.setAccOffsets(xyzFloat(0, 0, 0)); imu6500.setGyrOffsets(xyzFloat(0, 0, 0));
        return false;
    }
    Serial.println("[BENCH] offsets saved");
    return saveBenchOffsets(b);
}
