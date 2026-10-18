#include "connections.h"
#include "configurations.h"
#include <SPI.h>
#include <math.h>

// Two independent SPI buses (pins are final, see configurations.cpp)
static SPIClass spi6500(VSPI);
static SPIClass spi9250(HSPI);

MPU6500_WE imu6500 = MPU6500_WE(&spi6500, csPin_6500, mosiPin_6500, misoPin_6500, sckPin_6500, useSPI);
MPU9250_WE imu9250 = MPU9250_WE(&spi9250, csPin_9250, mosiPin_9250, misoPin_9250, sckPin_9250, useSPI);

// The upper-arm slot normally holds an MPU9250 (WHO_AM_I 0x71). If a plain MPU6500 (WHO_AM_I 0x70) is fitted there
// instead, MPU9250_WE::init() refuses it and never wakes the chip, so a second driver object on the same
// pins is used for that case (6-axis only, no magnetometer).
MPU6500_WE imuUpper6500 = MPU6500_WE(&spi9250, csPin_9250, mosiPin_9250, misoPin_9250, sckPin_9250, useSPI);
static MPU6500_WE *upper = &imu9250;
static bool upperIs9250 = true;

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

    upper = &imu9250; upperIs9250 = true; magOk = false;
    if (imu9250.init()) {
        Serial.println("[IMU] upper arm: MPU9250 found");
        magOk = imu9250.initMagnetometer();
        if (!magOk) Serial.println("[IMU] AK8963 magnetometer does not respond -> 6-axis only");
    } else if (imuUpper6500.init()) {
        upper = &imuUpper6500; upperIs9250 = false;
        Serial.println("[IMU] upper arm: MPU6500 found (no magnetometer) -> 6-axis only");
    } else {
        Serial.println("[IMU] upper arm: no MPU9250/MPU6500 answers - check wiring CS=27 MOSI=12 MISO=14 SCK=13 and 3.3 V");
        ok = false;
    }
    if (imu6500.init()) Serial.println("[IMU] forearm: MPU6500 found");
    else { Serial.println("[IMU] forearm: MPU6500 does not respond - check wiring CS=19 MOSI=21 MISO=23 SCK=22 and 3.3 V"); ok = false; }
    if (!ok) return false;

    BenchOffsets b;
    if (loadBenchOffsets(b)) {
        upper->setAccOffsets(b.acc9250);
        upper->setGyrOffsets(b.gyr9250);
        imu6500.setAccOffsets(b.acc6500);
        imu6500.setGyrOffsets(b.gyr6500);
    } else {
        Serial.println("[IMU] no bench offsets stored: hold the button while powering on to create them");
    }

    configureIMU(*upper);
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
    i9250 = upperIs9250 ? obtainIMUData(imu9250) : obtainIMUData(*upper);
    i6500 = obtainIMUData(imu6500);
    const bool ok = plausible(i9250) && plausible(i6500);
    static uint32_t bad = 0;
    if (!ok && (bad++ % 200) == 0)       // visible diagnosis instead of a silent stall
        Serial.printf("[IMU] implausible data: upper |acc|=%.2f g |gyr|=%.0f dps, forearm |acc|=%.2f g |gyr|=%.0f dps (0 = chip asleep / not wired)\n",
                      vnorm(i9250.acc), vnorm(i9250.gyr), vnorm(i6500.acc), vnorm(i6500.gyr));
    return ok;
}

bool benchCalibrateIMUs() {
    Serial.println("[BENCH] keep both IMUs flat (z up) and still...");
    upper->autoOffsets();
    imu6500.autoOffsets();

    BenchOffsets b;
    b.acc9250 = upper->getAccOffsets();  b.gyr9250 = upper->getGyrOffsets();
    b.acc6500 = imu6500.getAccOffsets();  b.gyr6500 = imu6500.getGyrOffsets();

    // autoOffsets assumes z = +1 g. An IMU that is not flat gives huge offsets: refuse them.
    const xyzFloat *acc[2] = { &b.acc9250, &b.acc6500 };
    const xyzFloat *gyr[2] = { &b.gyr9250, &b.gyr6500 };
    bool sane = true;
    for (int i = 0; i < 2; i++) {
        if (fabsf(acc[i]->x) > 4000.0f || fabsf(acc[i]->y) > 4000.0f || fabsf(acc[i]->z) > 4000.0f) sane = false;
        if (fabsf(gyr[i]->x) > 2000.0f || fabsf(gyr[i]->y) > 2000.0f || fabsf(gyr[i]->z) > 2000.0f) sane = false;
    }
    configureIMU(*upper);        // autoOffsets changed the ranges / DLPF
    configureIMU(imu6500);
    if (!sane) {
        Serial.println("[BENCH] offsets implausible (IMU not flat or moving) - NOT saved");
        upper->setAccOffsets(xyzFloat(0, 0, 0)); upper->setGyrOffsets(xyzFloat(0, 0, 0));
        imu6500.setAccOffsets(xyzFloat(0, 0, 0)); imu6500.setGyrOffsets(xyzFloat(0, 0, 0));
        return false;
    }
    Serial.println("[BENCH] offsets saved");
    return saveBenchOffsets(b);
}
