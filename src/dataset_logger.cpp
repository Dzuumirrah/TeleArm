// /***************************************************************************
// * MPU6500 DLPF Evaluation Script
// * Records 60 s of data for each DLPF configuration at 200 Hz.
// * Only the DLPF changes between tests. Calibration is performed once.
// ***************************************************************************/

// #include <MPU6500_WE.h>

// const int csPin   = 19;
// const int mosiPin = 21;
// const int misoPin = 23;
// const int sckPin  = 22;

// bool useSPI = true;

// MPU6500_WE myMPU6500(&SPI, csPin, mosiPin, misoPin, sckPin, useSPI);

// const uint16_t SAMPLE_RATE   = 200;
// const uint32_t SAMPLE_PERIOD = 1000000UL / SAMPLE_RATE;

// bool finished = false;

// void configureCommSetup() {

//     myMPU6500.setSPIClockSpeed(20000000);

//     myMPU6500.setSampleRateDivider(5);  // Default sample rate = 1 kHz. Divided by 5 gives 200 Hz sample rate.

//     myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
//     myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);

//     myMPU6500.disableI2CinSPI(true);

//     delay(100);
// }

// void configureSetting(uint8_t dlpf)
// {
//     Serial.println();
//     Serial.println("========================================");

//     if (dlpf == 0)
//     {
//         Serial.println("# TEST : DLPF OFF");

//         myMPU6500.disableGyrDLPF(MPU6500_BW_WO_DLPF_8800);
//         myMPU6500.enableAccDLPF(false);
//     }
//     else
//     {
//         MPU9250_DLPF setting = static_cast<MPU9250_DLPF>(dlpf - 1);

//         Serial.printf("# TEST : DLPF %u\n", setting);

//         myMPU6500.enableGyrDLPF();
//         myMPU6500.enableAccDLPF(true);

//         myMPU6500.setGyrDLPF(setting);
//         myMPU6500.setAccDLPF(setting);
//     }

//     Serial.println("# Sample Rate : 200 Hz");
//     Serial.println("# Gyro Range  : ±250 dps");
//     Serial.println("# Acc Range   : ±2 g");
//     Serial.println("#");

//     delay(500);      // Allow filter to settle
// }

// void recordData(uint16_t durationSec)
// {
//     uint32_t totalSamples = durationSec * SAMPLE_RATE;

//     uint32_t nextSample = micros();

//     uint32_t startMillis = millis();

//     uint32_t lastMicros = micros();

//     Serial.println("time_ms,dt_us,ax,ay,az,gx,gy,gz");

//     for (uint32_t i = 0; i < totalSamples; i++)
//     {
//         while ((int32_t)(micros() - nextSample) < 0);

//         nextSample += SAMPLE_PERIOD;

//         uint32_t nowMicros = micros();
//         uint32_t dt = nowMicros - lastMicros;
//         lastMicros = nowMicros;

//         xyzFloat acc = myMPU6500.getGValues();
//         xyzFloat gyr = myMPU6500.getGyrValues();

//         Serial.print(millis() - startMillis);
//         Serial.print(",");

//         Serial.print(dt);
//         Serial.print(",");

//         Serial.print(acc.x, 6);
//         Serial.print(",");
//         Serial.print(acc.y, 6);
//         Serial.print(",");
//         Serial.print(acc.z, 6);
//         Serial.print(",");

//         Serial.print(gyr.x, 6);
//         Serial.print(",");
//         Serial.print(gyr.y, 6);
//         Serial.print(",");
//         Serial.println(gyr.z, 6);
//     }
// }

// void setup()
// {
//     Serial.begin(115200);

//     if (!myMPU6500.init())
//     {
//         Serial.println("MPU6500 initialization failed.");

//         while (1);
//     }

//     configureCommSetup();

//     Serial.println();
//     Serial.println("Place the IMU on a flat surface...");
//     delay(1000);

//     myMPU6500.autoOffsets();      // Calibrate ONCE only

//     Serial.println("Calibration complete.");
//     delay(1000);
// }

// void loop()
// {
//     if (finished)
//         return;

//     for (uint8_t dlpf = 0; dlpf <= 8; dlpf++)
//     {
//         configureSetting(dlpf);

//         if (dlpf == 0)
//             Serial.println("# START DLPF_OFF");
//         else
//             Serial.printf("# START DLPF_%d\n", dlpf - 1);

//         recordData(60);

//         Serial.println("# END");
//         Serial.println();

//         delay(2000);
//     }

//     Serial.println("========== ALL TESTS COMPLETE ==========");

//     finished = true;
// }