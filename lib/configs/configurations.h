#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

// Master side  =============================================
// MPU6500
const int csPin_6500;  // Chip Select Pin
const int mosiPin_6500;  // "MOSI" Pin
const int misoPin_6500;  // "MISO" Pin
const int sckPin_6500;  // SCK Pin

// MPU9250
const int csPin_9250;  // Chip Select Pin
const int mosiPin_9250;  // "MOSI" Pin
const int misoPin_9250;  // "MISO" Pin
const int sckPin_9250;  // SCK Pin

// Others
const int btn_cal; // MPU calibration button  
const int vibration_motor; // Vibration motor pin (or led pin )

const bool useSPI;    // SPI use flag

#endif
