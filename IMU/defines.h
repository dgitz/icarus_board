#include "eROS_Definitions.h"

//Program Loop Definitions
#define VERYVERYSLOWLOOP_RATE 30000 //mS
#define VERYSLOWLOOP_RATE 10000 //mS
#define SLOWLOOP_RATE 1000 //mS
#define MEDIUMLOOP_RATE 200 //mS
#define FASTLOOP_RATE 20 //mS

//Pin Definitions
#define HW_LED_PIN 13 
#define MPU9250_INT_PIN 4

//IMU Definitions
#define DMP_SAMPLE_RATE    100 // Logging/DMP sample rate(4-200 Hz)
#define IMU_COMPASS_SAMPLE_RATE 100 // Compass sample rate (4-100 Hz)
#define IMU_AG_SAMPLE_RATE 100 // Accel/gyro sample rate Must be between 4Hz and 1kHz
#define IMU_GYRO_FSR       2000 // Gyro full-scale range (250, 500, 1000, or 2000)
#define IMU_ACCEL_FSR      2 // Accel full-scale range (2, 4, 8, or 16)
#define IMU_AG_LPF         5 // Accel/Gyro LPF corner frequency (5, 10, 20, 42, 98, or 188 Hz)
