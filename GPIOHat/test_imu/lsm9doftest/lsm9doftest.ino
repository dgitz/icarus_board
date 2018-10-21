#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#define IMU1_SCLK 21
#define IMU1_MISO 19
#define IMU1_MOSI 20
#define IMU1_XM_CS 22
#define IMU1_GYRO_CS 23
#define IMU2_SCLK 4
#define IMU2_MISO 18
#define IMU2_MOSI 5
#define IMU2_XM_CS 7
#define IMU2_GYRO_CS 13
// i2c
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();  O
// You can also use software SPI
Adafruit_LSM9DS0 lsm1 = Adafruit_LSM9DS0(IMU1_SCLK,IMU1_MISO,IMU1_MOSI,IMU1_XM_CS, IMU1_GYRO_CS);
Adafruit_LSM9DS0 lsm2 = Adafruit_LSM9DS0(IMU2_SCLK,IMU2_MISO,IMU2_MOSI,IMU2_XM_CS, IMU2_GYRO_CS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(3, 2);  OK
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(22, 23);
void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm1.setupAccel(lsm1.LSM9DS0_ACCELRANGE_2G);
  lsm2.setupAccel(lsm2.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm1.setupMag(lsm1.LSM9DS0_MAGGAIN_2GAUSS);
  lsm2.setupMag(lsm2.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm1.setupGyro(lsm1.LSM9DS0_GYROSCALE_245DPS);
  lsm2.setupGyro(lsm2.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}
bool lsm1_initialized = false;
bool lsm2_initialized = false;
bool lsm1_available = true;
bool lsm2_available = false;
void setup() 
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(115200);
  Serial.println("LSM1 raw read demo");
  if(lsm1_available == true)
  {
  // Try to initialise and warn if we couldn't detect the chip
  if(lsm1_initialized == false)
  {
    if (!lsm1.begin())
    {
      Serial.println("Oops ... unable to initialize the LSM9DS0 IMU1. Check your wiring!");
      delay(2000);
      //setup();
    }
    else
    {
      Serial.println("IMU1 Initialized.");
      lsm1_initialized = true;
    }
  }
}
if(lsm2_available == true)
{
  
  if(lsm2_initialized == false)
  {
    if (!lsm2.begin())
    {
      Serial.println("Oops ... unable to initialize the LSM9DS0 IMU2. Check your wiring!");
      delay(2000);
      //setup();
    }
    else
    {
      Serial.println("IMU2 Initialized.");
      lsm2_initialized = true;
    }
  }
}
  if(((lsm1_available == true) and (lsm1_initialized == false)) or ((lsm2_available == true) and (lsm2_initialized == false)))
  {
    setup();
  }
  Serial.println("Found LSM9DS0 9DOF");
  Serial.println("");
  Serial.println("");
}

void loop() 
{
  if(lsm1_available == true)
  {
  lsm1.read();
  }
  if(lsm2_available == true)
  {
  lsm2.read();
  }
  if(lsm1_available == true)
  {
  Serial.print("1Accel X: "); Serial.print((int)lsm1.accelData.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm1.accelData.y);       Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm1.accelData.z);     Serial.print(" ");
  }
  if(lsm2_available == true)
  {
  Serial.print("2Accel X: "); Serial.print((int)lsm2.accelData.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm2.accelData.y);       Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm2.accelData.z);     Serial.print(" ");
  }
  if(lsm1_available == true)
  {
  Serial.print("1 Mag X: "); Serial.print((int)lsm1.magData.x);     Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm1.magData.y);         Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm1.magData.z);       Serial.print(" ");
  }
  if(lsm2_available == true)
  {
  Serial.print("2 Mag X: "); Serial.print((int)lsm2.magData.x);     Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm2.magData.y);         Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm2.magData.z);       Serial.print(" ");
  }

 if(lsm1_available == true)
  {
  Serial.print("1 Gyro X: "); Serial.print((int)lsm1.gyroData.x);   Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm1.gyroData.y);        Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm1.gyroData.z);      Serial.println(" ");
  }
  if(lsm2_available == true)
  {
  Serial.print("2 Gyro X: "); Serial.print((int)lsm2.gyroData.x);   Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm2.gyroData.y);        Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm2.gyroData.z);      Serial.println(" ");
  }
  /*
  Serial.print("Temp: "); Serial.print((int)lsm1.temperature);    Serial.println(" ");
  */
  delay(1);
}
