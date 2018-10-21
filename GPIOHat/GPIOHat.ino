/*
 * TARGET: FAST ROBOTICS 100007 GPIOHAT
 * Pololu ASTAR 32U4 https://www.pololu.com/docs/pdf/0J66/a-star_32u4_robot_controller.pdf
 * Arduino Board: Pololu A-Star 32u4
 * NOTES:
 * 
 * RESOURCE USAGE:
 * PIN6: BUZZER
 */


#include "eROS_Definitions.h"
#include "config.h"
#include "i2cmessage.h"
#include <Wire.h>
#include <AStar32U4.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#define G_ACC 9.80655

PololuBuzzer buzzer;

//MAIN PROGRAM DEFINITIONS
#define VERYVERYSLOWLOOP_RATE 30000 //mS
#define VERYSLOWLOOP_RATE 10000 //mS
#define SLOWLOOP_RATE 1000 //mS
#define MEDIUMLOOP_RATE 200 //mS
#define FASTLOOP_RATE 10 //mS
//Main Program Variables
unsigned char diag_component = GPIO_NODE;
unsigned char diag_type = SENSORS;
unsigned char diag_message = INITIALIZING;
unsigned char diag_level = NOTICE;
void(*resetBoard)(void) = 0;

//Timing Variables
long prev_time = 0;
long veryveryslowloop_timer = 0;
long veryslowloop_timer = 0;
long slowloop_timer = 0;
long mediumloop_timer = 0;
long fastloop_timer = 0;

//I2C Variables
//Defines for I2C Comm between Raspberry Pi and Arduino Board
long lastcom_rx = 0;
byte current_command = 0;
unsigned char outputBuffer_AB12[13];
unsigned char outputBuffer_AB14[13];
unsigned char outputBuffer_AB19[13];
unsigned char outputBuffer_AB27[13];
unsigned char outputBuffer_AB28[13];
unsigned char outputBuffer_AB29[13];

int process_AB12_Query();
int process_AB14_Query();
int process_AB19_Query();
int process_AB27_Query();
int process_AB28_Query();
int process_AB29_Query();
long I2C_TestMessageCounter_ID_rx = 0;
long I2C_Diagnostic_ID_rx = 0;
long I2C_Get_DIO_Port1_ID_rx = 0;
long I2C_Get_IMUAcc_ID_rx = 0;
long I2C_Get_IMUGyro_ID_rx = 0;
long I2C_Get_IMUMag_ID_rx = 0;
byte transmit_testcounter = 0;
void i2c_rx();
void i2c_tx();

//Sonar Variables/Definitions
unsigned int FLSonar_Distance = 0;
unsigned int FRSonar_Distance = 0;
unsigned int BLSonar_Distance = 0;
unsigned int BRSonar_Distance = 0;

//IMU Variables/Definitions
Adafruit_LSM9DS0 imu1 = Adafruit_LSM9DS0(IMU1_SCLK,IMU1_MISO,IMU1_MOSI,IMU1_XM_CS, IMU1_GYRO_CS, (int32_t)IMU1_ID);
bool imu1_initialized = false;
bool imu1_available = true;
Adafruit_LSM9DS0 imu2 = Adafruit_LSM9DS0(IMU2_SCLK,IMU2_MISO,IMU2_MOSI,IMU2_XM_CS, IMU2_GYRO_CS, (int32_t)IMU2_ID);
bool imu2_initialized = false;
bool imu2_available = true;
float imu1_acc_range = LSM9DS0_ACCEL_MG_LSB_4G;
float imu2_acc_range = LSM9DS0_ACCEL_MG_LSB_4G;
float imu1_gyro_range = LSM9DS0_GYRO_DPS_DIGIT_245DPS;
float imu2_gyro_range = LSM9DS0_GYRO_DPS_DIGIT_245DPS;
float imu1_mag_range = LSM9DS0_MAG_MGAUSS_2GAUSS;
float imu2_mag_range = LSM9DS0_MAG_MGAUSS_2GAUSS;
double scale_imuacc(double scale,double v);
double scale_imugyro(double scale,double v);
double scale_imumag(double scale,double v);
bool configure_imus()
{
  bool return_value = false;
    imu1.setupAccel(imu1.LSM9DS0_ACCELRANGE_4G);
    imu1.setupMag(imu1.LSM9DS0_MAGGAIN_2GAUSS);
    imu1.setupGyro(imu1.LSM9DS0_GYROSCALE_245DPS);
    imu2.setupAccel(imu1.LSM9DS0_ACCELRANGE_4G);
    imu2.setupMag(imu2.LSM9DS0_MAGGAIN_2GAUSS);
    imu2.setupGyro(imu2.LSM9DS0_GYROSCALE_245DPS);
    return_value = true;
  return return_value;
}




int get_distance(int pin);
//Message processing functions.  This should be as fast as possible
int process_AB12_Query()
{
  int msg_length;
  encode_DiagnosticI2C(outputBuffer_AB12,&msg_length,
   DIAG_SYSTEM,DIAG_SUBSYSTEM,diag_component,diag_type,diag_level,diag_message);
}
int process_AB14_Query()
{
  int msg_length;
 encode_TestMessageCounterI2C(outputBuffer_AB14,&msg_length,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter--,
    transmit_testcounter--,
    transmit_testcounter--);
}
int process_AB19_Query()
{
  int msg_length;
 encode_Get_DIO_Port1I2C(outputBuffer_AB19,&msg_length,
    FLSonar_Distance,FRSonar_Distance,BLSonar_Distance,BRSonar_Distance);
}
int process_AB27_Query()
{
  int msg_length;
  unsigned x1,y1,z1,x2,y2,z2;
  encode_Get_IMUAccI2C(outputBuffer_AB27,&msg_length,
    (unsigned int)((scale_imuacc(imu1_acc_range,imu1.accelData.x)*100.0)+32768.0),
    (unsigned int)((scale_imuacc(imu1_acc_range,imu1.accelData.y)*100.0)+32768.0),
    (unsigned int)((scale_imuacc(imu1_acc_range,imu1.accelData.z)*100.0)+32768.0),
    (unsigned int)((scale_imuacc(imu2_acc_range,imu2.accelData.x)*100.0)+32768.0),
    (unsigned int)((scale_imuacc(imu2_acc_range,imu2.accelData.y)*100.0)+32768.0),
    (unsigned int)((scale_imuacc(imu2_acc_range,imu2.accelData.z)*100.0)+32768.0));
}
int process_AB28_Query()
{
  int msg_length;
  encode_Get_IMUGyroI2C(outputBuffer_AB28,&msg_length,
    (unsigned int)((scale_imugyro(imu1_gyro_range,imu1.gyroData.x)*1000.0)+32768.0),
    (unsigned int)((scale_imugyro(imu1_gyro_range,imu1.gyroData.y)*1000.0)+32768.0),
    (unsigned int)((scale_imugyro(imu1_gyro_range,imu1.gyroData.z)*1000.0)+32768.0),
    (unsigned int)((scale_imugyro(imu2_gyro_range,imu2.gyroData.x)*1000.0)+32768.0),
    (unsigned int)((scale_imugyro(imu2_gyro_range,imu2.gyroData.y)*1000.0)+32768.0),
    (unsigned int)((scale_imugyro(imu2_gyro_range,imu2.gyroData.z)*1000.0)+32768.0));
}
int process_AB29_Query()
{
  int msg_length;
  encode_Get_IMUMagI2C(outputBuffer_AB29,&msg_length,
    (unsigned int)((scale_imumag(imu1_mag_range,imu1.magData.x)*1000.0)+32768.0),
    (unsigned int)((scale_imumag(imu1_mag_range,imu1.magData.y)*1000.0)+32768.0),
    (unsigned int)((scale_imumag(imu1_mag_range,imu1.magData.z)*1000.0)+32768.0),
    (unsigned int)((scale_imumag(imu2_mag_range,imu2.magData.x)*1000.0)+32768.0),
    (unsigned int)((scale_imumag(imu2_mag_range,imu2.magData.y)*1000.0)+32768.0),
    (unsigned int)((scale_imumag(imu2_mag_range,imu2.magData.z)*1000.0)+32768.0));
}
void setup() {
  int retry_count = 0;
  Serial.begin(115200);
  while(Serial.read() >= 0);
  Serial.flush();
   if(DEBUG_PRINT == 1)
  {
    print_deviceinfo();
    Serial.println("[Status]: Booting.");
  }
  Wire.begin(DEVICEID);
  Wire.onReceive(i2c_rx);
  Wire.onRequest(i2c_tx);
  prev_time = millis();
  if(imu1_initialized == false)
  {
    while(imu1.begin() == false)
    {
        Serial.println("IMU1 Not Found.  Retrying.");
        delay(6000);
        retry_count++;
        if(retry_count > 2)
        {
          Serial.println("[Status] IMU1 Not Available.");
          imu1_available = false;
          break;
        }
    }
    
    if(imu1_available == true)
    {
    imu1_initialized = true;
    }
   
  }
  retry_count = 0;
  if(imu2_initialized == false)
  {
    while(imu2.begin() == false)
    {
       Serial.println("IMU2 Not Found.  Retrying.");
       delay(6000);
       retry_count++;
       if(retry_count > 2)
        {
          Serial.println("[Status] IMU2 Not Available.");
          imu2_available = false;
          break;
        }
    }
    if(imu2_available == true)
    {
      imu2_initialized = true;
    }
   
  }
  configure_imus();
  buzzer.play("v10>>g16>>>c16");
  
  run_powerup();
  Serial.println("[Status]: Running");

}


long idle_counter = 0;
long loop_counter = 0;
void loop() {
  loop_counter++;
  
  long now = millis();
  long dt = now - prev_time;
  prev_time = now;
 
  if(true == true)
  {
    bool nothing_ran = true;
    fastloop_timer += dt;
    mediumloop_timer += dt;
    slowloop_timer += dt;
    veryslowloop_timer += dt;
    veryveryslowloop_timer += dt;
    if(fastloop_timer > FASTLOOP_RATE)
    {
      fastloop_timer = 0;
      run_fastloop(FASTLOOP_RATE);
      nothing_ran = false;
    }
    if(mediumloop_timer > MEDIUMLOOP_RATE)
    {
      mediumloop_timer = 0;
      run_mediumloop(MEDIUMLOOP_RATE);
      nothing_ran = false;
    }
    
    if(slowloop_timer > SLOWLOOP_RATE)
    {
      slowloop_timer = 0;
      run_slowloop(SLOWLOOP_RATE);
      nothing_ran = false;
    }
    if(veryslowloop_timer > VERYSLOWLOOP_RATE)
    {
      veryslowloop_timer = 0;
      run_veryslowloop(VERYSLOWLOOP_RATE);
      nothing_ran = false;
    }
    if(veryveryslowloop_timer > VERYVERYSLOWLOOP_RATE)
    {
      veryveryslowloop_timer = 0;
      run_veryveryslowloop(VERYVERYSLOWLOOP_RATE);
      nothing_ran = false;
    }
    if(nothing_ran == true)
    {
      idle_counter++;
    }
  //delay(1);
  }
  if(loop_counter > 1000000)
  {
    loop_counter = idle_counter = 0;
  }
}


void run_powerup()
{
  
}
void i2c_rx(int numBytes)
{
  while(Wire.available())
  {
    char c = Wire.read();    
    lastcom_rx = millis();
    if(c == I2C_Diagnostic_ID)
    {
      
      I2C_Diagnostic_ID_rx++;
      current_command = I2C_Diagnostic_ID;
      process_AB12_Query();
    }
    else if(c == I2C_TestMessageCounter_ID)
    {
      
      I2C_TestMessageCounter_ID_rx++;
      current_command = I2C_TestMessageCounter_ID;
      process_AB14_Query();
    }
    else if(c == I2C_Get_DIO_Port1_ID)
    {
      
      I2C_Get_DIO_Port1_ID_rx++;
      current_command = I2C_Get_DIO_Port1_ID;
      process_AB19_Query();
    }
    else if(c == I2C_Get_IMUAcc_ID)
    {
      
      I2C_Get_IMUAcc_ID_rx++;
      current_command = I2C_Get_IMUAcc_ID;
      process_AB27_Query();
    }
    else if(c == I2C_Get_IMUGyro_ID)
    {
      I2C_Get_IMUGyro_ID_rx++;
      current_command = I2C_Get_IMUGyro_ID;
      process_AB28_Query();
    }
    else if(c == I2C_Get_IMUMag_ID)
    {
      
      I2C_Get_IMUMag_ID_rx++;
      current_command = I2C_Get_IMUMag_ID;
      process_AB29_Query();
    }
  }
}
void i2c_tx()
{
  if(current_command == I2C_TestMessageCounter_ID)
  {
    Wire.write(outputBuffer_AB14,13);
  }
  else if(current_command == I2C_Diagnostic_ID)
  {
    Wire.write(outputBuffer_AB12,13);
  }
  else if(current_command == I2C_Get_DIO_Port1_ID)
  {
    Wire.write(outputBuffer_AB19,13);
  }
  else if(current_command == I2C_Get_IMUAcc_ID)
  {
    Wire.write(outputBuffer_AB27,13);
  }
  else if(current_command == I2C_Get_IMUGyro_ID)
  {
    Wire.write(outputBuffer_AB28,13);
  }
  else if(current_command == I2C_Get_IMUMag_ID)
  {
    Wire.write(outputBuffer_AB29,13);
  }
  //unsigned char char_ar[16] = "Hi Raspberry Pi"; //Create String
  //Wire.write(char_ar,16); //Write String to Pi.
}
bool run_veryveryslowloop(long dt)
{
  if(DEBUG_PRINT)
  {
    Serial.println();
    Serial.print("[Time]: ");
    Serial.println(millis());

    Serial.print("I2C: 0xAB");
    Serial.print(I2C_Diagnostic_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(I2C_Diagnostic_ID_rx);

    Serial.print("I2C: 0xAB");
    Serial.print(I2C_TestMessageCounter_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(I2C_TestMessageCounter_ID_rx);


    Serial.print("I2C: 0xAB");
    Serial.print(I2C_Get_DIO_Port1_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(I2C_Get_DIO_Port1_ID_rx);

    Serial.print("I2C: 0xAB");
    Serial.print(I2C_Get_IMUAcc_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(I2C_Get_IMUAcc_ID_rx);

    Serial.print("I2C: 0xAB");
    Serial.print(I2C_Get_IMUGyro_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(I2C_Get_IMUGyro_ID_rx);

    Serial.print("I2C: 0xAB");
    Serial.print(I2C_Get_IMUMag_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(I2C_Get_IMUMag_ID_rx);
/*
    Serial.print("SPI: 0xAB");
    Serial.print(SPI_Get_ANA_Port1_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(SPI_Get_ANA_Port1_ID_rx);

    Serial.print("SPI: 0xAB");
    Serial.print(SPI_LEDStripControl_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(SPI_LEDStripControl_ID_rx);
    Serial.println();
    */
    
    
  }

}
bool run_veryslowloop(long dt)
{
  if(DEBUG_PRINT)
  {
    print_deviceinfo();
  }
  if(DEBUG_PRINT == 1)
  {
    
      Serial.print("[Status]: Idle Time: ");
      double idle_perc = 100.0*(double)idle_counter/(double)loop_counter;
      Serial.print(idle_perc);
      Serial.println(" %");
  }
}

bool run_slowloop(long dt)
{
  /*
    Serial.print("IMU1 Accel X: "); Serial.print(accel1.acceleration.x); Serial.print(" ");
    Serial.print("Y: "); Serial.print(accel1.acceleration.y);       Serial.print(" ");
    Serial.print("Z: "); Serial.println(accel1.acceleration.z);

    Serial.print("IMU2 Accel X: "); Serial.print((int)imu2.accelData.x); Serial.print(" ");
    Serial.print("Y: "); Serial.print((int)imu2.accelData.y);       Serial.print(" ");
    Serial.print("Z: "); Serial.println((int)imu2.accelData.z);
    */
  long rx_dt = millis() - lastcom_rx;
  if(DEBUG_PRINT == 1)
  {
      
      if(rx_dt > 2500)
      {
        Serial.print("[WARN]: Haven't received any I2C Data in ");
        Serial.print(rx_dt);
        Serial.println(" mS.");
      }
  }
  if(imu1_available == false)
    {
      Serial.println("[Status] IMU1 Not Available.");
    }
    if(imu2_available == false)
    {
       Serial.println("[Status] IMU2 Not Available.");
    }
}
bool run_mediumloop(long dt)
{
  long rx_dt = millis() - lastcom_rx;
    //FLSonar_Distance = get_distance_in(FLSONAR);
  FRSonar_Distance = get_distance_in(FRSONAR);
  BLSonar_Distance = get_distance_in(BLSONAR);
  BRSonar_Distance = get_distance_in(BRSONAR);
  if((imu1_available == false) or (imu2_available == false))
  {
    diag_component = GPIO_NODE;
    diag_type = SENSORS;
    diag_level = ERROR;
    diag_message = INITIALIZING_ERROR;
  }
  else if(rx_dt > 10000)
  {
    diag_component = GPIO_NODE;
    diag_type = COMMUNICATIONS;
    diag_level = ERROR;
    diag_message = DROPPING_PACKETS;
  }
  else if(rx_dt > 2500)
  {
    diag_component = GPIO_NODE;
    diag_type = COMMUNICATIONS;
    diag_level = WARN;
    diag_message = DROPPING_PACKETS;
  }
  else 
  {
    diag_component = GPIO_NODE;
    diag_type = SENSORS;
    diag_level = NOTICE;
    diag_message = NOERROR;
    
    
  }
  
 
  
}
bool run_fastloop(long dt)
{
  if(imu1_available == true)
  {
    imu1.read();
  }
  if(imu2_available == true)
  {
  imu2.read();
  }
  //imu1.getEvent(&accel1, &mag1, &gyro1, &temp1);
  //imu2.getEvent(&accel2, &mag2, &gyro2, &temp2);
  /*
  int rx_count = i2c_handler();
  if(rx_count > 0)
  {
    lastcom_rx =  millis();
  }
  else
  {
   
  }
  */
}
int get_distance_in(int pin)
{
  if(pin < 0)
  {
    return -1;
  }
  long duration;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH);
  
  return duration/74/2;
}
void print_deviceinfo()
{
  Serial.print("[Time]: ");
  Serial.print(millis());
  Serial.print("(mS) Device Type: ");
  Serial.print(DEVICETYPE);
  Serial.print(" Part Number: ");
  Serial.print(PARTNUMBER);
  Serial.print(" Name: ");
  Serial.print(DEVICENAME);
  Serial.print(" ID: ");
  Serial.println(DEVICEID);
}
double scale_imuacc(double scale,double v)
{
  return scale*v*G_ACC/1000.0;
}
double scale_imugyro(double scale,double v)
{
  return scale*v;
}
double scale_imumag(double scale,double v)
{
  return scale*v/1000.0;
}
