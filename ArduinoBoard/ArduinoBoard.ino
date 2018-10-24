/*
 * NOTES:
 * 
 * RESOURCE USAGE:
 * 2 INTERRUPTS: LEFT ENCODER AND RIGHT ENCODER
 */



#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include "eROS_Definitions.h"
#include "spimessage.h"
#include <SPI.h>
#include "config.h"
#define G_ACC 9.80655

//MAIN PROGRAM DEFINITIONS
#define VERYVERYSLOWLOOP_RATE 30000 //mS
#define VERYSLOWLOOP_RATE 10000 //mS
#define SLOWLOOP_RATE 1000 //mS
#define MEDIUMLOOP_RATE 200 //mS
#define FASTLOOP_RATE 20 //mS
Adafruit_NeoPixel led_strip = Adafruit_NeoPixel(LEDSTRIP_PIXELCOUNT, LEDSTRIP_PIN, NEO_RGBW + NEO_KHZ800);

//Main Program Variables
unsigned char diag_component = GPIO_NODE;
unsigned char diag_type = SENSORS;
unsigned char diag_message = INITIALIZING;
unsigned char diag_level = NOTICE;

//Encoder Variables
double EncoderLeft_TickSpeed = 0.0;  // Ticks/mS
double EncoderRight_TickSpeed = 0.0; // Ticks/mS
volatile double LeftEncoder_pos = 0.0;
double last_LeftEncoder_pos = 0.0;
volatile double RightEncoder_pos = 0;
double last_RightEncoder_pos = 0.0;

//Timing Variables
long prev_time = 0;
long veryveryslowloop_timer = 0;
long veryslowloop_timer = 0;
long slowloop_timer = 0;
long mediumloop_timer = 0;
long fastloop_timer = 0;

//SPI Variables
//Defines for SPI Comm between Raspberry Pi and Arduino Board
//unsigned char transmitBuffer[14];
//unsigned char receiveBuffer[14];
long lastcom_rx = 0;
unsigned char outputBuffer_AB12[13];
unsigned char outputBuffer_AB14[13];
unsigned char outputBuffer_AB19[13];
unsigned char outputBuffer_AB27[13];
unsigned char outputBuffer_AB28[13];
unsigned char outputBuffer_AB29[13];
unsigned char inputBuffer_AB42[13];
bool run_spi_handler = false;
byte current_command = 0;
int outputBuffer_index = 0;
int received_command = 0;
int message_ready_to_send = 0;
int receive_index = 0;
int message_index = 0;
byte marker = 0;
unsigned char dat;
int compute_checksum(unsigned char * outputbuffer);
int process_AB12_Query();
int process_AB14_Query();
int process_AB19_Query();
int process_AB20_Query();
int process_AB27_Query();
int process_AB28_Query();
int process_AB29_Query();
int process_AB41_Query();
int process_AB42_Command(int checksum);
byte transmit_testcounter = 0;
long SPI_Diagnostic_ID_rx = 0;
long SPI_TestMessageCounter_ID_rx = 0;
long SPI_Get_DIO_Port1_ID_rx = 0;
long SPI_Get_ANA_Port1_ID_rx = 0;
long SPI_Get_IMUAcc_ID_rx = 0;
long SPI_Get_IMUGyro_ID_rx = 0;
long SPI_Get_IMUMag_ID_rx = 0;
long SPI_LEDStripControl_ID_rx = 0;

int current_ledpixel = 0;
bool led_direction = 1;
int led_timeduration = 0;
unsigned char ledpixel_mode = LEDPIXELMODE_ERROR;
int current_ledcolor = LEDPIXELCOLOR_OFF;
int led_errortime = 1000; //mS
int led_state = 0;

//IMU Definitions
Adafruit_LSM9DS0 imu1 = Adafruit_LSM9DS0(IMU1_ID);
Adafruit_LSM9DS0 imu2 = Adafruit_LSM9DS0(IMU2_ID);
bool imu1_initialized = false;
bool imu2_initialized = false;
bool imu1_available = true;
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

//Message processing functions.  This should be as fast as possible
int process_AB12_Query()
{
  int msg_length;
  encode_DiagnosticSPI(outputBuffer_AB12,&msg_length,
   DIAG_SYSTEM,DIAG_SUBSYSTEM,diag_component,diag_type,diag_level,diag_message);
}
int process_AB14_Query()
{
  int msg_length;
  encode_TestMessageCounterSPI(outputBuffer_AB14,&msg_length,
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
  encode_Get_DIO_Port1SPI(outputBuffer_AB19,&msg_length,
    (int)EncoderLeft_TickSpeed+BYTE2_OFFSET,
    (int)EncoderRight_TickSpeed+BYTE2_OFFSET);
}
int process_AB27_Query()
{
  int msg_length;
  unsigned x1,y1,z1,x2,y2,z2;
  encode_Get_IMUAccSPI(outputBuffer_AB27,&msg_length,
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
  encode_Get_IMUGyroSPI(outputBuffer_AB28,&msg_length,
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
  encode_Get_IMUMagSPI(outputBuffer_AB29,&msg_length,
    (unsigned int)((scale_imumag(imu1_mag_range,imu1.magData.x)*1000.0)+32768.0),
    (unsigned int)((scale_imumag(imu1_mag_range,imu1.magData.y)*1000.0)+32768.0),
    (unsigned int)((scale_imumag(imu1_mag_range,imu1.magData.z)*1000.0)+32768.0),
    (unsigned int)((scale_imumag(imu2_mag_range,imu2.magData.x)*1000.0)+32768.0),
    (unsigned int)((scale_imumag(imu2_mag_range,imu2.magData.y)*1000.0)+32768.0),
    (unsigned int)((scale_imumag(imu2_mag_range,imu2.magData.z)*1000.0)+32768.0));
}
int process_AB42_Command(unsigned char checksum)
{
  unsigned char Param1,Param2;
  int msg_length;
  unsigned char v1,v2,v3;
  if(decode_LEDStripControlSPI(inputBuffer_AB42,&msg_length,checksum,&v1,&v2,&v3) == 1)
  {
    ledpixel_mode = v1;
    Param1 = v2;
    Param2 = v3;
    
  }
  else
  {
  }
  return 1;
  
}

void setup() {
  Serial.begin(115200);
  while(Serial.read() >= 0);
  Serial.flush();
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);

  pinMode(ENCODERLEFTA_PIN,INPUT);
  digitalWrite(ENCODERLEFTA_PIN,HIGH);
  pinMode(ENCODERLEFTB_PIN,INPUT);
  digitalWrite(ENCODERLEFTB_PIN,HIGH);
  pinMode(ENCODERRIGHTA_PIN,INPUT);
  digitalWrite(ENCODERRIGHTA_PIN,HIGH);
  pinMode(ENCODERRIGHTB_PIN,INPUT);
  digitalWrite(ENCODERRIGHTB_PIN,HIGH);
  attachInterrupt(digitalPinToInterrupt(ENCODERLEFTA_PIN),ISR_ENCODERLEFTA,CHANGE);
 // attachInterrupt(digitalPinToInterrupt(ENCODERLEFTB_PIN),ISR_ENCODERLEFTB,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODERRIGHTA_PIN),ISR_ENCODERRIGHTA,CHANGE);
 // attachInterrupt(digitalPinToInterrupt(ENCODERRIGHTB_PIN),ISR_ENCODERRIGHTB,CHANGE);
  if(imu1_available == true)
  {
    // Try to initialise and warn if we couldn't detect the chip
    if(imu1_initialized == false)
    {
      if (!imu1.begin())
      {
        Serial.println("Oops ... unable to initialize the imu9DS0 IMU1. Check your wiring!");
        delay(2000);
      }
      else
      {
      Serial.println("IMU1 Initialized.");
      imu1_initialized = true;
      }
    }
  }
  if(imu2_available == true)
  {
    if(imu2_initialized == false)
    {
      if (!imu2.begin())
      {
        Serial.println("Oops ... unable to initialize the imu9DS0 IMU2. Check your wiring!");
        delay(2000);
      }
      else
      {
        Serial.println("IMU2 Initialized.");
        imu2_initialized = true;
      }
    }
  }
  if(((imu1_available == true) and (imu1_initialized == false)) or ((imu2_available == true) and (imu2_initialized == false)))
  {
    setup();
  }
  configure_imus();
 
  prev_time = millis();
  led_strip.begin();
  led_strip.show();

  if(DEBUG_PRINT == 1)
  {
    print_deviceinfo();
    Serial.println("[Status]: Booting.");
  }
  run_powerup();


}


long idle_counter = 0;
long loop_counter = 0;
void loop() {
  loop_counter++;
  bool anything_ran = false;
  long now = millis();
  long dt = now - prev_time;
  prev_time = now;
  if((SPSR & (1 << SPIF)) != 0)
  {
    anything_ran = true;
    run_spi_handler = true;
    spiHandler();
    lastcom_rx = now;
  }
  else
  {
    run_spi_handler = false;
  }
  if(run_spi_handler == false)
  {
    
    fastloop_timer += dt;
    mediumloop_timer += dt;
    slowloop_timer += dt;
    veryslowloop_timer += dt;
    veryveryslowloop_timer += dt;
    if(fastloop_timer > FASTLOOP_RATE)
    {
      anything_ran = true;
      fastloop_timer = 0;
      run_fastloop(FASTLOOP_RATE);
    }
    if(mediumloop_timer > MEDIUMLOOP_RATE)
    {
      anything_ran = true;
      mediumloop_timer = 0;
      run_mediumloop(MEDIUMLOOP_RATE);
    }
    if(slowloop_timer > SLOWLOOP_RATE)
    {
      anything_ran = true;
      slowloop_timer = 0;
      run_slowloop(SLOWLOOP_RATE);
    }
    if(veryslowloop_timer > VERYSLOWLOOP_RATE)
    {
      anything_ran = true;
      veryslowloop_timer = 0;
      run_veryslowloop(VERYSLOWLOOP_RATE);
    }
    if(veryveryslowloop_timer > VERYVERYSLOWLOOP_RATE)
    {
      anything_ran = true;
      veryveryslowloop_timer = 0;
      run_veryveryslowloop(VERYVERYSLOWLOOP_RATE);
    }
    if(anything_ran == false)
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
void ISR_ENCODERLEFTA()
{
  if(digitalRead(ENCODERLEFTA_PIN) == digitalRead(ENCODERLEFTB_PIN))
  {
    LeftEncoder_pos+=1.0*LEFTENCODER_SCALE;
  }
  else
  {
    LeftEncoder_pos-=1.0*LEFTENCODER_SCALE;
  }
}
void ISR_ENCODERRIGHTA()
{
  if(digitalRead(ENCODERRIGHTA_PIN) == digitalRead(ENCODERRIGHTB_PIN))
  {
    RightEncoder_pos+=1.0*RIGHTENCODER_SCALE;
  }
  else
  {
    RightEncoder_pos-=1.0*RIGHTENCODER_SCALE;
  }
}
void spiHandler()
{
  if(marker == 0)
  {
    dat = SPDR;
    if(dat == 0xAB)
    {
      SPDR = 'a';
      marker++;
    }
  }
  else if(marker == 1)
  {
    dat = SPDR;
    current_command = dat;
    if(current_command == SPI_Diagnostic_ID)
    {
      SPI_Diagnostic_ID_rx++;
      process_AB12_Query();
    }
    else if(current_command == SPI_TestMessageCounter_ID)
    {
      SPI_TestMessageCounter_ID_rx++;
      process_AB14_Query();
    }
    else if(current_command == SPI_Get_DIO_Port1_ID)
    {
      SPI_Get_DIO_Port1_ID_rx++;
      process_AB19_Query();
    }
    else if(current_command == SPI_Get_IMUAcc_ID)
    {
      SPI_Get_IMUAcc_ID_rx++;
      process_AB27_Query();
    }
    else if(current_command == SPI_Get_IMUGyro_ID)
    {
      SPI_Get_IMUGyro_ID_rx++;
      process_AB28_Query();
    }
    else if(current_command == SPI_Get_IMUMag_ID)
    {
      SPI_Get_IMUMag_ID_rx++;
      process_AB29_Query();
    }
    else if(current_command == SPI_LEDStripControl_ID)
    {
      SPI_LEDStripControl_ID_rx++;
    }
    
    
    marker++;
  }
  
  else
  {
    if(current_command == SPI_Diagnostic_ID)
    {
      SPDR = outputBuffer_AB12[outputBuffer_index];
    }
    else if(current_command == SPI_TestMessageCounter_ID)
    {
      SPDR = outputBuffer_AB14[outputBuffer_index];
    }
    else if(current_command == SPI_Get_DIO_Port1_ID)
    {
      SPDR = outputBuffer_AB19[outputBuffer_index];
    }
    else if(current_command == SPI_Get_IMUAcc_ID)
    {
      SPDR = outputBuffer_AB27[outputBuffer_index];
    }
    else if(current_command == SPI_Get_IMUGyro_ID)
    {
      SPDR = outputBuffer_AB28[outputBuffer_index];
    }
    else if(current_command == SPI_Get_IMUMag_ID)
    {
      SPDR = outputBuffer_AB29[outputBuffer_index];
    }
    else if(current_command == SPI_LEDStripControl_ID)
    {
      dat = SPDR;
     inputBuffer_AB42[outputBuffer_index] = dat;
    }
    outputBuffer_index++;
    marker++;
    if(outputBuffer_index == 13)
    {
      if(current_command == SPI_LEDStripControl_ID)
      {
        process_AB42_Command(SPDR);
      }

      outputBuffer_index = 0;
      marker = 0;
      run_spi_handler = false;
    }
  }
  
  
}

void run_powerup()
{
  for(uint16_t i = 0; i < led_strip.numPixels(); i++)
  {
    led_strip.setPixelColor(i,led_strip.Color(0,255,0,0));
    led_strip.show();
    delay(20);
  }
  for(uint16_t i = led_strip.numPixels()-1; i >0; i--)
  {
    led_strip.setPixelColor(i,led_strip.Color(0,0,0,0));
    led_strip.show();
    delay(20);
  }
  
}
bool run_veryveryslowloop(long dt)
{
  if(DEBUG_PRINT)
  {
    Serial.println();
    Serial.print("[Time]: ");
    Serial.println(millis());
    Serial.print("SPI: 0xAB");
    Serial.print(SPI_Diagnostic_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(SPI_Diagnostic_ID_rx);
    
    Serial.print("SPI: 0xAB");
    Serial.print(SPI_TestMessageCounter_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(SPI_TestMessageCounter_ID_rx);

    Serial.print("SPI: 0xAB");
    Serial.print(SPI_Get_DIO_Port1_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(SPI_Get_DIO_Port1_ID_rx);

    Serial.print("SPI: 0xAB");
    Serial.print(SPI_Get_IMUAcc_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(SPI_Get_IMUAcc_ID_rx);

    Serial.print("SPI: 0xAB");
    Serial.print(SPI_Get_IMUGyro_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(SPI_Get_IMUGyro_ID_rx);

    Serial.print("SPI: 0xAB");
    Serial.print(SPI_Get_IMUMag_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(SPI_Get_IMUMag_ID_rx);

    Serial.print("SPI: 0xAB");
    Serial.print(SPI_Get_ANA_Port1_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(SPI_Get_ANA_Port1_ID_rx);

    Serial.print("SPI: 0xAB");
    Serial.print(SPI_LEDStripControl_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(SPI_LEDStripControl_ID_rx);
    Serial.println();
    
  }

}
bool run_veryslowloop(long dt)
{
  if(DEBUG_PRINT)
  {
    print_deviceinfo();
  }
}

bool run_slowloop(long dt)
{
  long rx_dt = millis() - lastcom_rx;
  if(DEBUG_PRINT == 1)
  {
    if((idle_counter > 0) and (loop_counter > 0))
    {
      Serial.print("[Status]: Idle Time: ");
      double idle_perc = 100.0*(double)idle_counter/(double)loop_counter;
      Serial.print(idle_perc);
      Serial.println(" %");
      
      if(rx_dt > 2500)
      {
        Serial.print("[WARN]: Haven't received any SPI Data in ");
        Serial.print(rx_dt);
        Serial.println(" mS.");
      }
    }
    
    Serial.print("Left Tick Speed: ");
    Serial.print(EncoderLeft_TickSpeed);
    Serial.print(" Left Pos: ");
    Serial.print(LeftEncoder_pos);
    Serial.print(" Right Tick Speed: ");
    Serial.print(EncoderRight_TickSpeed);
    Serial.print(" Right Pos: ");
    Serial.println(RightEncoder_pos);
  }
}
bool run_mediumloop(long dt)
{
 
  /*
  if(imu1_available == true)
  {
  Serial.print("1Accel X: "); Serial.print((int)imu1.accelData.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)imu1.accelData.y);       Serial.print(" ");
  Serial.print("Z: "); Serial.print((int)imu1.accelData.z);     Serial.print(" ");
  }
  if(imu2_available == true)
  {
  Serial.print(" 2Accel X: "); Serial.print((int)imu2.accelData.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)imu2.accelData.y);       Serial.print(" ");
  Serial.print("Z: "); Serial.print((int)imu2.accelData.z);     Serial.print(" ");
  }
  
  Serial.println();
  */
  long rx_dt = millis() - lastcom_rx;
  if(rx_dt > 10000)
  {
    ledpixel_mode = LEDPIXELMODE_ERROR;
    diag_component = GPIO_NODE;
    diag_type = COMMUNICATIONS;
    diag_level = ERROR;
    diag_message = DROPPING_PACKETS;
  }
  else if(rx_dt > 2500)
  {
    ledpixel_mode = LEDPIXELMODE_WARN;
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
    
    if(SPI_LEDStripControl_ID_rx == 0)
    {
      ledpixel_mode = LEDPIXELMODE_NORMAL;
    }
  }
  if(ledpixel_mode == LEDPIXELMODE_NONE)
  {
    led_errormodeupdate(ledpixel_mode,dt);
  }
  else if(ledpixel_mode == LEDPIXELMODE_NORMAL)
  {
    led_cylonmodeupdate();
  }
  else if(ledpixel_mode == LEDPIXELMODE_WARN)
  {
    led_errormodeupdate(ledpixel_mode,dt);
  }
  else if(ledpixel_mode == LEDPIXELMODE_ERROR)
  {
    led_errormodeupdate(ledpixel_mode,dt);
  }
  else if(ledpixel_mode == LEDPIXELMODE_FOCUS)
  {
    led_focusmodeupdate(current_ledpixel);
    if(current_ledpixel == (LEDSTRIP_PIXELCOUNT-1))
    {
      led_direction = 0;
    }
    if(current_ledpixel == 0)
    {
      led_direction = 1;
    }
    if(led_direction == 1)
    {
      current_ledpixel++;
    }
    else
    {
      current_ledpixel--;
    }
  }
  else if(ledpixel_mode == LEDPIXELMODE_COLORSELECT)
  {
    led_colorselectmodeupdate(current_ledcolor);
    current_ledcolor++;
    if(current_ledcolor > LEDPIXELCOLOR_WHITE)
    {
      current_ledcolor = 0;
    }
  }
  else
  {
    led_errormodeupdate(LEDPIXELMODE_ERROR,dt);
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
  double v1 = 1000.0*((LeftEncoder_pos - last_LeftEncoder_pos)/(double)dt);
  double v2 = 1000.0*((RightEncoder_pos - last_RightEncoder_pos)/(double)dt);
  EncoderLeft_TickSpeed = v1;
  EncoderRight_TickSpeed = v2;
  last_LeftEncoder_pos = LeftEncoder_pos;
  last_RightEncoder_pos = RightEncoder_pos;
  
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
void led_colorselectmodeupdate(int c)
{
  for(uint16_t i = 0; i < led_strip.numPixels();i++)
  {
    switch (c)
    {
      case LEDPIXELCOLOR_OFF:
        led_strip.setPixelColor(i,led_strip.Color(0,0,0,0));
        break;
         case LEDPIXELCOLOR_RED:
        led_strip.setPixelColor(i,led_strip.Color(0,255,0,0));
        break;
        case LEDPIXELCOLOR_GREEN:
        led_strip.setPixelColor(i,led_strip.Color(255,0,0,0));
        break;
        case LEDPIXELCOLOR_BLUE:
        led_strip.setPixelColor(i,led_strip.Color(0,0,255,0));
        break;
        case LEDPIXELCOLOR_WHITE:
        led_strip.setPixelColor(i,led_strip.Color(0,0,0,255));
        break;
      default:
        led_strip.setPixelColor(i,led_strip.Color(0,0,0,0));
        break;
    }
  }
  led_strip.show();
}

void led_focusmodeupdate(int ledpixel)
{
   for(uint16_t i = 0; i < led_strip.numPixels();i++)
  {
    led_strip.setPixelColor(i,led_strip.Color(0,0,0,0));
  }
  led_strip.setPixelColor(ledpixel,led_strip.Color(200,0,0,0));
  
  if(ledpixel -2 >= 0)
  {
    led_strip.setPixelColor(ledpixel-2,led_strip.Color(0,0,10,0));
  }
  
  if(ledpixel -1 >= 0)
  {
    led_strip.setPixelColor(ledpixel-1,led_strip.Color(0,0,200,0));
  }
  
  if(ledpixel +1 < led_strip.numPixels())
  {
    led_strip.setPixelColor(ledpixel+1,led_strip.Color(0,0,200,0));
  }
  if(ledpixel +2 < led_strip.numPixels())
  {
    led_strip.setPixelColor(ledpixel+2,led_strip.Color(0,0,10,0));
  }
  
  led_strip.show();
  
}
void led_errormodeupdate(int mode,long dt)
{
  led_timeduration += dt;
  if(led_timeduration > led_errortime)
  {
    led_timeduration = 0;
    if(led_state == 0)
    {
      for(int i = 0; i < led_strip.numPixels(); i++)
      {
        if(mode == LEDPIXELMODE_ERROR)
        {
          led_strip.setPixelColor(i,led_strip.Color(0,255,0,0));
        }
        else if(mode == LEDPIXELMODE_WARN)
        {
          led_strip.setPixelColor(i,led_strip.Color(255,255,0,0));
        }
      }
      led_strip.show();
      led_state = 1;
    }
    else if(led_state == 1)
    {
      for(int i = 0; i < led_strip.numPixels(); i++)
      {
         led_strip.setPixelColor(i,led_strip.Color(0,0,0,0));
      }
      led_strip.show();
      led_state = 0;
    }
  }
}
void led_cylonmodeupdate()
{

  if(current_ledpixel == 0) //First
  {
    led_strip.setPixelColor(current_ledpixel,led_strip.Color(0,255,0,0));
    led_strip.setPixelColor(current_ledpixel+1,led_strip.Color(0,20,0,0));
    led_strip.setPixelColor(current_ledpixel+2,led_strip.Color(0,0,0,0));
  }
  if(current_ledpixel == 1) //Second
  {
    led_strip.setPixelColor(current_ledpixel-1,led_strip.Color(0,0,0,0));
    led_strip.setPixelColor(current_ledpixel,led_strip.Color(0,255,0,0));
    led_strip.setPixelColor(current_ledpixel+1,led_strip.Color(0,20,0,0));
    led_strip.setPixelColor(current_ledpixel+2,led_strip.Color(0,0,0,0));
  }
  if(current_ledpixel == (LEDSTRIP_PIXELCOUNT-1)) //Second to Last
  {
    led_strip.setPixelColor(current_ledpixel-1,led_strip.Color(0,0,0,0));
    led_strip.setPixelColor(current_ledpixel,led_strip.Color(0,255,0,0));
    led_strip.setPixelColor(current_ledpixel+1,led_strip.Color(0,20,0,0));
  }
  else if(current_ledpixel == (LEDSTRIP_PIXELCOUNT-1)) //Last
  {
    led_strip.setPixelColor(current_ledpixel-2,led_strip.Color(0,0,0,0));
    led_strip.setPixelColor(current_ledpixel-1,led_strip.Color(0,20,0,0));
    led_strip.setPixelColor(current_ledpixel,led_strip.Color(0,255,0,0));
  }
  else
  {
    led_strip.setPixelColor(current_ledpixel-2,led_strip.Color(0,0,0,0));
    led_strip.setPixelColor(current_ledpixel-1,led_strip.Color(0,20,0,0));
    led_strip.setPixelColor(current_ledpixel,led_strip.Color(0,255,0,0));
    led_strip.setPixelColor(current_ledpixel+1,led_strip.Color(0,20,0,0));
    led_strip.setPixelColor(current_ledpixel+2,led_strip.Color(0,0,0,0));
  }
  led_strip.show();
  if(current_ledpixel == (LEDSTRIP_PIXELCOUNT-1))
  {
    led_direction = 0;
  }
  if(current_ledpixel == 0)
  {
    led_direction = 1;
  }
  if(led_direction == 1)
  {
    current_ledpixel++;
  }
  else
  {
    current_ledpixel--;
  }
  
}
