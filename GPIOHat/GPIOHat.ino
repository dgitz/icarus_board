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
//#include <Adafruit_Sensor.h>  // not used in this demo but required!
#define G_ACC 9.80655

PololuBuzzer buzzer;

//MAIN PROGRAM DEFINITIONS

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
unsigned char outputBuffer_AB47[13];

int process_AB12_Query();
int process_AB14_Query();
int process_AB19_Query();
int process_AB27_Query();
int process_AB28_Query();
int process_AB29_Query();
int process_AB47_Query();
long I2C_TestMessageCounter_ID_rx = 0;
long I2C_Diagnostic_ID_rx = 0;
long I2C_Get_DIO_Port1_ID_rx = 0;
long I2C_TestProgram_ID_rx = 0;
byte transmit_testcounter = 0;
void i2c_rx();
void i2c_tx();

//Test Program Variables/Definitions
unsigned int testprogram_state = TESTPROGRAMSTATE_START; //Run Test Program at boot
unsigned int testprogram_stepcounter = 0;
unsigned int testprogram_steplimit = 4; // Number of different tests to run

//Sonar Variables/Definitions
unsigned int FLSonar_Distance = 0;
unsigned int FRSonar_Distance = 0;
unsigned int BLSonar_Distance = 0;
unsigned int BRSonar_Distance = 0;

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
int process_AB47_Query()
{
  if(testprogram_state == TESTPROGRAMSTATE_IDLE)
  {
    testprogram_state = TESTPROGRAMSTATE_START;
  }
  int msg_length;
  encode_TestProgramI2C(outputBuffer_AB47,&msg_length,
    testprogram_state);
  if((testprogram_state == TESTPROGRAMSTATE_PASSED) or (testprogram_state == TESTPROGRAMSTATE_FAILED))
  {
    testprogram_state = TESTPROGRAMSTATE_IDLE;
  }
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
    else if(c == I2C_TestProgram_ID)
    {
      I2C_TestProgram_ID_rx++;
      current_command = I2C_TestProgram_ID;
      process_AB47_Query();
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
  else if(current_command == I2C_TestProgram_ID)
  {
    Wire.write(outputBuffer_AB47,13);
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
    Serial.print(I2C_TestProgram_ID,HEX);
    Serial.print(" Rx: ");
    Serial.println(I2C_TestProgram_ID_rx);

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
  if(testprogram_state == TESTPROGRAMSTATE_START)
  {
    testprogram_stepcounter = 0;
    testprogram_state = TESTPROGRAMSTATE_RUNNING;
  }
  else if(testprogram_state == TESTPROGRAMSTATE_RUNNING)
  {
    testprogram_stepcounter += 1;
    if(testprogram_stepcounter > testprogram_steplimit)
    {
      testprogram_stepcounter = 0;
      testprogram_state = TESTPROGRAMSTATE_PASSED;
    }
    if(testprogram_stepcounter == 1)
    {
      for(int i = 0; i < 10; ++i)
      {
        int v = get_distance_in(FLSONAR);
        if(v == 0)
        {
          testprogram_state = TESTPROGRAMSTATE_FAILED;
        }
        delay(500);
      }
    }
    else if(testprogram_stepcounter == 2)
    {
      for(int i = 0; i < 10; ++i)
      {
        int v = get_distance_in(BLSONAR);
        if(v == 0)
        {
          testprogram_state = TESTPROGRAMSTATE_FAILED;
        }
        delay(500);
      }
    }
    else if(testprogram_stepcounter == 3)
    {
      for(int i = 0; i < 10; ++i)
      {
        int v = get_distance_in(BRSONAR);
        if(v == 0)
        {
          testprogram_state = TESTPROGRAMSTATE_FAILED;
        }
        delay(500);
      }
    }
    else if(testprogram_stepcounter == 4)
    {
      for(int i = 0; i < 10; ++i)
      {
        int v = get_distance_in(FRSONAR);
        if(v == 0)
        {
          testprogram_state = TESTPROGRAMSTATE_FAILED;
        }
        delay(500);
      }
    }
  }


}
bool run_mediumloop(long dt)
{
  long rx_dt = millis() - lastcom_rx;
  if(testprogram_state == TESTPROGRAMSTATE_IDLE)
  {
    FLSonar_Distance = get_distance_in(FLSONAR);
    FRSonar_Distance = get_distance_in(FRSONAR);
    BLSonar_Distance = get_distance_in(BLSONAR);
    BRSonar_Distance = get_distance_in(BRSONAR);
  }
  if(rx_dt > 10000)
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
  duration = pulseIn(pin, HIGH,2000000);
  
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
