
#include <Adafruit_NeoPixel.h>
#include "eROS_Definitions.h"
#include "spimessage.h"
#include <SPI.h>
#include "config.h"

//MAIN PROGRAM DEFINITIONS
#define VERYSLOWLOOP_RATE 10000 //mS
#define SLOWLOOP_RATE 1000 //mS
#define MEDIUMLOOP_RATE 200 //mS
Adafruit_NeoPixel led_strip = Adafruit_NeoPixel(LEDSTRIP_PIXELCOUNT, LEDSTRIP_PIN, NEO_RGBW + NEO_KHZ800);

//Timing Variables
long prev_time = 0;
long veryslowloop_timer = 0;
long slowloop_timer = 0;
long mediumloop_timer = 0;
long fastloop_timer = 0;

//SPI Variables
//Defines for SPI Comm between Raspberry Pi and Arduino Board
//unsigned char transmitBuffer[14];
//unsigned char receiveBuffer[14];
long lastcom_rx = 0;
unsigned char outputBuffer_AB14[13];
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
int process_AB14_Query();
int process_AB19_Query();
int process_AB20_Query();
int process_AB41_Query();
int process_AB42_Command(int checksum);
byte transmit_testcounter = 0;

int current_ledpixel = 0;
bool led_direction = 1;
int led_timeduration = 0;
unsigned char ledpixel_mode = LEDPIXELMODE_ERROR;
int current_ledcolor = LEDPIXELCOLOR_OFF;
int led_errortime = 1000; //mS
int led_state = 0;

//Message processing functions.  This should be as fast as possible
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
  long now = millis();
  long dt = now - prev_time;
  prev_time = now;

  if((SPSR & (1 << SPIF)) != 0)
  {
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
    mediumloop_timer += dt;
    slowloop_timer += dt;
    veryslowloop_timer += dt;
    if(mediumloop_timer > MEDIUMLOOP_RATE)
    {
      mediumloop_timer = 0;
      run_mediumloop(MEDIUMLOOP_RATE);
    }
    else if(slowloop_timer > SLOWLOOP_RATE)
    {
      slowloop_timer = 0;
      run_slowloop(SLOWLOOP_RATE);
    }
    else if(veryslowloop_timer > VERYSLOWLOOP_RATE)
    {
      veryslowloop_timer = 0;
      run_veryslowloop(VERYSLOWLOOP_RATE);
    }
    else
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
    if(current_command == SPI_TestMessageCounter_ID)
    {
      process_AB14_Query();
    }
    
    marker++;
  }
  else
  {
    if(current_command == SPI_TestMessageCounter_ID)
    {
      SPDR = outputBuffer_AB14[outputBuffer_index];
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
    if(rx_dt > 5000)
    {
      ledpixel_mode = LEDPIXELMODE_ERROR;
    }
  }
}
bool run_mediumloop(long dt)
{
  if(ledpixel_mode == LEDPIXELMODE_NONE)
  {
    led_errormodeupdate(dt);
  }
  else if(ledpixel_mode == LEDPIXELMODE_NORMAL)
  {
    led_cylonmodeupdate();
  }
  else if(ledpixel_mode == LEDPIXELMODE_ERROR)
  {
    led_errormodeupdate(dt);
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
    Serial.print(current_ledcolor);
    led_colorselectmodeupdate(current_ledcolor);
    current_ledcolor++;
    if(current_ledcolor > LEDPIXELCOLOR_WHITE)
    {
      current_ledcolor = 0;
    }
  }
  else
  {
    led_errormodeupdate(dt);
  }
  
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
void led_errormodeupdate(long dt)
{
  led_timeduration += dt;
  if(led_timeduration > led_errortime)
  {
    led_timeduration = 0;
    if(led_state == 0)
    {
      for(int i = 0; i < led_strip.numPixels(); i++)
      {
         led_strip.setPixelColor(i,led_strip.Color(0,255,0,0));
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

