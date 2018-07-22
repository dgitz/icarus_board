
#include <Adafruit_NeoPixel.h>
#include "eROS_Definitions.h"
#include "spimessage.h"
#include <SPI.h>
#include "config.h"

//MAIN PROGRAM DEFINITIONS
#define SLOWLOOP_RATE 1000 //mS
#define MEDIUMLOOP_RATE 200 //mS
Adafruit_NeoPixel led_strip = Adafruit_NeoPixel(LEDSTRIP_PIXELCOUNT, LEDSTRIP_PIN, NEO_RGBW + NEO_KHZ800);

//Timing Variables
long prev_time = 0;
long slowloop_timer = 0;
long mediumloop_timer = 0;
long fastloop_timer = 0;

//SPI Variables
char buf [100];
volatile byte pos;
volatile boolean process_it;

//LED Strip Variables
enum LEDPIXEL_MODE {
  LEDPIXELMODE_NONE,
  LEDPIXELMODE_CYLON,
  LEDPIXELMODE_ERROR,
  LEDPIXELMODE_COLORSELECT,
  LEDPIXELMODE_FOCUS,
};
enum LEDPIXEL_COLOR {
  LEDPIXEL_COLOR_OFF,
  LEDPIXEL_COLOR_RED,
  LEDPIXEL_COLOR_GREEN,
  LEDPIXEL_COLOR_BLUE,
  LEDPIXEL_COLOR_WHITE
};
int current_ledpixel = 0;
bool led_direction = 1;
int led_timeduration = 0;
LEDPIXEL_MODE ledpixel_mode = LEDPIXELMODE_CYLON;
int current_ledcolor = LEDPIXEL_COLOR_OFF;
int led_errortime = 1000; //mS
int led_state = 0;
void setup() {
  Serial.begin(115200);
  SPCR |= bit (SPE);
   // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  
  // get ready for an interrupt 
  pos = 0;   // buffer empty
  process_it = false;

  // now turn on interrupts
  SPI.attachInterrupt();

  
  prev_time = millis();
  led_strip.begin();
  led_strip.show();

  run_powerup();


}
ISR (SPI_STC_vect)
{
byte c = SPDR;  // grab byte from SPI Data Register
  
  // add to buffer if room
  if (pos < (sizeof (buf) - 1))
    buf [pos++] = c;
    
  // example: newline means time to process buffer
  if (c == '\n')
    process_it = true;
      
}  // end of interrupt routine SPI_STC_vect


void loop() {
  long dt = millis() - prev_time;
  prev_time = millis();

  if (process_it)
    {
    buf [pos] = 0;  
    Serial.println (buf);
    pos = 0;
    process_it = false;
    }  // end of flag set
  mediumloop_timer += dt;
  if(mediumloop_timer > MEDIUMLOOP_RATE)
  {
    mediumloop_timer = 0;
    run_mediumloop(MEDIUMLOOP_RATE);
  }
  delay(1);
  
  

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

bool run_mediumloop(long dt)
{
  if(ledpixel_mode == LEDPIXELMODE_CYLON)
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
    led_colorselectmodeupdate(current_ledcolor);
    current_ledcolor++;
    if(current_ledcolor > LEDPIXEL_COLOR_WHITE)
    {
      current_ledcolor = 0;
    }
  }
  
}
void led_colorselectmodeupdate(LEDPIXEL_COLOR c)
{
  for(uint16_t i = 0; i < led_strip.numPixels();i++)
  {
    switch (c)
    {
      case LEDPIXEL_COLOR_OFF:
        led_strip.setPixelColor(i,led_strip.Color(0,0,0,0));
        break;
         case LEDPIXEL_COLOR_RED:
        led_strip.setPixelColor(i,led_strip.Color(0,255,0,0));
        break;
        case LEDPIXEL_COLOR_GREEN:
        led_strip.setPixelColor(i,led_strip.Color(255,0,0,0));
        break;
        case LEDPIXEL_COLOR_BLUE:
        led_strip.setPixelColor(i,led_strip.Color(0,0,255,0));
        break;
        case LEDPIXEL_COLOR_WHITE:
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
    Serial.println("b");
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

