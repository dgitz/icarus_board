
#include <Adafruit_NeoPixel.h>

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

//LED Strip Variables
enum LEDPIXEL_MODE {
  LEDPIXELMODE_NONE,
  LEDPIXELMODE_CYLON,
  LEDPIXELMODE_ERROR,
};
int current_ledpixel = 0;
bool led_direction = 1;
int led_timeduration = 0;
LEDPIXEL_MODE ledpixel_mode = LEDPIXELMODE_CYLON;
int led_errortime = 1000; //mS
int led_state = 0;
void setup() {
  Serial.begin(115200);
  prev_time = millis();
  led_strip.begin();
  led_strip.show();

  run_powerup();


}

void loop() {
  long dt = millis() - prev_time;
  prev_time = millis();

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

