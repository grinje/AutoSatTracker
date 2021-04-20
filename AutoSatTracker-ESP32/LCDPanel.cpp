#include "lcdpanel.h"

// Constructor
LCDPanel::LCDPanel () 
{
  if (!LCD_available) 
    return;

  lcdpanel = new LiquidCrystal_I2C(LCD_ADDRESS, 20, 4);

  lcdpanel->init(); // LiquidCrystal_I2C init
  lcdpanel->backlight();
  lcdpanel->setCursor(0,0);
  lcdpanel->print("Satellite Tracker");
  lcdpanel->setCursor(0,1);
  lcdpanel->print("by Andreas G / LB5ZH");
}

void LCDPanel::action(String text)
{
  if (!LCD_available) 
    return;

  lcdpanel->setCursor(0,1);
  lcdpanel->print(text);
  line1 = true;
}

void LCDPanel::status(String text)
{
  if (!LCD_available) 
    return;
    
  lcdpanel->setCursor(0,2);
  lcdpanel->print(text);
  line2 = true;
}

void LCDPanel::gyro(String text)
{
  if (!LCD_available) 
    return;
    
  lcdpanel->setCursor(0,3); 
  lcdpanel->print(text);
}

void LCDPanel::gps(String text)
{
  if (!LCD_available) 
    return;
    
  lcdpanel->setCursor(14,3); 
  lcdpanel->print(text);
}

void LCDPanel::gpsLock()
{
  if (!LCD_available) 
    return;
    
  lcdpanel->setCursor(17,3); 
  lcdpanel->print("+"); 
}

void LCDPanel::gpsSats(uint8_t sats)
{
  if (!LCD_available) 
    return;
    
  lcdpanel->setCursor(18,3); 
  if (sats < 10) 
     lcdpanel->print(" ");
  lcdpanel->print(sats);
}

void LCDPanel::printIP(String text)
{
  if (!LCD_available) 
    return;
    
  lcdpanel->setCursor(0,2);
  lcdpanel->print("IP                  ");
  lcdpanel->setCursor(3,2); 
  lcdpanel->print(text);
  line2 = true;
}

void LCDPanel::printTarget(String text)
{
  if (!LCD_available) 
    return;
    
  lcdpanel->setCursor(0,0);
  lcdpanel->print("                    ");
  lcdpanel->setCursor(0,0); 
  lcdpanel->print(text);  
}

void LCDPanel::printTracking(String text)
{
  if (!LCD_available) 
    return;

  lcdpanel->setCursor(19,0);
  lcdpanel->print(text);
}

void LCDPanel::print(String text)
{
  if (!LCD_available) 
    return;
    
  lcdpanel->print(text);
}

void LCDPanel::printAz(float az)
{

  if (!LCD_available) 
    return;
    
  lcdpanel->setCursor(0,3);
  if (az < 100)
    lcdpanel->print(" ");
  if (az < 10)
    lcdpanel->print(" ");   
  lcdpanel->print(az); 
}

void LCDPanel::printEl(float el)
{

  if (!LCD_available) 
    return;
    
  lcdpanel->setCursor(7,3); 
  if (el > 0) lcdpanel->print(" ");
  if (el >= 0 && el < 10) lcdpanel->print(" ");
  if (el < 0 && el > -10) lcdpanel->print(" ");
 
  lcdpanel->print(el);  
  lcdpanel->setCursor(13,3); lcdpanel->print(" ");
}

void LCDPanel::gyroLock(bool locked)
{
  if (!LCD_available) 
    return;
    
  lcdpanel->setCursor(6,3);
  if (locked)
    lcdpanel->print(" ");
  else
    lcdpanel->print("!");
}

void LCDPanel::nextRise(float v)
{
  lcdpanel->setCursor(0,1);
  if (line1)
    lcdpanel->print("                    ");
  lcdpanel->setCursor(0,1);
  lcdpanel->print("Rise in:   "); 
  bool isneg = v < 0;
  if (isneg) {
      v = -v;
      lcdpanel->print("-");
      isUp = true;
  } else {
    lcdpanel->print(" ");
    isUp = false;
  }

  uint8_t h = (uint8_t) v;
  v = (v - h)*60;
  uint8_t m = (uint8_t) v;
  v = (v - m)*60;
  uint8_t s = (uint8_t) v;
  if (h < 10) lcdpanel->print(" ");
  lcdpanel->print(h);lcdpanel->print(":");
  if (m < 10) lcdpanel->print("0");
  lcdpanel->print(m);lcdpanel->print(":");
  if (s < 10) lcdpanel->print("0");
  lcdpanel->print(s);
  line1 = false;
}

void LCDPanel::nextSet(float v)
{
  if (!isUp)
    return;
  lcdpanel->setCursor(0,2);
  if (line2)
    lcdpanel->print("                    ");
  lcdpanel->setCursor(8,2);
  lcdpanel->print("Set:"); 

  uint8_t h = (uint8_t) v;
  v = (v - h)*60;
  uint8_t m = (uint8_t) v;
  v = (v - m)*60;
  uint8_t s = (uint8_t) v;
  if (h < 10) lcdpanel->print(" ");
  lcdpanel->print(h);lcdpanel->print(":");
  if (m < 10) lcdpanel->print("0");
  lcdpanel->print(m);lcdpanel->print(":");
  if (s < 10) lcdpanel->print("0");
  lcdpanel->print(s);
  line2 = false;
}

void LCDPanel::printDuration(float v)
{
  if (isUp)
    return;
    
  lcdpanel->setCursor(0,2);
  if (line2)
    lcdpanel->print("                    ");
  lcdpanel->setCursor(8,2);
  lcdpanel->print("Dur:"); 

  uint8_t h = (uint8_t) v;
  v = (v - h)*60;
  uint8_t m = (uint8_t) v;
  v = (v - m)*60;
  uint8_t s = (uint8_t) v;
  if (h < 10) lcdpanel->print(" ");
  lcdpanel->print(h);lcdpanel->print(":");
  if (m < 10) lcdpanel->print("0");
  lcdpanel->print(m);lcdpanel->print(":");
  if (s < 10) lcdpanel->print("0");
  lcdpanel->print(s);
  line2 = false;
}

void LCDPanel::printElevation(float v)
{
  if (isUp)
    return;
    
  lcdpanel->setCursor(0,2);
  if (line2)
    lcdpanel->print("                    ");
  lcdpanel->setCursor(0,2);
  lcdpanel->print("El: "); 
  lcdpanel->print((int)round(v));
  line2 = false;
}
