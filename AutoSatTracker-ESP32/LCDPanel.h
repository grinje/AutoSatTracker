#ifndef _LCDPANEL_H
#define _LCDPANEL_H

#include <LiquidCrystal_I2C.h>

static bool LCD_available = true;

#define LCD_ADDRESS 0x27

class LCDPanel 
{
  private:
    LiquidCrystal_I2C *lcdpanel;
    bool line1 = true;
    bool line2 = true;
    bool isUp = false;

  public:
    LCDPanel();
    void status(String);
    void action(String);
    void gyro(String);
    void gps(String);
    void gpsLock();
    void gpsSats(uint8_t);
    void print(String);
    void printAz(float);
    void printEl(float);
    void gyroLock(bool);
    void printIP(String);
    void printTarget(String);
    void nextRise(float);
    void nextSet(float);
    void printDuration(float);
    void printTracking(String);
    void printElevation(float);
};

extern LCDPanel *lcd;

#endif // _LCDPANEL_H
