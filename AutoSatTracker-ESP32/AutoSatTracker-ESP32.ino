/* Autonomous Satellite Tracker.
 *
 * Connections on Adafruit ESP8266 Huzzah:
 *   Servo controller connects to SDA and SCL
 *   GPS TX connects to 16, RX to 2
 *
 */

#include "LiquidCrystal_I2C.h"
#include "AutoSatTracker-ESP.h"
#include "NV.h"
#include "Sensor.h"
#include "Circum.h"
//#include "Gimbal.h"
#include "Stepper.h"
#include "Target.h"
#include "Webpage.h"

#define LCD_ADDRESS 0x27

// Initiate LCD screen
LiquidCrystal_I2C lcd(LCD_ADDRESS, 20, 4);

NV *nv;
Sensor *sensor;
Circum *circum;
//Gimbal *gimbal;
StepController *stepper;
Target *target;
Webpage *webpage;
uint8_t lcdprint = 10;

/* called first and once
 */
void
setup()
{
    lcd.init(); // LiquidCrystal_I2C init
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("Satellite Tracker");
    lcd.setCursor(0,1);
    lcd.print("by Andreas G / LB5ZH");
 
    // init serial monitor
    Serial.begin (115200);
    delay(500);

    // this just resets the soft timeout, the hard timeout is still 6 seconds
    //ESP.wdtDisable();

    // scan for I2C devices
    resetWatchdog();
    Serial.println();
    lcd.setCursor(0,2);lcd.print("Scanning I2C");
    Serial.print (F("...Scanning I2C:"));
    for (uint8_t a = 1; a < 127; a++) {
	Wire.beginTransmission(a);
	if (Wire.endTransmission() == 0) {
	    Serial.print (' '); Serial.print (a, HEX);
	}
    }
    Serial.println();

    // instantiate each module
    Serial.println (F("making NV"));
    nv = new NV();
    Serial.println (F("making Sensor"));
    lcd.setCursor(0,3); lcd.print("9DOF      ");
    sensor = new Sensor(lcd);
    Serial.println (F("making Circum"));
    lcd.setCursor(14,3); lcd.print("NO GPS");
    circum = new Circum(lcd);
    Serial.println (F("making StepController")); 
    stepper = new StepController();
    Serial.println (F("making Target"));
    target = new Target();
    Serial.println (F("making Webpage"));
    webpage = new Webpage(lcd);

    // all set to go.. see you in loop()
    Serial.println (F("Ready"));

}

/* called repeatedly forever
 */
void loop()
{   
    // check for ethernet activity
    webpage->checkEthernet();

    // check for new GPS info
    circum->checkGPS(lcd);

    // print out Sensor values on LCD
    sensor->printLCD(lcd);

    // follow the target
    target->track(lcd);
    delay(1);
}

void
resetWatchdog()
{
    //ESP.wdtFeed();
    yield();
}
