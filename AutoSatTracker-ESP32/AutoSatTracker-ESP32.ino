/* Autonomous Satellite Tracker.
 *
 * Connections on Adafruit ESP8266 Huzzah:
 *   Servo controller connects to SDA and SCL
 *   GPS TX connects to 16, RX to 2
 *
 */

#include "LCDPanel.h"
#include "AutoSatTracker-ESP.h"
#include "NV.h"
#include "Sensor.h"
#include "Circum.h"
#include "Stepper.h"
#include "Target.h"
#include "Webpage.h"


NV *nv;
Sensor *sensor;
Circum *circum;
//Gimbal *gimbal;
StepController *stepper;
Target *target;
Webpage *webpage;
LCDPanel *lcd;


/* called first and once
 */
void
setup()
{
  // init serial monitor
  Serial.begin (115200);
  delay(500);

  // init LCD screen
  lcd = new LCDPanel();

  // this just resets the soft timeout, the hard timeout is still 6 seconds
  //ESP.wdtDisable();

  // scan for I2C devices
  resetWatchdog();
  Serial.println();
  lcd->status("Scanning I2C");
   
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
  lcd->gyro("9DOF      ");

  sensor = new Sensor();
  Serial.println (F("making Circum"));
  lcd->gps("NO GPS");

    circum = new Circum();
    Serial.println (F("making StepController")); 
    stepper = new StepController();
    Serial.println (F("making Target"));
    target = new Target();
    Serial.println (F("making Webpage"));
    webpage = new Webpage();

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
    circum->checkGPS();

    // print out Sensor values on LCD
    sensor->printLCD();

    // follow the target
    target->track();
    delay(1);
}

void
resetWatchdog()
{
    //ESP.wdtFeed();
    yield();
}
