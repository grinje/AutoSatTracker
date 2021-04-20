/* this class contains information about the 9 dof spatial sensor
 */

#ifndef _SENSOR_H
#define _SENSOR_H

#include <Wire.h>
#include <WiFiClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// In case you have this as a compass
#include <Adafruit_LSM303DLH_Mag.h>

#include "AutoSatTracker-ESP.h"
#include "Circum.h"

class Sensor {

    private:

	Adafruit_BNO055 *bno, *bnocp;		// sensor detail
  Adafruit_LSM303DLH_Mag_Unified compass = Adafruit_LSM303DLH_Mag_Unified(12345);

  bool LSM303_compass;
	bool sensor_found;		// whether sensor is connected
  bool bnocompass;
	bool calibrated(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag);
  void displaySensorDetails(void);
	enum {
	    BNO055_I2CADDR = 0x28,	// I2C bus address of BNO055
      BNO055_I2CCOMPASS = 0x29,
	};
  bool trycal = 0;
  void tryCalibrate();
  bool initialized = false;  // true when we have a calibration in nv
  float prevX = 0.0;
  float prevY = 0.0;

      // Only update the display every $lcd_counter times
    uint8_t lcd_counter = 100;


    public:

	Sensor();
	int8_t getTempC();
	void getAzEl (float *azp, float *elp);
  void printLCD ();
	void sendNewValues (WiFiClient client);
	bool connected() { return sensor_found; };
	void saveCalibration(void);
	void installCalibration(void);
	bool overrideValue (char *name, char *value);
  float getCompass();
  float getLSMcompass();
  float getEulerCompass();
  float getElevation();

};

extern Sensor *sensor;

#endif // _SENSOR_H
