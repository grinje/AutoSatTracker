/* Class to organize variables stored in EEPROM.
 * The public class variables are mirrored in RAM, so change nv then call put(), or call get() then access nv.
 * The validity of EEPROM is checked using a magic constant that must match. All values will be inited to 0.
 */

#ifndef _NV_H
#define _NV_H

#include <Adafruit_BNO055.h>
#include <WiFi.h>
#include <EEPROM.h>

#include "AutoSatTracker-ESP.h"

class NV {

    private:

  int eeAddress = 0;

    public:

  enum {
      MAGIC  = 0x5a5aa5a5,
      EEBYTES = 250,
  };

	// these variables are stored in EEPROM
	uint32_t magic;
	IPAddress IP, GW, NM;
	char ssid[64];
	char pw[64];
  uint32_t calibrationMagic = 0x00000000;
  adafruit_bno055_offsets_t calibrationData;

	NV() {
	    EEPROM.begin(EEBYTES);
	}

	void get() {
    eeAddress = 0;
    Serial.println("Reading data from EEPROM");
    EEPROM.get(eeAddress, magic); eeAddress += sizeof(uint32_t);
    EEPROM.get(eeAddress, IP); eeAddress += sizeof(IPAddress);
    EEPROM.get(eeAddress, GW); eeAddress += sizeof(IPAddress);
    EEPROM.get(eeAddress, NM); eeAddress += sizeof(IPAddress);
    EEPROM.get(eeAddress, ssid); eeAddress += 64;
    EEPROM.get(eeAddress, pw); eeAddress += 64;
    EEPROM.get(eeAddress, calibrationMagic); eeAddress += sizeof(uint32_t);
    EEPROM.get(eeAddress, calibrationData);
    Serial.print("Using SSID: "); Serial.println(ssid);

    /*
	    // fill this object from EEPROM
	    byte *this_addr = (byte *)this;
	    for (size_t i = 0; i < sizeof(*this); i++)
		this_addr[i] = EEPROM.read(i);
	    // it no magic cookie, init and save in EEPROM
	    if (magic != MAGIC) {
		memset (this, 0, sizeof(*this));
		magic = MAGIC;
		put();
	    }
     */
	}

	void put() {
    eeAddress = 0;
    Serial.println("Writing data to EEPROM"); 
    Serial.println(ssid);
    EEPROM.put(eeAddress, magic); eeAddress += sizeof(uint32_t);
    EEPROM.put(eeAddress, IP); eeAddress += sizeof(IPAddress);
    EEPROM.put(eeAddress, GW); eeAddress += sizeof(IPAddress);
    EEPROM.put(eeAddress, NM); eeAddress += sizeof(IPAddress);
    EEPROM.put(eeAddress, ssid); eeAddress += 64;
    EEPROM.put(eeAddress, pw); eeAddress += 64;
    EEPROM.put(eeAddress, calibrationMagic); eeAddress += sizeof(uint32_t);
    EEPROM.put(eeAddress, calibrationData);
    EEPROM.commit();
	}
    /*
	    // save this object in EEPROM
	    byte *this_addr = (byte *)this;
	    for (size_t i = 0; i < sizeof(*this); i++)
		EEPROM.write(i, this_addr[i]);
	    EEPROM.commit();
	}
 */
};

extern NV *nv;

#endif // _NV_H
