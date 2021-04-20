/* this class contains information about the 9 dof spatial sensor
 */

#include "Sensor.h"


/* class constructor
 */
Sensor::Sensor()
{
	// instantiate, discover and initialize
	resetWatchdog();

  // Initiate BNO compass
  bnocp = new Adafruit_BNO055(-1, BNO055_I2CCOMPASS);
  
  //bnocompass = bnocp->begin(Adafruit_BNO055::OPERATION_MODE_COMPASS);
  bnocompass = bnocp->begin(Adafruit_BNO055::OPERATION_MODE_NDOF);
  
  if (bnocompass) 
    Serial.println("Found BNO055 Compass.");

  // Initiate 9DOF sensor
	bno = new Adafruit_BNO055(-1, BNO055_I2CADDR);
	resetWatchdog();
	sensor_found = bno->begin(Adafruit_BNO055::OPERATION_MODE_NDOF);
	if (sensor_found)
	    Serial.println (F("Found 9DOF sensor."));
	else {
	    Serial.println (F("9DOF Sensor not found"));
      lcd->gyro("NO 9DOF!");
      return;
	}
	installCalibration();
  //displaySensorDetails();

  // Crystal must be configured AFTER loading calibration data into BNO055. 
  bno->setExtCrystalUse(true);
  bnocp->setExtCrystalUse(true);

  // Initiate the LSM303 compass
    /* Enable auto-gain */
  compass.enableAutoRange(true);
  /* Initialise the sensor */
  if(!compass.begin())
    LSM303_compass = false;
  else {
    LSM303_compass = true;
    Serial.println("Found LSM303 Compass.");
  }
}

/* read the current temperature, in degrees C
 */
int8_t Sensor::getTempC()
{
	if (sensor_found)
	    return bno->getTemp();
	return (-1);
}

/* return whether sensor is connected and calibrated
 */
bool Sensor::calibrated(uint8_t& sys, uint8_t& gyro, uint8_t& accel, uint8_t& mag)
{
	if (!sensor_found)
	    return (false);

	sys = 0;
	gyro = 0;
	accel = 0;
	mag = 0;;
  if (bnocompass) 
    bnocp->getCalibration(&sys, &gyro, &accel, &mag);    
  else
	  bno->getCalibration(&sys, &gyro, &accel, &mag);
	return (sys >= 1 && gyro >= 1 && accel >= 1 && mag >= 1);
}

void Sensor::displaySensorDetails(void)
{
  sensor_t sensor;
  bno->getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);

  if (LSM303_compass) {
    compass.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
  }
}

// In case you use the optional BNO 055 in compass mode
float Sensor::getCompass()
{
  imu::Vector<3> vector;
  float heading;

  if (true) {
      vector = bnocp->getVector(Adafruit_BNO055::VECTOR_EULER);
      heading = myfmod (vector.x() + circum->magdeclination +360, 360);
  } else {
    vector = bnocp->getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    heading = atan2(vector.y(), vector.x());

    // Correct for when signs are reversed.
    if(heading < 0)
      heading += 2*PI;
 
    // Convert radians to degrees
    heading = heading * 180/M_PI;
  }

  return heading;
}

// In case you have the LSM303D compass
float Sensor::getLSMcompass()
{
  sensors_event_t event;
  float heading;
      
  compass.getEvent(&event);
  heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / PI;
  // Normalize to 0-360
  if (heading < 0) {
    heading = 360 + heading;
  }
 // Serial.print("Compass: ");Serial.println(heading);
  return heading;
}

float Sensor::getEulerCompass()
{
  float heading;
  
  imu::Vector<3> eulers = bno->getVector(Adafruit_BNO055::VECTOR_EULER);
  heading = myfmod (eulers.x() + circum->magdeclination +360+90 , 360);
  //Serial.print("Euler Heading: "); Serial.println(heading); 
  return heading;
}
  
float Sensor::getElevation()
{
  sensors_event_t ev; 
  bno->getEvent(&ev);
 /* Serial.print(" X: "); Serial.print(ev.orientation.x);
  Serial.print(" Y: "); Serial.print(ev.orientation.y);
  Serial.print(" YG: "); Serial.print(ev.gyro.y);
  Serial.print(" Z: "); Serial.print(ev.orientation.z);*/
 /* for (int i = 0; i < 20; i++) {
    if ((ev.orientation.x != ev.orientation.z) && (ev.orientation.y != ev.orientation.z)) {
      if (fabs(ev.orientation.y - prevY) < 10)
        break;
       //Serial.println("Error, reading again");
    }
    delay(200);
    bno->getEvent(&ev);
 /*   Serial.print(" X: "); Serial.print(ev.orientation.x);
    Serial.print(" Y: "); Serial.print(ev.orientation.y);
     Serial.print(" YG: "); Serial.print(ev.gyro.y);
    Serial.print(" Z: "); Serial.println(ev.orientation.z);*/
  //}
  prevY = ev.orientation.y;
  return ev.orientation.y;
}
  
/* read the current az and el, corrected for mag decl but not necessarily calibrated.
 * N.B. we assume this will only be called if we know the sensor is connected.
 * N.B. Adafruit board:
 *   the short dimension is parallel to the antenna boom,
 *   the populated side of the board faces upwards and
 *   the side with the control signals (SDA, SCL etc) points in the rear direction of the antenna pattern.
 * Note that az/el is a left-hand coordinate system.
 */
void Sensor::getAzEl (float *azp, float *elp)
{  
  int i;
	//Wire.setClockStretchLimit(2000);
  // Read out elevation
  /*
	imu::Vector<3> euler = bno->getVector(Adafruit_BNO055::VECTOR_EULER);
 Serial.print("Euler y: ");Serial.print(euler.y());

  // Read 5 times to find a meaningful value
  for (i = 0; i < 20; i++) {
    if (fabs(euler.y() - prevY) < 10)
      break;
    //Serial.println("Error, reading again");
    euler = bno->getVector(Adafruit_BNO055::VECTOR_EULER);
  }
  Serial.print(" Euler y2: ");Serial.print(euler.y());
  prevY = euler.y();
  // Serial.println(*elp);
*/
  *elp = getElevation();
  lcd->printEl(*elp);
  //    Serial.print(" Euler y2: ");Serial.println(*elp);

  // Find compass heading
  for (i = 0; i < 20; i++) {
    if (bnocompass) 
      *azp = getCompass();
    else if (LSM303_compass) 
      *azp = getLSMcompass();
    else
      *azp = getEulerCompass();
      
    if (fabs(*azp - prevX) < 10)
      break;     
  }  
  prevX = *azp; 
  lcd->printAz(*azp);
  //  Serial.print(" Euler y3: ");Serial.println(*elp);

  // Serial.print("  H: "); Serial.println(*azp, 4);
 
  // Serial.println(*elp);

 /* sensors_event_t event; 
  bno->getEvent(&event);
  for (int i = 0; i < 5; i++) {
    if (fabs(event.orientation.x - prevX) < 10 && fabs(event.orientation.y - prevY) < 10)
      break;
    //Serial.println("Error, reading again");
   bno->getEvent(&event);
  }

 
  //*azp = myfmod (event.orientation.x + circum->magdeclination + 540, 360);
  *azp = myfmod(event.orientation.x +90, 360);
  //Serial.print("MagDeclination: ");Serial.println(circum->magdeclination);
  *elp = event.orientation.y;
  //Serial.print("  H: "); Serial.print(*azp);Serial.print(" ");Serial.println(event.orientation.x);

  prevX = event.orientation.x;
  prevY = event.orientation.y;
  */
  /*Serial.print("H: "); Serial.print(myfmod (euler.x() + circum->magdeclination + 540, 360));
  Serial.print("\tEULER: ");
  Serial.print("X: "); Serial.print(euler.x(), 4);
  Serial.print("\tY: "); Serial.print(euler.y(), 4);
  Serial.print("\tZ: "); Serial.println(euler.z(), 4);
/*
    sensors_event_t event; 
  bno->getEvent(&event);
  Serial.print("\tEVENT: ");
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.print("");

  // Convert quaternion to Euler, because BNO055 Euler data is broken 
  imu::Quaternion q = bno->getQuat();
  q.normalize();
  float temp = q.x();  q.x() = -q.y();  q.y() = temp;
  q.z() = -q.z();
  euler = q.toEuler();
  Serial.print(F("\tOrientation: "));
  Serial.print(-180/M_PI * euler.x());  // heading, nose-right is positive, z-axis points up
  Serial.print(F(" "));
  Serial.print(-180/M_PI * euler.y());  // roll, rightwing-up is positive, y-axis points forward
  Serial.print(F(" "));
  Serial.print(-180/M_PI * euler.z());  // pitch, nose-down is positive, x-axis points right
  Serial.print("\tH: "); Serial.print(myfmod ((-180/M_PI * euler.x()) + circum->magdeclination + 540, 360));
  Serial.println(F(""));
  */

}

/* process name = value pair
 * return whether we recognize it
 */
bool Sensor::overrideValue (char *name, char *value)
{
	if (!strcmp (name, "SS_Save")) {
	    saveCalibration();
	    webpage->setUserMessage (F("Sensor calibrations saved to EEPROM+"));
	    return (true);
	}

	return (false);
}

/* Print sensor reading on lcd */
void Sensor::printLCD ()
{
  if (!sensor_found) {
    return;
  }
  lcd_counter--;
  if (lcd_counter) 
    return;
  lcd_counter = 100;
  
  float az, el;
  getAzEl (&az, &el);
  //lcd->printAz(az);
  //lcd->printEl(el);

  // Check calibration
  uint8_t sys, gyro, accel, mag;
  bool calok = calibrated (sys, gyro, accel, mag);

  if (calok) {
    lcd->gyroLock(true);
    if (!initialized) 
      saveCalibration();
  } else {
    lcd->gyroLock(false);
    if (trycal) {
      trycal = false;
      stepper->goFullTurn();
    }
  }
}

/* send latest values to web page
 * N.B. labels must match ids in wab page
 */
void Sensor::sendNewValues (WiFiClient client)
{
	if (!sensor_found) {
	    client.println (F("SS_Status=Not found!"));
	    client.println (F("SS_Save=false"));
	    return;
	}

	float az, el;
	getAzEl (&az, &el);
	client.print (F("SS_Az=")); client.println (az);
	client.print (F("SS_El=")); client.println (el);

	uint8_t sys, gyro, accel, mag;
	bool calok = calibrated (sys, gyro, accel, mag);
	if (calok)
	    client.println (F("SS_Status=Ok+"));
	else
	    client.println (F("SS_Status=Uncalibrated!"));
	client.print (F("SS_SStatus=")); client.println (sys);
	client.print (F("SS_GStatus=")); client.println (gyro);
	client.print (F("SS_MStatus=")); client.println (mag);
	client.print (F("SS_AStatus=")); client.println (accel);

	client.print (F("SS_Save="));
  if (bnocompass) {
    if (bnocp->isFullyCalibrated())
      client.println (F("true"));
    else
      client.println (F("false"));     
  } else {
	  if (bno->isFullyCalibrated()) 
	    client.println (F("true"));
	  else
	    client.println (F("false"));
  }
  
	client.print (F("SS_Temp="));
	client.println (getTempC());
}

/* read the sensor calibration values and save into EEPROM.
 * Wanted to stick with stock Adafruit lib so pulled from
 * post by protonstorm at https://forums.adafruit.com/viewtopic.php?f=19&t=75497
 */
void Sensor::saveCalibration()
{
  /*
	// put into config mode
	bno->setMode (Adafruit_BNO055::OPERATION_MODE_CONFIG);
	delay(25);

	// request all bytes starting with the ACCEL
	byte nbytes = (byte)sizeof(nv->BNO055cal);
	Wire.beginTransmission((uint8_t)BNO055_I2CADDR);
	Wire.write((uint8_t)(Adafruit_BNO055::ACCEL_OFFSET_X_LSB_ADDR));
	Wire.endTransmission();
	Wire.requestFrom((uint8_t)BNO055_I2CADDR, nbytes);

	// wait for all 22 bytes to be available
	while (Wire.available() < nbytes);

	// copy to NV
	Serial.println (F("Saving sensor values"));
	for (uint8_t i = 0; i < nbytes; i++) {
	    nv->BNO055cal[i] = Wire.read();
	    Serial.println (nv->BNO055cal[i]);
	}
*/
  Serial.println("Fetching calibration data");
  if (bnocompass) {
    bnocp->getSensorOffsets(nv->calibrationData);
  	// restore NDOF mode
  	bnocp->setMode (Adafruit_BNO055::OPERATION_MODE_NDOF);
  	delay(25);

    // Crystal must be configured AFTER loading calibration data into BNO055. 
    bnocp->setExtCrystalUse(true);
  } else {
    bno->getSensorOffsets(nv->calibrationData);
    // restore NDOF mode
    bno->setMode (Adafruit_BNO055::OPERATION_MODE_NDOF);
    delay(25);

    // Crystal must be configured AFTER loading calibration data into BNO055. 
    bno->setExtCrystalUse(true);    
  }

	// save in EEPROM
  nv->calibrationMagic = nv->MAGIC;
	nv->put();
  initialized = true;
}

/* install previously stored calibration data from EEPROM if it looks valid.
 * Wanted to stick with stock Adafruit lib so pulled from
 * post by protonstorm at https://forums.adafruit.com/viewtopic.php?f=19&t=75497
 */
void Sensor::installCalibration()
{
	resetWatchdog();
	byte nbytes = (byte)sizeof(nv->calibrationData);

  Serial.println("Looking for calibration data in EEPROM");
	// read from EEPROM, qualify
	nv->get();
  if (nv->calibrationMagic != nv->MAGIC) {
    Serial.println("...No calibration found!");
    return;
  }

  if (bnocompass) {
    // put into config mode
    bnocp->setMode (Adafruit_BNO055::OPERATION_MODE_CONFIG);
    delay(25);
  
    Serial.println("Restoring COMPASS calibration data!");
    bnocp->setSensorOffsets(nv->calibrationData);

  	// restore NDOF mode
  	bnocp->setMode (Adafruit_BNO055::OPERATION_MODE_NDOF);
  } else {
    // put into config mode
    bno->setMode (Adafruit_BNO055::OPERATION_MODE_CONFIG);
    delay(25);
  
    Serial.println("Restoring 9DOF calibration data!");
    bno->setSensorOffsets(nv->calibrationData);

    // restore NDOF mode
    bno->setMode (Adafruit_BNO055::OPERATION_MODE_NDOF);    
  }
  
  initialized = true;
	delay(25);
}
