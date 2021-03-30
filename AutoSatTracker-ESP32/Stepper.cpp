// class to control two motors to track a target az and el using the A4988 stepper contoller
#include "Stepper.h"

/* issue raw motor command in microseconds pulse width, clamped at limit
 */
void StepController::setMotorPosition (uint8_t motn, uint16_t newpos)
{
  if (motn >= NMOTORS)
      return;
  MotorInfo *mip = &motor[motn];

  mip->atmin = (newpos <= mip->min);
  if (mip->atmin)
      newpos = mip->min;
  mip->atmax = (newpos >= mip->max);
  if (mip->atmax)
      newpos = mip->max;

  mip->del_pos = (int)newpos - (int)mip->pos;
  mip->pos = newpos;
  //pwm->setPWM(mip->step_num, 0, mip->pos/US_PER_BIT);
  // Serial.print(mip->step_num); Serial.print(" "); Serial.println (newpos);
}

void StepController::speed_Full(uint8_t motn) { digitalWrite(motor[motn].MS1Pin, LOW); digitalWrite(motor[motn].MS2Pin, LOW); digitalWrite(motor[motn].MS3Pin, LOW); }
void StepController::speed_Half(uint8_t motn) { digitalWrite(motor[motn].MS1Pin, HIGH); digitalWrite(motor[motn].MS2Pin, LOW); digitalWrite(motor[motn].MS3Pin, LOW); }
void StepController::speed_Quarter(uint8_t motn) { digitalWrite(motor[motn].MS1Pin, LOW); digitalWrite(motor[motn].MS2Pin, HIGH); digitalWrite(motor[motn].MS3Pin, LOW); }
void StepController::speed_Eight(uint8_t motn) { digitalWrite(motor[motn].MS1Pin, HIGH); digitalWrite(motor[motn].MS2Pin, HIGH); digitalWrite(motor[motn].MS3Pin, LOW); }
void StepController::speed_Sixteenth(uint8_t motn) { digitalWrite(motor[motn].MS1Pin, HIGH); digitalWrite(motor[motn].MS2Pin, HIGH); digitalWrite(motor[motn].MS3Pin, HIGH); }

boolean StepController::endstopPressed() {
  resetWatchdog();

  // Are we using physical endstop switches?
  if (endstopSwitches) {
    if (!digitalRead(endstopA)) {
       // Serial.println("Endstop A");
       // Serial.println(endstopA);
        delayMicroseconds(stepDelay);
        return true;
      }

      if (!digitalRead(endstopB)) {
      //  Serial.println("Endstop B");
      //   Serial.println(endstopB);
      delayMicroseconds(stepDelay);
      return true;
    }
    return false;
  }

  // only check at some given time interval
  uint32_t now = millis();
  if (now - last_sensorread < (UPD_PERIOD * 3))
      return false;
      
  last_sensorread = now;

  // Turn off our motors that act as huge magnets
  digitalWrite(enableMotor, LOW);
  delay(1);
  
  // read current sensor direction
  float azimuth, elevation;
  sensor->getAzEl (&azimuth, &elevation);  

  // Remember to enable our motors again
  digitalWrite(enableMotor, HIGH);
  delay(1);

  // Are we hitting our virtual azimuth endstop?
  if ((prevsensor_az > 300 && azimuth < 50) || (prevsensor_az < 50 && azimuth > 300)) {
    Serial.print("Hitting North Azimuth endstop!");Serial.print(prevsensor_az, 4);Serial.print(" "); Serial.println(azimuth);
    delay(500);
    return true;
  }
  Serial.print("Prev: ");Serial.print(prevsensor_az, 4); Serial.print(" Now: "); Serial.println(azimuth, 4);

  prevsensor_az = azimuth;

  // Or are we hitting our virtual elevation switch?
  //Serial.print("Elevation: "); Serial.println(elevation);
  if (elevation < -5 || elevation > 75) {
    return true;
  }

  // No endstop, keep turning
  return false;
}

void StepController::gotoMin(uint8_t motn)
{
  int i = 0;
  
  // set motor direction
  Serial.print("Setting pin HIGH: "); Serial.println(motor[motn].dirPin);
  digitalWrite(motor[motn].dirPin, HIGH);

  // set speed
  speed_Half(motn);

  // enable motor
  digitalWrite(enableMotor, HIGH);

  // Turn until we hit the endstop
  while (!endstopPressed()) {
    resetWatchdog();
   // Serial.print(motn);Serial.print(" FORWARD ");
    //Serial.println(motor[motn].stepPin);
    digitalWrite(motor[motn].stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(motor[motn].stepPin, LOW);
    delayMicroseconds(stepDelay);
  }

  // Lets move out of endstop
  Serial.print("Setting pin LOW: "); Serial.println(motor[motn].dirPin);
  digitalWrite(motor[motn].dirPin, LOW);
  digitalWrite(enableMotor, LOW);delay(UPD_PERIOD * 3);digitalWrite(enableMotor, HIGH);
  while (endstopPressed()) {
    resetWatchdog();
   // Serial.print(motn);Serial.print(" BACKWARD ");
   // Serial.println(motor[motn].stepPin);
   for (i = 0; i < 100; i++) {
    digitalWrite(motor[motn].stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(motor[motn].stepPin, LOW);
    delayMicroseconds(stepDelay);
   }
   digitalWrite(enableMotor, LOW);delay(UPD_PERIOD * 3);digitalWrite(enableMotor, HIGH);
  } 
  // Lets move a safe distance from the endstop (so not to influnce the other axis when testing that)
  for (i = 0; i < 500; i++) {
    digitalWrite(motor[motn].stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(motor[motn].stepPin, LOW);
    delayMicroseconds(stepDelay);
   }

  // disable motor
  digitalWrite(enableMotor, LOW);

}

void StepController::runStepMotor(uint8_t motn, int steps)
{
  /*
   * AZ: HIGH turning left, atmax on left side 
   * EL: HIHG turning down
   */
  bool direction = (steps >= 0 ? LOW : HIGH);
  Serial.print("Motor: "); Serial.print(motn); Serial.print("\tSteps: ");Serial.print(steps);Serial.print(" Direction: "); Serial.println(direction);

  // If at end position, just stop turning
/*  if (direction && motor[motn].atmin) {
    Serial.println("At end position, doing nothing");
    return;
  }

  if ((!direction) && motor[motn].atmax) {
    Serial.println("At end of the world, doing nothing");
    return;
  }

  // Reset end signal, as we are moving away
  motor[motn].atmax = false;
  motor[motn].atmin = false;*/
  
  // set motor direction
  digitalWrite(motor[motn].dirPin, direction);
    

  // set speed
  speed_Half(motn);

  // enable motor
  digitalWrite(enableMotor, HIGH);
  delayMicroseconds(stepDelay);

  // Turn the number of steps
  steps = abs(steps);
  while (steps--) {
    resetWatchdog();
    digitalWrite(motor[motn].stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(motor[motn].stepPin, LOW);
    delayMicroseconds(stepDelay);
    
    if (endstopPressed()) {
      Serial.println("Reached the endstop! Turning back.");
      // Bummer, we reached a full turn. Lets move out of endstop
      digitalWrite(motor[motn].dirPin, !direction);
      speed_Half(motn);
      while (endstopPressed()) {
        resetWatchdog();
        digitalWrite(motor[motn].stepPin, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(motor[motn].stepPin, LOW);
        delayMicroseconds(stepDelay);
      } 
      
      if (!direction)
        motor[motn].atmax = true;
      else
        motor[motn].atmin = true;
        
      // disable motor
      digitalWrite(enableMotor, LOW);
      return;
    }
  }

  // disable motor
  digitalWrite(enableMotor, LOW);
}

// An attempt to calibrate the magnetometer
void StepController::goFullTurn()
{
  gotoMin(1);
  runStepMotor(1, -50000);
  gotoMin(1); 
  runStepMotor(1, -50000);
  gotoMin(1); 
}

/* constructor 
 */
StepController::StepController ()
{
  resetWatchdog();

  // record axis assignments
 // motor[0].step_num = stepPinA;
 // motor[1].step_num = stepPinB;

  // init each motor state
  nv->get();
//  motor[0].min = nv->mot0min;
//  motor[0].max = nv->mot0max;
  motor[0].pos = 0;
  motor[0].atmin = false;
  motor[0].atmax = false;
  motor[0].az_scale = 0;
  motor[0].el_scale = 0;
  motor[0].dirPin = dirPinA;
  motor[0].stepPin = stepPinA;
  motor[0].MS1Pin = speedA_MS1;
  motor[0].MS2Pin = speedA_MS2;
  motor[0].MS3Pin = speedA_MS3;

 // motor[1].min = nv->mot1min;
 // motor[1].max = nv->mot1max;
  motor[1].pos = 0;
  motor[1].atmin = false;
  motor[1].atmax = false;
  motor[1].az_scale = 0;
  motor[1].el_scale = 0;
  motor[1].dirPin = dirPinB;
  motor[1].stepPin = stepPinB;
  motor[1].MS1Pin = speedB_MS1;
  motor[1].MS2Pin = speedB_MS2;
  motor[1].MS3Pin = speedB_MS3;

  // init to arbitrary, but at least defined, state
  init_step = 0;
  best_azmotor = 0;
  last_update = 0;
  prevfast_az = prevfast_el = -1000;
  prevstop_az = prevstop_el = -1000;
  prevsensor_az = 200;

  // Declare GPIO pins as Outputs
  pinMode(stepPinA, OUTPUT);
  pinMode(dirPinA, OUTPUT);
  pinMode(speedA_MS1, OUTPUT);
  pinMode(speedA_MS2, OUTPUT);
  pinMode(speedA_MS3, OUTPUT);

  pinMode(stepPinB, OUTPUT);
  pinMode(dirPinB, OUTPUT);
  pinMode(speedB_MS1, OUTPUT);
  pinMode(speedB_MS2, OUTPUT);
  pinMode(speedB_MS3, OUTPUT);

  pinMode(enableMotor, OUTPUT);

  // Endstop switches
  pinMode(endstopA, INPUT_PULLUP);
  pinMode(endstopB, INPUT_PULLUP);

  // Put it in a known, stopped state
  digitalWrite(stepPinA, LOW);
  digitalWrite(stepPinB, LOW);
  digitalWrite(dirPinA, HIGH);
  digitalWrite(dirPinB, HIGH);
  digitalWrite(enableMotor, LOW);
}

/* move motors towards the given new target az and el 
 */
void StepController::moveToAzEl (LiquidCrystal_I2C& lcd, float az_t, float el_t)
{
  // only update every UPD_PERIOD
  uint32_t now = millis();
  if (now - last_update < UPD_PERIOD)
      return;
  last_update = now;

  // read current sensor direction
  float az_s, el_s;
  sensor->getAzEl (&az_s, &el_s);

  // relcalibrate endstop position if needed
  Serial.print("Read AZ: "); Serial.println(az_s);
  if (motor[best_azmotor].atmax || motor[best_azmotor].atmin)
    min_az = az_s;

  // only check further when motion has stopped as evidenced by stable sensor values
  if (fabs (azDist (prevfast_az, az_s)) < MAX_SETTLE && fabs (el_s - prevfast_el) < MAX_SETTLE) {

      // calibrate if not already else seek target
      if (!calibrated()) {
        lcd.setCursor(0,1); lcd.print("Calibrating motors  ");
        calibrate (lcd, az_s, el_s);

      }  else {
        lcd.setCursor(0,1); lcd.print("Tracking target!     ");
        if (el_t < 0)
          el_t = 0;
          seekTarget (az_t, el_t, az_s, el_s);

      }  

      // preserve for next stopped iteration
      Serial.print("Setting prev AZ: "); Serial.println(az_s);
      prevstop_az = az_s;
      prevstop_el = el_s;

  }

  // preserve for next fast iteration
  prevfast_az = az_s;
  prevfast_el = el_s;
}

/* run the next step of the initial scale calibration series.
 * steps proceed using init_step up to N_INIT_STEPS
 */
void StepController::calibrate (LiquidCrystal_I2C& lcd, float& az_s, float& el_s)
{
  // handy step ranges
  uint16_t range0 = motor[0].max - motor[0].min;
  uint16_t range1 = motor[1].max - motor[1].min;

  switch (init_step++) {

  case 0:
      lcd.setCursor(19,1); lcd.print("0");
      Serial.println("Calibrate 0");
      // move near min of each range
      gotoMin(0);
      gotoMin(1);
   
      break;

  case 1:
      // store minimum values
      min_az = az_s;
      min_el = el_s;
      prevstop_az = az_s;
      prevstop_el = el_s;
      Serial.print("Case 1: Setting prev AZ: "); Serial.println(az_s);

      
      lcd.setCursor(19,1); lcd.print("1");
      Serial.println("Calibrate 1");
      // move just motor 0 a subtantial distance
      /*
      Serial.print(F("Init 1: Mot 0 starts at:\t"));
    Serial.print(az_s); Serial.print(F("\t"));
    Serial.print (el_s); Serial.print(F("\tMoves\t"));
    Serial.println(CAL_FRAC*range0, 0);
      */
      runStepMotor(0, (calibrateSteps * 1));

      break;

  case 2:
      lcd.setCursor(19,1); lcd.print("2");
      Serial.println("Calibrate 2");
      // calculate scale of motor 0
      motor[0].az_scale = fabs(calibrateSteps/azDist(prevstop_az, az_s));
      motor[0].el_scale = fabs(calibrateSteps/(el_s - prevstop_el));
     Serial.print(F("Calibrate 0 AZ scale ")); Serial.print(az_s); Serial.print(F("\tPrevstop: "));Serial.print(prevstop_az);Serial.print(F("\t Dist: ")); Serial.print(az_s - prevstop_az); Serial.print("\t Scale: ");Serial.println(motor[0].az_scale);
     Serial.print(F("Calibrate 0 EL scale ")); Serial.print(el_s); Serial.print(F("\tPrevstop: "));Serial.print(prevstop_el);Serial.print(F("\t Dist: ")); Serial.print(el_s - prevstop_el); Serial.print("\t Scale: ");Serial.println(motor[0].el_scale);
      /*
      Serial.print(F("Init 2: Mot 0 ended  at:\t"));
    Serial.print(az_s); Serial.print(F("\t"));
    Serial.print (el_s); Serial.print(F("\tusec:\t"));
    Serial.print (CAL_FRAC*range0); Serial.print (F("\tDel usec/Deg:\t"));
    Serial.print (motor[0].az_scale); Serial.print (F("\t"));
    Serial.println (motor[0].el_scale);
      */

      // repeat procedure for motor 1
      /*
      Serial.print(F("Init 2: Mot 1 starts at:\t"));
    Serial.print(az_s); Serial.print(F("\t"));
    Serial.print (el_s); Serial.print(F("\tMoves\t"));
    Serial.println(CAL_FRAC*range1, 0);
      */
      
      runStepMotor(1, (calibrateSteps * 1));
      break;

  case 3:
      lcd.setCursor(19,1); lcd.print("3");
        Serial.println("Calibrate 3");
      // calculate scale of motor 1
      //motor[1].az_scale = CAL_FRAC*range1/azDist(prevstop_az, az_s);
      //motor[1].el_scale = CAL_FRAC*range1/(el_s - prevstop_el);
      motor[1].az_scale = fabs(calibrateSteps/azDist(prevstop_az, az_s));
      motor[1].el_scale = fabs(calibrateSteps/(el_s - prevstop_el));
      Serial.print(F("Calibrate 1 AZ scale ")); Serial.print(az_s); Serial.print(F("\tPrevstop: "));Serial.print(prevstop_az);Serial.print(F("\t Dist: ")); Serial.print(az_s - prevstop_az); Serial.print("\t Scale: ");Serial.println(motor[1].az_scale);
      Serial.print(F("Calibrate 1 EL scale ")); Serial.print(el_s); Serial.print(F("\tPrevstop: "));Serial.print(prevstop_el);Serial.print(F("\t Dist: ")); Serial.print(el_s - prevstop_el); Serial.print("\t Scale: ");Serial.println(motor[1].el_scale);
     
 
      /*
      Serial.print(F("Init 3: Mot 1 ended  at:\t"));
    Serial.print(az_s); Serial.print(F("\t"));
    Serial.print (el_s); Serial.print(F("\tusec:\t"));
    Serial.print (CAL_FRAC*range1); Serial.print (F("\tDel usec/Deg:\t"));
    Serial.print (motor[1].az_scale); Serial.print (F("\t"));
    Serial.println (motor[1].el_scale);
      */

      // select best motor for az
      best_azmotor = fabs(motor[0].az_scale) < fabs(motor[1].az_scale) ? 0 : 1;
      
      // Override the whole calibration
      best_azmotor = 1;
      motor[1].az_scale = 60.0;
      motor[1].el_scale = 30.0;
      motor[0].az_scale = 60.0;
      motor[0].el_scale = 30.0;

      Serial.print (F("Best Az motor:\t"));
      Serial.print (best_azmotor); Serial.print (F("\tScale:\t"));
      Serial.print (motor[best_azmotor].az_scale);
      Serial.print (F("\tEl motor:\t"));
      Serial.print (!best_azmotor); Serial.print (F("\tScale:\t"));
      Serial.println (motor[!best_azmotor].el_scale);

      // report we have finished calibrating
      target->setTrackingState (true);
      break;

  default:
      lcd.setCursor(19,1); lcd.print("!");
      webpage->setUserMessage (F("BUG! Bogus init_step"));
      break;
  }
}

/* run the next step of seeking the given target given the current stable az/el sensor values
 */
void StepController::seekTarget (float& az_t, float& el_t, float& az_s, float& el_s)
{
  // find pointing error in each dimension as a move from sensor to target
  //float az_err = moveDist (az_s, az_t);
  float az_err = moveAbs0 (az_s, az_t);
  float el_err = el_t - el_s;

  // correct each error using motor with most effect in that axis
  MotorInfo *azmip = &motor[best_azmotor];
  MotorInfo *elmip = &motor[!best_azmotor];

  
  Serial.print (F("Az:\t"));
  Serial.print(az_s); Serial.print(F("\tTarget: "));
  Serial.print(az_t); Serial.print (F("\tDist: "));
  Serial.print(az_err, 1); Serial.print (F("\tSteps: "));
  Serial.print(az_err*azmip->az_scale, 0);
  Serial.print("\tMIN AZ: ");Serial.print(min_az);
  Serial.print(F("\tEl:\t"));
  Serial.print(el_s); Serial.print(F("\t"));
  Serial.print(el_t); Serial.print (F("\t"));
  Serial.print(el_err, 1); Serial.print (F("\t"));
  Serial.println(el_err*elmip->el_scale, 0);


  // tweak scale if move was substantial and sanity check by believing only small changes
  const float MIN_ANGLE = 30;   // min acceptable move
  const float MAX_CHANGE = 0.1;   // max fractional scale change
  float az_move = azDist (prevstop_az, az_s);
  if (fabs(az_move) >= MIN_ANGLE) {
      float new_az_scale = azmip->del_pos/az_move;
      if (fabs((new_az_scale - azmip->az_scale)/azmip->az_scale) < MAX_CHANGE) {
        Serial.print (F("New Az scale\t"));
        Serial.print (azmip->az_scale); Serial.print (F("\t->\t"));
        Serial.println(new_az_scale);
        azmip->az_scale = fabs(new_az_scale);
      }
  }
  float el_move = el_s - prevstop_el;
  if (fabs(el_move) >= MIN_ANGLE) {
      float new_el_scale = elmip->del_pos/el_move;
      if (fabs((new_el_scale - elmip->el_scale)/elmip->el_scale) < MAX_CHANGE) {
        Serial.print (F("New El scale\t"));
        Serial.print (elmip->el_scale); Serial.print (F("\t->\t"));
        Serial.println(new_el_scale);
        elmip->el_scale = fabs(new_el_scale);
      }
  }


  // move each motor to reduce error, but if at Az limit then swing back to near opposite limit
  //if (azmip->atmin) {
      // Serial.println (F("At Az Min"));
    //  setMotorPosition (best_azmotor, azmip->min + 0.8*(azmip->max - azmip->min));
  //} else if (azmip->atmax) {
      // Serial.println (F("At Az Max"));
    //  setMotorPosition (best_azmotor, azmip->min + 0.2*(azmip->max - azmip->min));
  //} else
      //setMotorPosition (best_azmotor, azmip->pos + az_err*azmip->az_scale);
  //setMotorPosition (!best_azmotor, elmip->pos + el_err*elmip->el_scale);
  runStepMotor(best_azmotor, az_err*azmip->az_scale);

  if (el_t < 0) 
    return;
  if (el_t > 80)
    return;

  runStepMotor(!best_azmotor, (el_err * elmip->el_scale));

}

/* given two azimuth values, return path length going shortest direction, keeping track of the endstop switches
 */
float StepController::moveDist (float &from, float &to)
{
  float d = to - from;
  if (to > min_az && from <= min_az)
    d = ((360 - to) + from) * -1;
  else if (to < min_az && from >= min_az)
    d = (360 - from) + to;
 
  return (d);
}

float StepController::moveAbs0(float &from, float &to)
{
    return (to - from);  
}

float StepController::azDist (float &from, float &to)
{
  float d = to - from;
  if (d < -180)
      d += 360;
  else if (d > 180)
      d -= 360;
      
  return (d);
}


/* send latest web values.
 * N.B. must match id's in main web page
 */
void StepController::sendNewValues (WiFiClient client)
{
 
 /* client.print (F("G_Mot1Pos="));
  if (init_step > 0)
      client.println (motor[0].pos);
  else
      client.println (F(""));
  client.print (F("G_Mot1Min=")); client.println (motor[0].min);
  client.print (F("G_Mot1Max=")); client.println (motor[0].max);

  client.print (F("G_Mot2Pos="));
  if (init_step > 0)
      client.println (motor[1].pos);
  else
      client.println (F(""));
  client.print (F("G_Mot2Min=")); client.println (motor[1].min);
  client.print (F("G_Mot2Max=")); client.println (motor[1].max);

  client.print (F("G_Status="));
      if (motor[0].atmin)
    client.println (F("1 at Min!"));
      else if (motor[0].atmax)
    client.println (F("1 at Max!"));
      else if (motor[1].atmin)
    client.println (F("2 at Min!"));
      else if (motor[1].atmax)
    client.println (F("2 at Max!"));
      else
    client.println (F("Ok+"));
*/
}
