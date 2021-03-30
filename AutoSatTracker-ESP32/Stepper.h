// class to control two motors to track a target az and el using the A4988 stepper controller

#ifndef _STEPPER_H
#define _STEPPER_H

#include <Wire.h>
#include <WiFiClient.h>

#include "AutoSatTracker-ESP.h"
#include "Target.h"
#include "Sensor.h"

class StepController {

    private:

      // Define pin connections & motor's steps per revolution
      // Stepper motor A 
      static const uint8_t dirPinA = 14; // Direction of step
      static const uint8_t stepPinA = 32; // Initiate a step
      static const uint8_t speedA_MS1 = 27; // MS1
      static const uint8_t speedA_MS2 = 33; // MS2
      static const uint8_t speedA_MS3 = 15; // MS3

      // Stepper motor B
      static const uint8_t dirPinB = 21;
      static const uint8_t stepPinB = 12;
      static const uint8_t speedB_MS1 = 26;
      static const uint8_t speedB_MS2 = 25;
      static const uint8_t speedB_MS3 = 4;

      // Connects to the RESET pin 
      static const uint8_t enableMotor = 13; // Enable the step controllers

      // In case you use the endstoppers
      static const uint8_t endstopA = 34;  // End-stop switch for stepper A
      static const uint8_t endstopB = 39;  // End-stop switch for stepper B
      
      boolean stepper_found = 1; // Do we find the stepper motors?
      static const boolean endstopSwitches = false; // We don´t use these switches

      static const int calibrateSteps = 4000; // # Number of steps during calibration

      static const int stepDelay = 1000; // Microseconds to keep the step pin high
 
      // motor info
      typedef struct {
        float az_scale, el_scale;     // az and el scale: steps (del usec) per degree
        uint16_t min, max;        // position limits, usec
        uint16_t pos;       // last commanded position, usec
        int16_t del_pos;        // change in pos since previous move
        bool atmin, atmax;        // (would have been commanded to) limit
        uint8_t step_num;        // Stepper address
        uint8_t dirPin;
        uint8_t stepPin;
        uint8_t MS1Pin;
        uint8_t MS2Pin;
        uint8_t MS3Pin;
      } MotorInfo;
 
      static const uint8_t NMOTORS = 2;   // not easily changed
      MotorInfo motor[NMOTORS];

      // search info
      static const uint16_t UPD_PERIOD = 500;   // ms between updates
      static constexpr float MAX_SETTLE = 2.0;  // considered stopped, degs
      static const uint8_t N_INIT_STEPS = 4;    // number of init_steps
      static constexpr float CAL_FRAC = 0.333;  // fraction of full range to move for calibration
              // N.B.: max physical motion must be < 180/CAL_FRAC

      uint8_t init_step;        // initialization sequencing
      uint8_t best_azmotor;       // after cal, motor[] index with most effect in az
      uint32_t last_update;       // millis() time of last move
      uint32_t last_sensorread;
      float prevfast_az, prevfast_el;     // previous pointing position
      float prevstop_az, prevstop_el;     // previous stopped position
      float prevsensor_az;
      float min_az, min_el;     // endstop positions

      void setMotorPosition (uint8_t motn, uint16_t newpos);
      void calibrate (LiquidCrystal_I2C& lcd, float &az_s, float &el_s);
      void seekTarget (float& az_t, float& el_t, float& az_s, float& el_s);
      float azDist (float &from, float &to);
      float moveDist (float &from, float &to);
      float moveAbs0 (float &from, float &to);

      void speed_Full(uint8_t motn);
      void speed_Half(uint8_t motn);
      void speed_Quarter(uint8_t motn);
      void speed_Eight(uint8_t motn);
      void speed_Sixteenth(uint8_t motn);

      boolean endstopPressed();
      void runStepMotor(uint8_t motn, int steps);
      void gotoMin(uint8_t motn);

  
    public:

      StepController();
      void goFullTurn();
      void moveToAzEl (LiquidCrystal_I2C& lcd, float az_t, float el_t);
      void sendNewValues (WiFiClient client);
      bool connected() { return stepper_found; };
      bool calibrated() { return (init_step >= N_INIT_STEPS); }
};

extern StepController *stepper;

#endif // _STEPPER_H
