#ifndef MOTORPID_H
#define MOTORPID_H

#include <Arduino.h>

//-------------------------
//  PID variables
//-------------------------

const float dT = 0.02; //50Hz - can't go too high because of lack of encoder pulses
const float Kp = 0.5;
const float Ki = 10.0;
const float Kd = 0.01;

//-------------------------
//  Motor control variables
//-------------------------

int linDmdMax = 200;
int angDmdMax = 100;

class MotorController {

    const int encPin_;
    const int motorA_;
    const int motorB_;

    volatile unsigned long lastTime_ = 0;
    volatile double spd_ = 0;
    volatile int count_ = 0;

    float lastError_ = 0.0, errorSum_ = 0.0;

    int drivePin_, gndPin_;

  public:
  
    int dir = 0; // -1 = bkwrds, 0 = static, 1 = frwrds (need to think how to sort this out)

    volatile float encCount = 0.0;

    MotorController (int encPin, int motorA, int motorB)
      : encPin_(encPin), motorA_(motorA), motorB_(motorB)
    { }

    void setup_pins (void(*isr)(void))
    {
      //Motor pins
      pinMode(motorA_, OUTPUT);
      pinMode(motorB_, OUTPUT);
      //Encoder pin
      pinMode(encPin_, INPUT);

      attachInterrupt(digitalPinToInterrupt(encPin_), isr, CHANGE);
    }

    //PID loop:
    void process_pid (int demand)
    {
      //Allow spd_to reach zero if no motion
      if ((micros() - lastTime_) > 50000)
      {
        spd_ = 0;
        dir = 0;
      }

      //Coast if demand is zero
      if(demand == 0)
      {
        analogWrite(motorA_, 0);
        analogWrite(motorB_, 0);
        return;
      }
      
      //Brake if travelling in wrong direction
      if (spd_ != 0 && dir*demand < 0)
      {
        analogWrite(motorA_, 255);
        analogWrite(motorB_, 255);
        return;
      }

      //Set drive and gnd pins
      if (demand >= 0)
      {
        drivePin_ = motorA_;
        gndPin_ = motorB_;
        analogWrite(gndPin_, 0);
        dir = 1;
      }
      if (demand < 0)
      {
        drivePin_ = motorB_;
        gndPin_ = motorA_;
        analogWrite(gndPin_, 0);
        dir = -1;
        demand = -demand;
      }

      //Temporary variables:
      int     pwm;
      int     scaledSpd = map(spd_, 0, 170, 0, 255);
      float   error;

      //Calculate error and set PWM level:
      error       = demand - scaledSpd;
      errorSum_   += error * dT;
      pwm         = Kp * error + Ki * errorSum_ + Kd * (error - lastError_) / dT;
      lastError_  = error;

      //Constrain PWM and ensure it gets to zero (immediately):
      if (demand == 0) pwm = 0;
      if (pwm < 0)        pwm = 0;
      if (pwm > 255)      pwm = 255;
      analogWrite(drivePin_, pwm);
    }

    //Called from ISR#
    void handle_irq ()
    {
      unsigned long   timeNow;
      float           tickTime;

      //Do nothing for first 2 times:
      count_++;
      encCount++;
      if (count_ < 3) return;
      count_ = 0;

      //Set time variables and calculate speed:
      timeNow     = micros();
      tickTime    = float(timeNow - lastTime_);
      spd_        = 1000000 / tickTime; // gives Hz for 0.5 encoder revs. RECONSIDER
      lastTime_   = timeNow;
    }
};

void set_levels(geometry_msgs::Twist &confirm, int dmd[], int num_motors)
{
  for (int i = 0; i < num_motors; i++)
  {
    dmd[i] = confirm.linear.x + (i % 2 ? 1 : -1) * confirm.angular.z;
  }
}

#endif
