#include <Arduino.h>

#include "MotorPID.h"

//-------------------------
//  PID variables
//-------------------------

static const float Kp = 0.5;
static const float Ki = 10.0;
static const float Kd = 0.01;

//-------------------
// MotorController Class
//-------------------

void 
MotorController::setup_pins (void(*isr)(void))
{
  //Motor pins
  pinMode(motorA_, OUTPUT);
  pinMode(motorB_, OUTPUT);

  encoder_.setup_pins(isr);
}

//PID loop:
void 
MotorController::process_pid (const geometry_msgs::Twist &twist)
{
  auto demand = twist.linear.x + (RHS ? 1 : -1) * twist.angular.z;
  auto speed  = encoder_.speed();

  //Coast if demand is zero
  if(demand == 0)
  {
    analogWrite(motorA_, 0);
    analogWrite(motorB_, 0);
    return;
  }
  
  //Brake if travelling in wrong direction
  if (speed != 0 && dir*demand < 0)
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
  /* XXX This 170 should be the max expected encoder speed. How is it
   * derived? */
  int     scaledSpd = map(speed, 0, 170, 0, 255);
  float   error;

  //Calculate error and set PWM level:
  error       = demand - scaledSpd;
  errorSum_   += error * dT;
  errorSum_   = constrain(errorSum_, -255, 255);
  pwm         = Kp * error + Ki * errorSum_ + Kd * (error - lastError_) / dT;
  lastError_  = error;

  //Constrain PWM and ensure it gets to zero (immediately):
  if (demand == 0) pwm = 0;
  if (pwm < 0)        pwm = 0;
  if (pwm > 255)      pwm = 255;
  analogWrite(drivePin_, pwm);
}
