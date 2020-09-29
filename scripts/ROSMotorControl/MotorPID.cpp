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
  //Encoder pin
  pinMode(encPin_, INPUT);

  attachInterrupt(digitalPinToInterrupt(encPin_), isr, CHANGE);
}

//PID loop:
void 
MotorController::process_pid (const geometry_msgs::Twist &twist)
{
  auto demand  = twist.linear.x + (RHS ? 1 : -1) * twist.angular.z;

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
  errorSum_   = constrain(errorSum_, -255, 255);
  pwm         = Kp * error + Ki * errorSum_ + Kd * (error - lastError_) / dT;
  lastError_  = error;

  //Constrain PWM and ensure it gets to zero (immediately):
  if (demand == 0) pwm = 0;
  if (pwm < 0)        pwm = 0;
  if (pwm > 255)      pwm = 255;
  analogWrite(drivePin_, pwm);
}

void
MotorController::reset_count()
{
  encCount = 0;
}

//Called from ISR#
void 
MotorController::handle_irq ()
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
