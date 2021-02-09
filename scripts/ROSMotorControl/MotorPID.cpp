#include <Arduino.h>

#include "MotorPID.h"
#include "Chassis.h"
#include "Debug.h"

/* This turns on errors for all implicit int<->float conversions and so
 * on. I think it is worth keeping these explicit, because the results
 * can be confusing otherwise. Note that by default the Arduino build
 * tools turn all warnings off (with -w) which will prevent this; the
 * option needs to be changed in the GUI. */
#pragma GCC diagnostic error "-Wconversion"

//-------------------------
//  PID variables
//-------------------------

/* These need to be tunable from outside the MotorController in the future. */
static const float Kp = 0.0f;
static const float Ki = 5.0f;
static const float Kd = 0.0f;

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

void 
MotorController::coast()
{
  digitalWrite(motorA_, 0);
  analogWrite(motorB_, 0);
  errorSum_ = 0.0f;
}

Direction 
MotorController::find_direction(float demand)
{
  if (demand > 0)
    return Forward;

  if (demand < 0)
    return Backward;

  return Stopped;
}

int
MotorController::find_pwm(float demand, float speed)
{
  float   error, pwm;

  //Calculate error and set PWM level:
  error       = demand - speed;
  errorSum_   += error * dT;
  errorSum_   = constrain(errorSum_, -200, 200);
  pwm         = Kp * error + Ki * errorSum_ + Kd * (error - lastError_) / dT;
  lastError_  = error;

  /* XXX This truncates towards zero. Is this the right thing to do? */
  return int(pwm);
}

void
MotorController::write_to_pins(Direction dir, int pwm)
{
  //Constrain PWM:
  pwm = constrain(pwm, 0, 255);
  if(RHS) debugInfo.pwm = (buggy_ctrl::ctrl_info::_pwm_type)pwm;

  switch (dir) {
  case Forward:
    digitalWrite(motorA_, 0);
    analogWrite(motorB_, pwm);
    break;
  case Backward:
    digitalWrite(motorA_, 1);
    analogWrite(motorB_, pwm);
    break;
  case Stopped:
    coast();
    break;
  }
}

//PID loop:
void 
MotorController::process_pid (const geometry_msgs::Twist &twist)
{
  /* We don't want to work in doubles, it's expensive and not worth it. */
  float twX   = float(twist.linear.x);
  float twTh  = float(twist.angular.z);

  //Demanded floor speed for this motor
  float demandFS  = twX + (RHS ? 1 : -1) * twTh * wheelbase / 2;
  //Demanded encoder speed for this motor
  float demandEnc = demandFS * gearboxRatio / (float(PI) * wheelDia); 
  // (Revs per sec for the encoder)
  float speed     = encoder_.speed(); 

  //Coast if demand is zero
  if(demandEnc == 0)
  {
    coast();
    return;
  }

  if (speed < 0) speed = -speed;

  //Set drive and gnd pins
  auto dir  = find_direction(demandEnc);
  if (demandEnc < 0) demandEnc = -demandEnc;

  auto pwm  = find_pwm(demandEnc, speed);
  write_to_pins(dir, pwm);
}
