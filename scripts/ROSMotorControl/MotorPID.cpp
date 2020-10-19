#include <Arduino.h>

#include "MotorPID.h"

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
static const float Kp = 0.5f;
static const float Ki = 10.0f;
static const float Kd = 0.01f;

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
  analogWrite(motorA_, 0);
  analogWrite(motorB_, 0);
  errorSum_ = 0.0f;
}

void 
MotorController::brake()
  {
    analogWrite(motorA_, 255);
    analogWrite(motorB_, 255);
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

  switch (dir) {
  case Forward:
    analogWrite(motorA_, pwm);
    analogWrite(motorB_, 0);
    break;
  case Backward:
    analogWrite(motorB_, pwm);
    analogWrite(motorA_, 0);
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
  float demandFS  = twX + (RHS ? 1 : -1) * twTh * distFromCentreline_;
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

  //Brake if travelling in wrong direction
  if (speed*demandEnc < 0)
  {
    brake();
    return;
  }
  if (speed < 0) speed = -speed;

  //Set drive and gnd pins
  auto dir  = find_direction(demandEnc);
  if (demandEnc < 0) demandEnc = -demandEnc;

  auto pwm  = find_pwm(demandEnc, speed);
  write_to_pins(dir, pwm);
}
