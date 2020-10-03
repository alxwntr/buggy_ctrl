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

void 
MotorController::coast()
{
  analogWrite(motorA_, 0);
  analogWrite(motorB_, 0);
}

void 
MotorController::brake()
  {
    analogWrite(motorA_, 255);
    analogWrite(motorB_, 255);
  }

void 
MotorController::set_pins(int& drivePin, int& gndPin, int demand)
{
  if (demand >= 0)
  {
    drivePin = motorA_;
    gndPin = motorB_;
    dir = 1;
  }
  if (demand < 0)
  {
    drivePin = motorB_;
    gndPin = motorA_;
    dir = -1;
  }
}

void 
MotorController::set_pwm(int demand, float speed)
{
  /*  At full 255 pwm, wheel speed is around 3 Hz
   *  Roughly, 3 Hz wheel freq = 150 Hz encoder freq
   */
  int     scaledSpd = map(speed, 0, 150, 0, 255);
  float   error;

  //Calculate error and set PWM level:
  error       = demand - scaledSpd;
  errorSum_   += error * dT;
  errorSum_   = constrain(errorSum_, -255, 255);
  pwm         = Kp * error + Ki * errorSum_ + Kd * (error - lastError_) / dT;
  lastError_  = error;
}

void
MotorController::write_to_pins(int gndPin, int drivePin)
{
  //Constrain PWM:
  pwm = constrain(pwm, 0, 255);

  //write to pins
  analogWrite(gndPin, 0);
  analogWrite(drivePin, pwm);
}

//PID loop:
void 
MotorController::process_pid (const geometry_msgs::Twist &twist)
{
  auto demand = twist.linear.x + (RHS ? 1 : -1) * twist.angular.z * distFromCentreline_;
  float speed  = encoder_.speed(); // (Hz for the encoder)
  int drivePin, gndPin;

  //Coast if demand is zero
  if(demand == 0)
  {
    coast();
    return;
  }

  //Brake if travelling in wrong direction
  if (speed != 0 && dir*demand < 0)
  {
    brake();
    return;
  }

  //Set drive and gnd pins
  set_pins(drivePin, gndPin, demand);
  if (demand < 0) demand = -demand;

  set_pwm(demand, speed);
  write_to_pins(gndPin, drivePin);
}
