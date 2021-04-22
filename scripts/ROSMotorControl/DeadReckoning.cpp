#include <Arduino.h>

#include "DeadReckoning.h"
#include "MotorArray.h"
#include "Chassis.h"

// Odometer object:
Odometer odometer;

//-------------------------
//  Movement functions
//-------------------------

void
Odometer::calculate_moves()
{
  float runningTotal = 0.0;
  float angTotal = 0.0;
  float fwdDist = 0.0;
  float dTheta = 0.0;

  for (auto &m : motors)
  {
    auto wheelDist = PI * wheelDia * m.revolutions() / gearboxRatio;
    runningTotal += wheelDist;
    //Pos anti-clockwise, so add right, sub left
    angTotal += (m.RHS ? -1 : 1) * wheelDist;
  }
  fwdDist = runningTotal / motors.size();

  dTheta = angTotal / wheelbase;
  x += fwdDist * cos(theta + dTheta / 2);
  y += fwdDist * sin(theta + dTheta / 2);
  theta += dTheta;
}
