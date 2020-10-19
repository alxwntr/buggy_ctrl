#include <Arduino.h>

#include "DeadReckoning.h"
#include "MotorArray.h"

// Odometer object: wheelbase[m], wheel diameter[m], gearbox ratio[-]
Odometer odometer { 0.15, 0.035, 51.45 };

//-------------------------
//  Movement functions
//-------------------------

void
Odometer::calculate_moves()
{
  float runningTotal = 0.0;
  float angTotal = 0.0;
  float fwdDist = 0.0;
  float angDist = 0.0;
  float dTheta = 0.0;

  for (auto &m : motors)
  {
    auto wheelDist = PI * wheelDia_ * m.revolutions() / gearboxRatio_;
    runningTotal += wheelDist;
    //Pos anti-clockwise, so add right, sub left
    angTotal += (m.RHS ? -1 : 1) * wheelDist;
  }
  fwdDist = runningTotal / motors.size();
  //Arc length traced by one side about the centre of car
  angDist = angTotal / 2;

  dTheta = angDist / wheelbase_;
  x += fwdDist * cos(theta + dTheta / 2);
  y += fwdDist * sin(theta + dTheta / 2);
  theta += dTheta;
}
