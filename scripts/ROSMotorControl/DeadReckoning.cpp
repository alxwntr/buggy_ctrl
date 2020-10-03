#include <Arduino.h>

#include "DeadReckoning.h"
#include "MotorArray.h"

Odometer odometer { 0.2, 0.035, 51.45 }; // Dist between wheel centres: CHECK THIS

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
    // XXX where does this 300 come from?
    auto wheelDist = PI * wheelDia_ * m.revolutions() / gearboxRatio_;
    debug = m.dir;
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
