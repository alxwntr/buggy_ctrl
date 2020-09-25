#include "DeadReckoning.h"
#include "MotorArray.h"

const float wheelbase = 0.12; // Dist between wheel centres. CHECK THIS
const float wheelDia = 0.035;

//-------------------------
//  Movement functions
//-------------------------

void
calculate_moves(float &x, float &y, float &theta)
{
  float runningTotal = 0.0;
  float angTotal = 0.0;
  float fwdDist = 0.0;
  float angDist = 0.0;
  float dTheta = 0.0;

  for (auto m : motors)
  {
    auto wheelDist = PI * wheelDia * m.encCount * m.dir / 300;
    runningTotal += wheelDist;
    //Pos anti-clockwise, so add right, sub left
    angTotal += (m.RHS ? -1 : 1) * wheelDist;
    m.encCount = 0;
  }
  fwdDist = runningTotal / motors.size();
  //Arc length traced by one side about the centre of car
  angDist = angTotal / 2;

  dTheta = angDist / wheelbase;
  x += fwdDist * cos(theta + dTheta / 2);
  y += fwdDist * sin(theta + dTheta / 2);
  theta += dTheta;
}
