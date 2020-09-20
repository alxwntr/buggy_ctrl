#ifndef DEADRECKONING_H
#define DEADRECKONING_H

#include <Arduino.h>

const int num_motors = 4;
const float wheelbase = 0.12; // Dist between wheel centres. CHECK THIS
const float wheelDia = 0.035;

/* Motor array:
    {Front left,
     Front right,
     Back left,
     Back right}

     Entries in the form [Enc, M1, M2]*/

MotorController motors[num_motors] = {
  { 0, 9, 10 },
  { 1, 11, 12 },
  { 3, 7, SCL },
  { 4, A4, A5 },
};

//-------------------------
//  Interrupt functions
//-------------------------

void ISR1() { motors[0].handle_irq(); }
void ISR2() { motors[1].handle_irq(); }
void ISR3() { motors[2].handle_irq(); }
void ISR4() { motors[3].handle_irq(); }

//-------------------------
//  Movement functions
//-------------------------

void calculate_moves(float &x, float &y, float &theta)
{
  float wheelDist[num_motors] = {0, 0, 0, 0};
  float runningTotal = 0.0;
  float angTotal = 0.0;
  float fwdDist = 0.0;
  float angDist = 0.0;
  float dTheta = 0.0;

  for (int i = 0; i < num_motors; i++)
  {
    wheelDist[i] = PI * wheelDia * motors[i].encCount / 300; //NEEDS DIRECTION
    runningTotal += wheelDist[i];
    angTotal += (i % 2 ? -1 : 1) * wheelDist[i]; //Pos anti-clockwise, so add right, sub left
    motors[i].encCount = 0;
  }
  fwdDist = runningTotal / num_motors;
  angDist = angTotal / 2; //Arc length traced by one side about the centre of car

  dTheta = angDist / wheelbase;
  x += fwdDist*cos(theta+dTheta/2);
  y += fwdDist*sin(theta+dTheta/2);
  theta += dTheta;
}

#endif
