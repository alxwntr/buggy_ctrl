#include "DeadReckoning.h"

const float wheelbase = 0.12; // Dist between wheel centres. CHECK THIS
const float wheelDia = 0.035;

/* Motor array:
    {Front left,
     Front right,
     Back left,
     Back right}

     Entries in the form [Enc, M1, M2]*/

MotorController motors[num_motors] = {
  { A0, 12, 11 },
  { A1, 10, 9 },
  { A2, 2, 4 },
  { A3, A4, A5 },
}; //Pins verified as working motor by motor, need to test all four together

//-------------------------
//  Interrupt functions
//-------------------------

static void ISR1() { motors[0].handle_irq(); }
static void ISR2() { motors[1].handle_irq(); }
static void ISR3() { motors[2].handle_irq(); }
static void ISR4() { motors[3].handle_irq(); }

void
setup_motors()
{
    motors[0].setup_pins(ISR1);
    motors[1].setup_pins(ISR2);
    motors[2].setup_pins(ISR3);
    motors[3].setup_pins(ISR4);
}

//-------------------------
//  Movement functions
//-------------------------

void
calculate_moves(float &x, float &y, float &theta)
{
  float wheelDist[num_motors] = {0, 0, 0, 0};
  float runningTotal = 0.0;
  float angTotal = 0.0;
  float fwdDist = 0.0;
  float angDist = 0.0;
  float dTheta = 0.0;

  for (int i = 0; i < num_motors; i++)
  {
    wheelDist[i] = PI * wheelDia * motors[i].encCount * motors[i].dir / 300;
    runningTotal += wheelDist[i];
    angTotal += (i % 2 ? -1 : 1) * wheelDist[i]; //Pos anti-clockwise, so add right, sub left
    motors[i].encCount = 0;
  }
  fwdDist = runningTotal / num_motors;
  angDist = angTotal / 2; //Arc length traced by one side about the centre of car

  dTheta = angDist / wheelbase;
  x += fwdDist * cos(theta + dTheta / 2);
  y += fwdDist * sin(theta + dTheta / 2);
  theta += dTheta;
}
